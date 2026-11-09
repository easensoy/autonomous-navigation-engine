// graph_navigator: the ROS 2 adapter around this repository's NavigationCore.
//
// Inputs   odom (nav_msgs/Odometry)          pose feedback; in the trials the fault injector relays the
//                                            simulator's ground-truth odometry here
//          scan (sensor_msgs/LaserScan)      lidar; unexplained returns become SENSOR blockages
//          ane/blockage_reports              OPERATOR or SCRIPTED blockage notifications
//          ane/start_mission, ane/stop, ane/retry (services)
// Outputs  cmd_vel_nav (geometry_msgs/Twist) to cmd_vel_gate
//          ane/status, ane/events            state and every transition, plan, replan and blockage
//          ane/planned_route, ane/invalidated_route (nav_msgs/Path, latched)
//          ane/graph_markers, ane/blocked_markers (latched), ane/live_markers  for RViz2
//
// All planning goes through ane::MissionExecutive, which calls NavigationCore (A* via the
// DynamicReplanner). This node only converts messages, applies the static map->odom and
// base->lidar transforms and publishes what the executive reports.
#include "ane_core/blockage_mapper.hpp"
#include "ane_core/mission_executive.hpp"
#include "ane_core/roadmap.hpp"

#include <ane_interfaces/msg/blockage_report.hpp>
#include <ane_interfaces/msg/navigator_event.hpp>
#include <ane_interfaces/msg/navigator_status.hpp>
#include <ane_interfaces/srv/start_mission.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

namespace {

using ane_interfaces::msg::BlockageReport;
using ane_interfaces::msg::NavigatorEvent;
using ane_interfaces::msg::NavigatorStatus;
using ane_interfaces::srv::StartMission;

builtin_interfaces::msg::Time toStamp(double seconds) {
    return rclcpp::Time(static_cast<int64_t>(std::llround(seconds * 1e9)), RCL_ROS_TIME);
}

// 2-D rigid transform: p_parent = R(yaw) p_child + (x, y).
struct Transform2 {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    ane::Pose2 apply(const ane::Pose2& p) const {
        const double c = std::cos(yaw);
        const double s = std::sin(yaw);
        return {c * p.x - s * p.y + x, s * p.x + c * p.y + y, ane::wrapAngle(p.yaw + yaw)};
    }
};

Transform2 toTransform2(const geometry_msgs::msg::TransformStamped& t) {
    return {t.transform.translation.x, t.transform.translation.y, tf2::getYaw(t.transform.rotation)};
}

std_msgs::msg::ColorRGBA rgba(float r, float g, float b, float a = 1.0f) {
    std_msgs::msg::ColorRGBA c;
    c.r = r;
    c.g = g;
    c.b = b;
    c.a = a;
    return c;
}

}  // namespace

class GraphNavigatorNode : public rclcpp::Node {
public:
    GraphNavigatorNode() : Node("graph_navigator") {
        const std::string roadmapFile = declare_parameter<std::string>("roadmap_file", "");
        map_frame_ = declare_parameter<std::string>("map_frame", "map");
        base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
        const double controlRate = declare_parameter<double>("control_rate_hz", 20.0);
        scan_pose_tolerance_ = declare_parameter<double>("scan_pose_tolerance_s", 0.1);

        ane::ExecutiveConfig e;
        e.association_radius = declare_parameter<double>("association_radius_m", e.association_radius);
        e.goal_tolerance = declare_parameter<double>("goal_tolerance_m", e.goal_tolerance);
        e.stop_arrival_radius = declare_parameter<double>("stop_arrival_radius_m", e.stop_arrival_radius);
        e.max_cross_track = declare_parameter<double>("max_cross_track_m", e.max_cross_track);
        e.progress_timeout = declare_parameter<double>("progress_timeout_s", e.progress_timeout);
        e.progress_distance = declare_parameter<double>("progress_distance_m", e.progress_distance);
        e.feedback_timeout = declare_parameter<double>("feedback_timeout_s", e.feedback_timeout);
        e.recovery_window = declare_parameter<double>("recovery_window_s", e.recovery_window);
        e.rejoin_radius = declare_parameter<double>("rejoin_radius_m", e.rejoin_radius);
        e.turn_threshold = declare_parameter<double>("turn_threshold_rad", e.turn_threshold);
        ane::ControllerConfig& c = e.controller;
        c.limits.max_linear = declare_parameter<double>("max_linear_mps", c.limits.max_linear);
        c.limits.max_angular = declare_parameter<double>("max_angular_radps", c.limits.max_angular);
        c.limits.max_linear_accel = declare_parameter<double>("max_linear_accel_mps2", c.limits.max_linear_accel);
        c.limits.max_linear_decel = declare_parameter<double>("max_linear_decel_mps2", c.limits.max_linear_decel);
        c.limits.max_angular_accel = declare_parameter<double>("max_angular_accel_radps2", c.limits.max_angular_accel);
        c.lookahead = declare_parameter<double>("lookahead_m", c.lookahead);
        c.rotate_enter = declare_parameter<double>("rotate_enter_rad", c.rotate_enter);
        c.rotate_exit = declare_parameter<double>("rotate_exit_rad", c.rotate_exit);
        c.rotate_gain = declare_parameter<double>("rotate_gain", c.rotate_gain);
        c.min_rotate_rate = declare_parameter<double>("min_rotate_rate_radps", c.min_rotate_rate);
        c.creep_speed = declare_parameter<double>("creep_speed_mps", c.creep_speed);
        c.stop_tolerance = declare_parameter<double>("stop_tolerance_m", c.stop_tolerance);

        ane::BlockageMapperConfig m;
        m.blocking_radius = declare_parameter<double>("blocking_radius_m", m.blocking_radius);
        m.static_tolerance = declare_parameter<double>("static_tolerance_m", m.static_tolerance);
        m.min_points = static_cast<int>(declare_parameter<int64_t>("min_points", m.min_points));
        m.consecutive_scans = static_cast<int>(declare_parameter<int64_t>("consecutive_scans", m.consecutive_scans));

        ane::Footprint footprint;
        footprint.length = declare_parameter<double>("footprint_length_m", footprint.length);
        footprint.width = declare_parameter<double>("footprint_width_m", footprint.width);
        const double margin = declare_parameter<double>("clearance_margin_m", 0.10);

        if (roadmapFile.empty()) {
            throw std::runtime_error("parameter roadmap_file is required");
        }
        roadmap_ = std::make_shared<const ane::Roadmap>(ane::Roadmap::load(roadmapFile));
        const ane::ClearanceRequirement need = ane::clearanceRequirement(footprint, e.max_cross_track, margin);
        const ane::ClearanceReport clearance = ane::checkClearance(*roadmap_, need);
        if (!clearance.violations.empty()) {
            for (const std::string& v : clearance.violations) {
                RCLCPP_FATAL(get_logger(), "clearance: %s", v.c_str());
            }
            throw std::runtime_error("roadmap does not meet the footprint clearance contract");
        }
        RCLCPP_INFO(get_logger(),
                    "roadmap %s: %zu nodes, %zu edges; min clearance %.3f m at nodes (need %.3f), %.3f m along edges (need %.3f)",
                    roadmap_->name().c_str(), roadmap_->graph().getNodeCount(), roadmap_->edges().size(),
                    clearance.min_node_clearance, need.node, clearance.min_edge_clearance, need.edge);

        exec_ = std::make_unique<ane::MissionExecutive>(roadmap_, e);
        mapper_ = std::make_unique<ane::BlockageMapper>(*roadmap_, m);

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        const auto latched = rclcpp::QoS(1).reliable().transient_local();
        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel_nav", 10);
        status_pub_ = create_publisher<NavigatorStatus>("ane/status", 20);
        event_pub_ = create_publisher<NavigatorEvent>("ane/events", rclcpp::QoS(500).reliable());
        route_pub_ = create_publisher<nav_msgs::msg::Path>("ane/planned_route", latched);
        invalidated_pub_ = create_publisher<nav_msgs::msg::Path>("ane/invalidated_route", latched);
        // Separate latched topics: a latched topic keeps only its newest message, so the static roadmap and
        // the changing blocked-node markers must not share one.
        graph_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("ane/graph_markers", latched);
        blocked_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("ane/blocked_markers", latched);
        live_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("ane/live_markers", 10);

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "odom", rclcpp::QoS(50), [this](const nav_msgs::msg::Odometry& msg) { onOdom(msg); });
        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", rclcpp::SensorDataQoS(), [this](const sensor_msgs::msg::LaserScan& msg) { onScan(msg); });
        report_sub_ = create_subscription<BlockageReport>(
            "ane/blockage_reports", rclcpp::QoS(50).reliable(), [this](const BlockageReport& msg) { onReport(msg); });
        start_srv_ = create_service<StartMission>(
            "ane/start_mission", [this](const std::shared_ptr<StartMission::Request> request,
                                        std::shared_ptr<StartMission::Response> response) { onStart(*request, *response); });
        stop_srv_ = create_service<std_srvs::srv::Trigger>(
            "ane/stop", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
                exec_->stop(now().seconds(), "ane/stop service");
                publishCommand({});
                response->success = true;
                response->message = "stopped; zero velocity until a new mission";
                flush();
            });
        retry_srv_ = create_service<std_srvs::srv::Trigger>(
            "ane/retry", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
                response->success = exec_->retry(now().seconds());
                response->message = response->success ? "route restored" : "no route (or not BLOCKED)";
                flush();
            });

        period_ = 1.0 / controlRate;
        timer_ = rclcpp::create_timer(this, get_clock(), rclcpp::Duration::from_seconds(period_), [this] { onControl(); });
        publishGraph();
        publishBlocked();
    }

private:
    void onOdom(const nav_msgs::msg::Odometry& msg) {
        if (!map_from_odom_) {
            try {
                map_from_odom_ = toTransform2(tf_buffer_->lookupTransform(map_frame_, msg.header.frame_id, tf2::TimePointZero));
                RCLCPP_INFO(get_logger(), "static %s -> %s: x %.3f y %.3f yaw %.4f", map_frame_.c_str(),
                            msg.header.frame_id.c_str(), map_from_odom_->x, map_from_odom_->y, map_from_odom_->yaw);
            } catch (const tf2::TransformException& error) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "waiting for %s -> %s: %s", map_frame_.c_str(),
                                     msg.header.frame_id.c_str(), error.what());
                return;
            }
        }
        const ane::Pose2 inOdom{msg.pose.pose.position.x, msg.pose.pose.position.y, tf2::getYaw(msg.pose.pose.orientation)};
        exec_->updatePose(rclcpp::Time(msg.header.stamp).seconds(), map_from_odom_->apply(inOdom));
    }

    void onScan(const sensor_msgs::msg::LaserScan& scan) {
        if (!exec_->hasPose()) {
            return;
        }
        if (!base_from_lidar_) {
            try {
                base_from_lidar_ = toTransform2(tf_buffer_->lookupTransform(base_frame_, scan.header.frame_id, tf2::TimePointZero));
            } catch (const tf2::TransformException& error) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "waiting for %s -> %s: %s", base_frame_.c_str(),
                                     scan.header.frame_id.c_str(), error.what());
                return;
            }
        }
        const double stamp = rclcpp::Time(scan.header.stamp).seconds();
        if (std::abs(stamp - exec_->poseStamp()) > scan_pose_tolerance_) {
            ++scans_skipped_;
            return;  // no pose close enough in time to place this scan
        }
        const ane::Pose2 sensor = Transform2{exec_->pose().x, exec_->pose().y, exec_->pose().yaw}.apply(
            {base_from_lidar_->x, base_from_lidar_->y, base_from_lidar_->yaw});
        const std::vector<ane::Point2> points = ane::scanToMapPoints(
            sensor, scan.angle_min, scan.angle_increment, scan.ranges, scan.range_min, scan.range_max);
        const ane::ScanBlockages result = mapper_->processScan(points);
        publishUnexplained(result.unexplained, scan.header.stamp);
        if (!result.newly_blocked.empty()) {
            std::ostringstream detail;
            detail << result.unexplained.size() << " of " << result.valid_points
                   << " returns unexplained by the static map in the scan stamped " << std::fixed << stamp;
            exec_->reportBlockage(now().seconds(), result.newly_blocked, "SENSOR", detail.str());
            flush();
        }
    }

    void onReport(const BlockageReport& msg) {
        if (msg.source == "SENSOR") {
            RCLCPP_WARN(get_logger(), "ignoring a blockage report with source SENSOR: sensor blockages come from the lidar only");
            return;
        }
        std::vector<int> nodes(msg.node_ids.begin(), msg.node_ids.end());
        if (msg.use_region) {
            const std::vector<int> inside = roadmap_->nodesInRect({msg.min_x, msg.min_y, msg.max_x, msg.max_y});
            nodes.insert(nodes.end(), inside.begin(), inside.end());
        }
        const std::string source = msg.source.empty() ? "OPERATOR" : msg.source;
        exec_->reportBlockage(now().seconds(), nodes, source, msg.label);
        flush();
    }

    void onStart(const StartMission::Request& request, StartMission::Response& response) {
        const int goal = request.goal_station.empty() ? request.goal_node : roadmap_->station(request.goal_station);
        if (goal < 0) {
            response.accepted = false;
            response.message = "unknown goal station '" + request.goal_station + "'";
            return;
        }
        const ane::StartResult result = exec_->start(now().seconds(), goal);
        response.accepted = result.accepted;
        response.message = result.message;
        response.start_node = result.start_node;
        response.goal_node = result.goal_node;
        response.route.assign(result.route.begin(), result.route.end());
        response.route_cost = result.cost;
        RCLCPP_INFO(get_logger(), "start_mission to node %d: %s (%s)", goal, result.accepted ? "accepted" : "rejected",
                    result.message.c_str());
        flush();
    }

    void onControl() {
        const double t = now().seconds();
        const double dt = last_control_ < 0.0 ? period_ : std::clamp(t - last_control_, 0.0, 0.2);
        last_control_ = t;
        const ane::VelocityCommand command = exec_->step(t, dt);
        publishCommand(command);
        flush();
        publishStatus(t, command);
        publishLive();
    }

    void publishCommand(const ane::VelocityCommand& command) {
        geometry_msgs::msg::Twist twist;
        twist.linear.x = command.linear;
        twist.angular.z = command.angular;
        cmd_pub_->publish(twist);
    }

    // Publishes pending executive events, and the routes and blocked-node markers when they changed.
    void flush() {
        for (const ane::MissionEvent& e : exec_->takeEvents()) {
            NavigatorEvent msg;
            msg.stamp = toStamp(e.time);
            msg.sequence = ++event_sequence_;
            msg.wall_time_ns = e.wall_ns;
            msg.type = e.type;
            msg.source = e.source;
            msg.nodes.assign(e.nodes.begin(), e.nodes.end());
            msg.value = e.value;
            msg.detail = e.detail;
            event_pub_->publish(msg);
            RCLCPP_INFO(get_logger(), "[%s/%s] %s value=%.4f nodes=%zu", e.type.c_str(), e.source.c_str(),
                        e.detail.c_str(), e.value, e.nodes.size());
        }
        if (exec_->routeVersion() != published_route_version_ || exec_->state() != published_state_) {
            published_route_version_ = exec_->routeVersion();
            published_state_ = exec_->state();
            route_pub_->publish(path(exec_->route()));
            invalidated_pub_->publish(path(exec_->invalidatedRoute()));
        }
        if (exec_->blockedNodes().size() != published_blocked_) {
            published_blocked_ = exec_->blockedNodes().size();
            publishBlocked();
        }
    }

    nav_msgs::msg::Path path(const std::vector<int>& nodes) const {
        nav_msgs::msg::Path p;
        p.header.frame_id = map_frame_;
        p.header.stamp = now();
        for (std::size_t i = 0; i < nodes.size(); ++i) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = p.header;
            const ane::Point2 here = roadmap_->nodePosition(nodes[i]);
            pose.pose.position.x = here.x;
            pose.pose.position.y = here.y;
            const ane::Point2 next = roadmap_->nodePosition(nodes[std::min(i + 1, nodes.size() - 1)]);
            const ane::Point2 prev = roadmap_->nodePosition(nodes[i == 0 ? 0 : i - 1]);
            const double yaw = (i + 1 < nodes.size()) ? std::atan2(next.y - here.y, next.x - here.x)
                                                       : std::atan2(here.y - prev.y, here.x - prev.x);
            pose.pose.orientation.z = std::sin(yaw / 2.0);
            pose.pose.orientation.w = std::cos(yaw / 2.0);
            p.poses.push_back(pose);
        }
        return p;
    }

    void publishStatus(double t, const ane::VelocityCommand& command) {
        NavigatorStatus s;
        s.stamp = toStamp(t);
        s.mission_state = ane::missionStateName(exec_->state());
        s.engine_state = navigationStateName(exec_->engineState());
        s.control_mode = ane::controlModeName(exec_->controller().mode());
        s.start_node = exec_->startNode();
        s.goal_node = exec_->goalNode();
        s.current_node = exec_->currentNode();
        s.target_node = exec_->targetNode();
        s.returning_to_node = exec_->returningToNode();
        s.route_index = static_cast<uint32_t>(exec_->routeIndex());
        s.route_version = exec_->routeVersion();
        s.replan_count = exec_->replanCount();
        const std::vector<int> route = exec_->route();
        s.route.assign(route.begin(), route.end());
        const std::vector<int> invalid = exec_->invalidatedRoute();
        s.invalidated_route.assign(invalid.begin(), invalid.end());
        const std::vector<int> blocked = exec_->blockedNodes();
        s.blocked_nodes.assign(blocked.begin(), blocked.end());
        s.remaining_cost = exec_->remainingCost();
        s.cross_track_m = exec_->controller().crossTrack();
        s.feedback_age_s = exec_->feedbackAge(t);
        s.cmd_linear_mps = command.linear;
        s.cmd_angular_radps = command.angular;
        status_pub_->publish(s);
    }

    visualization_msgs::msg::Marker marker(const std::string& ns, int id, int type) const {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = map_frame_;
        m.ns = ns;
        m.id = id;
        m.type = type;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose.orientation.w = 1.0;
        m.scale.x = m.scale.y = m.scale.z = 0.05;
        return m;
    }

    void publishGraph() {
        visualization_msgs::msg::MarkerArray array;
        // Sizes are chosen so the markers stay visible when the whole hall is shown in a few hundred
        // pixels (the recordings' top-down view); they have no effect on navigation.
        auto edges = marker("roadmap_edges", 0, visualization_msgs::msg::Marker::LINE_LIST);
        edges.scale.x = 0.05;
        edges.color = rgba(0.40f, 0.42f, 0.48f, 0.9f);
        for (const ane::RoadmapEdge& e : roadmap_->edges()) {
            for (int id : {e.from, e.to}) {
                geometry_msgs::msg::Point p;
                p.x = roadmap_->nodePosition(id).x;
                p.y = roadmap_->nodePosition(id).y;
                p.z = 0.01;
                edges.points.push_back(p);
            }
        }
        array.markers.push_back(edges);
        auto nodes = marker("roadmap_nodes", 0, visualization_msgs::msg::Marker::SPHERE_LIST);
        nodes.scale.x = nodes.scale.y = nodes.scale.z = 0.11;
        nodes.color = rgba(0.32f, 0.34f, 0.40f, 1.0f);
        for (int id : roadmap_->nodeIds()) {
            geometry_msgs::msg::Point p;
            p.x = roadmap_->nodePosition(id).x;
            p.y = roadmap_->nodePosition(id).y;
            p.z = 0.01;
            nodes.points.push_back(p);
        }
        array.markers.push_back(nodes);
        int zoneId = 0;
        for (const ane::CostZone& zone : roadmap_->zones()) {
            auto z = marker("cost_zones", zoneId++, visualization_msgs::msg::Marker::CUBE);
            z.pose.position.x = (zone.rect.min_x + zone.rect.max_x) / 2.0;
            z.pose.position.y = (zone.rect.min_y + zone.rect.max_y) / 2.0;
            z.scale.x = zone.rect.max_x - zone.rect.min_x;
            z.scale.y = zone.rect.max_y - zone.rect.min_y;
            z.scale.z = 0.005;
            z.color = rgba(0.95f, 0.55f, 0.15f, 0.25f);
            array.markers.push_back(z);
        }
        int obstacleId = 0;
        for (const ane::StaticObstacle& o : roadmap_->obstacles()) {
            auto box = marker("static_map", obstacleId++, visualization_msgs::msg::Marker::CUBE);
            box.pose.position.x = (o.rect.min_x + o.rect.max_x) / 2.0;
            box.pose.position.y = (o.rect.min_y + o.rect.max_y) / 2.0;
            box.pose.position.z = 0.005;
            box.scale.x = o.rect.max_x - o.rect.min_x;
            box.scale.y = o.rect.max_y - o.rect.min_y;
            box.scale.z = 0.01;
            box.color = rgba(0.55f, 0.58f, 0.62f, 0.55f);
            array.markers.push_back(box);
        }
        int stationId = 0;
        for (const auto& [name, node] : roadmap_->stations()) {
            auto label = marker("stations", stationId++, visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
            label.pose.position.x = roadmap_->nodePosition(node).x;
            label.pose.position.y = roadmap_->nodePosition(node).y + 0.55;
            label.pose.position.z = 0.3;
            label.scale.z = 0.48;
            label.color = rgba(0.1f, 0.1f, 0.1f, 1.0f);
            label.text = name;
            array.markers.push_back(label);
        }
        graph_pub_->publish(array);
    }

    void publishBlocked() {
        visualization_msgs::msg::MarkerArray array;
        auto sensor = marker("blocked_sensor", 0, visualization_msgs::msg::Marker::SPHERE_LIST);
        auto other = marker("blocked_operator", 0, visualization_msgs::msg::Marker::SPHERE_LIST);
        sensor.scale.x = sensor.scale.y = sensor.scale.z = 0.34;
        other.scale = sensor.scale;
        sensor.color = rgba(0.85f, 0.10f, 0.10f, 0.95f);
        other.color = rgba(0.70f, 0.20f, 0.80f, 0.95f);
        for (const auto& [id, source] : exec_->blockageSources()) {
            geometry_msgs::msg::Point p;
            p.x = roadmap_->nodePosition(id).x;
            p.y = roadmap_->nodePosition(id).y;
            p.z = 0.05;
            (source == "SENSOR" ? sensor : other).points.push_back(p);
        }
        // An empty SPHERE_LIST is valid and clears the previous one.
        array.markers.push_back(sensor);
        array.markers.push_back(other);
        blocked_pub_->publish(array);
    }

    void publishUnexplained(const std::vector<ane::Point2>& points, const builtin_interfaces::msg::Time& stamp) {
        auto m = marker("unexplained_returns", 0, visualization_msgs::msg::Marker::POINTS);
        m.header.stamp = stamp;
        m.scale.x = m.scale.y = 0.11;
        m.color = rgba(1.0f, 0.15f, 0.05f, 1.0f);
        for (const ane::Point2& q : points) {
            geometry_msgs::msg::Point p;
            p.x = q.x;
            p.y = q.y;
            p.z = 0.18;
            m.points.push_back(p);
        }
        visualization_msgs::msg::MarkerArray array;
        array.markers.push_back(m);
        live_pub_->publish(array);
    }

    void publishLive() {
        auto target = marker("target_node", 0, visualization_msgs::msg::Marker::SPHERE);
        const int node = exec_->targetNode();
        if (node >= 0) {
            target.pose.position.x = roadmap_->nodePosition(node).x;
            target.pose.position.y = roadmap_->nodePosition(node).y;
            target.pose.position.z = 0.05;
            target.scale.x = target.scale.y = target.scale.z = 0.26;
            target.color = rgba(0.10f, 0.70f, 0.25f, 1.0f);
        } else {
            target.action = visualization_msgs::msg::Marker::DELETE;
        }
        visualization_msgs::msg::MarkerArray array;
        array.markers.push_back(target);
        live_pub_->publish(array);
    }

    std::shared_ptr<const ane::Roadmap> roadmap_;
    std::unique_ptr<ane::MissionExecutive> exec_;
    std::unique_ptr<ane::BlockageMapper> mapper_;
    std::string map_frame_;
    std::string base_frame_;
    double scan_pose_tolerance_ = 0.1;
    double period_ = 0.05;
    double last_control_ = -1.0;
    std::size_t scans_skipped_ = 0;
    std::optional<Transform2> map_from_odom_;
    std::optional<Transform2> base_from_lidar_;
    uint64_t event_sequence_ = 0;
    unsigned published_route_version_ = 0;
    ane::MissionState published_state_ = ane::MissionState::IDLE;
    std::size_t published_blocked_ = 0;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<NavigatorStatus>::SharedPtr status_pub_;
    rclcpp::Publisher<NavigatorEvent>::SharedPtr event_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr route_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr invalidated_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr graph_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr blocked_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr live_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<BlockageReport>::SharedPtr report_sub_;
    rclcpp::Service<StartMission>::SharedPtr start_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr retry_srv_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<GraphNavigatorNode>());
    } catch (const std::exception& error) {
        RCLCPP_FATAL(rclcpp::get_logger("graph_navigator"), "%s", error.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
