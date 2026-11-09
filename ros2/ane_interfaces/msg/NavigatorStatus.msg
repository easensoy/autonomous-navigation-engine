# Navigator state, published every control period (20 Hz) by graph_navigator.
builtin_interfaces/Time stamp       # node clock: simulation time in Gazebo
string mission_state                # IDLE EXECUTING HOLDING_FEEDBACK COMPLETED BLOCKED FAILED_CONTROLLER FAILED_LOCALIZATION STOPPED
string engine_state                 # NavigationCore::getNavigationState(): IDLE ACTIVE PAUSED COMPLETED BLOCKED
string control_mode                 # SegmentController: IDLE ROTATE DRIVE BRAKE
int32 start_node
int32 goal_node
int32 current_node                  # NavigationCore::getCurrentPosition()
int32 target_node                   # node being driven to; -1 when not executing
bool returning_to_node              # driving back to current_node after a route replacement
uint32 route_index                  # index of current_node in route
uint32 route_version                # increments whenever the route changes
uint32 replan_count                 # route replacements made by the engine
int32[] route                       # NavigationCore::getCurrentPath()
int32[] invalidated_route           # NavigationCore::getInvalidatedRoute(); non-empty only in BLOCKED
int32[] blocked_nodes               # every node reported blocked, any source
float64 remaining_cost              # NavigationCore::getRemainingDistance(); infinity in BLOCKED
float64 cross_track_m               # signed cross-track error to the current leg
float64 feedback_age_s              # age of the newest pose feedback on the node clock
float64 cmd_linear_mps              # command published in this period
float64 cmd_angular_radps
