# A blockage notification from outside the navigator: an operator, or a scenario script acting as
# one. Sensor-derived blockages are produced inside graph_navigator from the lidar and never arrive
# on this topic; a report with source SENSOR is rejected.
builtin_interfaces/Time stamp
string source                       # OPERATOR or SCRIPTED
string label
int32[] node_ids                    # roadmap nodes to close
bool use_region                     # also close every roadmap node inside the region below
float64 min_x                       # region in the map frame, metres
float64 min_y
float64 max_x
float64 max_y
