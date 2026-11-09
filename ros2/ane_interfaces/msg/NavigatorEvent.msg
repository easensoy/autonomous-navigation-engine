# One event from the navigator (ane/events), the fault injector (ane/fault_events) or the scenario
# runner (ane/scenario_events). Types and sources are listed in ros2/README.md.
builtin_interfaces/Time stamp       # node clock: simulation time in Gazebo
uint64 sequence                     # per-publisher event counter; orders events that share a stamp
int64 wall_time_ns                  # system clock when the event happened, ns since the epoch (informative:
                                    # the system clock can be stepped; durations use steady clocks)
string type
string source                       # SENSOR OPERATOR SCRIPTED ENGINE EXECUTIVE
int32[] nodes
float64 value                       # type-specific; see ros2/README.md
string detail
