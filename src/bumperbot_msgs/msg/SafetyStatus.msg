string STATE_INITIALIZING=INITIALIZING     # waiting for scan / TF / odom: locked
string STATE_ACTIVE=ACTIVE                 # protection running
string STATE_DOCK_APPROACH=DOCK_APPROACH   # ACTIVE with the bounded rear dock profile
string STATE_FAULT=FAULT                   # stale/invalid input or internal error: locked
string STATE_ESTOP=ESTOP                   # latched until ~/reset succeeds: locked
string STATE_SHUTDOWN=SHUTDOWN             # node exiting: locked
string ZONE_CLEAR=CLEAR
string ZONE_WARNING=WARNING
string ZONE_STOP=STOP
string ZONE_CONTACT=CONTACT
string ZONE_UNKNOWN=UNKNOWN                # no valid scan
builtin_interfaces/Time stamp
string state
string enforcement                         # filter | lock_only
string active_zone                         # zone of the travel direction; worst of both when stationary
string front_zone
string rear_zone
string reason                              # empty only in ACTIVE with both zones CLEAR
bool locked                                # last value published on the lock topic
bool braking                               # brake pulse active
bool estop_latched
bool dock_profile_active
float32 nearest_obstacle_distance          # m, footprint edge to nearest valid point; inf none; -1 unknown
float32 nearest_obstacle_bearing           # rad in robot_frame (0 when none/unknown)
float32 front_clearance                    # m in the front corridor; inf none; -1 unknown
float32 rear_clearance
float32 max_forward_speed                  # m/s allowed now (>= 0)
float32 max_reverse_speed                  # m/s magnitude allowed now (>= 0)
float32 scan_age                           # s (steady clock), -1 never received
float32 odom_age
float32 cmd_age
