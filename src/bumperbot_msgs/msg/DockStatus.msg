string STATE_UNDOCKED=UNDOCKED
string STATE_NAVIGATING_TO_STAGING=NAVIGATING_TO_STAGING
string STATE_WAITING_FOR_DETECTION=WAITING_FOR_DETECTION
string STATE_ALIGNING=ALIGNING
string STATE_DOCKING=DOCKING
string STATE_DOCKED=DOCKED
string STATE_CHARGING=CHARGING
string STATE_UNDOCKING=UNDOCKING
string STATE_FAILED=FAILED
builtin_interfaces/Time stamp
string state
bool docked
bool charging                       # docked and /battery_state reports CHARGING
float32 battery_percentage          # 0..1, -1 if unknown
uint32 attempt
float32 position_error              # m, -1 if unknown
float32 yaw_error                   # rad, -1 if unknown
bool detection_used
string frame_id                     # global frame of the two poses below
geometry_msgs/Pose2D staging_pose
geometry_msgs/Pose2D dock_pose      # charger pose (docked base_footprint pose)
string message
