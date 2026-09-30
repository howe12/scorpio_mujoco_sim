# SE(2) Grid Map message
# Ported from ZJU-FAST-Lab/seb_naver (ROS1) to ROS2

std_msgs/Header header

SE2GridInfo info

# Layer names (e.g., "elevation", "var", "risk", "sdf")
string[] layers

# Whether each layer has SO(2) (yaw) dimension
bool[] has_so2

# Grid map data: one Float32MultiArray per layer
# For non-SO(2) layers: single 2D matrix [rows x cols]
# For SO(2) layers: concatenated yaw slices [rows x cols x n_yaw]
std_msgs/Float32MultiArray[] data

# Row start index for circular buffer (default 0)
uint16 start_index_row

# Column start index for circular buffer (default 0)
uint16 start_index_col
