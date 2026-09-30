# SE(2) Grid Map metadata
# Resolution of the grid in position [m/cell]
float64 pos_resolution

# Resolution of the grid in yaw [rad/cell]
float64 yaw_resolution

# Length in x-direction [m]
float64 length_x

# Length in y-direction [m]
float64 length_y

# Length in yaw-direction [rad] (typically 2*pi)
float64 length_yaw

# Pose of the se2 grid center in the frame defined in header
geometry_msgs/Pose pose
