std_msgs/Header header

# ROV Connection
bool connected

# OAK-D Camera (RGB)
uint8[] oakd_image
uint32 oakd_height
uint32 oakd_width

# Camera 1 (RGB)
uint8[] camera_1_image
uint32 camera_1_height
uint32 camera_1_width

# Camera 2 (RGB)
uint8[] camera_2_image
uint32 camera_2_height
uint32 camera_2_width

# Camera 3 (RGB)
uint8[] camera_3_image
uint32 camera_3_height
uint32 camera_3_width

# Camera 4 (RGB)
uint8[] camera_4_image
uint32 camera_4_height
uint32 camera_4_width

# IMU - MPU-9255
float32[3] accel_m_s2
float32[3] gyro_deg_s
float32[3] mag_ut
float32 temperature_c

# Robotic Arms
bool[2] arm_extension_percent
bool[2] gripper_opening_percent

# Lighting and Operating Mode
bool light_on
bool operating_mode

# Thrusters T1 to T8
float32[8] thruster_output_percent