# Six-wheel drive & four-wheel steering
std_msgs/Header header
float32[6] wheel_velocity   # rad/s or m/s at wheel
float32[6] wheel_position   # integrated angle, optional
float32[6] wheel_current    # optional, 0 if unused
float32[4] steering_angle   # front-left, front-right, rear-left, rear-right
