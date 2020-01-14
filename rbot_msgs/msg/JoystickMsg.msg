# This contains data about position of the digital axes on a gaming joystick
#
# We have packed float before int before (possibly) single byte data members
#
# @TODO: Sometime in future
# Further byte level bitmapping can be done for the boolean values
    # Currently, the ROS message format stops us from doing that
rbot_msgs/JoystickAxisFloat32 left_axis
rbot_msgs/JoystickAxisFloat32 right_axis
float32 LT
float32 RT
rbot_msgs/JoystickAxisInt32 digital_axis
bool LB
bool RB
bool A
bool B
bool X
bool Y
bool left_axis_button
bool right_axis_button
bool logo
bool center_left
bool center_right
char mode
