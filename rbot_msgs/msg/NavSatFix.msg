# This contains a message for a better gps fix info than the one in ROS

std_msgs/Header header
sensor_msgs/NavSatStatus status

# in degrees, positive is N/E, negative is S/W
float64 latitude
float64 longitude

# in meters, wrt mean sea level
float64 altitude

rbot_msgs/NED velocity

# some data missing from NMEA format

float64[9] position_covariance
float64[9] velocity_covariance

uint8 position_covariance_type
uint8 velocity_covariance_type

# constants for covariance_type
uint8 COVARIANCE_TYPE_UNKNOWN=0
uint8 COVARIANCE_TYPE_APPROXIMATED=1
uint8 COVARIANCE_TYPE_DIAGONAL_KNOWN=2
uint8 COVARIANCE_TYPE_KNOWN=3
