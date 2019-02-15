# Target information message
Header header

# Target Id
uint16 targetId

# Camera which the target is located in
uint16 cameraId
geometry_msgs/TransformStamped cameraTransform

# Target Location in the Image
float64 xPixelLocation
float64 yPixelLocation
float64 xPosition
float64 yPosition

# Transform to the target center
geometry_msgs/TransformStamped targetTransform

# Target Box Area (Centered at PositonX and PositionY)
float64 width
float64 height

float64 boxWidth
float64 boxHeight

# NavSatFix for Target
sensor_msgs/NavSatFix navSatFix

# Latitude [degrees]. Positive is north of equator; negative is south.
# float64 latitude

# Longitude [degrees]. Positive is east of prime meridian; negative is west.
# float64 longitude

# Altitude [m]. Positive is above the WGS 84 ellipsoid
# (quiet NaN if no altitude is available).
# float64 altitude

# Odom at Target Aquisition
nav_msgs/Odometry odom

# X/Y/Z Position
# float64 positionX
# float64 positionY
# float64 positionZ
