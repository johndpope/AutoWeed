# Sprayer information message
Header header

# Sprayer Id
uint16 sprayerIDs

TargetInfo targetInfo

# Transform for the sprayers upon spraying
geometry_msgs/TransformStamped sprayerTransform
geometry_msgs/TransformStamped sprayerMidTransform

uint16 sprayerTime

# Odom at Target Aquisition
nav_msgs/Odometry odom
