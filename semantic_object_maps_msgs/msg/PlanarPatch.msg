# This message describes one planar patch.

uint8 ORIENTATION_UNKNOWN    = 0
uint8 ORIENTATION_HORIZONTAL = 1
uint8 ORIENTATION_VERTICAL   = 2

std_msgs/Header header

# a unique ID for this patch
string id

# centroid of bounding box in header.frame_id coordinate system [m]
geometry_msgs/Point centroid

# normal vector in header.frame_id coordinate system (normalized to length 1)
geometry_msgs/Vector3 normal

# dimensions of axis-parallel bounding box [m]
geometry_msgs/Vector3 bbox

# surface area after hole closing [m^2]
float32 area

# orientation: 0/1/2 = unknown/horizontal/vertical
uint8 orientation

# Set of points forming the contour of the planar patch in header.frame_id coordinate system [m]
geometry_msgs/Point[] contour
