# Supported primitive geometry. Mesh support can be added when the optimizer
# produces stable mesh resources or triangle lists.
uint8 BOX=1
uint8 SPHERE=2
uint8 CYLINDER=3

string id
uint8 primitive_type

# BOX: [x, y, z], SPHERE: [radius], CYLINDER: [height, radius].
float64[] dimensions
geometry_msgs/Pose pose
