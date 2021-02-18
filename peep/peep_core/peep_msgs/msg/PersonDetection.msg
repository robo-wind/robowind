uint32 id
float32 confidence
string name
float32 dist_to_camera

# Boxes for faces and body
BoundingBox body_bbox
FaceDetection face

# This is the overall pose of this person
geometry_msgs/PoseWithCovariance pose

#For Objects associated with this person
ObjectDetection[] left_hand_objects
ObjectDetection[] right_hand_objects
ObjectDetection[] body_objects


# The raw joint positions
BodyPartDetection[] body_parts
BodyPartDetection[] facial_landmarks
HandDetection[] hands