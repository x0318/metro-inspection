# One defect observed in one source image. The header uses the source image
# timestamp and camera optical frame.
std_msgs/Header header
string event_id
string detection_id
string camera_name
string class_name
float32 confidence

# Engineering severity may be unknown when the detector first publishes the
# event. A later assessment stage can update the same event_id.
uint8 SEVERITY_UNKNOWN=0
uint8 SEVERITY_MINOR=1
uint8 SEVERITY_MODERATE=2
uint8 SEVERITY_SEVERE=3

uint8 severity
vision_msgs/BoundingBox2D bbox
uint32 image_width
uint32 image_height

# A 3D result can be added after localization. position.header identifies the
# coordinate frame and timestamp of that result.
uint8 LOCALIZATION_NONE=0
uint8 LOCALIZATION_CURRENT_CLOUD=1
uint8 LOCALIZATION_ACCUMULATED_MAP=2
uint8 LOCALIZATION_TUNNEL_MODEL=3

bool has_3d_position
geometry_msgs/PointStamped position
uint8 localization_method
float32 localization_confidence

# Traceability fields for platform display and later review.
string model_name
string snapshot_uri

# Engineering location can be added after tunnel semantic mapping.
bool has_semantic_location
float64 chainage_m
string chainage
string segment_name
int64 segment_id
float32 segment_offset_m
float32 clock_position_hours
string structure_area
