Detects European green crabs (class '0') in the OAK camera image using YOLO
and publishes an annotated image. This node only publishes; it has no viewer.

TOPICS
------
Subscribes:  /oak/image_raw        (sensor_msgs/msg/Image)
Publishes:   /crab_detections/image    (sensor_msgs/msg/Image)

