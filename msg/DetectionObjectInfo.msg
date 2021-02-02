# Detection
uint8                        type 		           # The category of object
# unknow     0
# pedestrain 1
# motor      2
# car        3
# truck      4

uint8                        dt_center_x
uint8                        dt_center_y
uint8                        dt_center_z
uint8                        dt_yaw                # The orientation angle of object
uint8                        dt_confidence         # The confidence of object
uint16                       dt_height             # The height of object
uint16                       dt_width              # The width of object
uint16                       dt_length             # The length of object
int16                        dt_distance_xv        # The longitudinal distance of object to ego vehicle coordinate
int16                        dt_distance_yv        # The lateral distance of object to ego vehicle coordinate