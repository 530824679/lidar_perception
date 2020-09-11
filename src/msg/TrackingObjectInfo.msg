# Tracking
uint16                       id                    # The No. of object
uint8                        type 		           # The category of object
# unknow     0
# pedestrain 1
# motor      2
# car        3
# truck      4

uint8                        motion_state          # The motion status of object
# unknow     0
# moving     1
# stationary 2

uint8                        tk_center_x
uint8                        tk_center_y
uint8                        tk_center_z
uint16                       tk_height
uint16                       tk_width
uint16                       tk_length
uint8                        tk_yaw
int16                        tk_distance_xv
int16                        tk_distance_yv
int16                        tk_velocity_xv        # The longitudinal velocity of object to ego vehicle coordinate
int16                        tk_velocity_yv        # The lateral velocity of object to ego vehicle coordinate
int16                        tk_accelerate_xv      # The longitudinal accelerated velocity of object to ego vehicle coordinate
int16                        tk_accelerate_yv      # The lateral accelerated velocity of object to ego vehicle coordinate

