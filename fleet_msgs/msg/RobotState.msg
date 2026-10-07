int32 robot_id
int32 row
int32 col
uint8 STATUS_IDLE=0
uint8 STATUS_TO_PICKUP=1
uint8 STATUS_TO_DROPOFF=2
uint8 STATUS_DEAD=3
uint8 status
int32 task_id          # -1 if none
float32 battery        # 0..1
builtin_interfaces/Time stamp
