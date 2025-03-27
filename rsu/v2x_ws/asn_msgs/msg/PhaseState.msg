int64 LIGHT_STATE_UNAVAILABLE     = 0
int64 LIGHT_STATE_DARK	          = 1
int64 LIGHT_STATE_FLASHING_RED    = 2
int64 LIGHT_STATE_RED	          = 3
int64 LIGHT_STATE_FLASHING_GREEN   = 4
int64 LIGHT_STATE_PERMISSIVE_GREEN = 5
int64 LIGHT_STATE_PROTECTED_GREEN  = 6
int64 LIGHT_STATE_YELLOW	       = 7
int64 LIGHT_STATE_FLASHING_YELLOW  = 8

int64   light

bool mode     # 运行模式

int64 current_time_remaining  # 当前剩余时间

int64 flashing_red_times 
int64 red_times         
int64 flashing_green_times 
int64 green_times         
int64 flashing_yellow_times 
int64 yellow_times          

uint32 set_cmd                # 设置命令    1: 设置丛机模式,  2: 设置红黄绿时间,  4:  强制从机器状态



