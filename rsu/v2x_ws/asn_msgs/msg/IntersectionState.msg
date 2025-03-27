int64 intersection_id
uint8[]	 status

bool mode     # 运行模式
uint32 set_cmd                # 设置命令    1: 设置丛机模式,  2: 设置红黄绿时间,  4:  强制从机器状态

# MinuteOfTheYear_t	*moy	          /* OPTIONAL */;
# DSecond_t	*timeStamp	              /* OPTIONAL */;
# TimeConfidence_t	*time_confidence	  /* OPTIONAL */;

Phase[]	 phases

