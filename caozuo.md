### 启动场景操作步骤

1.链接华测组合导航。 在 shared_dir 里面启动 docker。编译两个包，使用launch里面的 demo_1.py ,然后就可以看到组合导航的输出话题。使用了/chcnav/devpvt话题。  

2.进入 `pm_v2x` 这个工作空间，里面的几个包：
-  `msg_interfaces` 是定义的消息类型，需要在  `gps_to_xyz` 之前编译 
-  `obu_json` 是处理map的，需要编译一下，这个节点写到了 `gps_to_xyz` 下的launch里面
-  !!!!`gps_to_xyz` 是场景的基础功能，朗逸 可以使用 `shiche.launch.py` 
-  !!!!`position_determination` 是六个场景实现的包，可以使用 `changjing.launch.py` 启动  。在这个包里面另两个launch是分别发送两个故障的launch。
  
3.搭建前端通信页面
- 在 docker 里面编译 grpc_bridge_gui 包，然后启动。 直接 `ros2 run grpc_bridge_gui grpc_bridge_gui` 即可。 （这个docker脚本在other文件夹下）
- 启动页面 `/usr/local/gui-obu-client/bin/gui-obu-client.sh` 


### 朗逸 docker 进入pm_v2x 启动 docker 
- 1.启动华测驱动 souce 之后 ros2 launch chcnav demo_1.py
- 2.再次进入 docker exec -it xiaoche /bin/bash 
- 3.进入 pm_v2x， souce 之后 ros2 launch gps_to_xyz shiche.launch.py
