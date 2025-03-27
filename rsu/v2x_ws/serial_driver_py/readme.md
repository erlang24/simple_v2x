
### Depends
```sh
#sudo apt-get install ros-galactic-nmea-msgs 
sudo apt-get update 
sudo apt-get install -f 
sudo apt-get install python3-serial
sudo apt-get install ros-galactic-gps-msgs
sudo apt-get install ros-galactic-tf-transformations
sudo pip3 install transforms3d==0.3.1
```


### Launch

```sh
sudo chmod a+wrx /dev/ttyS0
ros2 launch gnss_device_driver gnss_device_driver.launch port:=/dev/ttyS0
```


### Publish

|topic               |type                  |
|--------------------|---------------------|
| `/gnss/fix`        | **gps_msgs/msg/GPSFix**   |
| `/gnss/navsat_fix` | **sensor_msgs/msg/NavSatFix** |
| `/gnss/imu_raw`    | **sensor_msgs/msg/Imu**  |

