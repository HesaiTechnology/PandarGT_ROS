# PandarGT_ROS
### Check
```
mkdir -p <ros_workspace>/src
cd <ros_workspace>/src
git clone https://github.com/HesaiTechnology/PandarGT_ROS.git
```

### Build
```
cd <ros_workspace>
catkin_make
```

### Run Pcap
```
roslaunch hesai_gt hesai_gt.launch pcap_file:=<pcap path>
```

### Run(Receive udp from GT)
```
roslaunch hesai_gt hesai_gt.launch
```
