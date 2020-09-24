# Xavier-SLAM-module
Warning: currently doesn't work on Xavier. Recommend platforms based on x86 architecture.

1. On your master computer, check master computer local IP address, and add it to your ~/.bashrc file
```
gedit ~/.bashrc
export ROS_MASTER_URI=http://<masterIP>:11311
export ROS_IP=<masterIP>
source ~/.bashrc
```

2. Then, launch roscore
```
roscore
```

3. Log into your mobile computer of choice
```
ssh <username>@<mobileIP>
```

4. On your mobile computer, check mobile computer local IP address, and add it to your ~/.bashrc file
```
gedit ~/.bashrc
export ROS_MASTER_URI=http://<masterIP>:11311
export ROS_IP=<mobileIP>
source ~/.bashrc
```
