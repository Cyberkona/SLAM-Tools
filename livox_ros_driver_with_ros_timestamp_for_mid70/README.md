# SLAM-Tools

> Modify from https://github.com/ziv-lin/livox_ros_driver_for_R2LIVE

## Why to modify the livox_ros_driver
- the LIVOX MID-70 without IMU, the setup of 'init_lidar_tim' in function 'Lddc::PublishImuData' (https://github.com/ziv-lin/livox_ros_driver_for_R2LIVE/blob/main/livox_ros_driver/livox_ros_driver/lddc.cpp#L589) is unavailable.
- sometimes we need the Pointcloud msg type of 'Pointcloud2' and 'CustomPointcloud', so we will change the ros timestamp to system base in the function 'uint32_t Lddc::PublishPointcloud2(LidarDataQueue *queue, uint32_t packet_num, uint8_t handle)' and 'uint32_t Lddc::PublishCustomPointcloud(LidarDataQueue *queue, uint32_t packet_num, uint8_t handle)'

## Usage: you can replace the lddc.cpp in livox_ros_driver with this.

## Setup variables to record the start time of lidar and ros system time

```c++
/****** Modified: add variables to change timestamp ******/
double g_ros_init_start_time = -3e8;
double init_lidar_tim = 3e10;
double init_ros_time = 0;
double skip_frame = 10;
/*********************************************************/
```

## Change timestamp in function PublishPointcloud2

```c++
  /****** Modified: Setup ros timestamp to system base ******/
  if (1)
  {
    if (skip_frame)
    {
      skip_frame--;
      init_ros_time = ros::Time::now().toSec();
      init_lidar_tim = timestamp;
      g_ros_init_start_time = timestamp;
      ROS_INFO("========================");
      ROS_INFO("Init time stamp = %lf", g_ros_init_start_time);
      ROS_INFO("========================");
    }
    // change timestamp in function PublishPointcloud2
    cloud.header.stamp = ros::Time((timestamp - init_lidar_tim) / 1e9 + init_ros_time);
  }
  /**********************************************************/
```
  
## Change timestamp in function PublishCustomPointcloud
```c++
  /****** Modified: Setup ros timestamp to system base ******/
  if (1)
  {
    if (skip_frame)
    {
      skip_frame--;
      init_ros_time = ros::Time::now().toSec();
      init_lidar_tim = timestamp;
      g_ros_init_start_time = timestamp;
      ROS_INFO("========================");
      ROS_INFO("Init time stamp = %lf", g_ros_init_start_time);
      ROS_INFO("========================");
    }
    // change timestamp in function PublishCustomPointcloud
    livox_msg.header.stamp = ros::Time((timestamp - init_lidar_tim - packet_offset_time )  / 1e9 + init_ros_time);
  }
  /**********************************************************/
```
  
