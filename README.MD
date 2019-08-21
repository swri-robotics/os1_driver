# os1_driver

*copied one point twelve*  
*included zero copy*  
*open source summer*   

A rewrite of the ouster example using the original headers, including nodelets and PTP support.

# os1_driver nodelet
This nodelet reads lidar and IMU pacets from the OS1 sensor, and publishes them as a message. The output should go into `os1_cloud`.

This nodelet advertises a service at `os1_config`, and the `os1_cloud` and `os1_image` nodelets should be able to resolve that service.

```xml
<node pkg="swri_nodelet" 
    type="nodelet" 
    name="ouster_driver" 
    args="os1_driver/os1_driver ouster_nodelet_manager">
    <param name="lidar_mode"          value="1024x10"/>
    <param name="replay"              value="false"/>
    <param name="os1_hostname"        value="192.168.15.210"/>
    <param name="os1_udp_destination" value="192.168.15.100"/>
    <param name="os1_lidar_port"      value="7502"/>
    <param name="os1_imu_port"        value="7503"/>
    <param name="use_ptp"             value="true"/>
</node>
```

## params

`lidar_mode` (string, default none)  
Sets the horizontal resolution and refresh rate. Options are `512x10`, `512x20`, `1024x10`, `1024x20`, and `2048x20`. 

`replay` (bool, default: false)  
The driver will not attempt to connect to sensor hardware in replay mode

`os1_hostname` (string, default: false)  
The IP address or hostname of the lidar

`os1_udp_destination` (string, default: none)  
The IP address to send packets to.

`os1_lidar_port` (int, default: 7502)  
The port at the `udp_destination` to send lidar packets. Change this if you have multiple lidars.

`os1_imu_port` (int, default 7503)  
The port at the `udp_destination` to send imu packets to. Change this if you have multiple lidars.


## subscribed topics
There are no subscribed topics

## published topics
`/lidar_packets` (os1_driver/PacketMsg)  
lidar packets. published at over 600 Hz on an OS1-64

`/imu_packets` (os1_driver/PacketMsg)  
imu packets.

# os1_cloud nodelet
The `os1_cloud_nodelet` turns packets into pointclouds.

```xml
<node pkg="swri_nodelet" 
    type="nodelet" 
    name="ouster_cloud" 
    args="os1_driver/os1_packets_to_point_cloud ouster_nodelet_manager">
    <param name="tf_prefix"             value="lidar_top" />
    <param name="use_system_timestamp"  value="true" />
</node>
```

## params
`sensor_frame` (string)  
Required argument. The nodelet publishes a static transform from this frame to {sensor_frame}/imu and to {sensor_frame}/lidar.

`use_system_timestamp` (bool, default: false)
determines whether the point clouds and IMU messages are stamped with the timestamp from the sensor, or ros::Time::now(). You should only use the system timestamp if you do not have PTP and still want Unix time.

## subscribed topics
`/lidar_packets` (os1_driver/PacketMsg)  
lidar packets. published at over 600 Hz on an OS1-64

`/imu_packets` (os1_driver/PacketMsg)  
imu packets.

## published topics
`/points` (sensor_msgs/PointCloud2)  
one pointcloud is one rotation.

`/imu` (sensor_msgs/Imu)  
unfiltered IMU data

# os1_image nodelet
An exciting feature of the OS1 sensors is the ability to create images out of lidar data, giving you a unique type of mono, 360 degree camera.

```xml
<node pkg="swri_nodelet" 
    type="nodelet" 
    name="ouster_image" 
    args="os1_driver/os1_point_cloud_to_image ouster_nodelet_manager" />
```

## params
This node has no parameters

## subscribed topics
`points` (sensor_msgs/PointCloud2)  
the output from `os1_cloud`

## published topics
`intensity_image` (sensor_msgs/Image)
An interesting subject for segmentation, this maps the intensity of returns to pixels.

`noise_image` (sensor_msgs/Image)  
I can't say this one is very useful

`range_image` (sensor_msgs/Image)  
It reminds me of the depth image from Kinect, back when those were the easiest depth sensors to acquire.
