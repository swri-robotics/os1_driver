// *****************************************************************************
//
// Copyright (C) 2019 All Right Reserved, Southwest Research Institute® (SwRI®)
//
// THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
// KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
// PARTICULAR PURPOSE.
//
// *****************************************************************************

// ROS libraries

#include <nodelet/nodelet.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <ros/service.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <chrono>
#include <thread>

#include "os1_driver/ouster/os1_packet.h"
#include "os1_driver/ouster/os1_util.h"
#include "os1_driver/OS1ConfigSrv.h"
#include "os1_driver/PacketMsg.h"
#include "os1_driver/ouster_ros/os1_ros.h"

// Intel TBB
#include <tbb/tbb.h>

using CloudOS1 = ouster_ros::OS1::CloudOS1;
using PointOS1 = ouster_ros::OS1::PointOS1;

namespace OS1 = ouster::OS1;

namespace os1_driver
{
class OS1PacketsToPointCloud : public nodelet::Nodelet
{
public:
  void onInit() override
  {
    cloud_loop_ = std::thread(&OS1PacketsToPointCloud::run, this);
  }

  ~OS1PacketsToPointCloud()
  {
    cloud_loop_.join();
  }
private:
  bool use_system_timestamp_; // TODO: make a better parameter name
  uint32_t h_, w_;

  std::string sensor_frame_, imu_frame_, lidar_frame_;
  std::thread cloud_loop_;
  std::vector<double> xyz_lut_;

  ros::Publisher imu_pub_, lidar_pub_;
  ros::Subscriber imu_packet_sub_, lidar_packet_sub_;
  tf2_ros::StaticTransformBroadcaster tf_bcast_;

  CloudOS1 cloud_;

  std::function<void(const uint8_t*, CloudOS1::iterator it)> batch_and_publish_;

  void run()
  {
    NODELET_INFO_STREAM("Starting os1_cloud");

    ros::NodeHandle nh = getNodeHandle();
    ros::NodeHandle private_nh = getPrivateNodeHandle();

    sensor_frame_ = private_nh.param("sensor_frame", std::string{"os1_lidar"});
    use_system_timestamp_ = private_nh.param("use_system_timestamp", false);
    imu_frame_ = sensor_frame_ + "/imu";
    lidar_frame_ = sensor_frame_ + "/lidar";

    // wait until packet driver is ready, then get the config
    OS1ConfigSrv cfg{};
    auto client = nh.serviceClient<OS1ConfigSrv>("os1_config");
    client.waitForExistence();
    if (!client.call(cfg)) {
      NODELET_FATAL_STREAM("Calling os1 config service failed");
      ros::shutdown();
      return;
    }

    h_ = OS1::pixels_per_column;
    w_ = OS1::n_cols_of_lidar_mode(OS1::lidar_mode_of_string(cfg.response.lidar_mode));
    xyz_lut_ = OS1::make_xyz_lut(w_, h_, cfg.response.beam_azimuth_angles, cfg.response.beam_altitude_angles);
    cloud_ = CloudOS1{w_, h_};

    // publish transforms
    tf_bcast_ = tf2_ros::StaticTransformBroadcaster();
    tf_bcast_.sendTransform(ouster_ros::OS1::transform_to_tf_msg(
              cfg.response.imu_to_sensor_transform,
              sensor_frame_,
              imu_frame_));
    tf_bcast_.sendTransform(ouster_ros::OS1::transform_to_tf_msg(
              cfg.response.lidar_to_sensor_transform,
              sensor_frame_,
              lidar_frame_));

    // setup publishers
    lidar_pub_ = nh.advertise<sensor_msgs::PointCloud2>("points", 10);
    imu_pub_ = nh.advertise<sensor_msgs::Imu>("imu", 100);


    batch_and_publish_ = OS1::batch_to_iter<CloudOS1::iterator>(
            xyz_lut_, w_, h_, {}, &PointOS1::make,
            [&](uint64_t scan_ts) mutable {
               auto msg = ouster_ros::OS1::cloud_to_cloud_msg(cloud_, std::chrono::nanoseconds{scan_ts}, lidar_frame_);

              if (use_system_timestamp_)
              {
                // if packets are not PTP-timestamped, then the header is the time since the sensor was initialized,
                // rather than the time since the epoch.
                msg->header.stamp = ros::Time::now();
              }

               lidar_pub_.publish(msg);
            });

    imu_packet_sub_ = nh.subscribe<PacketMsg, const PacketMsgConstPtr&>("imu_packets", 100, [&](const PacketMsgConstPtr& p) {
      imu_pub_.publish(ouster_ros::OS1::packet_to_imu_msg(*p, imu_frame_));
    });

    lidar_packet_sub_ = nh.subscribe<PacketMsg, const PacketMsgConstPtr&>("lidar_packets", 2048, [&](const PacketMsgConstPtr& p) {
      auto it = cloud_.begin();
      batch_and_publish_(p->buf.data(), it);
    });
  }
};
}

// Register nodelet plugin
#include <swri_nodelet/class_list_macros.h>
SWRI_NODELET_EXPORT_CLASS(os1_driver, OS1PacketsToPointCloud)