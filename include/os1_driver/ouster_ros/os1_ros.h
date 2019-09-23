/**
 * @file
 * @brief Higher-level functions to read data from the OS-1 as ROS messages
 */

#pragma once

#include <geometry_msgs/TransformStamped.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <chrono>
#include <string>

#include "os1_driver/ouster/os1.h"
#include "os1_driver/PacketMsg.h"
#include "point_os1.h"

namespace ouster_ros {
namespace OS1 {

using CloudOS1 = pcl::PointCloud<PointOS1>;
using ns = std::chrono::nanoseconds;
using PacketMsg = os1_driver::PacketMsg;

/**
 * Read an imu packet into a ROS message. Blocks for up to a second if no data
 * is available.
 * @param cli the OS1 client
 * @return shared pointer to packet message.
 */
os1_driver::PacketMsgPtr read_imu_packet(const ouster::OS1::client& cli);

/**
 * Read a lidar packet into a ROS message. Blocks for up to a second if no data
 * is available.
 * @param cli the OS1 client
 * @return shared pointer to packet message.
 */
os1_driver::PacketMsgPtr read_lidar_packet(const ouster::OS1::client& cli);

/**
 * Parse an imu packet message into a ROS imu message
 * @param pm packet message populated by read_imu_packet
 * @param frame the frame to set in the resulting ROS message
 * @return ROS sensor message with fields populated from the OS1 packet
 */
  boost::shared_ptr<sensor_msgs::Imu> packet_to_imu_msg(const PacketMsg& pm,
                                   const std::string& frame);

/**
 * Serialize a PCL point cloud to a ROS message
 * @param cloud the PCL point cloud to convert
 * @param timestamp the timestamp to give the resulting ROS message
 * @param frame the frame to set in the resulting ROS message
 * @return a ROS message containing the point cloud
 */
  boost::shared_ptr<sensor_msgs::PointCloud2> cloud_to_cloud_msg(const CloudOS1& cloud, ns timestamp,
                                            const std::string& frame);

/**
 * Convert transformation matrix return by sensor to ROS transform
 * @param mat transformation matrix return by sensor
 * @param frame the parent frame of the published transform
 * @param child_frame the child frame of the published transform
 * @return ROS message suitable for publishing as a transform
 */
geometry_msgs::TransformStamped transform_to_tf_msg(
    const std::vector<double>& mat, const std::string& frame,
    const std::string& child_frame);
}
}
