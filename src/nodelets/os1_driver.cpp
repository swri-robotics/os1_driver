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

#include <thread>
#include <nodelet/nodelet.h>
#include <ros/ros.h>

#include "os1_driver/PacketMsg.h"
#include "os1_driver/OS1ConfigSrv.h"
#include "os1_driver/ouster/os1_util.h"
#include "os1_driver/ouster/os1.h"
#include "os1_driver/ouster_ros/os1_ros.h"

namespace OS1 = ouster::OS1;

namespace os1_driver {
  class OS1Driver : public nodelet::Nodelet
  {
  public:
    ~OS1Driver() override
    {
      connection_loop_.join();
    }

    void onInit() override
    {
      connection_loop_ = std::thread(&OS1Driver::run, this);
    }

  private:
    std::thread connection_loop_;
    std::string hostname_, udp_dest_, lidar_mode_;
    int lidar_port_, imu_port_;
    bool replay_mode_, use_ptp_;

    ros::Publisher lidar_packet_pub_, imu_packet_pub_;
    ros::ServiceServer ouster_config_service_;

    void run()
    {
      NODELET_INFO_STREAM("starting ouster_driver");

      ros::NodeHandle nh = getNodeHandle();
      ros::NodeHandle private_nh = getPrivateNodeHandle();

      // read in required and optional parameters
      bool parameter_success = true;
      parameter_success &= private_nh.getParam("replay", replay_mode_);
      parameter_success &= private_nh.getParam("lidar_mode", lidar_mode_);
      parameter_success &= private_nh.param<std::string>("os1_hostname", hostname_, "UNKNOWN");

      if (parameter_success && !replay_mode_)
      {
        // get parameters specific to reading information to the sensor.
        parameter_success &= private_nh.getParam("os1_udp_destination", udp_dest_);
        parameter_success &= private_nh.param<int>("os1_lidar_port", lidar_port_, 7501);
        parameter_success &= private_nh.param<int>("os1_imu_port", imu_port_, 7502);
        parameter_success &= private_nh.param<bool>("use_ptp", use_ptp_, true);
      }

      if (!parameter_success)
      {
        NODELET_FATAL_STREAM("You are missing a few required parameters. Check the documentation");
        return;
      }

      // service lets subscribers know when to start
      ouster_config_service_ = nh.advertiseService<OS1ConfigSrv::Request, OS1ConfigSrv::Response>(
              "os1_config", [&](OS1ConfigSrv::Request &, OS1ConfigSrv::Response &res) {
                res.hostname = hostname_;
                res.lidar_mode = lidar_mode_;
                // TODO: add options to grab these from config files.
                res.beam_azimuth_angles = ouster::OS1::beam_azimuth_angles;
                res.beam_altitude_angles = ouster::OS1::beam_altitude_angles;
                res.imu_to_sensor_transform = ouster::OS1::imu_to_sensor_transform;
                res.lidar_to_sensor_transform = ouster::OS1::lidar_to_sensor_transform;
                return true;
              });

      if (replay_mode_)
      {
        // there is no need to publish anything, or connect to a sensor that does not exist!
        NODELET_WARN_STREAM("driver is in replay mode, not connecting to sensor");
        return;
      }

      // initialize publishers
      lidar_packet_pub_ = nh.advertise<PacketMsg>("lidar_packets", 1280);
      imu_packet_pub_ = nh.advertise<PacketMsg>("imu_packets", 100);

      // reconfigure the sensor according to the parameters
      NODELET_INFO_STREAM("Connecting to sensor at " << hostname_ << "...");
      NODELET_INFO_STREAM("Sending data to " << udp_dest_ << " using lidar_mode: " << lidar_mode_);

      auto ouster_client = ouster::OS1::init_client(
              hostname_, udp_dest_, ouster::OS1::lidar_mode_of_string(lidar_mode_), lidar_port_, imu_port_);

      if (use_ptp_)
      {
        if (OS1::init_ptp_client(ouster_client))
        {
          NODELET_INFO_STREAM("Successfully enabled PTP mode on " << hostname_);
        }
        else
        {
          NODELET_ERROR_STREAM("Failed to enable PTP mode");
        }
      }

      if (!ouster_client)
      {
        NODELET_FATAL_STREAM("Failed to initialize sensor at: " << hostname_);
        return;
      }
      NODELET_INFO_STREAM("Sensor reconfigured successfully, waiting for data...");

      // the actual connection loop
      while (ros::ok())
      {
        auto state = ouster::OS1::poll_client(*ouster_client);

        if (state == ouster::OS1::EXIT)
        {
          NODELET_FATAL_STREAM("poll_client: caught signal, exiting");
          return;
        }
        if (state & ouster::OS1::ERROR)
        {
          NODELET_FATAL_STREAM("poll_client: returned error");
          continue;
        }
        if (state & ouster::OS1::LIDAR_DATA)
        {
          auto lidar_packet_msg = ouster_ros::OS1::read_lidar_packet(*ouster_client);
          if (lidar_packet_msg != nullptr)
          {
            lidar_packet_pub_.publish(lidar_packet_msg);
          }
        }
        if (state & ouster::OS1::IMU_DATA)
        {
          auto imu_packet_msg = ouster_ros::OS1::read_imu_packet(*ouster_client);
          if (imu_packet_msg != nullptr)
          {
            imu_packet_pub_.publish(imu_packet_msg);
          }
        }
      }
    }
  };
}

// Register nodelet plugin
#include <swri_nodelet/class_list_macros.h>
SWRI_NODELET_EXPORT_CLASS(os1_driver, OS1Driver)