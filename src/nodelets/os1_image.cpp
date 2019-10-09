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

// ROS packages
#include <thread>
#include <image_transport/image_transport.h>
#include <nodelet/nodelet.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include "os1_driver/ouster/os1_packet.h"
#include "os1_driver/ouster/os1_util.h"
#include "os1_driver/OS1ConfigSrv.h"
#include "os1_driver/ouster_ros/os1_ros.h"

// Intel TBB
#include <tbb/tbb.h>

namespace OS1 = ouster::OS1;

namespace os1_driver {
  class OS1PointCloudToImage : public nodelet::Nodelet
  {
  public:
    ~OS1PointCloudToImage() override
    {
      image_thread_.join();
    }

    void onInit() override
    {
      image_thread_ = std::thread(&OS1PointCloudToImage::run, this);
    }

  private:
    uint32_t h_, w_;
    std::vector<int> px_offset_;

    std::thread image_thread_;

    image_transport::Publisher intensity_image_pub_, noise_image_pub_, range_image_pub_;
    ros::Subscriber point_cloud_sub_;


    void run()
    {
      ros::NodeHandle nh = getNodeHandle();
      ros::NodeHandle private_nh = getPrivateNodeHandle();
      NODELET_INFO_STREAM("starting os1_image nodelet");

      image_transport::ImageTransport it(nh);

      // wait for the driver to be ready
      os1_driver::OS1ConfigSrv cfg{};
      auto client = nh.serviceClient<os1_driver::OS1ConfigSrv>("os1_config");
      client.waitForExistence();
      if (!client.call(cfg))
      {
        NODELET_FATAL_STREAM("Calling os1 config service failed");
        return;
      }

      h_ = OS1::pixels_per_column;
      w_ = OS1::n_cols_of_lidar_mode(OS1::lidar_mode_of_string(cfg.response.lidar_mode));
      px_offset_ = OS1::get_px_offset(w_);

      // advertise topics
      intensity_image_pub_ = it.advertise("intensity_image", 100);
      noise_image_pub_ = it.advertise("noise_image", 100);
      range_image_pub_ = it.advertise("range_image", 100);

      point_cloud_sub_ = nh.subscribe<sensor_msgs::PointCloud2, const sensor_msgs::PointCloud2::ConstPtr &>(
              "points", 500, [&](const sensor_msgs::PointCloud2::ConstPtr &msg) {
                // sensor_msgs::Image intensity_image, noise_image, range_image;

                auto intensity_image = boost::make_shared<sensor_msgs::Image>();
                auto noise_image = boost::make_shared<sensor_msgs::Image>();
                auto range_image = boost::make_shared<sensor_msgs::Image>();



                ouster_ros::OS1::CloudOS1 cloud{};

                // convert the pointcloud_msg into a PCL structure
                pcl::fromROSMsg(*msg, cloud);

                // ensure that the pointcloud is the same size as what the driver things
                if ((int) cloud.size() != w_ * h_)
                {
                  ROS_FATAL("Unexpected cloud size, check lidar_mode");
                  ros::shutdown();
                }

                range_image->width = w_;
                range_image->height = h_;
                range_image->step = w_;
                range_image->encoding = "mono8";
                range_image->data.resize(w_ * h_);
                range_image->header = msg->header;

                noise_image->width = w_;
                noise_image->height = h_;
                noise_image->step = w_;
                noise_image->encoding = "mono8";
                noise_image->data.resize(w_ * h_);
                noise_image->header = msg->header;

                intensity_image->width = w_;
                intensity_image->height = h_;
                intensity_image->step = w_;
                intensity_image->encoding = "mono8";
                intensity_image->data.resize(w_ * h_);
                intensity_image->header = msg->header;

                tbb::parallel_for(
                        tbb::blocked_range2d<size_t>(0, h_, 0, w_), [&](const tbb::blocked_range2d<size_t> &r) {
                          for (int u = r.rows().begin(), u_end = r.rows().end(); u < u_end; u++)
                          {
                            for (int v = r.cols().begin(), v_end = r.cols().end(); v < v_end; v++)
                            {
                              const size_t vv = (v + px_offset_[u]) % w_;
                              const size_t index = vv * h_ + u;
                              const auto &pt = cloud[index];

                              // write pixels
                              if (pt.range == 0) {
                                range_image->data[u * w_ + v] = 0;
                              }
                              else
                              {
                                range_image->data[u * w_ + v] =
                                        255 - static_cast<char>(std::min(std::round(pt.range * 5e-3), 255.0));
                              }
                              noise_image->data[u * w_ + v] = std::min(pt.noise, (uint16_t) 255);
                              intensity_image->data[u * w_ + v] = static_cast<char>(std::min(pt.intensity, 255.f));
                            }
                          }
                        });

                if (range_image_pub_.getNumSubscribers() > 0)
                {
                  range_image_pub_.publish(range_image);
                }
                if (noise_image_pub_.getNumSubscribers() > 0)
                {
                  noise_image_pub_.publish(noise_image);
                }
                if (intensity_image_pub_.getNumSubscribers() > 0)
                {
                  intensity_image_pub_.publish(intensity_image);
                }
              });
    }
  };
}

// Register nodelet plugin
#include <swri_nodelet/class_list_macros.h>
SWRI_NODELET_EXPORT_CLASS(os1_driver, OS1PointCloudToImage)