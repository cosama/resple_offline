#pragma once

// Convert captured point clouds.

#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>

#include "sensor_msgs/msg/point_cloud2.hpp"

namespace pcl_conversions {

inline void toPCL(const sensor_msgs::msg::PointCloud2& msg, pcl::PCLPointCloud2& out) {
  out.header.stamp = static_cast<std::uint64_t>(msg.header.stamp.sec) * 1000000ULL +
                      msg.header.stamp.nanosec / 1000ULL;
  out.header.frame_id = msg.header.frame_id;
  out.header.seq = 0;

  out.height = msg.height;
  out.width = msg.width;
  out.is_bigendian = msg.is_bigendian;
  out.point_step = msg.point_step;
  out.row_step = msg.row_step;
  out.is_dense = msg.is_dense;

  out.fields.resize(msg.fields.size());
  for (std::size_t i = 0; i < msg.fields.size(); ++i) {
    out.fields[i].name = msg.fields[i].name;
    out.fields[i].offset = msg.fields[i].offset;
    out.fields[i].datatype = msg.fields[i].datatype;
    out.fields[i].count = msg.fields[i].count;
  }
  out.data = msg.data;
}

inline void fromPCL(const pcl::PCLPointCloud2& in, sensor_msgs::msg::PointCloud2& msg) {
  msg.header.frame_id = in.header.frame_id;
  msg.height = in.height;
  msg.width = in.width;
  msg.is_bigendian = in.is_bigendian;
  msg.point_step = in.point_step;
  msg.row_step = in.row_step;
  msg.is_dense = in.is_dense;
  msg.fields.resize(in.fields.size());
  for (std::size_t i = 0; i < in.fields.size(); ++i) {
    msg.fields[i].name = in.fields[i].name;
    msg.fields[i].offset = in.fields[i].offset;
    msg.fields[i].datatype = in.fields[i].datatype;
    msg.fields[i].count = in.fields[i].count;
  }
  msg.data = in.data;
}

}  // namespace pcl_conversions

namespace pcl {

template <typename PointT>
void fromROSMsg(const sensor_msgs::msg::PointCloud2& msg, pcl::PointCloud<PointT>& cloud) {
  pcl::PCLPointCloud2 intermediate;
  pcl_conversions::toPCL(msg, intermediate);
  pcl::fromPCLPointCloud2(intermediate, cloud);
}

template <typename PointT>
void toROSMsg(const pcl::PointCloud<PointT>& cloud, sensor_msgs::msg::PointCloud2& msg) {
  pcl::PCLPointCloud2 intermediate;
  pcl::toPCLPointCloud2(cloud, intermediate);
  pcl_conversions::fromPCL(intermediate, msg);
}

}  // namespace pcl
