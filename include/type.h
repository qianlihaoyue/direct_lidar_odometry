#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

typedef pcl::PointXYZI PointType;
using CloudType = pcl::PointCloud<PointType>;
using CloudPtr = pcl::PointCloud<PointType>::Ptr;