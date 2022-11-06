#ifndef LIDAR_PROCESSING_PIPELINE_HPP_
#define LIDAR_PROCESSING_PIPELINE_HPP_

// downsampling and filtering
#include "preprocessing.hpp"

// ground segmentation
#include "segmentation.hpp"

// clustering obstacles
#include "clustering.hpp"

// simplifying obstacles into polygons
#include "polygonization.hpp"

// dynamic object tracking
#include "tracking.hpp"

// File IO
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace pcp
{

} // namespace pcp

#endif // LIDAR_PROCESSING_PIPELINE_HPP_