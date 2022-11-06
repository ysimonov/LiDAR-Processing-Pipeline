#ifndef PREPROCESSING_HPP_
#define PREPROCESSING_HPP_

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace pcp
{
// Applies Voxel Grid downsampling on input cloud 'cloud_in' and places downsampled result into 'cloud_out'
template <typename PointT>
void applyVoxelGridDownsampling(const typename pcl::PointCloud<PointT>::ConstPtr &cloud_in,
                                typename pcl::PointCloud<PointT>::Ptr &cloud_out,
                                // Voxel Grid parameters
                                float lx = 0.05f, float ly = 0.05f, float lz = 0.05f)
{
    // Create the filtering object
    typename pcl::VoxelGrid<PointT> voxel_grid;
    voxel_grid.setInputCloud(cloud_in);
    voxel_grid.setLeafSize(lx, ly, lz);
    voxel_grid.filter(*cloud_out);
}
} // namespace pcp

#endif // PREPROCESSING_HPP_