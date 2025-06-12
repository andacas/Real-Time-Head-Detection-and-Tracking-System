#ifndef HEAD_DETECTOR_UTIL_CLOUDUTILS_IMPL_H_
#define HEAD_DETECTOR_UTIL_CLOUDUTILS_IMPL_H_

#include <pcl/io/ply_io.h>
#include <pcl/filters/voxel_grid.h>
#include <iostream>

namespace util {

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr loadPLY(const std::string& path)
{
    auto cloud = typename pcl::PointCloud<PointT>::Ptr
                 (new pcl::PointCloud<PointT>());
    if (pcl::io::loadPLYFile<PointT>(path, *cloud) != 0) {
        std::cerr << "[ERROR] Could not read PLY file: " << path << '\n';
        return nullptr;
    }
    return cloud;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr voxelize(
    const typename pcl::PointCloud<PointT>::ConstPtr& input_cloud,
    float leaf_size)
{
    auto filtered = typename pcl::PointCloud<PointT>::Ptr
                    (new pcl::PointCloud<PointT>());
    pcl::VoxelGrid<PointT> vox;
    vox.setInputCloud(input_cloud);
    vox.setLeafSize(leaf_size, leaf_size, leaf_size);
    vox.filter(*filtered);
    return filtered;
}

} // namespace util
#endif // HEAD_DETECTOR_UTIL_CLOUDUTILS_IMPL_H_
