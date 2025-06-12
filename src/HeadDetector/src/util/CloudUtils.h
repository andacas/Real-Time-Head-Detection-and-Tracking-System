#ifndef HEAD_DETECTOR_UTIL_CLOUDUTILS_H_
#define HEAD_DETECTOR_UTIL_CLOUDUTILS_H_

#include <pcl/point_cloud.h>
#include <string>

namespace util {

/**
 * Template helper: load a .ply file into a PCL point cloud.
 *
 * @tparam PointT   pcl::PointXYZ, pcl::PointXYZRGB, ...
 * @param  path     Absolute / relative path to the PLY file.
 * @return          Smart pointer to the loaded cloud (nullptr on failure).
 *
 * Implementation lives in CloudUtils.hpp (header-only for templates).
 */
template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr loadPLY(const std::string& path);

/**
 * Template helper: voxel-grid down-sample any point cloud.
 *
 * @param input_cloud   Input cloud (const ptr).
 * @param leaf_size     Cube edge length for the voxel grid.
 * @return              New down-sampled cloud.
 */
template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr voxelize(
    const typename pcl::PointCloud<PointT>::ConstPtr& input_cloud,
    float leaf_size);

} // namespace util

#include "util/CloudUtils.hpp"   // include template implementations
#endif // HEAD_DETECTOR_UTIL_CLOUDUTILS_H_
