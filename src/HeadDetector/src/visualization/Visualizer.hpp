#ifndef HEAD_DETECTOR_VISUALIZER_IMPL_H_
#define HEAD_DETECTOR_VISUALIZER_IMPL_H_

#include <thread>
#include <chrono>

namespace visualization {

inline Visualizer::Visualizer(const std::string& window_name)
    : viewer_(new pcl::visualization::PCLVisualizer(window_name))
{
    viewer_->setBackgroundColor(0.1, 0.1, 0.1);
    viewer_->addCoordinateSystem(0.2);
    viewer_->initCameraParameters();
}

// ---------- ConstPtr overload ----------
template <typename PointT>
void Visualizer::addPointCloud(
        const typename pcl::PointCloud<PointT>::ConstPtr& cloud,
        const std::string& id,
        const std::array<float,3>& rgb)
{
    pcl::visualization::PointCloudColorHandlerCustom<PointT> color(
        cloud,
        static_cast<int>(rgb[0]*255),
        static_cast<int>(rgb[1]*255),
        static_cast<int>(rgb[2]*255));

    viewer_->addPointCloud<PointT>(cloud, color, id);
    viewer_->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, id);
}

// ---------- Ptr overload (forwards to ConstPtr version) ----------
template <typename PointT>
void Visualizer::addPointCloud(
        const typename pcl::PointCloud<PointT>::Ptr& cloud,
        const std::string& id,
        const std::array<float,3>& rgb)
{
    addPointCloud<PointT>(
        typename pcl::PointCloud<PointT>::ConstPtr(cloud), id, rgb);
}

inline void Visualizer::addBoundingBox(const pcl::PointXYZ& center,
                                       const std::string& id,
                                       float side_length)
{
    const float h = side_length * 0.5f;
    viewer_->addCube(center.x - h, center.x + h,
                     center.y - h, center.y + h,
                     center.z - h, center.z + h,
                     1.0, 0.0, 0.0, id);     // red wireframe

    viewer_->setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
        pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, id);
}

inline void Visualizer::spin()
{
    while (!viewer_->wasStopped()) {
        viewer_->spinOnce(33);                      // ~30 FPS
        std::this_thread::sleep_for(std::chrono::milliseconds(33));
    }
}
inline void Visualizer::addOrientedCube(const pcl::PointXYZ& centre,
                                        float side,
                                        float yaw_rad,
                                        const std::string& id)
{
    Eigen::Quaternionf q(Eigen::AngleAxisf(yaw_rad, Eigen::Vector3f::UnitZ()));
    viewer_->addCube(Eigen::Vector3f(centre.x,centre.y,centre.z),
                     q,  side, side, side, id);
    viewer_->setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
        pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, id);
    viewer_->setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_COLOR, 1,0,0, id);
}

} // namespace visualization
#endif // HEAD_DETECTOR_VISUALIZER_IMPL_H_
