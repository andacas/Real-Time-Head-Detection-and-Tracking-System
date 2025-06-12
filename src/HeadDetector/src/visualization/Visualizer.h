#ifndef HEAD_DETECTOR_VISUALIZATION_VISUALIZER_H_
#define HEAD_DETECTOR_VISUALIZATION_VISUALIZER_H_
/*─────────────────────────────────────────────────────────────────────────────
 *  Visualizer – renders the input cloud plus a red, wire-frame cube for each
 *  detected head pose.  Keeps the **legacy API** expected by main.cpp:
 *
 *      showCloud<PointT>(cloud);          // adds the cloud
 *      drawDetections(poses [, side]);    // draws cubes
 *
 *  Internally uses the modern PCL overload
 *      addCube(translation, quaternion, x, y, z, id)
 *  so yaw / pitch / roll are fully honoured.
 *────────────────────────────────────────────────────────────────────────────*/
#include <array>
#include <Eigen/Geometry>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "detection/Detector.h"           // HeadPose struct

namespace visualization
{

class Visualizer
{
public:
    explicit Visualizer(const std::string& title = "HeadDetector")
      : viewer_(new pcl::visualization::PCLVisualizer(title))
    {
        viewer_->setBackgroundColor(0, 0, 0);
        viewer_->addCoordinateSystem(0.1);            // 10 cm axis widget
    }

    /* ------------------------------------------------------------------ */
    template <typename PointT>
    void showCloud(const typename pcl::PointCloud<PointT>::ConstPtr& cloud)
    {
        // White points
        viewer_->addPointCloud<PointT>(cloud, "cloud");
        viewer_->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0, "cloud");
    }

    /* ------------------------------------------------------------------ */
    void drawDetections(const std::vector<detection::HeadPose>& poses,
                        float side = 0.25f)           // cube side (m)
    {
        std::size_t idx = 0;
        for (const auto& p : poses)
        {
            /* quaternion built from intrinsic Z-Y-X (yaw-pitch-roll) */
            Eigen::Quaternionf q =
                Eigen::AngleAxisf(p.yaw,   Eigen::Vector3f::UnitZ()) *
                Eigen::AngleAxisf(p.pitch, Eigen::Vector3f::UnitY()) *
                Eigen::AngleAxisf(p.roll,  Eigen::Vector3f::UnitX());

            const std::string id = "cube_" + std::to_string(idx++);
            viewer_->addCube(Eigen::Vector3f(p.x, p.y, p.z), q,
                             side, side, side, id);

            viewer_->setShapeRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_COLOR,
                1.0, 0.0, 0.0, id);                       // red
            viewer_->setShapeRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
                pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,
                id);
        }
        viewer_->spin();
    }

private:
    pcl::visualization::PCLVisualizer::Ptr viewer_;
};

} // namespace visualization
#endif /* HEAD_DETECTOR_VISUALIZATION_VISUALIZER_H_ */
