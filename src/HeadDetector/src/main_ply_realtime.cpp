/******************************************************************************
 *  main_ply_realtime.cpp  –  rotate the cloud around Z; detect head
 *                           only every 5th visual frame for speed.
 ******************************************************************************/
#include <iostream>
#include <thread>
#include <chrono>

#include <Eigen/Geometry>
#include <pcl/io/ply_io.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>

#include "detection/Detector.h"

using CloudXYZ = pcl::PointCloud<pcl::PointXYZ>;

int main(int argc, char **argv)
{
    if (argc < 2) {
        std::cerr << "Usage: HeadDetectorRealtime <file.ply>\n";
        return 1;
    }

    /* -------- load shell cloud --------------------------------------- */
    CloudXYZ::Ptr cloud_raw(new CloudXYZ);
    if (pcl::io::loadPLYFile(argv[1], *cloud_raw) != 0) {
        std::cerr << "Error reading " << argv[1] << '\n';
        return 1;
    }
    std::cout << "Loaded " << cloud_raw->size() << " points\n";

    /* -------- visualiser --------------------------------------------- */
    pcl::visualization::PCLVisualizer::Ptr viewer(
        new pcl::visualization::PCLVisualizer("HeadDetector – realtime spin"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addCoordinateSystem(0.1);
    viewer->addPointCloud(cloud_raw, "cloud");
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0, "cloud");

    /* -------- detector ------------------------------------------------ */
    detection::Detector detector;

    /* -------- loop settings ------------------------------------------ */
    constexpr float DEG2RAD      = static_cast<float>(M_PI) / 180.f;
    constexpr float STEP_DEG     = 5.f;   // rotation per frame
    constexpr int   DETECT_EVERY = 5;     // heavy detection cadence

    float angle = 0.f;            // current Z-angle
    int   frame = 0;              // visual frame index
    bool  have_pose = false;      // whether we have a pose to draw
    detection::HeadPose pose{};   // last detected pose
    const std::string cube_id = "cube";

    while (!viewer->wasStopped())
    {
        /* --- rotate cloud -------------------------------------------- */
        Eigen::Affine3f tf = Eigen::Affine3f::Identity();
        tf.rotate(Eigen::AngleAxisf(angle * DEG2RAD, Eigen::Vector3f::UnitZ()));

        CloudXYZ::Ptr cloud_rot(new CloudXYZ);
        pcl::transformPointCloud(*cloud_raw, *cloud_rot, tf.matrix());

        /* --- run detector every DETECT_EVERY frames ------------------ */
        if (frame % DETECT_EVERY == 0) {
            auto heads = detector.detectHeads<pcl::PointXYZ>(cloud_rot);
            if (!heads.empty()) { pose = heads.front(); have_pose = true; }
            else                 { have_pose = false;                     }
        }

        /* --- update cloud -------------------------------------------- */
        if (!viewer->updatePointCloud(cloud_rot, "cloud"))
            viewer->addPointCloud(cloud_rot, "cloud");
        viewer->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0, "cloud");

        /* --- cube ---------------------------------------------------- */
        viewer->removeShape(cube_id);
        if (have_pose) {
            Eigen::Quaternionf q =
                Eigen::AngleAxisf(pose.yaw,   Eigen::Vector3f::UnitZ()) *
                Eigen::AngleAxisf(pose.pitch, Eigen::Vector3f::UnitY()) *
                Eigen::AngleAxisf(pose.roll,  Eigen::Vector3f::UnitX());

            viewer->addCube(Eigen::Vector3f(pose.x, pose.y, pose.z),
                            q, 0.25f, 0.25f, 0.25f, cube_id);
            viewer->setShapeRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_COLOR,
                1.0, 0.0, 0.0, cube_id);
            viewer->setShapeRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
                pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,
                cube_id);
        }

        /* --- display & advance -------------------------------------- */
        viewer->spinOnce(33);                    // ~30 fps
        std::this_thread::sleep_for(std::chrono::milliseconds(5));

        angle += STEP_DEG;
        if (angle >= 360.f) angle -= 360.f;
        ++frame;
    }
    return 0;
}
