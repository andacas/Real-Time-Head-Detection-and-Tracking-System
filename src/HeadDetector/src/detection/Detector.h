#pragma once
/******************************************************************************
 *  Detector – 3-D sliding-window Haar-cascade head detector
 *             (yaw-pitch-roll, symmetry-aware, shell contour input)
 *
 *  Returned type is **HeadPose** so the visualiser can show oriented cubes.
 ******************************************************************************/
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace detection
{

struct HeadPose
{
    float x{}, y{}, z{};        // centroid in metres             (world)
    float yaw{};                // radians  (rotation about +Z)   (world)
    float pitch{};              // radians  (rotation about +Y)
    float roll{};               // radians  (rotation about +X)
};

class Detector
{
public:
    Detector() = default;

    template <typename PointT>
    std::vector<HeadPose>                   // ← was std::vector<pcl::PointXYZ>
    detectHeads(const typename pcl::PointCloud<PointT>::ConstPtr& cloud);

private:
/* Detector.hpp — suggested values with the new cascade */
static constexpr float kCell_        = 0.045f;  // 2.5 cm voxel
static constexpr float kStep_        = kCell_ * 2.5f; // 1.25 cm stride
static constexpr int   kMinMasksHit_ = 3;       // ≥3 of 5 Stage-2 masks
static constexpr int   kSymThresh_   = 140;     // symmetry tolerance
static constexpr float kSymPenalty_  = 2.5f;    // penalty weight

};

} // namespace detection

#include "detection/Detector.hpp"   // template implementation
