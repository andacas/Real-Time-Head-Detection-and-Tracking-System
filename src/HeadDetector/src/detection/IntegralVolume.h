#ifndef HEAD_DETECTOR_INTEGRAL_VOLUME_H_
#define HEAD_DETECTOR_INTEGRAL_VOLUME_H_

#include <vector>
#include <pcl/point_cloud.h>

/**
 * @brief Summed-volume table (integral volume) on a regular 3-D grid.
 *
 * Cell size equals the voxel leaf size (0.025 m).  Integral lookup makes the
 * sum in ANY axis-aligned box an O(1) operation.
 */
class IntegralVolume
{
public:
    IntegralVolume(int nx, int ny, int nz, float cell,
                   float origin_x, float origin_y, float origin_z)
        : nx_(nx), ny_(ny), nz_(nz),
          cell_(cell),
          origin_x_(origin_x), origin_y_(origin_y), origin_z_(origin_z),
          data_(static_cast<std::size_t>((nx+1)*(ny+1)*(nz+1)), 0)
    {}

    template <typename PointT>
    static IntegralVolume buildFromCloud(
        const typename pcl::PointCloud<PointT>::ConstPtr& cloud,
        float cell);

    /** Inclusive sum in [x0,x1]×[y0,y1]×[z0,z1] (world coords). */
    int sum(float x0,float y0,float z0,
            float x1,float y1,float z1) const;

    /** World-space extents (min/max per axis). */
    void bounds(float& min_x,float& max_x,
                float& min_y,float& max_y,
                float& min_z,float& max_z) const
    {
        min_x = origin_x_;
        min_y = origin_y_;
        min_z = origin_z_;
        max_x = origin_x_ + nx_ * cell_;
        max_y = origin_y_ + ny_ * cell_;
        max_z = origin_z_ + nz_ * cell_;
    }

private:
    inline std::size_t idx(int ix,int iy,int iz) const
    { return static_cast<std::size_t>(iz*(ny_+1)*(nx_+1) + iy*(nx_+1) + ix); }

    int   nx_, ny_, nz_;
    float cell_, origin_x_, origin_y_, origin_z_;
    std::vector<int> data_;
};

/* template impl. */
#include "detection/IntegralVolume.hpp"
#endif
