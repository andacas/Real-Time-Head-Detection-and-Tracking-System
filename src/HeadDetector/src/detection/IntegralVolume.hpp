#ifndef HEAD_DETECTOR_DETECTION_INTEGRALVOLUME_HPP_
#define HEAD_DETECTOR_DETECTION_INTEGRALVOLUME_HPP_
/*─────────────────────────────────────────────────────────────────────────────
 *  IntegralVolume – header-only, GPU-aware prefix-sum volume
 *
 *  • Accepts a *contour* point-cloud, flood-fills it → solid volume.
 *  • Builds an inclusive 3-D prefix sum (Summed-Area Table).
 *  • Safe, clamped `sum()` and `bounds()` helpers are `inline`.
 *  • ▲  GPU helpers:
 *        – `const int*  data()   const`   → raw pointer for OpenMP -fopenmp-target
 *        – `std::size_t size()   const`   → #elements  (mapping extent)
 *        – `nx(), ny(), nz(), cell(), ox(), oy(), oz()`  → grid meta-data
 *
 *  Header-only so the compiler sees everything on device builds.
 *───────────────────────────────────────────────────────────────────────────*/
#include <algorithm>
#include <limits>
#include <queue>
#include <stdexcept>
#include <vector>

#include <pcl/point_cloud.h>
#include <omp.h>

namespace detection
{

class IntegralVolume
{
public:
    /* ------------------------------------------------------------------ */
    IntegralVolume(int nx, int ny, int nz,
                   float cell,
                   float ox,  float oy,  float oz)
      : nx_(nx), ny_(ny), nz_(nz),
        cell_(cell),
        ox_(ox),  oy_(oy),  oz_(oz),
        data_(static_cast<std::size_t>((nx + 1) * (ny + 1) * (nz + 1)), 0)
    {
        if (cell_ <= 0.f) throw std::invalid_argument("cell ≤ 0");
    }

    /* ------------------------------------------------------------------ *
     *  Factory: build solid integral volume from shell point-cloud       *
     * ------------------------------------------------------------------ */
    template <typename PointT>
    static IntegralVolume
    buildFromCloud(const typename pcl::PointCloud<PointT>::ConstPtr& cloud,
                   float cell);

    /* ------------------------------------------------------------------ *
     *  Inclusive sum in world coords (clamped, empty box → 0).           *
     * ------------------------------------------------------------------ */
    inline int sum(float x0,float y0,float z0,
                   float x1,float y1,float z1) const;

    /* grid bounds in world space --------------------------------------- */
    inline void bounds(float& min_x,float& max_x,
                       float& min_y,float& max_y,
                       float& min_z,float& max_z) const
    {
        min_x = ox_;              max_x = ox_ + nx_ * cell_;
        min_y = oy_;              max_y = oy_ + ny_ * cell_;
        min_z = oz_;              max_z = oz_ + nz_ * cell_;
    }

    /* ▲ GPU accessors (raw pointer & meta) ----------------------------- */
    inline const int*     data() const { return data_.data(); }
    inline std::size_t    size() const { return data_.size(); }
    inline int            nx()   const { return nx_; }
    inline int            ny()   const { return ny_; }
    inline int            nz()   const { return nz_; }
    inline float          cell() const { return cell_; }
    inline float          ox()   const { return ox_; }
    inline float          oy()   const { return oy_; }
    inline float          oz()   const { return oz_; }

private:
    /* flat index ------------------------------------------------------- */
    inline std::size_t idx(int x,int y,int z) const
    { return static_cast<std::size_t>(z*(ny_+1)*(nx_+1) + y*(nx_+1) + x); }

    int   nx_, ny_, nz_;
    float cell_;
    float ox_, oy_, oz_;
    std::vector<int> data_;
};

/* =========================================================================
 *  buildFromCloud  –  implementation
 * =========================================================================*/
template <typename PointT>
IntegralVolume IntegralVolume::buildFromCloud(
    const typename pcl::PointCloud<PointT>::ConstPtr& cloud,
    float cell)
{
    if (!cloud || cloud->empty())
        throw std::invalid_argument("empty cloud");

    /* 1. tight bounds -------------------------------------------------- */
    float min_x= std::numeric_limits<float>::max();
    float min_y=min_x, min_z=min_x;
    float max_x=-min_x, max_y=-min_x, max_z=-min_x;

    for (const auto& p: cloud->points) {
        min_x = std::min(min_x, p.x); max_x = std::max(max_x, p.x);
        min_y = std::min(min_y, p.y); max_y = std::max(max_y, p.y);
        min_z = std::min(min_z, p.z); max_z = std::max(max_z, p.z);
    }
    min_x -= 0.5f*cell;  min_y -= 0.5f*cell;  min_z -= 0.5f*cell;

    int nx = static_cast<int>((max_x - min_x) / cell) + 1;
    int ny = static_cast<int>((max_y - min_y) / cell) + 1;
    int nz = static_cast<int>((max_z - min_z) / cell) + 1;

    IntegralVolume iv(nx, ny, nz, cell, min_x, min_y, min_z);

    auto flat=[&](int x,int y,int z)->std::size_t{
        return static_cast<std::size_t>(z*(ny+1)*(nx+1)+y*(nx+1)+x); };

    /* 2. voxelise shell ------------------------------------------------ */
    std::vector<uint8_t> vox((nx+1)*(ny+1)*(nz+1), 0);
    for (const auto& p: cloud->points) {
        int ix = std::clamp(int((p.x - min_x)/cell), 0, nx);
        int iy = std::clamp(int((p.y - min_y)/cell), 0, ny);
        int iz = std::clamp(int((p.z - min_z)/cell), 0, nz);
        vox[flat(ix,iy,iz)] = 1;                          // shell voxel
    }

    /* 3. flood-fill from faces to mark OUTSIDE zeros ------------------ */
    std::queue<std::tuple<int,int,int>> Q;
    auto push=[&](int x,int y,int z){
        if(x<0||x>nx||y<0||y>ny||z<0||z>nz) return;
        auto& v = vox[flat(x,y,z)];
        if(v==0){ v=2; Q.emplace(x,y,z); } };

    for(int x=0;x<=nx;++x) for(int y=0;y<=ny;++y){ push(x,y,0); push(x,y,nz); }
    for(int x=0;x<=nx;++x) for(int z=0;z<=nz;++z){ push(x,0,z); push(x,ny,z); }
    for(int y=0;y<=ny;++y) for(int z=0;z<=nz;++z){ push(0,y,z); push(nx,y,z); }

    const int dx[6]={1,-1,0,0,0,0}, dy[6]={0,0,1,-1,0,0}, dz[6]={0,0,0,0,1,-1};
    while(!Q.empty()){
        auto [x,y,z]=Q.front(); Q.pop();
        for(int k=0;k<6;++k) push(x+dx[k], y+dy[k], z+dz[k]);
    }

    /* 4. interior fill + copy to int ---------------------------------- */
    auto& intg = iv.data_;
#pragma omp parallel for
    for(std::size_t i=0;i<vox.size();++i)
        intg[i] = (vox[i]==1 || vox[i]==0) ? 1 : 0;   // 1 = occupied

    /* 5. inclusive 3-D prefix scan ----------------------------------- */
#pragma omp parallel for collapse(2)
    for(int z=0;z<=nz;++z)
        for(int y=0;y<=ny;++y)
            for(int x=1;x<=nx;++x)
                intg[flat(x,y,z)] += intg[flat(x-1,y,z)];

#pragma omp parallel for collapse(2)
    for(int z=0;z<=nz;++z)
        for(int x=0;x<=nx;++x)
            for(int y=1;y<=ny;++y)
                intg[flat(x,y,z)] += intg[flat(x,y-1,z)];

#pragma omp parallel for collapse(2)
    for(int y=0;y<=ny;++y)
        for(int x=0;x<=nx;++x)
            for(int z=1;z<=nz;++z)
                intg[flat(x,y,z)] += intg[flat(x,y,z-1)];

    return iv;
}

/* =========================================================================
 *  sum() – safe, clamped
 * =========================================================================*/
inline int IntegralVolume::sum(float x0,float y0,float z0,
                               float x1,float y1,float z1) const
{
    if (x1<x0) std::swap(x0,x1);
    if (y1<y0) std::swap(y0,y1);
    if (z1<z0) std::swap(z0,z1);

    auto toI=[&](float v,float o,int n){
        int i=int((v-o)/cell_); return (i<0)?0:(i>n)?n:i; };

    int ix0=toI(x0,ox_,nx_), iy0=toI(y0,oy_,ny_), iz0=toI(z0,oz_,nz_);
    int ix1=toI(x1,ox_,nx_), iy1=toI(y1,oy_,ny_), iz1=toI(z1,oz_,nz_);

    if(ix1<ix0||iy1<iy0||iz1<iz0) return 0;

    auto at=[&](int x,int y,int z)->int{
        if(x<0||y<0||z<0) return 0;
        return data_[idx(x,y,z)]; };

    int A=at(ix1,iy1,iz1), B=at(ix0-1,iy1,iz1),
        C=at(ix1,iy0-1,iz1), D=at(ix1,iy1,iz0-1),
        E=at(ix0-1,iy0-1,iz1), F=at(ix0-1,iy1,iz0-1),
        G=at(ix1,iy0-1,iz0-1), H=at(ix0-1,iy0-1,iz0-1);
    return A-B-C-D+E+F+G-H;
}

} // namespace detection
#endif /* HEAD_DETECTOR_DETECTION_INTEGRALVOLUME_HPP_ */
