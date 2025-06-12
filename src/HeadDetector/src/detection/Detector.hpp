#pragma once
/******************************************************************************
 *  Detector.hpp – one-kernel GPU implementation, CPU fallback
 *
 *  • HAVE_GPU (set by CMake) 0 → pure OpenMP threads 1 → NVPTX off-load
 *  • Entire Nx×Ny×Nz × 9 × 5 × 5 sweep is evaluated inside one kernel,
 *    eliminating the long launch-latency pauses you observed.
 ******************************************************************************/
#include <omp.h>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>

#include "detection/IntegralVolume.hpp"
#include "detection/HaarMasks.h"
#include "detection/Detector.h"

namespace detection
{
/* ═══════════════════ device helpers ═══════════════════════════════════ */
#pragma omp declare target
inline void rotZYX(float yaw,float pitch,float roll,
                   float x,float y,float z,
                   float& rx,float& ry,float& rz)
{
    const float cy = cosf(yaw),   sy = sinf(yaw);
    const float cp = cosf(pitch), sp = sinf(pitch);
    const float cr = cosf(roll),  sr = sinf(roll);

    rx = cy*cp*x + (cy*sp*sr - sy*cr)*y + (cy*sp*cr + sy*sr)*z;
    ry = sy*cp*x + (sy*sp*sr + cy*cr)*y + (sy*sp*cr - cy*sr)*z;
    rz =       -sp*x +  cp*sr*y +  cp*cr*z;
}

/* inclusive 3-D prefix-sum query (device version) --------------------- */
inline int ivSumDev(const int* iv,
                    int nx,int ny,int nz,
                    float cell,float ox,float oy,float oz,
                    float x0,float y0,float z0,
                    float x1,float y1,float z1)
{
    if (x1 < x0) { float t=x0; x0=x1; x1=t; }
    if (y1 < y0) { float t=y0; y0=y1; y1=t; }
    if (z1 < z0) { float t=z0; z0=z1; z1=t; }

    auto toI=[&](float v,float o,int n){
        int i = int((v-o)/cell);
        return (i<0)?0:(i>n)?n:i;
    };
    int ix0=toI(x0,ox,nx), iy0=toI(y0,oy,ny), iz0=toI(z0,oz,nz);
    int ix1=toI(x1,ox,nx), iy1=toI(y1,oy,ny), iz1=toI(z1,oz,nz);

    auto at=[&](int x,int y,int z)->int{
        if(x<0||y<0||z<0) return 0;
        return iv[std::size_t(z*(ny+1)*(nx+1) + y*(nx+1) + x)];
    };

    int A=at(ix1,iy1,iz1), B=at(ix0-1,iy1,iz1),
        C=at(ix1,iy0-1,iz1), D=at(ix1,iy1,iz0-1),
        E=at(ix0-1,iy0-1,iz1), F=at(ix0-1,iy1,iz0-1),
        G=at(ix1,iy0-1,iz0-1), H=at(ix0-1,iy0-1,iz0-1);
    return A-B-C-D+E+F+G-H;
}
#pragma omp end declare target
/* ══════════════════════════════════════════════════════════════════════ */

/* ───────────────────────── detectHeads() ────────────────────────────── */
template<typename PointT>
std::vector<HeadPose>
Detector::detectHeads(const typename pcl::PointCloud<PointT>::ConstPtr& cloud)
{
    std::vector<HeadPose> dets;
    if(!cloud || cloud->empty()) return dets;

    /* build solid integral volume */
    auto iv = IntegralVolume::buildFromCloud<PointT>(cloud, kCell_);

    const int*       ivPtr = iv.data();
    const std::size_t ivSz = iv.size();

    /* search bounds --------------------------------------------------- */
    float min_x,max_x,min_y,max_y,min_z,max_z;
    iv.bounds(min_x,max_x,min_y,max_y,min_z,max_z);
    const float z0 = max_z - 0.30f;

    const int Nx = int((max_x-min_x)/kStep_);
    const int Ny = int((max_y-min_y)/kStep_);
    const int Nz = int((max_z-z0   )/kStep_);

    // Dynamically build yaw, pitch, and roll vectors from nsteps in [0, 2pi)
    int nsteps = 9; // or any desired number of steps
    std::vector<float> kYaw(nsteps), kPitch(nsteps), kRoll(nsteps);
    float step = 2.0f * static_cast<float>(M_PI) / static_cast<float>(nsteps);
    for (int i = 0; i < nsteps; ++i) {
        kYaw[i] = i * step;
        kPitch[i] = i * step;
        kRoll[i] = i * step;
    }

/* ═════════════════════ GPU SINGLE-KERNEL PATH ════════════════════════ */
#if HAVE_GPU
    #pragma omp target enter data map(to: ivPtr[0:ivSz]) map(to: kStage2)

    /* global best score & pose live in device memory */
    float    dBestScore = -1e30f;
    HeadPose dBestPose  {};

    #pragma omp target teams distribute parallel for collapse(5)         \
            map(to:ivPtr[0:ivSz]) map(tofrom:dBestScore,dBestPose)        \
            firstprivate(iv,min_x,min_y,z0)
    for(int xi=0; xi<Nx; ++xi)
      for(int yi=0; yi<Ny; ++yi)
        for(int zi=0; zi<Nz; ++zi)
          for(int iy=0; iy<9; ++iy)
            for(int ip=0; ip<5; ++ip)
    {
        float cx = min_x + xi*kStep_;
        float cy = min_y + yi*kStep_;
        float cz = z0    + zi*kStep_;
        float yaw   = kYaw[iy];
        float pitch = kPitch[ip];

        for (float roll : kRoll)
        {
            /* ─ Stage-1 shells ─ */
            bool ok = true;
            for (const HaarMask* mp : kStage1)
            {
                const HaarMask& m = *mp;
                float sm = 0.f;
                for (const auto& b : m.boxes)
                    if (b.w)
                        sm += b.w * ivSumDev(ivPtr, iv.nx(),iv.ny(),iv.nz(),
                                             iv.cell(),iv.ox(),iv.oy(),iv.oz(),
                                             cx+b.dx0,cy+b.dy0,cz+b.dz0,
                                             cx+b.dx1,cy+b.dy1,cz+b.dz1);
                if (sm <= 0.f) { ok = false; break; }
            }
            if (!ok) continue;

            /* ─ Stage-2 fine masks ─ */
            int   pass = 0;
            float occ  = 0.f;
            float sym  = 0.f;

            for (const HaarMask* mp : kStage2)
            {
                const HaarMask& m = *mp;
                float ms = 0.f;

                for (const auto& b : m.boxes)
                {
                    if (b.w == 0.f) break;
                    float rx0,ry0,rz0, rx1,ry1,rz1;
                    rotZYX(yaw,pitch,roll,
                           b.dx0,b.dy0,b.dz0, rx0,ry0,rz0);
                    rotZYX(yaw,pitch,roll,
                           b.dx1,b.dy1,b.dz1, rx1,ry1,rz1);

                    ms += b.w * ivSumDev(ivPtr, iv.nx(),iv.ny(),iv.nz(),
                                         iv.cell(),iv.ox(),iv.oy(),iv.oz(),
                                         cx+fminf(rx0,rx1), cy+fminf(ry0,ry1), cz+fminf(rz0,rz1),
                                         cx+fmaxf(rx0,rx1), cy+fmaxf(ry0,ry1), cz+fmaxf(rz0,rz1));
                }

                bool symM = (mp == &kSymLR || mp == &kSymFB);
                if (symM) {
                    float d = fabsf(ms);
                    if (d > kSymThresh_) { pass = -99; break; }
                    sym += d; ++pass; continue;
                }
                if (ms <= 0.f) { pass = -99; break; }

                occ += ms; ++pass;
            } /* fine masks */

            if (pass < kMinMasksHit_) continue;

            float score = occ - kSymPenalty_*sym;

            /* device-side critical to update global best */
            #pragma omp critical
            if (score > dBestScore)
            {
                dBestScore = score;
                dBestPose = { cx,cy,cz,yaw,pitch,roll };
            }
        } /* roll */
    } /* kernel */

    if (dBestScore > 0.f) dets.push_back(dBestPose);

/* ═════════════════════ CPU MULTI-THREAD PATH ════════════════════════ */
#else
    float    bestScore = -1e30f;
    HeadPose bestPose  {};

#pragma omp parallel
    {
        float    localBest = -1e30f;
        HeadPose localPose {};

#pragma omp for collapse(3) schedule(static)
        for (int xi=0; xi<Nx; ++xi)
            for (int yi=0; yi<Ny; ++yi)
                for (int zi=0; zi<Nz; ++zi)
                {
                    float cx = min_x + xi*kStep_;
                    float cy = min_y + yi*kStep_;
                    float cz = z0    + zi*kStep_;

                    bool stage1 = true;
                    for (const HaarMask* mp : kStage1)
                    {
                        const HaarMask& m = *mp;
                        float sm = 0.f;
                        for (const auto& b : m.boxes)
                            if (b.w)
                                sm += b.w * iv.sum(cx+b.dx0,cy+b.dy0,cz+b.dz0,
                                                   cx+b.dx1,cy+b.dy1,cz+b.dz1);
                        if (sm <= 0.f) { stage1 = false; break; }
                    }
                    if (!stage1) continue;

                    for (float yaw   : kYaw)
                        for (float pitch : kPitch)
                            for (float roll  : kRoll)
                            {
                                int   pass = 0;
                                float occ  = 0.f;
                                float sym  = 0.f;

                                for (const HaarMask* mp : kStage2)
                                {
                                    const HaarMask& m = *mp;
                                    float ms = 0.f;
                                    for (const auto& b : m.boxes)
                                    {
                                        if (b.w == 0.f) break;
                                        float rx0,ry0,rz0, rx1,ry1,rz1;
                                        rotZYX(yaw,pitch,roll,
                                               b.dx0,b.dy0,b.dz0, rx0,ry0,rz0);
                                        rotZYX(yaw,pitch,roll,
                                               b.dx1,b.dy1,b.dz1, rx1,ry1,rz1);

                                        ms += b.w * iv.sum(cx+std::min(rx0,rx1),
                                                           cy+std::min(ry0,ry1),
                                                           cz+std::min(rz0,rz1),
                                                           cx+std::max(rx0,rx1),
                                                           cy+std::max(ry0,ry1),
                                                           cz+std::max(rz0,rz1));
                                    }
                                    bool symM = (mp == &kSymLR || mp == &kSymFB);
                                    if (symM) {
                                        float d = fabsf(ms);
                                        if (d > kSymThresh_) { pass = -99; break; }
                                        sym += d; ++pass; continue;
                                    }
                                    if (ms <= 0.f) { pass = -99; break; }

                                    occ += ms; ++pass;
                                }
                                if (pass < kMinMasksHit_) continue;

                                float score = occ - kSymPenalty_*sym;
                                if (score > localBest) {
                                    localBest = score;
                                    localPose = {cx,cy,cz,yaw,pitch,roll};
                                }
                            } /* all RPY */
                } /* windows */

#pragma omp critical
        if (localBest > bestScore) {
            bestScore = localBest;
            bestPose  = localPose;
        }
    } /* parallel */

    if (bestScore > 0.f) dets.push_back(bestPose);
#endif /* HAVE_GPU */

    return dets;
}
} // namespace detection
