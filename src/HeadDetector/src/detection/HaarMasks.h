/*******************************************************************************
 * HaarMasks.h  —  sphere •  axis-cross •  lollipop •  shoulder-lollipop
 * ---------------------------------------------------------------------------
 * 2025-06-12  ·  BSD-3-Clause
 ******************************************************************************/
#ifndef HEAD_DETECTOR_HAAR_MASKS_H_
#define HEAD_DETECTOR_HAAR_MASKS_H_

#include <array>

/* ───── geometry constants (metres) ───── */
static constexpr float R   = 0.15f;   /* head radius (≈26 cm Ø)           */
static constexpr float T   = 0.02f;   /* shell thickness                  */
static constexpr float L   = R;       /* half-side of bounding cube       */
static constexpr float Cx  = 0.06f;   /* half thickness of neck / cross   */
static constexpr float NH  = 0.08f;   /* neck height                      */
static constexpr float SH  = 0.26f;   /* half shoulder span               */
static constexpr float SZ  = 0.08f;   /* shoulder thickness (Z)           */

struct HaarBox  { float dx0,dy0,dz0,  dx1,dy1,dz1,  w; };
struct HaarMask { const char* name;   std::array<HaarBox,8> boxes; };

/* ──── make masks visible to GPU off-load ──── */
#pragma omp declare target

/* ─────────────── Stage-1 : coarse gates ──────────────── */
/* 1️⃣ rotation-agnostic spherical shell */
static constexpr HaarMask kSphereShell = { "sphereShell", {{
    {  L-T,-L,-L,   L,  L,  L, +1.f}, { -L,-L,-L,-L+T,L, L,+1.f},
    { -L, L-T,-L,   L,  L,  L, +1.f}, { -L,-L,-L,   L,-L+T,L,+1.f},
    { -L,-L, L-T,   L,  L,  L, +1.f}, { -L,-L,-L,   L,  L,-L+T,+1.f},
    { -0.11f,-0.11f,-0.11f, 0.11f,0.11f,0.11f, -1.f},
    {}
}}};


/* 2️⃣ compact 8-box “sphere gate”  (−  +  −)                 *
 *    – six outer NEGATIVE slabs  (thickness T)               *
 *    – one middle POSITIVE slab  (thickness T)               *
 *    – one inner NEGATIVE core   (18 cm cube)                */
static constexpr HaarMask kSphereGate = { "sphereGate", {{
    /* outer NEGATIVE shell (−2) — 6 boxes */
    {  L-T,-L,-L,   L,  L,  L,   -2.f},
    { -L, -L,-L, -L+T, L,  L,   -2.f},
    { -L,  L-T,-L,  L,  L,  L,   -2.f},
    { -L, -L,-L,   L,-L+T, L,   -2.f},
    { -L, -L, L-T,  L,  L,  L,   -2.f},
    { -L, -L,-L,   L,  L,-L+T,  -2.f},

    /* middle POSITIVE shell (thickness T) — choose +Z face */
    { -L+T, -L+T,  L-2*T,  L-T,  L-T,  L-T,  +10.f},

    /* inner NEGATIVE core (18 cm cube) */
    { -0.09f, -0.09f, -0.09f,  0.09f, 0.09f, 0.09f,  -1.f}
}}};

/* Stage-1 list */
static constexpr std::array<const HaarMask*,1> kStage1 = {
     &kSphereGate
};

/* 2️⃣ axis-cross: three orthogonal slabs through the centre */
static constexpr HaarMask kAxisCross = { "axisCross", {{
    { -L, -Cx, -Cx,   L,  Cx,  Cx, +1.f},   /* X-axis slab */
    { -Cx, -L, -Cx,   Cx,  L,  Cx, +1.f},   /* Y-axis slab */
    { -Cx, -Cx, -L,   Cx,  Cx,  L, +1.f},   /* Z-axis slab */
    {}
}}};

/* ─────────────── Stage-2 : refined shells ────────────── */
/* (A) sphere + neck column (“lollipop”) */
static constexpr HaarMask kLollipop = { "lollipop", {{
    { -L,-L,-L,  L, L, L, +1.f},
    { -0.11f,-0.11f,-0.11f, 0.11f,0.11f,0.11f, -1.f},
    { -Cx,-Cx,-L-NH,  Cx, Cx,-L, +1.f},
    {}
}}};

/* (B) sphere + neck + shoulders */
static constexpr HaarMask kShoulderLollipop = { "shoulderLollipop", {{
    { -L,-L,-L,  L, L, L, +1.f},
    { -0.11f,-0.11f,-0.11f, 0.11f,0.11f,0.11f, -1.f},
    { -Cx,-Cx,-L-NH,   Cx,  Cx,-L,  +1.f},
    { -SH,-L-0.02f,-SZ-0.25f, -0.12f, L+0.02f,-0.15f, +1.f},
    { 0.12f,-L-0.02f,-SZ-0.25f,   SH, L+0.02f,-0.15f, +1.f},
    { -0.10f,-0.10f,-SZ-0.25f, 0.10f,0.10f,-0.15f,  -2.f},
    {}
}}};

/* Stage-2 list */
static constexpr std::array<const HaarMask*,3> kStage2 = {
    &kLollipop,
    &kShoulderLollipop,
    &kAxisCross
};

/* ── dummy symbols to satisfy legacy tests in Detector.hpp ── */
static constexpr HaarMask kSymLR = { "unusedLR", {{}} };
static constexpr HaarMask kSymFB = { "unusedFB", {{}} };

#pragma omp end declare target
#endif /* HEAD_DETECTOR_HAAR_MASKS_H_ */
