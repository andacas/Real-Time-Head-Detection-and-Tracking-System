#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

#include "detection/Detector.h"
#include "visualization/Visualizer.h"   // ← new path

int main(int argc, char** argv)
{
    if (argc < 2) {
        std::cerr << "Usage: HeadDetector <file.ply>\n";
        return 1;
    }

    /* ---------- load cloud ------------------------------------- */
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPLYFile(argv[1], *cloud) != 0) {
        std::cerr << "Error reading " << argv[1] << '\n';
        return 1;
    }

    /* ---------- detect heads ----------------------------------- */
    detection::Detector detector;
    auto heads = detector.detectHeads<pcl::PointXYZ>(cloud);

    /* ---------- visualise -------------------------------------- */
    visualization::Visualizer vis;
    vis.showCloud<pcl::PointXYZ>(cloud);
    vis.drawDetections(heads);
    return 0;
}
