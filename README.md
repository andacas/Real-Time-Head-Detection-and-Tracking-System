

# Real-Time 3-D Haar-Cascade Head Detector 👤💡

[![ROS 2 Humble](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/index.html)
[![License: BSD-3-Clause](https://img.shields.io/badge/License-BSD--3--Clause-green.svg)](LICENSE)

GPU-accelerated 3-D Haar cascade for deterministic, millisecond-scale head
detection in point-clouds. Runs on an **RTX-2060** in real time and integrates
seamlessly with ROS 2 + Gazebo.


---

## 🚀 Quick-Start (Simulation)

> Verified on **Ubuntu 22.04**, ROS 2 **Humble**, Gazebo Garden, CUDA 11.8.

```bash
# 1 Create workspace ────────────────────────────────────────────────
mkdir -p ~/haar_ws/src
cd       ~/haar_ws/src

# 2 Clone repository ────────────────────────────────────────────────
git clone https://github.com/rt3d-haar/head_detector.git
cd ..

# 3 Install ROS 2/pip deps ─────────────────────────────────────────
sudo apt update
rosdep install --from-paths src --ignore-src -r -y

# 4 Colcon build (Release) ─────────────────────────────────────────
colcon build --symlink-install \
             --cmake-args -DCMAKE_BUILD_TYPE=Release

# 5 Source the overlay ─────────────────────────────────────────────
source install/setup.bash
````

---

## 🕹️ Gazebo Demo — 3 Terminals

| Terminal # | Command                                                                                                                                                                                                                      |
| ---------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **1**      | `ros2 launch head_detector_sim person_sim.launch.py`                                                                                                                                                                         |
| **2**      | `bash<br>ros2 launch pointcloud_concatenate pointcloud_concatenate_launch.py \<br>  cloud_in1:=/camera_left/points \<br>  cloud_in2:=/camera_right/points \<br>  target_frame:=world hz:=10.0 cloud_out:=cloud_concatenated` |
| **3**      | `ros2 run head_detector head_detector_node`                                                                                                                                                                                  |

After a few seconds Gazebo spawns two RealSense D435i sensors and a walking
avatar; RViz opens and overlays a green cube on the detected head.

---

## 🧩 Package Overview

| Package                  | Purpose                                                                                         |
| ------------------------ | ----------------------------------------------------------------------------------------------- |
| `detector_cpp`           | Header-only cascade (`IntegralVolume.hpp`, `HaarMasks.h`, `Detector.hpp`) with OpenMP off-load. |
| `head_detector`          | ROS 2 node (`head_detector_node.cpp`) and config/launch files.                                  |
| `head_detector_sim`      | Gazebo world + two RealSense cameras + avatar.                                                  |
| `pointcloud_concatenate` | Merges two `sensor_msgs/PointCloud2` topics into one.                                           |

---

## ⚙️ Key Parameters

| ROS param            | Default | Description                    |
| -------------------- | ------- | ------------------------------ |
| `cell_size`          | `0.025` | Voxel edge length \[m]         |
| `hz`                 | `10.0`  | Fused cloud publish rate \[Hz] |
| `min_masks_hit`      | `3`     | Votes required in Stage-2      |
| `sym_penalty_lambda` | `1.2`   | Weight of asymmetry penalty    |

Override at runtime, e.g.:

```bash
ros2 run head_detector head_detector_node \
     --ros-args -p cell_size:=0.03 -p min_masks_hit:=4
```

---

## 🛠️ Troubleshooting

| Issue                  | Remedy                                                                                   |
| ---------------------- | ---------------------------------------------------------------------------------------- |
| `libcuda.so` not found | Verify `nvidia-smi`, reinstall driver/CUDA toolkit.                                      |
| Blank Gazebo world     | Ensure `GAZEBO_MODEL_PATH` includes `models/`; run `gazebo --verbose`.                   |
| PCL out-of-memory      | Lower RealSense resolution (`640x360@15fps`) or set `voxel_leaf:=0.03` in concat launch. |

---

## 📄 Citation

```bibtex
@misc{castro2025haar,
  title  = {Real-Time 3-D Haar-Cascade Head Detection},
  author = {Castro Peñaranda, A.~A. and Alikhani Najafabadi, M.},
  year   = {2025},
  howpublished = {\url{https://github.com/rt3d-haar/head_detector}}
}
```

---

## 🙋 Maintainers

| Name                             | Contact                                                       |
| -------------------------------- | ------------------------------------------------------------- |
| **Andres A. Castro Peñaranda**   | [andres.castro@upc.edu](mailto:andres.castro@upc.edu)         |
| **Mohammad Alikhani Najafabadi** | [mohammad.alikhani@upc.edu](mailto:mohammad.alikhani@upc.edu) |

Contributions are welcome—please open an issue to discuss large changes first.

---

## 📝 License

BSD 3-Clause. See [`LICENSE`](LICENSE) for full text.

```

Save this as `README.md` in your repository root, commit, and push.
```
