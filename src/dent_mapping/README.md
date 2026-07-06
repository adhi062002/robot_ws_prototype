# dent_mapping — ROS2 Humble

3-D dent mapping for the Intel RealSense D415.  
YOLO detects dents in 2-D → depth ROI is backprojected to 3-D → patch is
**frozen permanently in map frame**.  
SuperGlue + TEASER++ + Colored ICP handle frame-to-frame ego-motion to keep
the global map consistent.

---

## How dent freezing works

```
RGB frame ──► YOLO ──► bbox (x1,y1,x2,y2)
                             │
                    mask depth ROI pixels
                             │
                    backproject → Nx3 pts (camera frame)
                             │
                    apply current T_global + FLIP
                             │
                    store in dent_patches[]  ◄── NEVER MOVED AGAIN
                             │
                    publish /dent_mapping/dent_cloud (accumulated)
```

Each detection becomes a coloured `DentPatch` object (bright red in RViz2).
Multiple detections across frames all accumulate — the dent cloud only grows.

---

## Package layout

```
dent_mapping/
├── dent_mapping_node.py     ← single node, all logic
├── launch/
│   ├── live.launch.py       ← D415 live stream
│   └── bag.launch.py        ← offline / image files
├── config/
│   └── bag_config.yaml      ← YOLO + intrinsics + paths
├── package.xml
└── setup.py
```

---

## Prerequisites

### ROS2 Humble packages
```bash
sudo apt install -y \
  ros-humble-cv-bridge \
  ros-humble-sensor-msgs-py \
  ros-humble-tf2-ros \
  ros-humble-message-filters \
  ros-humble-realsense2-camera
```

### Python
```bash
pip install ultralytics open3d torch torchvision teaserpp-python scipy
# SuperGlue:
git clone https://github.com/magicleap/SuperGluePretrainedNetwork.git ~/SuperGlue
export PYTHONPATH=$PYTHONPATH:~/SuperGlue
```

---

## Build

```bash
cp -r dent_mapping ~/ros2_ws/src/
cd ~/ros2_ws
colcon build --packages-select dent_mapping
source install/setup.bash
```

---

## Running

### Live (D415 plugged in)
```bash
ros2 launch dent_mapping live.launch.py \
    yolo_model_path:=/path/to/dent_yolov8.pt \
    yolo_conf:=0.4
```

### Bag / offline
```bash
# Edit config/bag_config.yaml, then:
ros2 launch dent_mapping bag.launch.py \
    config:=$(ros2 pkg prefix dent_mapping)/share/dent_mapping/config/bag_config.yaml
```

---

## Topics

| Topic | Type | Description |
|---|---|---|
| `/dent_mapping/scene_cloud` | `PointCloud2` | Full accumulated scene |
| `/dent_mapping/dent_cloud` | `PointCloud2` | **Frozen** dent patches (red) |
| `/dent_mapping/detection_image` | `Image` | RGB with YOLO bboxes |
| `/dent_mapping/overlay_image` | `Image` | RGB with tinted dent fills |
| `/tf` | TF | `frame_N-1→frame_N` and `map→camera` |

---

## Parameters

| Parameter | Default | Description |
|---|---|---|
| `source` | `"live"` | `"live"` or `"bag"` |
| `yolo_model_path` | `yolov8n.pt` | Path to `.pt` weights |
| `yolo_conf` | `0.4` | YOLO confidence threshold |
| `yolo_class_ids` | `[]` | Class IDs = dents (`[]` = all) |
| `dent_min_depth` | `0.1` | Min depth for backprojection (m) |
| `dent_max_depth` | `3.5` | Max depth for backprojection (m) |
| `voxel_size` | `0.01` | Downsample voxel size (m) |
| `fx/fy/cx/cy` | D415 defaults | Auto-read in live mode |
| `map_frame` | `"map"` | Global TF frame |
| `camera_frame` | `"camera_color_optical_frame"` | Camera TF frame |

---

## Training your own dent YOLO model

```bash
# Label your dent images with any YOLO-format tool (Roboflow, CVAT, etc.)
# Then fine-tune:
yolo detect train \
    data=dent_dataset.yaml \
    model=yolov8n.pt \
    epochs=100 \
    imgsz=640

# Point yolo_model_path at runs/detect/train/weights/best.pt
```

---

## RViz2 setup

Add these displays:
- **PointCloud2** → `/dent_mapping/scene_cloud`  (white/grey)
- **PointCloud2** → `/dent_mapping/dent_cloud`   (set fixed colour → red)
- **Image**       → `/dent_mapping/detection_image`
- **TF**          → to see frame chain

Set **Fixed Frame** = `map`.
