# ROS 2 Bag to VDB Converter

Convert ROS 2 bag files containing LiDAR point clouds into OpenVDB level-set maps.

---

### Key Capabilities
- **Point-to-Plane ICP Registration** - Sub-centimeter alignment accuracy
- **Global Pose Graph Optimization** - Corrects accumulated drift across entire trajectories
- **Odometry Integration** - Uses wheel/visual odometry for better initial alignment
- **Loop Closure Detection** (Optional) - Finds and constrains revisited areas with FPFH feature matching
- **Z-Axis Leveling** - Prevents vertical drift in planar environments
- **Adaptive Frame Filtering** - Automatically skips low-quality registrations
- **OpenVDB Level-Set Output** - Narrow-band signed distance field compatible with most robotics planning and simulation frameworks

---

## 🛠 What You'll Need

### Required
- **Docker** ([Install Docker](https://docs.docker.com/get-docker/))
- **ROS 2 Bag File** with `sensor_msgs/PointCloud2` messages
- **8GB+ RAM** (16GB+ recommended for large datasets)

### Optional (Improves Results)
- Odometry topic (`nav_msgs/Odometry`) - For better initial alignment
- ~5-10GB free disk space per hour of bag data

---

## 🚀 Getting Started

### 1. Set Up Your Project

```bash
mkdir bag-to-vdb && cd bag-to-vdb
# Place Dockerfile and bag_to_vdb.py here
```


### 2. Build the Docker Image

```bash
docker build -t bag-to-vdb .
```

This downloads dependencies (~2GB) and takes 5–10 minutes on first build.

### 3. Organize Your Data

```bash
mkdir -p data/input data/output
# Copy your ROS 2 bag file to data/input/
```


### 4. Run Your First Conversion

**Basic command** (assumes point cloud topic is `points`):

```bash
docker run --rm \
  -v "$(pwd)/data:/app/data" \
  bag-to-vdb \
  /app/data/input/your_bag_file \
  /app/data/output
```

**Recommended command** (with odometry and floor leveling):

```bash
docker run --rm \
  -v "$(pwd)/data:/app/data" \
  bag-to-vdb \
  /app/data/input/your_bag_file \
  /app/data/output \
  --pc_topic /your/pointcloud/topic \
  --odom_topic /your/odom/topic \
  --icp_fitness_thresh 0.4 \
  --level_floor
```


### 5. View Your Results

Output files will be in `data/output/`:

- `your_bag_file_cloud.ply` - Debug point cloud (view in CloudCompare, MeshLab)
- `your_bag_file_map.vdb`  - OpenVDB level-set map (view in Houdini, Blender, or `vdb_view`)

---

## 💡 Common Use Cases

### Indoor Mapping with TurtleBot

```bash
docker run --rm \
  -v "$(pwd)/data:/app/data" \
  bag-to-vdb \
  /app/data/tb3_office_scan \
  /app/data/output \
  --pc_topic /scan/points \
  --odom_topic /odom \
  --voxel_size 0.02 \
  --icp_fitness_thresh 0.5 \
  --level_floor
```

**Why these settings?**

- `voxel_size 0.02` - Finer detail for small spaces
- `icp_fitness_thresh 0.5` - Balanced quality (indoor has good features)
- `level_floor` - Corrects floor tilt in multi-room scans

---

### Outdoor Mapping with DLIO/LIO-SAM

```bash
docker run --rm \
  -v "$(pwd)/data:/app/data" \
  bag-to-vdb \
  /app/data/outdoor_survey \
  /app/data/output \
  --pc_topic /dlio/odom_node/pointcloud/deskewed \
  --odom_topic /dlio/odom_node/odom \
  --icp_fitness_thresh 0.2 \
  --icp_dist_thresh 0.5 \
  --level_floor \
  --voxel_size 0.1 \
  --vdb_voxel_size 0.1
```

**Why these settings?**

- Lower fitness threshold (0.2) - Outdoor has fewer distinct features
- `vdb_voxel_size 0.1` - Coarser VDB resolution for large environments reduces memory
- Uses pre-deskewed clouds from SLAM system for better initial quality

---

### High-Quality Object Scanning

```bash
docker run --rm \
  -v "$(pwd)/data:/app/data" \
  bag-to-vdb \
  /app/data/object_scan \
  /app/data/output \
  --pc_topic /camera/depth/points \
  --voxel_size 0.005 \
  --icp_dist_thresh 0.05 \
  --icp_fitness_thresh 0.7 \
  --vdb_voxel_size 0.005 \
  --vdb_half_width 3
```

**Why these settings?**

- `voxel_size 0.005` - Maximum detail preservation for small objects
- Tight thresholds - Ensures precise alignment
- `vdb_voxel_size 0.005` - VDB resolution matches point cloud resolution

---

### Large Campus/Warehouse Mapping (with Loop Closure)

```bash
docker run --rm \
  -v "$(pwd)/data:/app/data" \
  bag-to-vdb \
  /app/data/warehouse_full \
  /app/data/output \
  --pc_topic /velodyne_points \
  --odom_topic /integrated_odom \
  --voxel_size 0.1 \
  --icp_fitness_thresh 0.3 \
  --level_floor \
  --enable_loop_closure \
  --vdb_voxel_size 0.1
```

**Why these settings?**

- `voxel_size 0.1` - Aggressive downsampling for large environments
- `enable_loop_closure` - Constrains drift when revisiting areas
- `icp_fitness_thresh 0.3` - More permissive to include more frames

---

## 📖 Parameter Reference

### Required Arguments

| Parameter | Purpose |
| :-- | :-- |
| `bag_path` | Path to the ROS 2 bag file |
| `output_dir` | Directory where output files are saved |

### Topic Arguments

| Parameter | Default | Purpose |
| :-- | :-- | :-- |
| `--pc_topic` | `points` | `sensor_msgs/PointCloud2` topic name |
| `--odom_topic` | `None` | Odometry topic (`nav_msgs/Odometry`); improves ICP initial guess |
| `--imu_topic` | `None` | IMU topic (parsed but currently unused in pipeline) |
| `--odom_max_latency` | 0.5 s | Staleness cutoff for odom↔pointcloud timestamp matching |

### Point Cloud Processing

| Parameter | Default | Range | Purpose |
| :-- | :-- | :-- | :-- |
| `--voxel_size` | `0.05` | 0.001–1.0 | Downsampling resolution for registration and cleaning (metres) |
| `--icp_dist_thresh` | `0.2` | 0.01–10.0 | Max point correspondence distance for ICP (metres) |
| `--icp_fitness_thresh` | `0.6` | 0.0–1.0 | Min fraction of points aligned to accept a frame |

### Loop Closure

| Parameter | Default | Purpose |
| :-- | :-- | :-- |
| `--enable_loop_closure` | `False` | Enable loop closure detection (disabled by default for speed) |
| `--loop_closure_radius` | `10.0` | Spatial radius to search for loop closure candidates (metres) |
| `--loop_closure_fitness_thresh` | `0.3` | Min ICP fitness to accept a loop closure constraint |
| `--loop_closure_search_interval` | `10` | Run loop closure search every N registered frames |

### Map Post-Processing

| Parameter | Default | Purpose |
| :-- | :-- | :-- |
| `--level_floor` | `False` | Detect ground plane via RANSAC and rotate map so Z points up |

### VDB Output

| Parameter | Default | Purpose |
| :-- | :-- | :-- |
| `--vdb_voxel_size` | same as `--voxel_size` | VDB grid voxel edge length (metres). Set independently to use a coarser or finer VDB resolution than the processing voxel size |
| `--vdb_half_width` | `3` | Narrow-band half-width in voxels. Truncation distance = `vdb_half_width × vdb_voxel_size`. Increase for thicker surfaces; rarely needs changing |


---

## 🎯 Pro Tips

### Finding Your Topics

```bash
ros2 bag info /path/to/your/bag_file
```

Look for:

- `sensor_msgs/msg/PointCloud2` - Your point cloud topic
- `nav_msgs/msg/Odometry` - Your odometry topic


### VDB Voxel Size vs Processing Voxel Size

`--voxel_size` controls the resolution used for point cloud downsampling, outlier removal, and ICP. `--vdb_voxel_size` controls the resolution of the final output grid. They default to the same value, which is usually correct. You may want a coarser `--vdb_voxel_size` for large outdoor maps to reduce file size, or a finer one for object scans where you want a high-detail SDF.

### Narrow-Band Width

The default `--vdb_half_width 3` creates a ±3 voxel band around each surface. For a 0.05 m voxel size this means the SDF is valid within ±0.15 m of any obstacle surface — sufficient for most collision-checking applications. Increasing to 5–6 is useful if your planner queries distances further from surfaces.

### Performance Trade-Offs

| Goal | `--voxel_size` | `--icp_fitness_thresh` | `--icp_dist_thresh` |
| :-- | :-- | :-- | :-- |
| 🚀 Fast preview | 0.1 | 0.3 | 0.5 |
| ⚖️ Balanced | 0.05 | 0.5 | 0.2 |
| 🎨 Maximum quality | 0.01 | 0.7 | 0.1 |

### Loop Closure (Disabled by Default)

Enable only for **large-scale mapping** where you revisit areas:

**Use it when:**

- ✅ Large outdoor surveys (>30 min of data)
- ✅ Multi-floor indoor mapping
- ✅ Areas with significant loops/revisits
- ✅ Accuracy is critical

**Skip it when:**

- ❌ Linear paths (corridors, roads)
- ❌ Small rooms or areas
- ❌ Speed is critical

**Performance impact:**

- Disabled (default): ~2–3 min per 1000 frames
- Enabled: ~5–15 min per 1000 frames


### When to Use `--level_floor`

✅ **Use it when:**

- Mapping single-floor indoor spaces
- Scanning parking lots or warehouses
- The output PLY shows an undulating floor

❌ **Don't use when:**

- Mapping multi-story buildings
- Scanning terrain or hills
- Working with ramps or significant elevation changes

---

## 🔧 Troubleshooting

### "Error: No messages found for topics"

**Problem:** Topic names don't match your bag file.

**Fix:**

```bash
ros2 bag info your_bag_file
# Use exact topic names from the output
docker run --rm -v "$(pwd)/data:/app/data" bag-to-vdb \
  /app/data/your_bag \
  /app/data/output \
  --pc_topic /exact/topic/name
```


---

### "Error: Registration failed (too few successful registrations)"

**Fix (in order of priority):**

1. **Lower fitness threshold** - Accept more frames:

```bash
--icp_fitness_thresh 0.3
```

2. **Increase distance threshold** - Allow looser matching:

```bash
--icp_dist_thresh 0.5
```

3. **Add odometry** if available — significantly improves initial guess:

```bash
--odom_topic /odom
```


---

### Point Clouds Look "Stacked" or Doubled

**Problem:** Severe registration failure causing ghost geometry.

**Fix:** Much stricter filtering to reject bad frames:

```bash
--icp_fitness_thresh 0.7 \
--icp_dist_thresh 0.15
```


---

### Processing is Too Slow

**Speed-up strategies (in priority order):**

1. **Increase voxel size** (biggest impact):

```bash
--voxel_size 0.1
```

2. **Disable loop closure** (if enabled)
3. **Pre-filter your bag file:**

```bash
ros2 bag filter input.bag output.bag \
  "topic == '/your/topic' and t.sec % 5 == 0"
```


---

### Vertical Drift (Z-axis Issues)

1. Try `--level_floor` for flat environments
2. Add `--enable_loop_closure`
3. Lower `--icp_fitness_thresh` to 0.3 to include more frames

---

### "Warning: VDB generation failed"

**Possible causes and fixes:**

1. **`pyopenvdb` not importable** - Verify the Docker image was built correctly:

```bash
docker run --rm bag-to-vdb python3 -c "import pyopenvdb; print(pyopenvdb.__version__)"
```

If this fails, rebuild with `docker build --no-cache -t bag-to-vdb .`
2. **`vdb.tools.topologyToLevelSet` not available** - The script automatically falls back to a kd-tree-based narrow-band SDF. Check the log for:

```
Warning: vdb.tools.topologyToLevelSet not available ... falling back to kd-tree narrow-band SDF.
```

The output is still a valid VDB level set; the fallback just produces an unsigned (non-negative) SDF rather than a signed one.
3. **Empty point cloud after cleaning** - Loosen the outlier removal by increasing `--voxel_size` to retain more points.

---

## ⚠️ Known Limitations

### Loop Closure

- Searches within a spatial radius and temporal window
- May produce false positives in highly repetitive environments
- Disabled by default due to performance impact


### Scale

- Tested up to ~5000 frames (~10 min of data at 10 Hz)
- Larger datasets may require 32GB+ RAM
- Consider splitting very large bags


### Point Cloud Types

- Only works with XYZ fields (FLOAT32 or FLOAT64)
- Intensity/colour fields are ignored


### Real-Time

- Not designed for real-time operation
- Typical speed: ~2–3 min per minute of recorded data (without loop closure)

---

## 📊 Performance Guide

### Processing Times (Without Loop Closure)

| Dataset Size | Point Clouds | Voxel Size | Time | Peak RAM |
| :-- | :-- | :-- | :-- | :-- |
| Small room | ~500 frames | 0.02 m | 1–2 min | 4 GB |
| Office floor | ~1500 frames | 0.05 m | 3–5 min | 8 GB |
| Large warehouse | ~3000 frames | 0.1 m | 8–12 min | 12 GB |
| Campus outdoor | ~5000 frames | 0.1 m | 15–25 min | 16 GB |

### Processing Times (With Loop Closure)

| Dataset Size | Point Clouds | Voxel Size | Time | Peak RAM |
| :-- | :-- | :-- | :-- | :-- |
| Office floor | ~1500 frames | 0.05 m | 10–20 min | 8 GB |
| Large warehouse | ~3000 frames | 0.1 m | 30–60 min | 12 GB |
| Campus outdoor | ~5000 frames | 0.1 m | 60–120 min | 16 GB |

### Hardware Recommendations

| Scenario | CPU | RAM | Storage |
| :-- | :-- | :-- | :-- |
| Fast preview | 4 cores | 8 GB | 5 GB |
| Standard mapping | 8 cores | 16 GB | 20 GB |
| Large datasets | 16+ cores | 32 GB | 50+ GB SSD |


---

## 📝 Output Files

### `*_cloud.ply`

- **Type:** Point cloud (debug artifact)
- **Contents:** Cleaned, registered world-frame point cloud; normals channel contains averaged view-ray directions from the merge step
- **Tools:** CloudCompare, MeshLab, Open3D
- **Typical size:** ~500 MB per 1000 frames at voxel 0.05 m


### `*_map.vdb`

- **Type:** OpenVDB FloatGrid, narrow-band level set
- **Contents:** Signed (or unsigned, see fallback note) distance field. Active voxels lie within `vdb_half_width × vdb_voxel_size` metres of obstacle surfaces. Grid name: `"Map"`
- **Tools:** Houdini (free Apprentice), Blender (built-in Volume support), `vdb_view` CLI tool from the OpenVDB package, RVIZ2 with a VDB plugin
- **Typical size:** ~50–200 MB depending on environment size and voxel resolution

---

## 🔗 Viewing Your Results

### Point Clouds (`.ply`)

- **CloudCompare** (free, cross-platform): https://www.cloudcompare.org/
- **MeshLab** (free): https://www.meshlab.net/


### VDB Maps (`.vdb`)

- **Houdini Apprentice** (free non-commercial): https://www.sidefx.com/products/houdini-apprentice/
- **Blender** (free): https://www.blender.org/ — import via *File → Import → OpenVDB*
- **`vdb_view`** CLI tool — installed inside the container via `libopenvdb-dev`:

```bash
docker run --rm -v "$(pwd)/data:/app/data" bag-to-vdb \
  vdb_view /app/data/output/your_bag_map.vdb
```

- **Online viewer**: https://3dviewer.net/ (supports `.vdb`)

---

## 📦 Project Files

- `bag_to_vdb.py` - Main conversion script
- `Dockerfile` - Container build instructions
- `README.md` - This file

---

**Built with:** Open3D • OpenVDB (pyopenvdb) • NumPy • SciPy • rosbags • Python 3.10+



