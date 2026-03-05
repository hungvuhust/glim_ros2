# GLIM Localization Mode - Graph-Based Architecture

## Overview

Localization mode sử dụng **factor graph optimization** (GTSAM iSAM2) để localize robot trong pre-built map. Khác với SLAM mode (tạo map mới), localization mode **cố định prebuilt submaps** và chỉ optimize pose của active submaps (submaps mới tạo từ sensor data hiện tại).

## Factor Graph Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                        FACTOR GRAPH STRUCTURE                        │
└─────────────────────────────────────────────────────────────────────┘

Pre-built Map (FIXED NODES - không optimize)
═══════════════════════════════════════════════

    [Submap_0]━━━[Submap_1]━━━[Submap_2]━━━[Submap_3]━━━[Submap_4]
        │            │            │            │            │
        │            │            │            │            │
     (fixed)      (fixed)      (fixed)      (fixed)      (fixed)
      pose         pose         pose         pose         pose


Active Submaps (VARIABLE NODES - được optimize)
═══════════════════════════════════════════════

                              [Active_0]━━━[Active_1]━━━[Active_2]
                                   │            │            │
                                   ╱            ╱            ╱
                          (odometry)   (odometry)   (odometry)
                                 ╱            ╱            ╱
                                ╱            ╱            ╱
                               ╱            ╱            ╱
Localization Factors      ╱            ╱            ╱
(matching với prebuilt)  ╱            ╱            ╱
                        ╱            ╱            ╱
                   [Submap_2]   [Submap_3]   [Submap_4]
                    (fixed)      (fixed)      (fixed)


Legend:
━━━  Between Factor (odometry constraints)
╱    Localization Factor (scan matching với prebuilt submap)
│    IMU Preintegration Factor
```

## Factor Types

### 1. **Localization Factors** (Quan trọng nhất!)

Kết nối active submaps với prebuilt submaps qua scan matching:

```cpp
// File: glim_localization/src/glim/mapping/localization.cpp

// Find prebuilt submaps gần active submap
auto nearby_submaps = find_nearby_submaps(active_submap->origin,
                                          max_localization_distance);

for (auto& prebuilt : nearby_submaps) {
  // Scan matching giữa active và prebuilt
  auto result = matching(active_submap, prebuilt_submap);

  // Check overlap score
  if (result.overlap > min_localization_overlap) {
    // Tạo localization factor với weight cao
    auto factor = RegistrationFactor(
      active_id, prebuilt_id,
      result.transformation,
      result.information_matrix * relocalization_factor_weight
    );
    graph.add(factor);
  }
}
```

**Parameters ảnh hưởng:**
- `max_localization_distance`: Tìm prebuilt submaps trong bán kính này (8m)
- `min_localization_overlap`: Overlap tối thiểu để chấp nhận match (0.35)
- `relocalization_factor_weight`: Weight của constraint (1e12 = rất mạnh)

### 2. **Between Factors** (Odometry Constraints)

Kết nối giữa các active submaps liên tiếp:

```cpp
// Between factor từ odometry
BetweenFactor<Pose3>(
  active_id_prev, active_id_curr,
  T_prev_curr,  // Relative transformation từ odometry
  noise_model   // Covariance từ odometry estimation
);
```

**Parameters:**
- `loc_between_factor_weight`: Weight của odometry constraints (1e4)

### 3. **IMU Preintegration Factors**

Kết nối IMU measurements giữa các poses:

```cpp
ImuFactor(
  pose_id_prev, vel_id_prev, bias_id_prev,
  pose_id_curr, vel_id_curr, bias_id_curr,
  preintegrated_imu_measurements
);
```

## Localization Process Flow

```
┌──────────────────────────────────────────────────────────────────┐
│ 1. INITIALIZATION                                                │
└──────────────────────────────────────────────────────────────────┘
   │
   ├─► Load pre-built map (submaps với poses đã optimize)
   │   └─► Add prebuilt submaps vào graph như FIXED nodes
   │       (dùng NonlinearEquality1 factors)
   │
   └─► Wait for relocalization trigger

┌──────────────────────────────────────────────────────────────────┐
│ 2. RELOCALIZATION                                                │
└──────────────────────────────────────────────────────────────────┘
   │
   ├─► Trigger: User sets initial pose hoặc call service
   │
   ├─► Search nearby prebuilt submaps (trong linear/angular window)
   │   Parameters:
   │   • linear_search_window: ±5m
   │   • angular_search_window: ±30°
   │
   ├─► Scan matching với từng prebuilt submap
   │   └─► Filter matches bằng min_relocalization_overlap (0.30)
   │
   ├─► Select best match (highest overlap score)
   │
   └─► Initialize active submap pose với match result

┌──────────────────────────────────────────────────────────────────┐
│ 3. CONTINUOUS LOCALIZATION                                       │
└──────────────────────────────────────────────────────────────────┘
   │
   ├─► Odometry estimation tạo estimation frames
   │
   ├─► Sub-mapping tạo active submaps từ frames
   │
   ├─► Insert active submap vào localization:
   │   │
   │   ├─► Find nearby prebuilt submaps (max_localization_distance: 8m)
   │   │
   │   ├─► Scan matching với từng nearby submap
   │   │   └─► Filter: overlap > min_localization_overlap (0.35)
   │   │
   │   ├─► Create localization factors:
   │   │   • Weight: relocalization_factor_weight (1e12)
   │   │   • Kết nối active submap → prebuilt submaps
   │   │
   │   ├─► Create between factors:
   │   │   • Weight: loc_between_factor_weight (1e4)
   │   │   • Kết nối active_prev → active_curr
   │   │
   │   ├─► Add to iSAM2 factor graph
   │   │
   │   └─► Optimize (incremental):
   │       └─► Only optimize active submap poses
   │           (prebuilt submaps FIXED)
   │
   └─► Manage active submaps:
       └─► Keep only num_keep_submaps (15) gần nhất
           └─► Marginalize cũ khỏi graph

┌──────────────────────────────────────────────────────────────────┐
│ 4. POSE OUTPUT                                                   │
└──────────────────────────────────────────────────────────────────┘
   │
   └─► Optimized active submap poses → Transform frames
       └─► Publish TF: map → odom → base_link
```

## Key Differences vs SLAM Mode

| Aspect | SLAM Mode | Localization Mode |
|--------|-----------|-------------------|
| **Prebuilt submaps** | Không có | FIXED nodes trong graph |
| **Active submaps** | Optimize tất cả | Chỉ optimize active |
| **Loop closure** | Tìm trong tất cả submaps | Chỉ match với prebuilt |
| **Map drift** | Có thể drift theo thời gian | Cố định bởi prebuilt map |
| **Relocalization** | Không cần | Required để khởi tạo |

## Parameters Tuning Guide

### Để tăng stability (giảm map jumping):

```json
{
  "min_localization_overlap": 0.35-0.50,     // ↑ Cao hơn = ít false matches
  "angular_search_window": 0.3-0.52,         // ↓ Nhỏ hơn = tighter search
  "relocalization_factor_weight": 1e12-1e13, // ↑ Cao hơn = giữ chặt prebuilt map
  "loc_between_factor_weight": 1e4-1e5       // ↑ Cao hơn = trust odometry more
}
```

### Để tăng robustness (matching được nhiều hơn):

```json
{
  "min_localization_overlap": 0.20-0.30,     // ↓ Thấp hơn = accept more matches
  "max_localization_distance": 10.0-15.0,    // ↑ Rộng hơn = search further
  "linear_search_window": 8.0-10.0           // ↑ Rộng hơn = bigger search area
}
```

### Trade-off:
- **High stability** → Khó localize khi environment thay đổi
- **High robustness** → Dễ false matches và map jumping

## Callbacks Flow

```
┌─────────────────────────────────────────────────────────────────┐
│                    CALLBACK ARCHITECTURE                         │
└─────────────────────────────────────────────────────────────────┘

Localization::load(map_path)
    │
    ├─► For each prebuilt submap:
    │   └─► LocalizationCallbacks::on_insert_localization_submap()
    │       └─► LocalizationViewer displays prebuilt submap
    │
    └─► After all loaded:
        └─► LocalizationCallbacks::on_update_localization_submaps()
            └─► LocalizationViewer updates all coordinates

Localization::insert_submap(active_submap)
    │
    ├─► Match with prebuilt submaps
    │
    ├─► Add to factor graph
    │
    ├─► Optimize
    │
    └─► LocalizationCallbacks::on_update_active_submaps()
        └─► LocalizationViewer displays active submaps
```

## Troubleshooting

### Map jumping issue:
**Nguyên nhân:** Matching với sai prebuilt submap
**Giải pháp:**
1. Tăng `min_localization_overlap` (0.35 → 0.45)
2. Giảm `angular_search_window` (0.52 → 0.3)
3. Tăng `relocalization_factor_weight` (1e12 → 1e13)

### Cannot localize:
**Nguyên nhân:** Overlap thấp hoặc environment thay đổi
**Giải pháp:**
1. Giảm `min_localization_overlap` (0.35 → 0.25)
2. Tăng `max_localization_distance` (8.0 → 12.0)
3. Tăng `linear_search_window` (5.0 → 8.0)

### Drift over time:
**Nguyên nhân:** Localization factors quá yếu
**Giải pháp:**
1. Tăng `relocalization_factor_weight` (1e12 → 1e13)
2. Giảm `loc_between_factor_weight` (1e4 → 1e3) - trust odometry less

## Implementation Files

Key source files:
- `glim/mapping/localization.cpp` - Main localization logic
- `glim/mapping/localization.hpp` - Localization class interface
- `glim/mapping/async_global_mapping.cpp` - Async wrapper
- `glim/viewer/localization_viewer.cpp` - Visualization
- `config/livox/config_global_mapping_gpu.json` - Parameters

## References

- GTSAM iSAM2: https://gtsam.org/
- Factor Graph SLAM: https://gtsam.org/tutorials/intro.html
- VGICP: https://github.com/koide3/small_gicp
