> **Note (AI-generated):** This repository was generated with the help of an AI system.

# `pogolm_landmark_usage` (ROS 2)

This repository contains a small ROS 2 package that demonstrates **landmark usage** with the **POGOLM API** (`pgo_with_visual::pogolm_api`) by inserting landmarks from RViz “clicked point” inputs and using POGOLM’s **neighbor query** as a geometric heuristic for **user-side data association**.

---

## What this example does

* Subscribes to **`/clicked_point`** (`geometry_msgs/msg/PointStamped`) — typically published by RViz’s *Publish Point* tool
* Converts the clicked point into a **local robot-relative position** (the format POGOLM expects as input) 
* Runs a **neighbor/range query** (geometric heuristic) around the clicked position
* Stores a new landmark with a generated label like `CLICK_0`, `CLICK_1`, …

This is intentionally a **minimal demo**: POGOLM stores/optimizes **positions** and provides neighbor queries, while **feature-based data association remains the user’s responsibility** (e.g., visual/semantic checks).  

---

## Landmark usage workflow

POGOLM’s intended landmark workflow looks like this:  

1. **Perceive a landmark** in your own pipeline (camera/LiDAR/etc.) and estimate its position **`pos ∈ R³` in the robot/local frame**. 
2. **Ask POGOLM for neighbors** near that position using a **range query** with parameters `r` and `ε` (conceptually “`r ± ε`”). POGOLM returns **neighbor landmark keys** (if any). 
3. **If no neighbors are found**, store the landmark directly and keep the returned **unique key**. 
4. **If neighbors exist**, do **user-side data association** (e.g., compare visual/geometric features you maintain outside POGOLM). Then store the new landmark and pass an **association list** of keys to be merged/replaced. 
5. During optimization, POGOLM adjusts **robot poses and landmark positions together**, yielding globally consistent landmark positions. 

**How this repo maps onto that workflow**

* RViz click = “perception output” (a stand-in for your real detector)
* The node uses POGOLM’s **neighbor query** as the geometric heuristic (Step 2)
* The demo can either:

  * store with **no associations** (simple mode), or
  * store while passing the returned neighbor keys as a naive association list (if enabled/added in code)

---

## Installation

```bash
cd ~/ros2_ws/src
git clone
cd ..
colcon build --packages-select pgo_api_test
source install/setup.bash
```

---

## Run

```bash
ros2 run pgo_api_test t_node
```

This example configures POGOLM in code (topic names may be hard-coded in `src/test_node.cpp`):

* LiDAR input: `/velodyne_points`
* Odometry input: `/odom`
* Operating frame: `odom`

---

## RViz: click-to-insert landmark

1. Open RViz.
2. Set **Fixed Frame** to **`odom`** (important; the node expects consistent frames).
3. Use **Publish Point** and click a location.
4. The node will:

   * convert that click into the expected **local** landmark position (robot-relative)
   * query neighbors
   * store the landmark with label `CLICK_<n>`

**Subscribed topic**

* `/clicked_point` (`geometry_msgs/msg/PointStamped`)

---

## Notes on keys, labels, and queries

* When storing a landmark, POGOLM returns a **unique key** for that landmark. 
* Landmarks can be queried by **key** or by **label** (labels are not necessarily unique). 
* POGOLM supports **range search** and **kNN search** for neighbors (used as a geometric heuristic). 
* POGOLM stores **landmark positions**; additional attributes (visual/semantic features) remain in your application. 
