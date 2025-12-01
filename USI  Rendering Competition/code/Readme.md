//Author : Xuanlin Chen
//Work E-mail : chenxu@usi.ch
//Personal E-mail : kissofazshara@gmail.com
//Exercise solved: Bonus Exercies.

## BVH Acceleration Implementation

This raytracer now includes a HLBVH (Hierarchical Linear Bounding Volume Hierarchy) acceleration structure to enable rendering of meshes with millions of triangles efficiently.

### Usage:

1. **Rendering**:
   ```bash
   ./main
   ```

2. **Performance Testing**:
   ```bash
   ./main --more
   ```
   This will generate `performance_data.csv` with timing data for different mesh configurations.

3. **Plot Performance Results**:
   ```bash
   python3 plot_performance.py
   ```
   This generates `bvh_performance.png` showing the sub-linear relationship between triangle count and rendering time.

### Files:
- `bvh.h`: BVH data structures and interface
- `main.cpp`: Complete BVH implementation and integration
- `plot_performance.py`: Python script for performance visualization