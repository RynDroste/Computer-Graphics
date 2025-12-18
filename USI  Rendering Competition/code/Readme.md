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

2. **Anti-Aliasing Comparison** (抗锯齿对比):
   ```bash
   ./main --compare-aa
   ```
   This will render two images at 2K resolution (1920x1080):
   - `result_no_aa.ppm`: Without anti-aliasing (faster, but with jagged edges)
   - `result_with_aa.ppm`: With anti-aliasing (slower, but smoother edges)
   
   You can compare these images to see the difference. Anti-aliasing uses 2x2 supersampling (4 samples per pixel) to smooth out jagged edges.

3. **Depth-of-Field Comparison** (景深效果对比):
   ```bash
   ./main --compare-dof
   ```
   This will render two images at 2K resolution (1920x1080):
   - `result_no_dof.ppm`: Without depth-of-field (all objects are sharp)
   - `result_with_dof.ppm`: With depth-of-field (objects out of focus are blurred)
   
   Depth-of-field simulates real camera behavior where objects at the focal distance (22.5 units) are sharp, while foreground and background objects are blurred. This creates a more realistic and cinematic look.

4. **Phong vs Ward Model Comparison** (Phong和Ward模型对比):
   ```bash
   ./main --compare-phong-ward
   ```
   This will render two images at 2K resolution (1920x1080):
   - `result_phong.ppm`: Using Phong model (isotropic specular reflection)
   - `result_ward.ppm`: Using Ward model (anisotropic specular reflection)
   
   **Phong Model**: Isotropic specular reflection where highlights are uniformly distributed in all directions. This is the classic lighting model suitable for most materials.
   
   **Ward Model**: Anisotropic specular reflection where highlights can have different roughness in different directions. This can simulate brushed metals, wood grain, and other anisotropic materials. The model uses tangent and bitangent vectors to define the anisotropic directions.
   
   Note: Ward model requires tangent and bitangent vectors. Spheres and cylinders support this, but planes and triangles may fall back to Phong model if these vectors are not available.

5. **Animation Mode**:
   ```bash
   ./main --animation
   ```
   Renders a 12-frame animation sequence.

6. **Performance Testing**:
   ```bash
   ./main --more
   ```
   This will generate `performance_data.csv` with timing data for different mesh configurations.

7. **Plot Performance Results**:
   ```bash
   python3 plot_performance.py
   ```
   This generates `bvh_performance.png` showing the sub-linear relationship between triangle count and rendering time.

### Files:
- `bvh.h`: BVH data structures and interface
- `main.cpp`: Complete BVH implementation and integration
- `plot_performance.py`: Python script for performance visualization

https://sketchfab.com/3d-models/16-mk6-9b6563ebc38d47f9bc45af5d2bbed745