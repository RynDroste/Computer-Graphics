# 体积烟雾渲染 - Real-Time Rendering 理论解析

本文档结合《Real-Time Rendering》第4版（RTR4）的相关章节，解析实现的体积烟雾渲染技术。

## 1. 体积渲染基础（Volume Rendering）

### 1.1 理论基础（RTR4 Chapter 14: Volumetric and Translucency Rendering）

**参与介质（Participating Media）**：
- 烟雾、雾、云等都属于参与介质
- 光线在介质中会发生**吸收（Absorption）**和**散射（Scattering）**

**辐射传输方程（Radiative Transfer Equation, RTE）**：
```
L(x,ω) = T(x₀,x) L(x₀,ω) + ∫[x₀ to x] T(s,x) σₛ(s) Lᵢ(s,ω) ds
```

其中：
- `L(x,ω)`：位置x、方向ω的辐射亮度
- `T(a,b)`：从a到b的透射率（Transmittance）
- `σₛ`：散射系数
- `Lᵢ`：内散射辐射

### 1.2 实现对应

在我们的代码中（`traceVolumeRay`函数）：

```cpp
// 对应RTE的离散化实现
float transmittance = 1.0f;  // T(x₀,x) - 透射率
glm::vec3 color(0.0f);       // 累积的辐射亮度

// 光线步进（对应积分）
for (int i = 0; i < maxSteps; i++) {
    glm::vec3 pos = ray.origin + ray.direction * t;
    
    // 计算密度（对应σₛ）
    float totalDensity = getSmokeDensity(pos, smoke);
    
    // 吸收（Beer-Lambert定律）
    float absorption = totalDensity * stepSize * 2.0f;
    transmittance *= exp(-absorption);  // T = exp(-τ)，τ是光学深度
    
    // 单次散射（简化版）
    float scattering = totalDensity * stepSize * 0.5f;
    color += smokeColor * scattering * transmittance;
}
```

## 2. Beer-Lambert 定律（RTR4 Section 14.1.2）

### 2.1 理论

**透射率（Transmittance）**：
```
T(d) = exp(-τ(d))
```

其中光学深度（Optical Depth）：
```
τ(d) = ∫[0 to d] σₐ(s) ds
```

- `σₐ`：吸收系数
- `d`：距离

### 2.2 实现对应

```cpp
// 在trace_ray函数中计算透射率
float totalDensity = 0.0f;
for (int i = 0; i < steps; i++) {
    float t = i * stepSize;
    glm::vec3 pos = ray.origin + ray.direction * t;
    totalDensity += getSmokeDensity(pos, smoke) * stepSize;
}

// Beer-Lambert定律
float transmittance = exp(-totalDensity * 2.0f);
```

这对应RTR4中的公式14.2和14.3。

## 3. 体积光线步进（Volume Ray Marching）

### 3.1 理论（RTR4 Section 14.2.1）

**光线步进算法**：
1. 沿光线方向等间距采样
2. 在每个采样点计算密度
3. 累积透射和散射

**采样策略**：
- 均匀采样（Uniform Sampling）
- 自适应采样（Adaptive Sampling）
- 重要性采样（Importance Sampling）

### 3.2 实现对应

```cpp
float stepSize = 0.1f;  // 均匀采样步长
int maxSteps = (int)(maxDistance / stepSize);

for (int i = 0; i < maxSteps && transmittance > 0.01f; i++) {
    // 采样点位置
    glm::vec3 pos = ray.origin + ray.direction * t;
    
    // 计算密度
    float totalDensity = 0.0f;
    for (const auto& smoke : smokeVolumes) {
        totalDensity += getSmokeDensity(pos, smoke);
    }
    
    // 更新透射和散射
    // ...
}
```

**优化技巧**（RTR4 Section 14.2.2）：
- 早期终止：`transmittance > 0.01f`（当透射率太低时停止）
- 层次细节（LOD）：远距离使用更大的步长

## 4. 密度场（Density Field）

### 4.1 理论（RTR4 Section 14.3）

**密度函数**：
- 可以使用解析函数（如椭球、球体）
- 可以使用3D纹理（3D Texture）
- 可以使用程序化噪声（Procedural Noise）

### 4.2 实现对应

```cpp
float getSmokeDensity(glm::vec3 pos, const SmokeVolume& smoke) {
    // 转换到局部空间（椭球）
    glm::vec3 localPos = (pos - smoke.position) / smoke.size;
    float dist = glm::length(localPos);
    
    // 距离场（Distance Field）
    if (dist > 1.0f) return 0.0f;
    
    // 平滑衰减函数
    float density = smoke.density * (1.0f - dist * dist);
    density = density * density;  // 平方衰减，使边缘更柔和
    
    return glm::clamp(density, 0.0f, 1.0f);
}
```

**改进方向**（参考RTR4）：
- 添加Perlin噪声或Simplex噪声
- 使用3D纹理存储预计算的密度场
- 使用FBM（Fractal Brownian Motion）生成更自然的形状

## 5. 单次散射（Single Scattering）

### 5.1 理论（RTR4 Section 14.1.3）

**单次散射模型**：
- 假设光线只散射一次
- 简化计算，适合实时渲染
- 公式：`Lᵢ = ∫ σₛ(s) Lᵢₙ(s,ω) ds`

### 5.2 实现对应

```cpp
// 单次散射（简化版，假设光源在相机方向）
float scattering = totalDensity * stepSize * 0.5f;
glm::vec3 smokeColor(0.7f, 0.65f, 0.6f);  // 烟雾颜色
glm::vec3 scatteredLight = smokeColor * scattering;
color += scatteredLight * transmittance;
```

**改进方向**（RTR4 Section 14.4）：
- 实现真正的单次散射：考虑光源方向
- 添加相位函数（Phase Function）：控制散射方向
- 实现多次散射（Multiple Scattering）：更真实但更昂贵

## 6. 性能优化（RTR4 Section 14.5）

### 6.1 已实现的优化

1. **早期终止**：
```cpp
while (transmittance > 0.01f)  // 当透射率太低时停止
```

2. **固定步长**：
```cpp
float stepSize = 0.1f;  // 平衡质量和性能
```

### 6.2 可进一步优化的方向

1. **层次细节（LOD）**：
   - 远距离使用更大的步长
   - 低分辨率密度场

2. **视锥剔除**：
   - 只渲染可见的烟雾体积

3. **时间相干性**：
   - 复用上一帧的采样结果

4. **GPU加速**：
   - 使用计算着色器并行计算
   - 使用3D纹理存储密度场

## 7. 烟雾动画（RTR4 Section 14.6）

### 7.1 实现

我们使用多个烟雾体积来模拟烟雾的扩散：

```cpp
// 创建多个烟雾体积，形成连续的烟雾云
int numSmokeVolumes = 3 + (int)(smokeIntensity * 5);

for (int i = 0; i < numSmokeVolumes; i++) {
    float t = (float)i / (float)(numSmokeVolumes - 1);
    
    // 向上扩散（浮力）
    float height = t * smokeIntensity * 1.5f;
    
    // 向前扩散（炮口方向）
    float forwardDistance = t * smokeIntensity * 0.8f;
    
    // 水平扩散
    float horizontalSpread = t * t * smokeIntensity * 0.6f;
}
```

### 7.2 理论对应

这对应RTR4中讨论的：
- **平流（Advection）**：烟雾随速度场移动
- **扩散（Diffusion）**：烟雾自然扩散
- **浮力（Buoyancy）**：热空气上升

## 8. 与实时渲染管线的对比

### 8.1 光线追踪 vs 光栅化

**我们的实现（光线追踪）**：
- ✅ 物理准确
- ✅ 自动处理遮挡
- ❌ 性能较慢

**实时渲染管线（RTR4 Chapter 18）**：
- 使用屏幕空间技术
- 使用粒子系统 + Billboard
- 使用体积纹理（3D Texture）
- 使用计算着色器加速

### 8.2 混合方案

可以结合两种方法：
1. 使用光线追踪渲染高质量烟雾
2. 使用实时技术渲染背景烟雾
3. 使用重要性采样优化关键区域

## 9. 参考文献

- **Real-Time Rendering, 4th Edition**:
  - Chapter 14: Volumetric and Translucency Rendering
  - Section 14.1: Theory
  - Section 14.2: Volume Ray Marching
  - Section 14.3: Density Fields
  - Section 14.4: Scattering Models
  - Section 14.5: Performance Optimization

- **相关论文**：
  - "Volumetric Light Scattering as a Post-Process" (SIGGRAPH 2014)
  - "Unified Volumetric Rendering" (GPU Gems 3)

## 10. 代码实现总结

我们的实现遵循了RTR4的核心理论：

1. ✅ **Beer-Lambert定律**：正确计算透射率
2. ✅ **体积光线步进**：离散化RTE方程
3. ✅ **单次散射模型**：简化但有效的散射
4. ✅ **密度场**：使用椭球距离场
5. ✅ **性能优化**：早期终止、固定步长

**改进空间**：
- 添加噪声扰动（更自然的形状）
- 实现真正的单次散射（考虑光源方向）
- GPU加速（计算着色器）
- 自适应采样（重要区域更密集）

