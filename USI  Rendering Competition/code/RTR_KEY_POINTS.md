# Real-Time Rendering 关键知识点总结

## 核心概念映射

### 1. 参与介质（Participating Media） - RTR4 Chapter 14.1

**理论**：烟雾、雾、云等介质会影响光线传播

**实现**：
```cpp
struct SmokeVolume {
    glm::vec3 position;  // 介质位置
    float density;      // 介质密度
};
```

### 2. Beer-Lambert 定律 - RTR4 Section 14.1.2

**公式**：`T(d) = exp(-τ(d))`，其中 `τ(d) = ∫ σₐ(s) ds`

**实现**：
```cpp
// 计算光学深度
float totalDensity = 0.0f;
for (int i = 0; i < steps; i++) {
    totalDensity += getSmokeDensity(pos, smoke) * stepSize;
}

// Beer-Lambert定律
float transmittance = exp(-totalDensity * 2.0f);
```

### 3. 辐射传输方程（RTE） - RTR4 Section 14.1.3

**理论**：`L(x,ω) = T(x₀,x) L(x₀,ω) + ∫ T(s,x) σₛ(s) Lᵢ(s,ω) ds`

**实现**：`traceVolumeRay()` 函数中的光线步进循环

### 4. 体积光线步进（Volume Ray Marching） - RTR4 Section 14.2.1

**算法**：
1. 沿光线等间距采样
2. 计算每点的密度
3. 累积透射和散射

**实现**：
```cpp
for (int i = 0; i < maxSteps; i++) {
    glm::vec3 pos = ray.origin + ray.direction * t;
    float density = getSmokeDensity(pos, smoke);
    
    // 更新透射率
    transmittance *= exp(-absorption);
    
    // 累积散射
    color += scatteredLight * transmittance;
}
```

### 5. 单次散射（Single Scattering） - RTR4 Section 14.1.3

**理论**：假设光线只散射一次（简化模型）

**实现**：
```cpp
float scattering = totalDensity * stepSize * 0.5f;
glm::vec3 scatteredLight = smokeColor * scattering;
color += scatteredLight * transmittance;
```

### 6. 密度场（Density Field） - RTR4 Section 14.3

**方法**：
- 解析函数（椭球、球体）✅ 我们使用
- 3D纹理
- 程序化噪声

**实现**：
```cpp
float getSmokeDensity(glm::vec3 pos, const SmokeVolume& smoke) {
    glm::vec3 localPos = (pos - smoke.position) / smoke.size;
    float dist = glm::length(localPos);
    return smoke.density * (1.0f - dist * dist);
}
```

## 性能优化技巧（RTR4 Section 14.5）

### 已实现：
1. ✅ **早期终止**：`transmittance > 0.01f`
2. ✅ **固定步长**：平衡质量和性能

### 可改进：
1. **自适应采样**：重要区域更密集
2. **层次细节（LOD）**：远距离使用更大步长
3. **GPU加速**：使用计算着色器
4. **时间相干性**：复用上一帧结果

## 与实时渲染的对比

| 特性 | 我们的实现（光线追踪） | 实时渲染管线 |
|------|---------------------|------------|
| 物理准确性 | ✅ 高 | ⚠️ 中等 |
| 性能 | ❌ 较慢 | ✅ 快 |
| 自动遮挡 | ✅ 是 | ⚠️ 需要额外处理 |
| 实现复杂度 | ⚠️ 中等 | ⚠️ 中等 |

## 关键公式速查

1. **透射率**：`T = exp(-τ)`
2. **光学深度**：`τ = ∫ σₐ(s) ds`
3. **单次散射**：`Lᵢ = ∫ σₛ(s) Lᵢₙ(s,ω) ds`
4. **密度衰减**：`density = max_density × (1 - dist²)`

## 学习路径

1. **基础**：理解参与介质和Beer-Lambert定律
2. **进阶**：掌握体积光线步进算法
3. **高级**：实现多次散射和相位函数
4. **优化**：GPU加速和自适应采样

## 推荐阅读顺序

1. RTR4 Chapter 14.1：理论基础
2. RTR4 Chapter 14.2：体积光线步进
3. RTR4 Chapter 14.3：密度场
4. RTR4 Chapter 14.5：性能优化

