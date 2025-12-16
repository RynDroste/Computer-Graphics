# 实时渲染中实现开炮烟雾效果的方法

## 1. 体积渲染（Volumetric Rendering）

### 原理
在光线路径上对烟雾密度进行采样，使用体积光线步进（Volume Ray Marching）计算透射和散射。

### 在光线追踪器中的实现

```cpp
// 体积烟雾结构
struct VolumeSmoke {
    glm::vec3 position;      // 烟雾中心位置
    glm::vec3 size;          // 烟雾体积大小
    float density;           // 密度
    float falloff;           // 衰减系数
};

// 计算烟雾密度函数（使用噪声或距离场）
float getSmokeDensity(glm::vec3 pos, const VolumeSmoke& smoke) {
    glm::vec3 localPos = (pos - smoke.position) / smoke.size;
    float dist = glm::length(localPos);
    
    // 使用距离场或Perlin噪声
    if (dist > 1.0f) return 0.0f;
    
    // 密度衰减：中心密度高，边缘密度低
    float density = smoke.density * (1.0f - dist);
    
    // 添加噪声扰动（可选）
    // density *= perlinNoise(pos * 0.5f);
    
    return density;
}

// 体积光线步进
glm::vec3 traceVolumeRay(Ray ray, const std::vector<VolumeSmoke>& smokes) {
    float stepSize = 0.05f;  // 步进大小
    float maxDistance = 20.0f;
    
    glm::vec3 color(0.0f);
    float transmittance = 1.0f;  // 透射率
    
    float t = 0.0f;
    while (t < maxDistance) {
        glm::vec3 pos = ray.origin + ray.direction * t;
        
        // 累积所有烟雾源的密度
        float totalDensity = 0.0f;
        for (const auto& smoke : smokes) {
            totalDensity += getSmokeDensity(pos, smoke);
        }
        
        if (totalDensity > 0.0f) {
            // 计算吸收和散射
            float absorption = totalDensity * stepSize;
            float scattering = totalDensity * stepSize * 0.3f;  // 散射系数
            
            // 更新透射率（Beer-Lambert定律）
            transmittance *= exp(-absorption);
            
            // 单次散射（简化版）
            glm::vec3 scatteredLight = glm::vec3(0.8f, 0.7f, 0.6f) * scattering;
            color += scatteredLight * transmittance;
        }
        
        t += stepSize;
    }
    
    return color;
}

// 修改trace_ray函数以支持体积渲染
glm::vec3 trace_ray_with_volume(Ray ray, const std::vector<VolumeSmoke>& smokes) {
    // 先进行常规光线追踪
    Hit closest_hit = trace_ray(ray);
    
    glm::vec3 color(0.0f);
    
    if (closest_hit.hit) {
        // 计算体积效果（从相机到物体）
        glm::vec3 volumeColor = traceVolumeRay(ray, smokes);
        
        // 物体颜色受体积透射影响
        glm::vec3 objectColor = PhongModel(...);
        color = volumeColor + objectColor * transmittance;
    } else {
        // 背景也受体积影响
        color = traceVolumeRay(ray, smokes);
    }
    
    return color;
}
```

## 2. 粒子系统（Particle Systems）

### 原理
使用大量小粒子（通常用billboard或球体表示）模拟烟雾。

### 实现步骤

```cpp
// 粒子结构
struct SmokeParticle {
    glm::vec3 position;
    glm::vec3 velocity;
    float size;
    float lifetime;
    float age;
    float alpha;
};

// 粒子系统
class SmokeParticleSystem {
private:
    std::vector<SmokeParticle> particles;
    glm::vec3 emitterPosition;
    
public:
    void update(float deltaTime) {
        for (auto& p : particles) {
            // 更新位置
            p.position += p.velocity * deltaTime;
            
            // 更新年龄
            p.age += deltaTime;
            
            // 速度衰减（空气阻力）
            p.velocity *= 0.98f;
            
            // 添加浮力（向上）
            p.velocity.y += 0.5f * deltaTime;
            
            // 更新大小（扩散）
            p.size += 0.1f * deltaTime;
            
            // 更新透明度
            p.alpha = 1.0f - (p.age / p.lifetime);
            
            // 移除死亡粒子
            if (p.age >= p.lifetime) {
                // 重新初始化或移除
            }
        }
    }
    
    void emit(int count, glm::vec3 position, glm::vec3 direction) {
        for (int i = 0; i < count; i++) {
            SmokeParticle p;
            p.position = position;
            p.velocity = direction + randomOffset();
            p.size = 0.1f;
            p.lifetime = 2.0f + random() * 1.0f;
            p.age = 0.0f;
            p.alpha = 1.0f;
            particles.push_back(p);
        }
    }
};
```

### 在实时渲染管线中的渲染

**OpenGL实现：**
```glsl
// Vertex Shader
#version 330 core
layout (location = 0) in vec3 position;
layout (location = 1) in float size;
layout (location = 2) in float alpha;

uniform mat4 view;
uniform mat4 projection;
uniform vec3 cameraRight;
uniform vec3 cameraUp;

out float fragAlpha;

void main() {
    // Billboard技术：始终面向相机
    vec3 pos = position + cameraRight * size + cameraUp * size;
    gl_Position = projection * view * vec4(pos, 1.0);
    fragAlpha = alpha;
}

// Fragment Shader
#version 330 core
in float fragAlpha;
out vec4 FragColor;

uniform sampler2D smokeTexture;  // 烟雾纹理（圆形渐变）

void main() {
    vec2 uv = gl_PointCoord;
    vec4 texColor = texture(smokeTexture, uv);
    
    // 边缘衰减
    float dist = length(uv - 0.5);
    float alpha = texColor.a * fragAlpha * (1.0 - smoothstep(0.3, 0.5, dist));
    
    FragColor = vec4(texColor.rgb, alpha);
}
```

## 3. 基于噪声的体积（Noise-based Volumes）

### 使用3D噪声纹理

```cpp
// 使用Perlin噪声或Simplex噪声生成体积密度
float sampleNoise3D(glm::vec3 pos) {
    // 多层噪声叠加
    float noise = 0.0f;
    float amplitude = 1.0f;
    float frequency = 0.5f;
    
    for (int i = 0; i < 4; i++) {
        noise += perlinNoise3D(pos * frequency) * amplitude;
        amplitude *= 0.5f;
        frequency *= 2.0f;
    }
    
    return noise;
}

// 在体积渲染中使用
float getSmokeDensity(glm::vec3 pos, glm::vec3 smokeCenter) {
    glm::vec3 localPos = (pos - smokeCenter) * 0.5f;
    
    // 基础形状（球体或椭球）
    float dist = length(localPos);
    float baseDensity = max(0.0f, 1.0f - dist);
    
    // 添加噪声扰动
    float noise = sampleNoise3D(localPos * 2.0f);
    noise = (noise + 1.0f) * 0.5f;  // 归一化到[0,1]
    
    return baseDensity * noise;
}
```

## 4. 屏幕空间体积光（Screen-Space Volumetric Lighting）

### 原理
在屏幕空间计算体积光效果，适合实时渲染。

```glsl
// Fragment Shader - 屏幕空间体积光
uniform sampler2D depthTexture;
uniform vec3 lightPosition;
uniform vec3 lightColor;
uniform float lightIntensity;

vec3 calculateVolumetricLight(vec2 screenUV, vec3 viewPos) {
    vec3 lightDir = normalize(lightPosition - viewPos);
    
    // 光线步进
    float stepSize = 0.1f;
    int numSteps = 32;
    
    vec3 color = vec3(0.0);
    float transmittance = 1.0;
    
    for (int i = 0; i < numSteps; i++) {
        vec3 samplePos = viewPos + lightDir * (stepSize * float(i));
        
        // 采样深度
        float depth = texture(depthTexture, screenUV).r;
        
        // 计算密度（简化）
        float density = calculateDensity(samplePos);
        
        // 散射
        float scattering = density * stepSize;
        color += lightColor * scattering * transmittance;
        
        // 透射
        transmittance *= exp(-density * stepSize);
    }
    
    return color;
}
```

## 5. 预计算的烟雾动画（Precomputed Smoke Animation）

### 使用纹理序列

```cpp
// 加载预渲染的烟雾帧序列
class SmokeAnimation {
private:
    std::vector<GLuint> textureFrames;
    int currentFrame;
    float frameTime;
    
public:
    void loadFrames(const std::vector<std::string>& framePaths) {
        for (const auto& path : framePaths) {
            GLuint tex = loadTexture(path);
            textureFrames.push_back(tex);
        }
    }
    
    GLuint getCurrentFrame(float time) {
        currentFrame = (int)(time / frameTime) % textureFrames.size();
        return textureFrames[currentFrame];
    }
};
```

## 6. 性能优化技巧

1. **LOD（细节层次）**：远距离使用低分辨率粒子
2. **视锥剔除**：只渲染可见的烟雾
3. **遮挡剔除**：被遮挡的烟雾不渲染
4. **GPU加速**：使用计算着色器更新粒子
5. **实例化渲染**：批量渲染相同类型的粒子

## 7. 在您的光线追踪器中实现

对于您的光线追踪器，推荐使用**体积光线步进**方法：

1. 在`trace_ray`函数中添加体积采样
2. 定义烟雾区域（从炮口位置开始）
3. 使用距离场或噪声函数计算密度
4. 应用Beer-Lambert定律计算透射和散射

这样可以保持光线追踪的一致性，同时实现真实的烟雾效果。

