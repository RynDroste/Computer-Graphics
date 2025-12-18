# OBJ 模型细分工具使用说明

## 功能
使用细分算法增加 OBJ 模型的面数（三角形数），每次迭代会将每个三角形分成 4 个，面数增加 4 倍。

## 使用方法

### 基本用法
```bash
python3 subdivide_mesh.py <输入文件> <输出文件> [迭代次数]
```

### 示例

1. **单次细分**（面数增加 4 倍）：
```bash
python3 subdivide_mesh.py meshes/barrel.obj meshes/barrel_high.obj 1
```

2. **两次细分**（面数增加 16 倍）：
```bash
python3 subdivide_mesh.py meshes/barrel.obj meshes/barrel_very_high.obj 2
```

3. **细分炮塔模型**：
```bash
python3 subdivide_mesh.py meshes/turret.obj meshes/turret_high.obj 1
```

## 细分效果

- **1 次迭代**: 面数 × 4
- **2 次迭代**: 面数 × 16
- **3 次迭代**: 面数 × 64

## 当前模型面数

- `meshes/barrel.obj`: 1638 个三角形
- `meshes/turret.obj`: 7824 个三角形

## 细分后预期面数

### barrel.obj
- 1 次迭代: ~6,552 个三角形
- 2 次迭代: ~26,208 个三角形
- 3 次迭代: ~104,832 个三角形

### turret.obj
- 1 次迭代: ~31,296 个三角形
- 2 次迭代: ~500,736 个三角形（非常大！）

## 高级选项

### 使用 trimesh 库（更好的细分质量）

如果需要更高质量的细分（Loop 细分算法），可以安装 trimesh：

```bash
pip install trimesh
```

安装后，脚本会自动使用 trimesh 进行细分，效果更好。

## 注意事项

1. **性能影响**: 面数增加会显著影响渲染性能，建议根据实际需求选择迭代次数
2. **文件大小**: 细分后的文件会变大，注意磁盘空间
3. **内存使用**: 高面数模型会占用更多内存

## 在代码中使用细分后的模型

修改 `main.cpp` 中的文件路径：

```cpp
// 原来
loadOBJ("./meshes/barrel.obj", verts, faces);

// 改为
loadOBJ("./meshes/barrel_high.obj", verts, faces);
```

