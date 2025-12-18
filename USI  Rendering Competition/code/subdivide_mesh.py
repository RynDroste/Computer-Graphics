#!/usr/bin/env python3
"""
OBJ 模型细分工具
使用 Loop 细分算法增加三角形数量
"""

import sys
import os

def subdivide_with_trimesh(input_file, output_file, iterations=1):
    """
    使用 trimesh 库进行细分
    需要安装: pip install trimesh
    """
    try:
        import trimesh
        
        # 加载模型
        mesh = trimesh.load(input_file)
        
        if not isinstance(mesh, trimesh.Trimesh):
            print(f"错误: {input_file} 不是有效的三角网格")
            return False
        
        original_faces = len(mesh.faces)
        print(f"原始面数: {original_faces}")
        
        # 进行细分
        for i in range(iterations):
            mesh = mesh.subdivide()
            print(f"细分迭代 {i+1} 后面数: {len(mesh.faces)}")
        
        # 保存结果
        mesh.export(output_file)
        print(f"已保存到: {output_file}")
        print(f"最终面数: {len(mesh.faces)} (增加了 {len(mesh.faces) / original_faces:.2f}x)")
        
        return True
        
    except ImportError:
        print("错误: 需要安装 trimesh 库")
        print("安装命令: pip install trimesh")
        return False
    except Exception as e:
        print(f"错误: {e}")
        return False

def subdivide_simple(input_file, output_file, iterations=1):
    """
    简单的细分方法：将每个三角形分成4个
    不需要额外库，但效果不如 Loop 细分
    """
    try:
        vertices = []
        faces = []
        
        # 读取 OBJ 文件
        with open(input_file, 'r') as f:
            for line in f:
                line = line.strip()
                if line.startswith('v '):
                    # 顶点
                    parts = line.split()
                    if len(parts) >= 4:
                        vertices.append([float(parts[1]), float(parts[2]), float(parts[3])])
                elif line.startswith('f '):
                    # 面（可能是三角形或四边形）
                    parts = line.split()[1:]
                    if len(parts) >= 3:
                        # 提取顶点索引（处理可能的纹理/法线索引，格式：v/vt/vn）
                        face_indices = []
                        for part in parts:
                            # 提取第一个数字（顶点索引）
                            vertex_idx_str = part.split('/')[0]
                            try:
                                idx = int(vertex_idx_str) - 1  # OBJ 索引从1开始
                                face_indices.append(idx)
                            except ValueError:
                                continue
                        
                        # 如果是三角形，直接添加
                        if len(face_indices) == 3:
                            faces.append(face_indices)
                        # 如果是四边形，分成两个三角形
                        elif len(face_indices) == 4:
                            faces.append([face_indices[0], face_indices[1], face_indices[2]])
                            faces.append([face_indices[0], face_indices[2], face_indices[3]])
                        # 如果是多边形，进行三角剖分（fan triangulation）
                        elif len(face_indices) > 4:
                            for i in range(1, len(face_indices) - 1):
                                faces.append([face_indices[0], face_indices[i], face_indices[i+1]])
        
        print(f"原始顶点数: {len(vertices)}")
        print(f"原始面数: {len(faces)}")
        original_face_count = len(faces)
        
        # 细分：将每个三角形分成4个
        for iteration in range(iterations):
            new_vertices = list(vertices)
            new_faces = []
            edge_midpoints = {}  # 缓存边的中点
            
            def get_midpoint(v1_idx, v2_idx):
                """获取两个顶点的中点，并缓存"""
                key = tuple(sorted([v1_idx, v2_idx]))
                if key not in edge_midpoints:
                    v1 = vertices[v1_idx]
                    v2 = vertices[v2_idx]
                    mid = [(v1[i] + v2[i]) / 2.0 for i in range(3)]
                    edge_midpoints[key] = len(new_vertices)
                    new_vertices.append(mid)
                return edge_midpoints[key]
            
            for face in faces:
                if len(face) == 3:
                    v0, v1, v2 = face
                    # 计算每条边的中点
                    m01 = get_midpoint(v0, v1)
                    m12 = get_midpoint(v1, v2)
                    m20 = get_midpoint(v2, v0)
                    
                    # 创建4个新三角形
                    new_faces.append([v0, m01, m20])
                    new_faces.append([v1, m12, m01])
                    new_faces.append([v2, m20, m12])
                    new_faces.append([m01, m12, m20])
            
            vertices = new_vertices
            faces = new_faces
            print(f"细分迭代 {iteration+1} 后: 顶点数={len(vertices)}, 面数={len(faces)}")
        
        # 写入 OBJ 文件
        with open(output_file, 'w') as f:
            f.write("# Subdivided mesh\n")
            for v in vertices:
                f.write(f"v {v[0]:.6f} {v[1]:.6f} {v[2]:.6f}\n")
            for face in faces:
                f.write(f"f {face[0]+1} {face[1]+1} {face[2]+1}\n")
        
        print(f"已保存到: {output_file}")
        print(f"最终面数: {len(faces)} (增加了 {len(faces) / original_face_count if original_face_count > 0 else 1:.2f}x)")
        
        return True
        
    except Exception as e:
        print(f"错误: {e}")
        import traceback
        traceback.print_exc()
        return False

def main():
    if len(sys.argv) < 3:
        print("用法: python3 subdivide_mesh.py <输入文件> <输出文件> [迭代次数]")
        print("示例: python3 subdivide_mesh.py meshes/barrel.obj meshes/barrel_high.obj 2")
        sys.exit(1)
    
    input_file = sys.argv[1]
    output_file = sys.argv[2]
    iterations = int(sys.argv[3]) if len(sys.argv) > 3 else 1
    
    if not os.path.exists(input_file):
        print(f"错误: 文件不存在: {input_file}")
        sys.exit(1)
    
    print(f"输入文件: {input_file}")
    print(f"输出文件: {output_file}")
    print(f"迭代次数: {iterations}")
    print()
    
    # 先尝试使用 trimesh（如果可用）
    if subdivide_with_trimesh(input_file, output_file, iterations):
        return
    
    # 否则使用简单方法
    print("\n使用简单细分方法...")
    subdivide_simple(input_file, output_file, iterations)

if __name__ == "__main__":
    main()

