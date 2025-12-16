/**
@file main.cpp
*/

#include <iostream>
#include <fstream>
#include <cmath>
#include <ctime>
#include <vector>
#include <string>
#include <sstream>
#include <limits>
#include "glm/glm.hpp"
#include "glm/gtx/transform.hpp"

#include "Image.h"
#include "Material.h"
#include "bvh.h"

using namespace std;

// Ray class is now defined in bvh.h
// Using the definition from bvh.h

class Object;

/**
 Structure representing the even of hitting an object
 */
struct Hit{
    bool hit; ///< Boolean indicating whether there was or there was no intersection with an object
    glm::vec3 normal; ///< Normal vector of the intersected object at the intersection point
    glm::vec3 intersection; ///< Point of Intersection
    float distance; ///< Distance from the origin of the ray to the intersection point
    Object *object; ///< A pointer to the intersected object
};

/**
 General class for the object
 */
class Object{
	
protected:
	glm::mat4 transformationMatrix; ///< Matrix representing the transformation from the local to the global coordinate system
	glm::mat4 inverseTransformationMatrix; ///< Matrix representing the transformation from the global to the local coordinate system
	glm::mat4 normalMatrix; ///< Matrix for transforming normal vectors from the local to the global coordinate system
	
public:
	glm::vec3 color; ///< Color of the object
	Material material; ///< Structure describing the material of the object
	/** A function computing an intersection, which returns the structure Hit */
    virtual Hit intersect(Ray ray) = 0;

	/** Function that returns the material struct of the object*/
	Material getMaterial(){
		return material;
	}
	/** Function that set the material
	 @param material A structure describing the material of the object
	*/
	void setMaterial(Material material){
		this->material = material;
	}
	/** Functions for setting up all the transformation matrices
	@param matrix The matrix representing the transformation of the object in the global coordinates */
	void setTransformation(glm::mat4 matrix){
			
		transformationMatrix = matrix;

		inverseTransformationMatrix = glm::inverse(matrix);
		normalMatrix = glm::transpose(inverseTransformationMatrix);
	}
};
 

class Plane : public Object{

private:
	glm::vec3 normal;
	glm::vec3 point;

public:
	Plane(glm::vec3 point, glm::vec3 normal) : point(point), normal(normal){
	}
	Plane(glm::vec3 point, glm::vec3 normal, Material material) : point(point), normal(normal){
		this->material = material;
	}
	Hit intersect(Ray ray){
		
		Hit hit;
		hit.hit = false;
		
        float DdotN = glm::dot(ray.direction, normal);
        if(DdotN < 0){
            
            float PdotN = glm::dot (point-ray.origin, normal);
            float t = PdotN/DdotN;
            
            if(t > 0){
                hit.hit = true;
                hit.normal = normal;
                hit.distance = t;
                hit.object = this;
                hit.intersection = t * ray.direction + ray.origin;
            }
        }
		
		return hit;
	}
};



class Triangle : public Object{

private:
    glm::vec3 v0;
    glm::vec3 v1;
    glm::vec3 v2;
    glm::vec3 faceNormal; 

public:
    Triangle(const glm::vec3 &a, const glm::vec3 &b, const glm::vec3 &c){
        v0 = a; v1 = b; v2 = c;
        faceNormal = glm::normalize(glm::cross(v1 - v0, v2 - v0));
    }
    Triangle(const glm::vec3 &a, const glm::vec3 &b, const glm::vec3 &c, const Material &mat){
        v0 = a; v1 = b; v2 = c;
        faceNormal = glm::normalize(glm::cross(v1 - v0, v2 - v0));
        material = mat;
    }

    Hit intersect(Ray ray){
        Hit hit; hit.hit = false;
        glm::vec3 edge1 = v1 - v0;
        glm::vec3 edge2 = v2 - v0;
        glm::vec3 pvec = glm::cross(ray.direction, edge2);
        float det = glm::dot(edge1, pvec);
        if (det == 0.0f) return hit;
        float invDet = 1.0f / det;
        glm::vec3 tvec = ray.origin - v0;
        float u = glm::dot(tvec, pvec) * invDet;
        if (u < 0.0f || u > 1.0f) return hit;
        glm::vec3 qvec = glm::cross(tvec, edge1);
        float v = glm::dot(ray.direction, qvec) * invDet;
        if (v < 0.0f || u + v > 1.0f) return hit;
        float t = glm::dot(edge2, qvec) * invDet;
        if (t <= 0.0f || t > ray.tMax) return hit;

        hit.hit = true;
        hit.distance = t;
        hit.intersection = ray.origin + t * ray.direction;
        hit.normal = faceNormal;
        hit.object = this;
        return hit;
    }
    
    // Get bounding box for BVH
    Bounds3 WorldBound() const {
        Bounds3 b(v0);
        b = b.Union(v1);
        b = b.Union(v2);
        return b;
    }
};

class Cylinder : public Object {
private:
    glm::vec3 center;      // 圆柱体中心点
    glm::vec3 axis;         // 高度轴方向（已归一化）
    float radius;           // 半径
    float halfHeight;       // 半高（高度的一半）

public:
    Cylinder(const glm::vec3 &center, const glm::vec3 &axis, float radius, float height) 
        : center(center), axis(glm::normalize(axis)), radius(radius), halfHeight(height * 0.5f) {
    }
    
    Cylinder(const glm::vec3 &center, const glm::vec3 &axis, float radius, float height, const Material &mat) 
        : center(center), axis(glm::normalize(axis)), radius(radius), halfHeight(height * 0.5f) {
        material = mat;
    }

    Hit intersect(Ray ray) {
        Hit hit;
        hit.hit = false;
        
        // 将射线转换到圆柱体局部坐标系
        glm::vec3 oc = ray.origin - center;
        
        // 计算射线方向在轴上的投影
        float rayAxisDot = glm::dot(ray.direction, axis);
        float ocAxisDot = glm::dot(oc, axis);
        
        // 计算垂直于轴的平面上的分量
        glm::vec3 rayPerp = ray.direction - rayAxisDot * axis;
        glm::vec3 ocPerp = oc - ocAxisDot * axis;
        
        // 二次方程系数：a*t^2 + b*t + c = 0
        float a = glm::dot(rayPerp, rayPerp);
        float b = 2.0f * glm::dot(rayPerp, ocPerp);
        float c = glm::dot(ocPerp, ocPerp) - radius * radius;
        
        float discriminant = b * b - 4.0f * a * c;
        if (discriminant < 0.0f || a < 1e-6f) return hit;
        
        float sqrtD = sqrtf(discriminant);
        float t1 = (-b - sqrtD) / (2.0f * a);
        float t2 = (-b + sqrtD) / (2.0f * a);
        
        // 检查两个交点
        float t = (t1 > 0.0f && t1 < ray.tMax) ? t1 : t2;
        if (t <= 0.0f || t > ray.tMax) return hit;
        
        glm::vec3 intersection = ray.origin + t * ray.direction;
        glm::vec3 toIntersection = intersection - center;
        float projOnAxis = glm::dot(toIntersection, axis);
        
        // 检查交点是否在圆柱体的高度范围内
        if (fabs(projOnAxis) > halfHeight) return hit;
        
        // 计算法向量（从轴到交点的方向）
        glm::vec3 normal = toIntersection - projOnAxis * axis;
        float normalLen = glm::length(normal);
        if (normalLen > 1e-6f) {
            normal = normal / normalLen;
        } else {
            // 如果交点在轴上（理论上不应该发生），使用垂直方向
            glm::vec3 up(0, 1, 0);
            if (fabs(glm::dot(axis, up)) > 0.9f) up = glm::vec3(1, 0, 0);
            normal = glm::normalize(glm::cross(axis, up));
        }
        
        hit.hit = true;
        hit.distance = t;
        hit.intersection = intersection;
        hit.normal = normal;
        hit.object = this;
        
        return hit;
    }
};

static void loadOBJ(const std::string &path,
                    std::vector<glm::vec3> &out_vertices,
                    std::vector<glm::ivec3> &out_faces){
    out_vertices.clear();
    out_faces.clear();
    std::ifstream in(path);
    std::string line;
    while(std::getline(in, line)){
        if(line.empty() || line[0] == '#') continue;
        std::istringstream iss(line);
        std::string tag; iss >> tag;
        if(tag == "v"){
            float x, y, z; iss >> x >> y >> z;
            out_vertices.push_back(glm::vec3(x,y,z));
        } else if(tag == "f"){
            std::vector<int> idx;
            std::string vertToken;
            while(iss >> vertToken){
                int vIndex = 0;
                size_t slash = vertToken.find('/');
                if(slash == std::string::npos){
                    vIndex = std::stoi(vertToken);
                }else{
                    vIndex = std::stoi(vertToken.substr(0, slash));
                }
                idx.push_back(vIndex);
            }
            if(idx.size() >= 3){
                for(size_t k=1; k+1<idx.size(); ++k){
                    out_faces.push_back(glm::ivec3(idx[0]-1, idx[k]-1, idx[k+1]-1));
                }
            }
        }
    }
}

// BVHAccel implementation
void BVHAccel::computeBounds(const std::vector<BVHPrimitiveInfo> &primitiveInfo,
                             int start, int end, Bounds3 *bounds) {
    *bounds = Bounds3();
    for (int i = start; i < end; ++i) {
        *bounds = bounds->Union(primitiveInfo[i].bounds);
    }
}

int BVHAccel::partitionPrimitives(std::vector<BVHPrimitiveInfo> &primitiveInfo,
                                  int start, int end, int dim, float split) {
    int mid = start;
    for (int i = start; i < end; ++i) {
        if (primitiveInfo[i].centroid[dim] < split) {
            std::swap(primitiveInfo[i], primitiveInfo[mid]);
            ++mid;
        }
    }
    return mid;
}

BVHBuildNode* BVHAccel::recursiveBuild(
    std::vector<BVHPrimitiveInfo> &primitiveInfo,
    int start, int end, int *totalNodes) {
    
    (*totalNodes)++;
    BVHBuildNode *node = new BVHBuildNode();
    
    // Compute bounds of all primitives
    Bounds3 bounds;
    computeBounds(primitiveInfo, start, end, &bounds);
    
    int nPrimitives = end - start;
    if (nPrimitives == 1) {
        // Create leaf node
        int firstPrimOffset = (int)primitiveInfo[start].primitiveNumber;
        node->InitLeaf(firstPrimOffset, nPrimitives, bounds);
        return node;
    } else {
        // Compute bound of primitive centroids
        Bounds3 centroidBounds;
        for (int i = start; i < end; ++i) {
            centroidBounds = centroidBounds.Union(primitiveInfo[i].centroid);
        }
        
        int dim = centroidBounds.MaximumExtent();
        
        // Partition primitives into two sets
        int mid = (start + end) / 2;
        
        if (centroidBounds.pMax[dim] == centroidBounds.pMin[dim]) {
            // All primitives have same centroid - create leaf
            int firstPrimOffset = (int)primitiveInfo[start].primitiveNumber;
            node->InitLeaf(firstPrimOffset, nPrimitives, bounds);
            return node;
        }
        
        // Partition primitives based on middle split (HLBVH simplified)
        float pmid = (centroidBounds.pMin[dim] + centroidBounds.pMax[dim]) / 2;
        mid = partitionPrimitives(primitiveInfo, start, end, dim, pmid);
        
        if (mid == start || mid == end) {
            mid = (start + end) / 2;
        }
        
        // Recursively build children
        node->InitInterior(dim,
            recursiveBuild(primitiveInfo, start, mid, totalNodes),
            recursiveBuild(primitiveInfo, mid, end, totalNodes));
    }
    
    return node;
}

BVHBuildNode* BVHAccel::HLBVHBuild(
    std::vector<BVHPrimitiveInfo> &primitiveInfo,
    int *totalNodes) {
    
    // HLBVH simplified: use recursive build with middle split
    return recursiveBuild(primitiveInfo, 0, (int)primitiveInfo.size(), totalNodes);
}

int BVHAccel::flattenBVHTree(BVHBuildNode *node, int *offset) {
    LinearBVHNode *linearNode = &nodes[*offset];
    linearNode->bounds = node->bounds;
    int myOffset = (*offset)++;
    
    if (node->nPrimitives > 0) {
        // Leaf node
        linearNode->primitivesOffset = node->firstPrimOffset;
        linearNode->nPrimitives = node->nPrimitives;
    } else {
        // Interior node
        linearNode->axis = node->splitAxis;
        linearNode->nPrimitives = 0;
        flattenBVHTree(node->children[0], offset);
        linearNode->secondChildOffset = flattenBVHTree(node->children[1], offset);
    }
    
    delete node; // Clean up build node
    return myOffset;
}

BVHAccel::BVHAccel(std::vector<Triangle*> &prims, int maxPrimsInNode)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), primitives(prims), nodes(nullptr), totalNodes(0) {
    
    if (primitives.empty()) return;
    
    // Initialize primitive info
    std::vector<BVHPrimitiveInfo> primitiveInfo(primitives.size());
    for (size_t i = 0; i < primitives.size(); ++i) {
        primitiveInfo[i] = BVHPrimitiveInfo(i, primitives[i]->WorldBound());
    }
    
    // Build BVH tree using HLBVH
    BVHBuildNode *root;
    root = HLBVHBuild(primitiveInfo, &totalNodes);
    
    // Flatten tree
    nodes = new LinearBVHNode[totalNodes];
    int offset = 0;
    flattenBVHTree(root, &offset);
}

BVHAccel::~BVHAccel() {
    delete[] nodes;
}

Hit BVHAccel::Intersect(const Ray &ray) const {
    Hit hit;
    hit.hit = false;
    hit.distance = std::numeric_limits<float>::max();
    
    if (!nodes) return hit;
    
    glm::vec3 invDir(1.0f / ray.direction.x, 1.0f / ray.direction.y, 1.0f / ray.direction.z);
    int dirIsNeg[3] = { invDir.x < 0, invDir.y < 0, invDir.z < 0 };
    
    int toVisitOffset = 0, currentNodeIndex = 0;
    int nodesToVisit[64];
    
    while (true) {
        const LinearBVHNode *node = &nodes[currentNodeIndex];
        
        if (node->bounds.IntersectP(ray, invDir, dirIsNeg)) {
            if (node->nPrimitives > 0) {
                // Leaf node - intersect with primitives
                for (int i = 0; i < node->nPrimitives; ++i) {
                    Hit h = primitives[node->primitivesOffset + i]->intersect(ray);
                    if (h.hit && h.distance < hit.distance) {
                        hit = h;
                        ray.tMax = h.distance; // Update ray tMax for early termination
                    }
                }
                
                if (toVisitOffset == 0) break;
                currentNodeIndex = nodesToVisit[--toVisitOffset];
            } else {
                // Interior node
                if (dirIsNeg[node->axis]) {
                    nodesToVisit[toVisitOffset++] = currentNodeIndex + 1;
                    currentNodeIndex = node->secondChildOffset;
                } else {
                    nodesToVisit[toVisitOffset++] = node->secondChildOffset;
                    currentNodeIndex = currentNodeIndex + 1;
                }
            }
        } else {
            if (toVisitOffset == 0) break;
            currentNodeIndex = nodesToVisit[--toVisitOffset];
        }
    }
    
    return hit;
}


/** Mesh object that holds triangles parsed from OBJ */
class Mesh : public Object{

private:
    std::vector<Triangle> triangles;
    BVHAccel *bvh;
    bool useBVH;

public:
    Mesh(const std::vector<glm::vec3> &vertices,
         const std::vector<glm::ivec3> &faces,
         const Material &mat,
         const glm::vec3 &translate,
         bool useBVHAccel = true){
        material = mat;
        useBVH = useBVHAccel;
        bvh = nullptr;
        triangles.reserve(faces.size());
        for(const auto &f : faces){
            glm::vec3 a = vertices[f.x] + translate;
            glm::vec3 b = vertices[f.y] + translate;
            glm::vec3 c = vertices[f.z] + translate;
            triangles.emplace_back(a,b,c, material);
        }
        
        // Build BVH if enabled and we have triangles
        if (useBVH && !triangles.empty()) {
            std::vector<Triangle*> triPtrs;
            triPtrs.reserve(triangles.size());
            for (auto &tri : triangles) {
                triPtrs.push_back(&tri);
            }
            bvh = new BVHAccel(triPtrs, 1);
        }
    }
    
    ~Mesh() {
        if (bvh) delete bvh;
    }

    Hit intersect(Ray ray){
        Hit closest; closest.hit = false; closest.distance = INFINITY; closest.object = this;
        
        if (useBVH && bvh) {
            // Use BVH acceleration
            closest = bvh->Intersect(ray);
            if (closest.hit) {
                closest.object = this;
            }
        } else {
            // Fallback to linear search
            for(auto &tri : triangles){
                Hit h = tri.intersect(ray);
                if(h.hit && h.distance < closest.distance){
                    closest = h;
                }
            }
        }
        return closest;
    }
    
    int GetTriangleCount() const { return (int)triangles.size(); }
    int GetBVHNodeCount() const { return bvh ? bvh->GetTotalNodes() : 0; }
};

	

/**
 Light class
 */
class Light{
public:
	glm::vec3 position; ///< Position of the light source
	glm::vec3 color; ///< Color/intentisty of the light source
	Light(glm::vec3 position): position(position){
		color = glm::vec3(1.0);
	}
	Light(glm::vec3 position, glm::vec3 color): position(position), color(color){
	}
};

vector<Light *> lights; ///< A list of lights in the scene
//glm::vec3 ambient_light(0.1,0.1,0.1);
// new ambient light
glm::vec3 ambient_light(0.001,0.001,0.001);
vector<Object *> objects; ///< A list of all objects in the scene

/**
 * 烟雾体积结构
 */
struct SmokeVolume {
    glm::vec3 position;      // 烟雾中心位置
    glm::vec3 size;          // 烟雾体积大小（椭球）
    float density;           // 最大密度
    float age;               // 烟雾年龄（用于动画）
};

vector<SmokeVolume> smokeVolumes; ///< 烟雾体积列表


/**
 * 计算烟雾密度（使用椭球距离场）
 * @param pos 世界空间位置
 * @param smoke 烟雾体积
 * @return 密度值 [0, 1]
 */
float getSmokeDensity(glm::vec3 pos, const SmokeVolume& smoke) {
    // 转换到烟雾局部空间（椭球）
    glm::vec3 localPos = (pos - smoke.position) / smoke.size;
    
    // 计算到椭球中心的归一化距离
    float dist = glm::length(localPos);
    
    // 超出范围则密度为0
    if (dist > 1.0f) return 0.0f;
    
    // 密度衰减：中心密度高，边缘密度低
    // 使用平滑的衰减函数
    float density = smoke.density * (1.0f - dist * dist);
    density = density * density;  // 平方衰减，使边缘更柔和
    
    // 考虑烟雾年龄（随时间扩散）
    float ageFactor = 1.0f / (1.0f + smoke.age * 0.5f);
    density *= ageFactor;
    
    return glm::clamp(density, 0.0f, 1.0f);
}

/**
 * 体积光线步进 - 计算烟雾的透射和散射
 * @param ray 光线
 * @param maxDistance 最大采样距离
 * @return 烟雾贡献的颜色
 */
glm::vec3 traceVolumeRay(Ray ray, float maxDistance) {
    if (smokeVolumes.empty()) return glm::vec3(0.0f);
    
    float stepSize = 0.1f;  // 步进大小
    int maxSteps = (int)(maxDistance / stepSize);
    
    glm::vec3 color(0.0f);
    float transmittance = 1.0f;  // 透射率（光线穿透烟雾的程度）
    
    float t = 0.0f;
    for (int i = 0; i < maxSteps && transmittance > 0.01f; i++) {
        glm::vec3 pos = ray.origin + ray.direction * t;
        
        // 累积所有烟雾源的密度
        float totalDensity = 0.0f;
        for (const auto& smoke : smokeVolumes) {
            totalDensity += getSmokeDensity(pos, smoke);
        }
        
        if (totalDensity > 0.001f) {
            // 吸收系数（烟雾吸收光线）
            float absorption = totalDensity * stepSize * 2.0f;
            
            // 散射系数（烟雾散射光线，使烟雾可见）
            float scattering = totalDensity * stepSize * 0.5f;
            
            // 更新透射率（Beer-Lambert定律）
            transmittance *= exp(-absorption);
            
            // 单次散射（简化版，假设光源在相机方向）
            // 烟雾颜色：灰白色，略带黄色
            glm::vec3 smokeColor(0.7f, 0.65f, 0.6f);
            
            // 散射光贡献
            glm::vec3 scatteredLight = smokeColor * scattering;
            color += scatteredLight * transmittance;
        }
        
        t += stepSize;
    }
    
    return color;
}

/**
 Function to check if a point is in shadow from a light source
 Uses BVH acceleration through Mesh::intersect() which automatically uses BVH for mesh objects
 @param point The point to check
 @param lightPosition The position of the light source
 @return true if the point is in shadow, false otherwise
 */
bool isInShadow(glm::vec3 point, glm::vec3 lightPosition) {
    glm::vec3 lightDirection = lightPosition - point;
    float lightDistance = glm::length(lightDirection);
    lightDirection = glm::normalize(lightDirection);
    
    // Offset the ray origin slightly along the light direction to avoid self-intersection
    glm::vec3 offsetPoint = point + lightDirection * 0.001f;
    Ray shadowRay(offsetPoint, lightDirection);
    shadowRay.tMax = lightDistance - 0.002f; // Slightly less than distance to light
    
    // Check for intersection with any object
    // Note: This automatically uses BVH acceleration for Mesh objects,
    // as Mesh::intersect() uses BVH internally when enabled
    for(int k = 0; k < objects.size(); k++){
        Hit hit = objects[k]->intersect(shadowRay);
        // Early exit: if we find any intersection before reaching the light, point is in shadow
        if(hit.hit && hit.distance < shadowRay.tMax && hit.distance > 0.0f) {
            return true; // Point is in shadow
        }
    }
    return false; // Point is not in shadow - no objects block the path to light
}

/** Function for computing color of an object according to the Phong Model
 @param point A point belonging to the object for which the color is computer
 @param normal A normal vector the the point
 @param view_direction A normalized direction from the point to the viewer/camera
 @param material A material structure representing the material of the object
*/
glm::vec3 PhongModel(glm::vec3 point, glm::vec3 normal, glm::vec3 view_direction, Material material){

	glm::vec3 color(0.0);
	for(int light_num = 0; light_num < lights.size(); light_num++){
		
		// Check if point is in shadow from this light
		if(isInShadow(point, lights[light_num]->position)) {
			continue; // Skip this light, point is in shadow
		}

		glm::vec3 light_direction = glm::normalize(lights[light_num]->position - point);
		glm::vec3 reflected_direction = glm::reflect(-light_direction, normal);

		float NdotL = glm::clamp(glm::dot(normal, light_direction), 0.0f, 1.0f);
		float VdotR = glm::clamp(glm::dot(view_direction, reflected_direction), 0.0f, 1.0f);

		glm::vec3 diffuse_color = material.diffuse;
		glm::vec3 diffuse = diffuse_color * glm::vec3(NdotL);
		glm::vec3 specular = material.specular * glm::vec3(pow(VdotR, material.shininess));
		
        float r = glm::distance(point,lights[light_num]->position);
        r = max(r, 0.1f);
        color += lights[light_num]->color * (diffuse + specular) / r/r;
	}
	color += ambient_light * material.ambient;
	color = glm::clamp(color, glm::vec3(0.0), glm::vec3(1.0));
	return color;
}

/**
 Functions that computes a color along the ray
 @param ray Ray that should be traced through the scene
 @return Color at the intersection point
 */
glm::vec3 trace_ray(Ray ray){

	Hit closest_hit;

	closest_hit.hit = false;
	closest_hit.distance = INFINITY;

	for(int k = 0; k<objects.size(); k++){
		Hit hit = objects[k]->intersect(ray);
		if(hit.hit == true && hit.distance < closest_hit.distance)
			closest_hit = hit;
	}

	glm::vec3 color(0.0);
	
	// 计算体积烟雾效果（从相机到物体或背景）
	float volumeDistance = closest_hit.hit ? closest_hit.distance : 50.0f;
	glm::vec3 volumeColor = traceVolumeRay(ray, volumeDistance);
	
	if(closest_hit.hit){
		// 计算物体颜色
		glm::vec3 objectColor = PhongModel(closest_hit.intersection, closest_hit.normal, glm::normalize(-ray.direction), closest_hit.object->getMaterial());
		
		// 计算从相机到物体的透射率（烟雾对物体的遮挡）
		// 通过计算体积中的总密度来估算透射率
		float totalDensity = 0.0f;
		float stepSize = 0.1f;
		int steps = (int)(closest_hit.distance / stepSize);
		
		for (int i = 0; i < steps; i++) {
            float t = i * stepSize;
            glm::vec3 pos = ray.origin + ray.direction * t;
            
            for (const auto& smoke : smokeVolumes) {
                totalDensity += getSmokeDensity(pos, smoke) * stepSize;
            }
        }
        
        // Beer-Lambert定律：透射率 = exp(-吸收系数 * 距离)
        float transmittance = exp(-totalDensity * 2.0f);
		
		// 最终颜色 = 体积散射 + 物体颜色 * 透射率
		color = volumeColor + objectColor * transmittance;
	}else{
		// 背景颜色 = 体积散射
		color = volumeColor;
	}
	
	return color;
}
/**
 Function defining the scene
 @param elevationAngle 炮管抬起角度（度），0表示水平，负值表示向上抬起
 @param recoilDistance 炮管缩退距离（沿炮管反方向），用于模拟后坐力效果
 @param smokeIntensity 烟雾强度（0-1），0表示无烟雾，1表示最大烟雾
 */
void sceneDefinition (float elevationAngle = -20.0f, float recoilDistance = 0.0f, float smokeIntensity = 0.0f){
	// 清空之前的烟雾体积
	smokeVolumes.clear();
	
	Material green_diffuse;
	green_diffuse.ambient = glm::vec3(0.7f, 0.9f, 0.7f);
	green_diffuse.diffuse = glm::vec3(0.7f, 0.9f, 0.7f);

	//Material green_diffuse;
	green_diffuse.ambient = glm::vec3(0.03f, 0.1f, 0.03f);
	green_diffuse.diffuse = glm::vec3(0.3f, 1.0f, 0.3f);

	
	lights.push_back(new Light(glm::vec3(0, 26, 5), glm::vec3(1.0, 1.0, 1.0)));
	lights.push_back(new Light(glm::vec3(0, 1, 12), glm::vec3(0.1)));
	lights.push_back(new Light(glm::vec3(0, 5, 1), glm::vec3(0.4)));
	
    Material red_diffuse;
    red_diffuse.ambient = glm::vec3(0.09f, 0.06f, 0.06f);
    red_diffuse.diffuse = glm::vec3(0.9f, 0.6f, 0.6f);
        
    Material blue_diffuse;
    blue_diffuse.ambient = glm::vec3(0.06f, 0.06f, 0.09f);
    blue_diffuse.diffuse = glm::vec3(0.6f, 0.6f, 0.9f);
    // Removed floor, back wall, left wall, right wall to keep only the mesh
    Material gray_diffuse;
    gray_diffuse.ambient = glm::vec3(0.07f);
    gray_diffuse.diffuse = glm::vec3(0.7f);
    gray_diffuse.specular = glm::vec3(0.2f);
    gray_diffuse.shininess = 32.0f;

    // 现在场景中加载两个网格：barrel 和 turret
    // -------------------------
    // 公共缩放与旋转参数（可根据需要调整）
    const float meshScale = 0.4f; 
    const float angleDeg  = 135.0f;    // 围绕 Y 轴的水平旋转
    const float angleRad  = glm::radians(angleDeg);
    const float c = std::cos(angleRad);
    const float s = std::sin(angleRad);

    // ---------- 加载 barrel ----------
    {
        std::vector<glm::vec3> verts; 
        std::vector<glm::ivec3> faces;
        loadOBJ("./meshes/barrel.obj", verts, faces);

        // 计算炮管长度（沿Z轴方向，考虑缩放）
        float barrelLength = 0.0f;
        if (!verts.empty()) {
            float minZ = verts[0].z;
            float maxZ = verts[0].z;
            for (const auto &v : verts) {
                if (v.z < minZ) minZ = v.z;
                if (v.z > maxZ) maxZ = v.z;
            }
            barrelLength = (maxZ - minZ) * meshScale;  // 考虑缩放后的长度
        }

        // 炮管的基准平移（连接点附近）
        glm::vec3 T_barrel_base(-0.2f, -2.0f, 15.4f);
        // 假设炮管在本地坐标中沿 +Z 轴指向前方，则经过相同的 Y 轴旋转后，
        // 世界空间中的指向方向为 (s, 0, c)
        glm::vec3 forwardDir(s, 0.0f, c);
        forwardDir = glm::normalize(forwardDir);
        
        // 先计算旋转轴（红色圆柱体的轴心）
        // 添加圆柱体：高度轴垂直于炮管方向（横过来）
        // 炮管方向是 (s, 0, c)，在XZ平面上
        // 垂直于炮管方向且在XZ平面内的向量是 (-c, 0, s)
        glm::vec3 cylinderAxis(-c, 0.0f, s);  // 高度轴横过来，垂直于炮管方向
        cylinderAxis = glm::normalize(cylinderAxis);  // 确保归一化
        // 沿炮管方向平移 3.0f，然后向炮管反方向平移 0.3f
        // 向上平移圆柱体，使其与炮管相交（炮管在Y=-2.0f附近，圆柱体中心需要与炮管中心对齐）
        glm::vec3 cylinderCenter = T_barrel_base + forwardDir * 3.0f - forwardDir * 0.3f;
        cylinderCenter.y += 0.4f;  // 向上平移0.5f后向下0.1f，净向上0.4f
        
        // 计算旋转轴心（红色圆柱体的轴心，即炮耳位置）
        glm::vec3 rotationAxisPoint(cylinderCenter.x, T_barrel_base.y, cylinderCenter.z);
        glm::vec3 rotationAxis = cylinderAxis;  // 旋转轴方向（炮耳轴方向）
        
        // 计算绕旋转轴旋转的旋转矩阵
        // elevationAngle参数：向上抬起角度（负值表示向上）
        float angleRad = glm::radians(elevationAngle);
        
        // 使用Rodrigues旋转公式构建旋转矩阵
        glm::vec3 axis = glm::normalize(rotationAxis);
        float cosA = cosf(angleRad);
        float sinA = sinf(angleRad);
        
        // Rodrigues旋转矩阵: R = I + sin(θ)[k]× + (1-cos(θ))[k]×²
        glm::mat3 K(0.0f, -axis.z, axis.y,
                    axis.z, 0.0f, -axis.x,
                    -axis.y, axis.x, 0.0f);
        glm::mat3 K2 = K * K;
        glm::mat3 rotationMatrix = glm::mat3(1.0f) + sinA * K + (1.0f - cosA) * K2;
        
        // offset 为沿着炮管指向方向移动的距离（>0 往炮口方向，<0 往反方向）
        const float barrelOffset = 0.3f; // 你可以根据效果自行调整这个值
        // 计算炮管位置：基础位置 + 偏移 - 缩退距离（向后移动）
        glm::vec3 T_barrel = T_barrel_base + forwardDir * barrelOffset - forwardDir * recoilDistance;
        
        std::vector<glm::vec3> transformedVerts = verts;
        for (auto &v : transformedVerts) {
            // 缩放
            v *= meshScale;
            // 水平旋转
            float x = v.x;
            float z = v.z;
            v.x = c * x + s * z;
            v.z = -s * x + c * z;
            
            // 计算顶点在最终位置（相对于T_barrel）
            glm::vec3 vWorld = T_barrel + v;
            
            // 计算相对于旋转轴心的位置
            glm::vec3 vRelative = vWorld - rotationAxisPoint;
            
            // 绕旋转轴旋转
            glm::vec3 vRotated = rotationMatrix * vRelative;
            
            // 计算旋转后的世界位置
            glm::vec3 vWorldRotated = rotationAxisPoint + vRotated;
            
            // 转换回相对于T_barrel的局部坐标
            v = vWorldRotated - T_barrel;
        }
        
        if (!transformedVerts.empty() && !faces.empty()) {
            objects.push_back(new Mesh(transformedVerts, faces, gray_diffuse, T_barrel));
        }
        
        float cylinderRadius = 0.04f;  // 圆柱体半径（减小五倍）
        float cylinderHeight = 10.0f;  // 圆柱体高度
        
        Material cylinderMaterial;
        cylinderMaterial.ambient = glm::vec3(0.1f, 0.1f, 0.8f);
        cylinderMaterial.diffuse = glm::vec3(0.2f, 0.2f, 0.9f);
        cylinderMaterial.specular = glm::vec3(0.5f);
        cylinderMaterial.shininess = 64.0f;
        
        objects.push_back(new Cylinder(cylinderCenter, cylinderAxis, cylinderRadius, cylinderHeight, cylinderMaterial));
        
        // 添加烟雾效果（在炮口位置）
        if (smokeIntensity > 0.0f) {
            // 计算旋转后的炮管方向
            glm::vec3 barrelDirection = forwardDir;
            glm::vec3 barrelDirRotated = rotationMatrix * barrelDirection;
            glm::vec3 rotatedBarrelDirection = glm::normalize(barrelDirRotated);
            
            // 计算炮口位置：从炮管模型的前端（maxZ）计算
            // 假设炮管模型沿+Z轴方向，炮口在Z的最大值处
            float maxZ = 0.0f;
            if (!verts.empty()) {
                for (const auto &v : verts) {
                    if (v.z > maxZ) maxZ = v.z;
                }
            }
            
            // 炮口在模型局部坐标系中的位置（相对于T_barrel）
            // 炮管模型沿+Z轴，炮口在maxZ处
            glm::vec3 muzzleLocalPos(0.0f, 0.0f, maxZ * meshScale);
            
            // 应用与炮管顶点相同的变换：
            // 1. 水平旋转（Y轴旋转）
            float x = muzzleLocalPos.x;
            float z = muzzleLocalPos.z;
            glm::vec3 muzzleRotated(c * x + s * z, muzzleLocalPos.y, -s * x + c * z);
            
            // 2. 计算相对于旋转轴心的位置
            glm::vec3 muzzleWorld = T_barrel + muzzleRotated;  // 先计算未旋转的世界位置
            glm::vec3 muzzleRelative = muzzleWorld - rotationAxisPoint;
            
            // 3. 应用俯仰旋转（绕炮耳轴）
            glm::vec3 muzzleRotated2 = rotationMatrix * muzzleRelative;
            
            // 4. 最终炮口位置
            glm::vec3 muzzlePosition = rotationAxisPoint + muzzleRotated2;
            
            // 创建多个烟雾体积，形成连续的烟雾云
            int numSmokeVolumes = 3 + (int)(smokeIntensity * 5);  // 3-8个烟雾体积
            
            for (int i = 0; i < numSmokeVolumes; i++) {
                float t = (float)i / (float)(numSmokeVolumes - 1);
                
                // 烟雾向上扩散（热空气上升）
                glm::vec3 upDirection(0.0f, 1.0f, 0.0f);
                float height = t * smokeIntensity * 1.5f;  // 最大高度
                
                // 向前方也有一定扩散（炮口方向）
                float forwardDistance = t * smokeIntensity * 0.8f;
                
                // 水平扩散（烟雾扩散）
                float horizontalSpread = t * t * smokeIntensity * 0.6f;
                
                // 计算烟雾位置
                glm::vec3 smokePos = muzzlePosition 
                    + upDirection * height
                    + rotatedBarrelDirection * forwardDistance;
                
                // 添加一些随机偏移（使烟雾更自然）
                float angle = t * 4.0f * M_PI;
                smokePos.x += cosf(angle) * horizontalSpread;
                smokePos.z += sinf(angle) * horizontalSpread;
                
                // 烟雾大小：底层小，上层大（扩散）
                glm::vec3 smokeSize(
                    0.3f + t * smokeIntensity * 0.8f,
                    0.4f + t * smokeIntensity * 1.0f,
                    0.3f + t * smokeIntensity * 0.8f
                );
                
                // 烟雾密度：底层高，上层低（扩散变淡）
                float smokeDensity = smokeIntensity * (1.0f - t * 0.6f);
                
                // 烟雾年龄：用于动画效果
                float smokeAge = t * 0.5f;
                
                SmokeVolume smoke;
                smoke.position = smokePos;
                smoke.size = smokeSize;
                smoke.density = smokeDensity;
                smoke.age = smokeAge;
                
                smokeVolumes.push_back(smoke);
            }
        }
        
        // 输出计算结果（可选，用于调试）
        // cout << "直线起点: (" << intersectionPoint.x << ", " << intersectionPoint.y << ", " << intersectionPoint.z << ")" << endl;
        // cout << "直线方向: (" << cylinderAxis.x << ", " << cylinderAxis.y << ", " << cylinderAxis.z << ")" << endl;
    }

    // ---------- 加载 turret ----------
    {
        std::vector<glm::vec3> verts; 
        std::vector<glm::ivec3> faces;
        loadOBJ("./meshes/turret.obj", verts, faces);

        std::vector<glm::vec3> transformedVerts = verts;
        for (auto &v : transformedVerts) {
            // 缩放
            v *= meshScale;
            // 水平旋转
            float x = v.x;
            float z = v.z;
            v.x = c * x + s * z;
            v.z = -s * x + c * z;
        }

        // 炮塔作为主体，稍微高一点
        glm::vec3 T_turret(0.0f, -2.0f, 15.0f);
        if (!transformedVerts.empty() && !faces.empty()) {
            objects.push_back(new Mesh(transformedVerts, faces, gray_diffuse, T_turret));
        }
    }
}
glm::vec3 toneMapping(glm::vec3 intensity){
	float gamma = 1.0/2.0;
	float alpha = 12.0f;
	return glm::clamp(alpha * glm::pow(intensity, glm::vec3(gamma)), glm::vec3(0.0), glm::vec3(1.0));
}

// Test function for --more: render 6 configurations without saving images
void renderMoreTest() {
    cout << "=== Rendering 6 Mesh Configurations ===" << endl;
    cout << "Testing with resolution (1280x720)..." << endl << endl;
    
    ofstream csvFile("performance_data.csv");
    csvFile << "TriangleCount,TimeSeconds" << endl;
    
    vector<string> testConfigs;
    // 当前只有一个配置：barrel + turret 组合
    testConfigs.push_back("barrel_turret");
    
    int width = 1280;
    int height = 720;
    float fov = 90;
    
    for (size_t configIdx = 0; configIdx < testConfigs.size(); configIdx++) {
        const string &config = testConfigs[configIdx];
        
        // Clear previous scene
        for (size_t i = 0; i < objects.size(); i++) delete objects[i];
        for (size_t i = 0; i < lights.size(); i++) delete lights[i];
        objects.clear();
        lights.clear();
        
        // Setup lights
        lights.push_back(new Light(glm::vec3(0, 26, 5), glm::vec3(1.0, 1.0, 1.0)));
        lights.push_back(new Light(glm::vec3(0, 1, 12), glm::vec3(0.1)));
        lights.push_back(new Light(glm::vec3(0, 5, 1), glm::vec3(0.4)));
        
        // Setup planes
        Material green_diffuse;
        green_diffuse.ambient = glm::vec3(0.03f, 0.1f, 0.03f);
        green_diffuse.diffuse = glm::vec3(0.3f, 1.0f, 0.3f);
        Material red_diffuse;
        red_diffuse.ambient = glm::vec3(0.09f, 0.06f, 0.06f);
        red_diffuse.diffuse = glm::vec3(0.9f, 0.6f, 0.6f);
        Material blue_diffuse;
        blue_diffuse.ambient = glm::vec3(0.06f, 0.06f, 0.09f);
        blue_diffuse.diffuse = glm::vec3(0.6f, 0.6f, 0.9f);
        // Removed floor, back wall, left wall, right wall for performance test as well
        
        Material gray_diffuse;
        gray_diffuse.ambient = glm::vec3(0.07f);
        gray_diffuse.diffuse = glm::vec3(0.7f);
        gray_diffuse.specular = glm::vec3(0.2f);
        gray_diffuse.shininess = 32.0f;
        
        int totalTriangles = 0;

        // 加载 barrel + turret 的性能测试配置
        if (config == "barrel_turret") {
            const float meshScale = 0.1f;
            const float angleDeg = 135.0f;
            const float angleRad = glm::radians(angleDeg);
            const float c = std::cos(angleRad);
            const float s = std::sin(angleRad);

            // barrel
            {
                std::vector<glm::vec3> verts; 
                std::vector<glm::ivec3> faces;
                loadOBJ("./meshes/barrel.obj", verts, faces);

                std::vector<glm::vec3> transformedVerts = verts;
                for (auto &v : transformedVerts) {
                    v *= meshScale;
                    float x = v.x;
                    float z = v.z;
                    v.x = c * x + s * z;
                    v.z = -s * x + c * z;
                }

                // 与主场景类似的相对位置，让炮管与炮塔在测试场景中也保持连接
                glm::vec3 T_barrel_base(0.0f, -2.4f, 10.4f);
                glm::vec3 forwardDir(s, 0.0f, c);
                forwardDir = glm::normalize(forwardDir);
                const float barrelOffset = 0.5f; // 与主场景使用相同的偏移
                glm::vec3 T = T_barrel_base + forwardDir * barrelOffset;
                if (!transformedVerts.empty() && !faces.empty()) {
                    Mesh *mesh = new Mesh(transformedVerts, faces, gray_diffuse, T, true);
                    objects.push_back(mesh);
                    totalTriangles += mesh->GetTriangleCount();
                }
            }

            // turret
            {
                std::vector<glm::vec3> verts; 
                std::vector<glm::ivec3> faces;
                loadOBJ("./meshes/turret.obj", verts, faces);

                std::vector<glm::vec3> transformedVerts = verts;
                for (auto &v : transformedVerts) {
                    v *= meshScale;
                    float x = v.x;
                    float z = v.z;
                    v.x = c * x + s * z;
                    v.z = -s * x + c * z;
                }

                glm::vec3 T(0.0f, -2.0f, 10.0f);
                if (!transformedVerts.empty() && !faces.empty()) {
                    Mesh *mesh = new Mesh(transformedVerts, faces, gray_diffuse, T, true);
                    objects.push_back(mesh);
                    totalTriangles += mesh->GetTriangleCount();
                }
            }
        }
        
        cout << "Rendering configuration: " << config << " (" << totalTriangles << " triangles)" << endl;
        
        // Render and measure time (without saving image)
        clock_t t = clock();
        
        float s = 2*tan(0.5*fov/180*M_PI)/width;
        float X = -s * width / 2;
        float Y = s * height / 2;
        
        int sampleCount = 0;
        for(int i = 0; i < width; i++) {
            for(int j = 0; j < height; j++) {
                float dx = X + i*s + s/2;
                float dy = Y - j*s - s/2;
                float dz = 1;
                
                glm::vec3 origin(0, 0, 0);
                glm::vec3 direction(dx, dy, dz);
                direction = glm::normalize(direction);
                
                Ray ray(origin, direction);
                trace_ray(ray);  // Render but don't save
                sampleCount++;
            }
        }
        
        t = clock() - t;
        float timeSeconds = ((float)t) / CLOCKS_PER_SEC;
        float fps = sampleCount / timeSeconds;
        
        cout << "  Time: " << timeSeconds << " seconds" << endl;
        cout << "  FPS: " << fps << endl;
        cout << "  Samples: " << sampleCount << endl << endl;
        
        // Write to CSV file
        csvFile << totalTriangles << "," << timeSeconds << endl;
    }
    
    csvFile.close();
    cout << "Performance data saved to performance_data.csv" << endl;
    cout << "All configurations rendered successfully!" << endl;
}

int main(int argc, const char * argv[]) {
    
    // Check for --more mode
    if (argc > 1 && string(argv[1]) == "--more") {
        renderMoreTest();
        return 0;
    }
    
    // Check for --animation mode
    if (argc > 1 && string(argv[1]) == "--animation") {
        cout << "=== Rendering 12-frame animation (8 elevation + 4 recoil) ===" << endl;
        
        // 先加载炮管模型计算长度
        std::vector<glm::vec3> verts;
        std::vector<glm::ivec3> faces;
        loadOBJ("./meshes/barrel.obj", verts, faces);
        
        float barrelLength = 0.0f;
        const float meshScale = 0.4f;  // 与sceneDefinition中的缩放一致
        if (!verts.empty()) {
            float minZ = verts[0].z;
            float maxZ = verts[0].z;
            for (const auto &v : verts) {
                if (v.z < minZ) minZ = v.z;
                if (v.z > maxZ) maxZ = v.z;
            }
            barrelLength = (maxZ - minZ) * meshScale;  // 考虑缩放后的长度
        }
        float maxRecoil = barrelLength / 20.0f;  // 最大缩退距离为炮管长度的1/20
        
        cout << "Barrel length: " << barrelLength << ", Max recoil: " << maxRecoil << endl;
        
        int width = 1280;
        int height = 720;
        float fov = 90;
        int numFrames = 12;  // 抬起8帧 + 缩退4帧
        int elevationFrames = 8;  // 抬起帧数
        int recoilFrames = 4;     // 缩退帧数
        float maxAngle = -20.0f;  // 最大抬起角度（向上）
        
        for (int frame = 0; frame < numFrames; frame++) {
            float elevationAngle = 0.0f;
            float recoilDistance = 0.0f;
            float smokeIntensity = 0.0f;
            
            if (frame < elevationFrames) {
                // 前8帧：炮管抬起动画（从0度到-20度）
                float t = (float)frame / (float)(elevationFrames - 1);  // 0到1
                elevationAngle = maxAngle * t;
                recoilDistance = 0.0f;  // 不缩退
                smokeIntensity = 0.0f;  // 无烟雾
            } else {
                // 后4帧：缩退动画（保持抬起角度为-20度）
                elevationAngle = maxAngle;  // 保持最大抬起角度
                
                // 缩退动画：前30%快速缩退，中间20%保持，后50%缓慢复进
                float recoilT = (float)(frame - elevationFrames) / (float)(recoilFrames - 1);  // 0到1
                if (recoilT < 0.3f) {
                    // 快速缩退阶段（0-30%）
                    float t = recoilT / 0.3f;
                    recoilDistance = maxRecoil * t;
                    smokeIntensity = t;  // 烟雾逐渐出现
                } else if (recoilT < 0.5f) {
                    // 保持最大缩退（30-50%）
                    recoilDistance = maxRecoil;
                    smokeIntensity = 1.0f;  // 最大烟雾
                } else {
                    // 缓慢复进阶段（50-100%）
                    float t = (recoilT - 0.5f) / 0.5f;
                    // 使用缓动函数（ease-out）
                    float easeOut = 1.0f - (1.0f - t) * (1.0f - t);
                    recoilDistance = maxRecoil * (1.0f - easeOut);
                    smokeIntensity = 1.0f - t * 0.7f;  // 烟雾逐渐消散（保留30%）
                }
            }
            
            cout << "Rendering frame " << frame + 1 << "/" << numFrames 
                 << " (angle: " << elevationAngle << " degrees, recoil: " 
                 << recoilDistance << ", smoke: " << smokeIntensity << ")..." << endl;
            
            // 清理之前的场景
            for (size_t i = 0; i < objects.size(); i++) delete objects[i];
            for (size_t i = 0; i < lights.size(); i++) delete lights[i];
            objects.clear();
            lights.clear();
            
            // 定义当前帧的场景
            sceneDefinition(elevationAngle, recoilDistance, smokeIntensity);
            
            // 渲染当前帧
            Image image(width, height);
            float s = 2*tan(0.5*fov/180*M_PI)/width;
            float X = -s * width / 2;
            float Y = s * height / 2;
            
            clock_t frameStart = clock();
            for(int i = 0; i < width ; i++) {
                for(int j = 0; j < height ; j++){
                    float dx = X + i*s + s/2;
                    float dy = Y - j*s - s/2;
                    float dz = 1;
                    
                    glm::vec3 origin(0, 0, 0);
                    glm::vec3 direction(dx, dy, dz);
                    direction = glm::normalize(direction);
                    
                    Ray ray(origin, direction);
                    image.setPixel(i, j, toneMapping(trace_ray(ray)));
                }
            }
            clock_t frameTime = clock() - frameStart;
            cout << "  Frame " << frame + 1 << " completed in " 
                 << ((float)frameTime)/CLOCKS_PER_SEC << " seconds" << endl;
            
            // 保存当前帧
            char filename[256];
            snprintf(filename, sizeof(filename), "./frame_%03d.ppm", frame);
            image.writeImage(filename);
        }
        
        cout << "Animation rendering complete! " << numFrames << " frames saved." << endl;
        return 0;
    }

    clock_t t = clock(); // variable for keeping the time of the rendering

    int width = 1280; //width of the image (increase for better quality)
    int height = 720; // height of the image (increase for better quality)
    float fov = 90; // field of view

	sceneDefinition(); // Let's define a scene

	Image image(width,height); // Create an image where we will store the result
	vector<glm::vec3> image_values(width*height);

    float s = 2*tan(0.5*fov/180*M_PI)/width;
    float X = -s * width / 2;
    float Y = s * height / 2;

    for(int i = 0; i < width ; i++)
        for(int j = 0; j < height ; j++){

			float dx = X + i*s + s/2;
            float dy = Y - j*s - s/2;
            float dz = 1;

			glm::vec3 origin(0, 0, 0);
            glm::vec3 direction(dx, dy, dz);
            direction = glm::normalize(direction);

            Ray ray(origin, direction);
            image.setPixel(i, j, toneMapping(trace_ray(ray)));
        }
	
    t = clock() - t;
    cout<<"It took " << ((float)t)/CLOCKS_PER_SEC<< " seconds to render the image."<< endl;
    cout<<"I could render at "<< (float)CLOCKS_PER_SEC/((float)t) << " frames per second."<<endl;

	// Writing the final results of the rendering
	if (argc == 2){
		image.writeImage(argv[1]);
	}else{
		image.writeImage("./result.ppm");
	}

	
    return 0;
}