#ifndef BVH_H
#define BVH_H

#include "glm/glm.hpp"
#include <vector>
#include <memory>
#include <algorithm>
#include <limits>
#include <cstdint>

class Triangle;
struct Hit;

class Ray {
public:
    glm::vec3 origin;
    glm::vec3 direction;
    mutable float tMax;
    
    Ray(glm::vec3 origin = glm::vec3(0), glm::vec3 direction = glm::vec3(0,0,1)) 
        : origin(origin), direction(direction), tMax(std::numeric_limits<float>::max()) {}
};

// Bounding box structure
struct Bounds3 {
    glm::vec3 pMin, pMax;
    
    Bounds3() : pMin(glm::vec3(std::numeric_limits<float>::max())),
                pMax(glm::vec3(std::numeric_limits<float>::lowest())) {}
    
    Bounds3(const glm::vec3 &p) : pMin(p), pMax(p) {}
    
    Bounds3(const glm::vec3 &p1, const glm::vec3 &p2) 
        : pMin(glm::min(p1, p2)), pMax(glm::max(p1, p2)) {}
    
    glm::vec3 Diagonal() const { return pMax - pMin; }
    
    float SurfaceArea() const {
        glm::vec3 d = Diagonal();
        return 2.0f * (d.x * d.y + d.x * d.z + d.y * d.z);
    }
    
    int MaximumExtent() const {
        glm::vec3 d = Diagonal();
        if (d.x > d.y && d.x > d.z) return 0;
        else if (d.y > d.z) return 1;
        else return 2;
    }
    
    glm::vec3 Centroid() const { return 0.5f * pMin + 0.5f * pMax; }
    
    Bounds3 Union(const Bounds3 &b) const {
        return Bounds3(glm::min(pMin, b.pMin), glm::max(pMax, b.pMax));
    }
    
    Bounds3 Union(const glm::vec3 &p) const {
        return Bounds3(glm::min(pMin, p), glm::max(pMax, p));
    }
    
    bool IntersectP(const Ray &ray, const glm::vec3 &invDir, const int dirIsNeg[3]) const {
        // Get the appropriate min/max based on ray direction
        float tMin = ((*this)[dirIsNeg[0]].x - ray.origin.x) * invDir.x;
        float tMax = ((*this)[1-dirIsNeg[0]].x - ray.origin.x) * invDir.x;
        float tyMin = ((*this)[dirIsNeg[1]].y - ray.origin.y) * invDir.y;
        float tyMax = ((*this)[1-dirIsNeg[1]].y - ray.origin.y) * invDir.y;
        
        if (tMin > tyMax || tyMin > tMax) return false;
        if (tyMin > tMin) tMin = tyMin;
        if (tyMax < tMax) tMax = tyMax;
        
        float tzMin = ((*this)[dirIsNeg[2]].z - ray.origin.z) * invDir.z;
        float tzMax = ((*this)[1-dirIsNeg[2]].z - ray.origin.z) * invDir.z;
        
        if (tMin > tzMax || tzMin > tMax) return false;
        if (tzMin > tMin) tMin = tzMin;
        if (tzMax < tMax) tMax = tzMax;
        
        return (tMin < ray.tMax) && (tMax > 0);
    }
    
    const glm::vec3& operator[](int i) const {
        return (i == 0) ? pMin : pMax;
    }
};

// BVH primitive info
struct BVHPrimitiveInfo {
    size_t primitiveNumber;
    Bounds3 bounds;
    glm::vec3 centroid;
    
    BVHPrimitiveInfo() : primitiveNumber(0), bounds(), centroid(0.0f) {}
    
    BVHPrimitiveInfo(size_t idx, const Bounds3 &b) 
        : primitiveNumber(idx), bounds(b), centroid(b.Centroid()) {}
};

// BVH build node
struct BVHBuildNode {
    Bounds3 bounds;
    BVHBuildNode *children[2];
    int splitAxis, firstPrimOffset, nPrimitives;
    
    BVHBuildNode() {
        children[0] = children[1] = nullptr;
        splitAxis = 0;
        firstPrimOffset = 0;
        nPrimitives = 0;
    }
    
    void InitLeaf(int first, int n, const Bounds3 &b) {
        firstPrimOffset = first;
        nPrimitives = n;
        bounds = b;
        children[0] = children[1] = nullptr;
    }
    
    void InitInterior(int axis, BVHBuildNode *c0, BVHBuildNode *c1) {
        children[0] = c0;
        children[1] = c1;
        bounds = c0->bounds.Union(c1->bounds);
        splitAxis = axis;
        nPrimitives = 0;
    }
};

// Linear BVH node for traversal
struct LinearBVHNode {
    Bounds3 bounds;
    union {
        int primitivesOffset;  // leaf
        int secondChildOffset; // interior
    };
    uint16_t nPrimitives;  // 0 -> interior node
    uint8_t axis;          // interior node: xyz
    uint8_t pad[1];        
};

// BVH Accelerator
class BVHAccel {
public:
    BVHAccel(std::vector<Triangle*> &prims, int maxPrimsInNode = 1);
    ~BVHAccel();
    
    Hit Intersect(const Ray &ray) const;
    
    int GetTotalNodes() const { return totalNodes; }
    int GetTotalPrimitives() const { return (int)primitives.size(); }
    
private:
    const int maxPrimsInNode;
    std::vector<Triangle*> primitives;
    LinearBVHNode *nodes;
    int totalNodes;
    
    BVHBuildNode* recursiveBuild(
        std::vector<BVHPrimitiveInfo> &primitiveInfo,
        int start, int end, int *totalNodes);
    
    BVHBuildNode* HLBVHBuild(
        std::vector<BVHPrimitiveInfo> &primitiveInfo,
        int *totalNodes);
    
    int flattenBVHTree(BVHBuildNode *node, int *offset);
    
    void computeBounds(const std::vector<BVHPrimitiveInfo> &primitiveInfo,
                      int start, int end, Bounds3 *bounds);
    
    int partitionPrimitives(std::vector<BVHPrimitiveInfo> &primitiveInfo,
                           int start, int end, int dim, float split);
};

#endif // BVH_H

