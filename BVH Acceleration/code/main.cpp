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
	if(closest_hit.hit){
		color = PhongModel(closest_hit.intersection, closest_hit.normal, glm::normalize(-ray.direction), closest_hit.object->getMaterial());
	}else{
		color = glm::vec3(0.0, 0.0, 0.0);
	}
	return color;
}
/**
 Function defining the scene
 */
void sceneDefinition (){

	
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
    // Four visible planes: floor, back wall, left wall, right wall
    objects.push_back(new Plane(glm::vec3(0,-3,0), glm::vec3(0.0,1,0))); // floor
    objects.push_back(new Plane(glm::vec3(0,1,30), glm::vec3(0.0,0.0,-1.0), green_diffuse)); // back wall
    objects.push_back(new Plane(glm::vec3(-15,1,0), glm::vec3(1.0,0.0,0.0), red_diffuse)); // left wall
    objects.push_back(new Plane(glm::vec3(15,1,0), glm::vec3(-1.0,0.0,0.0), blue_diffuse)); // right wall
    Material gray_diffuse;
    gray_diffuse.ambient = glm::vec3(0.07f);
    gray_diffuse.diffuse = glm::vec3(0.7f);
    gray_diffuse.specular = glm::vec3(0.2f);
    gray_diffuse.shininess = 32.0f;
    
	// Load armadillo mesh - Ta = (-4.0, -3.0, 10)
	std::vector<glm::vec3> verts2; std::vector<glm::ivec3> faces2;
	loadOBJ("./meshes/armadillo.obj", verts2, faces2);
	glm::vec3 Ta(-4.0f, -3.0f, 10.0f);
	if(!verts2.empty() && !faces2.empty()){
		objects.push_back(new Mesh(verts2, faces2, gray_diffuse, Ta));
	}
	
	// Load bunny mesh - Tb = (0.0, -3.0, 8.0)
    std::vector<glm::vec3> verts; std::vector<glm::ivec3> faces;
    loadOBJ("./meshes/bunny.obj", verts, faces);
    glm::vec3 Tb(0.0f, -3.0f, 8.0f);
    if(!verts.empty() && !faces.empty()){
        objects.push_back(new Mesh(verts, faces, gray_diffuse, Tb));
    }
	
	// Load lucy mesh - Tl = (4.0, -3.0, 10)
	std::vector<glm::vec3> verts3; std::vector<glm::ivec3> faces3;
	loadOBJ("./meshes/lucy.obj", verts3, faces3);
	glm::vec3 Tl(4.0f, -3.0f, 10.0f);
	if(!verts3.empty() && !faces3.empty()){
		objects.push_back(new Mesh(verts3, faces3, gray_diffuse, Tl));
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
    testConfigs.push_back("bunny_small");
    testConfigs.push_back("armadillo_small");
    testConfigs.push_back("lucy_small");
    testConfigs.push_back("bunny");
    testConfigs.push_back("armadillo");
    testConfigs.push_back("lucy");
    
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
        
        objects.push_back(new Plane(glm::vec3(0,-3,0), glm::vec3(0.0,1,0)));
        objects.push_back(new Plane(glm::vec3(0,1,30), glm::vec3(0.0,0.0,-1.0), green_diffuse));
        objects.push_back(new Plane(glm::vec3(-15,1,0), glm::vec3(1.0,0.0,0.0), red_diffuse));
        objects.push_back(new Plane(glm::vec3(15,1,0), glm::vec3(-1.0,0.0,0.0), blue_diffuse));
        
        Material gray_diffuse;
        gray_diffuse.ambient = glm::vec3(0.07f);
        gray_diffuse.diffuse = glm::vec3(0.7f);
        gray_diffuse.specular = glm::vec3(0.2f);
        gray_diffuse.shininess = 32.0f;
        
        int totalTriangles = 0;
        
        // Load mesh based on configuration
        if (config == "bunny") {
            std::vector<glm::vec3> verts; std::vector<glm::ivec3> faces;
            loadOBJ("./meshes/bunny.obj", verts, faces);
            glm::vec3 Tb(0.0f, -3.0f, 8.0f);
            if(!verts.empty() && !faces.empty()){
                Mesh *mesh = new Mesh(verts, faces, gray_diffuse, Tb, true);
                objects.push_back(mesh);
                totalTriangles += mesh->GetTriangleCount();
            }
        }
        
        if (config == "bunny_small") {
            std::vector<glm::vec3> verts; std::vector<glm::ivec3> faces;
            loadOBJ("./meshes/bunny_small.obj", verts, faces);
            glm::vec3 Tb(0.0f, -3.0f, 8.0f);
            if(!verts.empty() && !faces.empty()){
                Mesh *mesh = new Mesh(verts, faces, gray_diffuse, Tb, true);
                objects.push_back(mesh);
                totalTriangles += mesh->GetTriangleCount();
            }
        }
        
        if (config == "armadillo") {
            std::vector<glm::vec3> verts2; std::vector<glm::ivec3> faces2;
            loadOBJ("./meshes/armadillo.obj", verts2, faces2);
            glm::vec3 Ta(-4.0f, -3.0f, 10.0f);
            if(!verts2.empty() && !faces2.empty()){
                Mesh *mesh = new Mesh(verts2, faces2, gray_diffuse, Ta, true);
                objects.push_back(mesh);
                totalTriangles += mesh->GetTriangleCount();
            }
        }
        
        if (config == "armadillo_small") {
            std::vector<glm::vec3> verts2; std::vector<glm::ivec3> faces2;
            loadOBJ("./meshes/armadillo_small.obj", verts2, faces2);
            glm::vec3 Ta(-4.0f, -3.0f, 10.0f);
            if(!verts2.empty() && !faces2.empty()){
                Mesh *mesh = new Mesh(verts2, faces2, gray_diffuse, Ta, true);
                objects.push_back(mesh);
                totalTriangles += mesh->GetTriangleCount();
            }
        }
        
        if (config == "lucy") {
            std::vector<glm::vec3> verts3; std::vector<glm::ivec3> faces3;
            loadOBJ("./meshes/lucy.obj", verts3, faces3);
            glm::vec3 Tl(4.0f, -3.0f, 10.0f);
            if(!verts3.empty() && !faces3.empty()){
                Mesh *mesh = new Mesh(verts3, faces3, gray_diffuse, Tl, true);
                objects.push_back(mesh);
                totalTriangles += mesh->GetTriangleCount();
            }
        }
        
        if (config == "lucy_small") {
            std::vector<glm::vec3> verts3; std::vector<glm::ivec3> faces3;
            loadOBJ("./meshes/lucy_small.obj", verts3, faces3);
            glm::vec3 Tl(4.0f, -3.0f, 10.0f);
            if(!verts3.empty() && !faces3.empty()){
                Mesh *mesh = new Mesh(verts3, faces3, gray_diffuse, Tl, true);
                objects.push_back(mesh);
                totalTriangles += mesh->GetTriangleCount();
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

    clock_t t = clock(); // variable for keeping the time of the rendering

    int width = 2048; //width of the image (increase for better quality)
    int height = 1536; // height of the image (increase for better quality)
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
