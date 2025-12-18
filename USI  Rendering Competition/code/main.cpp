#include <iostream>
#include <fstream>
#include <cmath>
#include <ctime>
#include <vector>
#include <string>
#include <sstream>
#include <limits>
#include "glm/glm.hpp"

#include "Image.h"
#include "Material.h"
#include "bvh.h"
#include "bmpmini.hpp"

using namespace std;
using namespace image;

class Object;
class PerlinNoise {
private:
    static float hash(float n) {
        float result = sinf(n) * 43758.5453f;
        return result - floorf(result);
    }
    
    static float smoothstep(float t) {
        return t * t * (3.0f - 2.0f * t);
    }
    
    static float grad(int hash, float x, float y) {
        int h = hash & 3;
        float u = h < 2 ? x : y;
        float v = h < 1 ? y : (h == 1 ? -x : -y);
        return u + v;
    }

public:
    static float noise2D(float x, float y) {
        int X = (int)floorf(x) & 255;
        int Y = (int)floorf(y) & 255;
        x -= floorf(x);
        y -= floorf(y);
        
        float u = smoothstep(x);
        float v = smoothstep(y);
        
        int A = (int)(X + Y * 57) & 255;
        int B = (int)((X + 1) + Y * 57) & 255;
        int AA = (int)(X + (Y + 1) * 57) & 255;
        int BA = (int)((X + 1) + (Y + 1) * 57) & 255;
        
        float a = grad(A, x, y);
        float b = grad(B, x - 1.0f, y);
        float aa = grad(AA, x, y - 1.0f);
        float ba = grad(BA, x - 1.0f, y - 1.0f);
        
        float lerp1 = a + (b - a) * u;
        float lerp2 = aa + (ba - aa) * u;
        return lerp1 + (lerp2 - lerp1) * v;
    }
    
    static float fractalNoise2D(float x, float y, int octaves = 4, float persistence = 0.5f) {
        float value = 0.0f;
        float amplitude = 1.0f;
        float frequency = 1.0f;
        float maxValue = 0.0f;
        
        for (int i = 0; i < octaves; i++) {
            value += noise2D(x * frequency, y * frequency) * amplitude;
            maxValue += amplitude;
            amplitude *= persistence;
            frequency *= 2.0f;
        }
        
        return value / maxValue;
    }
    
    static float noise3D(float x, float y, float z) {
        int X = (int)floorf(x) & 255;
        int Y = (int)floorf(y) & 255;
        int Z = (int)floorf(z) & 255;
        x -= floorf(x);
        y -= floorf(y);
        z -= floorf(z);
        
        float u = smoothstep(x);
        float v = smoothstep(y);
        float w = smoothstep(z);
        
        int A = (int)(X + Y * 57 + Z * 131) & 255;
        int B = (int)((X + 1) + Y * 57 + Z * 131) & 255;
        int AA = (int)(X + (Y + 1) * 57 + Z * 131) & 255;
        int BA = (int)((X + 1) + (Y + 1) * 57 + Z * 131) & 255;
        int AB = (int)(X + Y * 57 + (Z + 1) * 131) & 255;
        int BB = (int)((X + 1) + Y * 57 + (Z + 1) * 131) & 255;
        int AAB = (int)(X + (Y + 1) * 57 + (Z + 1) * 131) & 255;
        int BAB = (int)((X + 1) + (Y + 1) * 57 + (Z + 1) * 131) & 255;
        
        float a = grad3D(A, x, y, z);
        float b = grad3D(B, x - 1.0f, y, z);
        float aa = grad3D(AA, x, y - 1.0f, z);
        float ba = grad3D(BA, x - 1.0f, y - 1.0f, z);
        float ab = grad3D(AB, x, y, z - 1.0f);
        float bb = grad3D(BB, x - 1.0f, y, z - 1.0f);
        float aab = grad3D(AAB, x, y - 1.0f, z - 1.0f);
        float bab = grad3D(BAB, x - 1.0f, y - 1.0f, z - 1.0f);
        
        float lerp1 = a + (b - a) * u;
        float lerp2 = aa + (ba - aa) * u;
        float lerp3 = ab + (bb - ab) * u;
        float lerp4 = aab + (bab - aab) * u;
        float lerp5 = lerp1 + (lerp2 - lerp1) * v;
        float lerp6 = lerp3 + (lerp4 - lerp3) * v;
        return lerp5 + (lerp6 - lerp5) * w;
    }
    
    static float grad3D(int hash, float x, float y, float z) {
        int h = hash & 15;
        float u = h < 8 ? x : y;
        float v = h < 4 ? y : (h == 12 || h == 14 ? x : z);
        return ((h & 1) == 0 ? u : -u) + ((h & 2) == 0 ? v : -v);
    }
    
    static float fractalNoise3D(float x, float y, float z, int octaves = 4, float persistence = 0.5f) {
        float value = 0.0f;
        float amplitude = 1.0f;
        float frequency = 1.0f;
        float maxValue = 0.0f;
        
        for (int i = 0; i < octaves; i++) {
            value += noise3D(x * frequency, y * frequency, z * frequency) * amplitude;
            maxValue += amplitude;
            amplitude *= persistence;
            frequency *= 2.0f;
        }
        
        return value / maxValue;
    }
};

struct Hit{
    bool hit; ///< Boolean indicating whether there was or there was no intersection with an object
    glm::vec3 normal; ///< Normal vector of the intersected object at the intersection point
    glm::vec3 intersection; ///< Point of Intersection
    float distance; ///< Distance from the origin of the ray to the intersection point
    Object *object; ///< A pointer to the intersected object
    glm::vec3 tangent; ///< Tangent vector (for anisotropic materials like Ward model)
    glm::vec3 bitangent; ///< Bitangent vector (for anisotropic materials like Ward model)
    float u; ///< Texture coordinate U (0-1)
    float v; ///< Texture coordinate V (0-1)
    
    Hit() : hit(false), u(0.0f), v(0.0f) {}
};

/**
 * Texture class for loading and sampling BMP textures
 */
class Texture {
private:
    mutable BMPMini bmp;  // mutable to allow get() in const function
    int width;
    int height;
    bool loaded;

public:
    Texture() : loaded(false), width(0), height(0) {}
    
    bool load(const std::string& filename) {
        try {
            bmp.read(filename);
            ImageView imageView = bmp.get();
            width = imageView.width;
            height = imageView.height;
            loaded = (width > 0 && height > 0);
            return loaded;
        } catch (...) {
            loaded = false;
            return false;
        }
    }
    
    bool isLoaded() const { return loaded; }
    
    /**
     * Sample texture at UV coordinates (0-1 range)
     * Returns RGB color as vec3 (0-1 range)
     */
    glm::vec3 sample(float u, float v) const {
        if (!loaded) return glm::vec3(1.0f);  // Return white if no texture
        
        // Get image view when needed
        ImageView imageView = bmp.get();
        
        // Clamp UV coordinates to [0, 1]
        u = glm::clamp(u, 0.0f, 1.0f);
        v = glm::clamp(v, 0.0f, 1.0f);
        
        // Convert to pixel coordinates
        int x = (int)(u * (width - 1));
        int y = (int)(v * (height - 1));
        
        // Ensure within bounds
        x = glm::clamp(x, 0, width - 1);
        y = glm::clamp(y, 0, height - 1);
        
        // Get pixel index (BMP is BGR format)
        int channels = imageView.channels;
        int index = (y * width + x) * channels;
        
        if (channels >= 3) {
            // BGR to RGB conversion, normalize to [0, 1]
            float r = imageView.data[index + 2] / 255.0f;
            float g = imageView.data[index + 1] / 255.0f;
            float b = imageView.data[index + 0] / 255.0f;
            return glm::vec3(r, g, b);
        } else if (channels == 1) {
            // Grayscale
            float gray = imageView.data[index] / 255.0f;
            return glm::vec3(gray);
        }
        
        return glm::vec3(1.0f);  // Default to white
    }
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
	Texture* texture; ///< Texture for diffuse color (nullptr if no texture)
	Texture* normalMap; ///< Normal map texture (nullptr if no normal map)
	Texture* aoMap; ///< Ambient occlusion texture (nullptr if no AO map)
	Texture* roughnessMap; ///< Roughness texture (nullptr if no roughness map)
	
	Object() : texture(nullptr), normalMap(nullptr), aoMap(nullptr), roughnessMap(nullptr) {}
	
	/** Function that sets the texture
	 @param tex Pointer to texture object
	*/
	void setTexture(Texture* tex) {
		texture = tex;
	}
	
	/** Function that sets the normal map
	 @param nm Pointer to normal map texture object
	*/
	void setNormalMap(Texture* nm) {
		normalMap = nm;
	}
	
	/** Function that sets the ambient occlusion map
	 @param ao Pointer to AO texture object
	*/
	void setAOMap(Texture* ao) {
		aoMap = ao;
	}
	
	/** Function that sets the roughness map
	 @param rough Pointer to roughness texture object
	*/
	void setRoughnessMap(Texture* rough) {
		roughnessMap = rough;
	}
	
	/** A function computing an intersection, which returns the structure Hit */
    virtual Hit intersect(Ray ray) = 0;
	
	/** Function that returns whether this object casts shadows
	 @return true if the object casts shadows, false otherwise
	*/
	virtual bool castsShadow() const {
		return true;
	}

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
                hit.tangent = glm::vec3(0.0f);
                hit.bitangent = glm::vec3(0.0f);
                
                float textureScale = 0.1f;
                hit.u = fmod(hit.intersection.x * textureScale, 1.0f);
                if (hit.u < 0.0f) hit.u += 1.0f;
                hit.v = fmod(hit.intersection.z * textureScale, 1.0f);
                if (hit.v < 0.0f) hit.v += 1.0f;
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
        
        glm::vec2 uv0(0.0f, 0.0f);
        glm::vec2 uv1(1.0f, 0.0f);
        glm::vec2 uv2(0.0f, 1.0f);
        glm::vec2 deltaUV1 = uv1 - uv0;
        glm::vec2 deltaUV2 = uv2 - uv0;
        
        float f = 1.0f / (deltaUV1.x * deltaUV2.y - deltaUV2.x * deltaUV1.y);
        
        glm::vec3 tangent;
        tangent.x = f * (deltaUV2.y * edge1.x - deltaUV1.y * edge2.x);
        tangent.y = f * (deltaUV2.y * edge1.y - deltaUV1.y * edge2.y);
        tangent.z = f * (deltaUV2.y * edge1.z - deltaUV1.y * edge2.z);
        tangent = glm::normalize(tangent);
        
        glm::vec3 bitangent;
        bitangent.x = f * (-deltaUV2.x * edge1.x + deltaUV1.x * edge2.x);
        bitangent.y = f * (-deltaUV2.x * edge1.y + deltaUV1.x * edge2.y);
        bitangent.z = f * (-deltaUV2.x * edge1.z + deltaUV1.x * edge2.z);
        bitangent = glm::normalize(bitangent);
        
        tangent = glm::normalize(tangent - glm::dot(tangent, hit.normal) * hit.normal);
        bitangent = glm::normalize(bitangent - glm::dot(bitangent, hit.normal) * hit.normal);
        if (glm::dot(glm::cross(tangent, bitangent), hit.normal) < 0.0f) {
            bitangent = -bitangent;
        }
        
        hit.tangent = tangent;
        hit.bitangent = bitangent;
        
        float w = 1.0f - u - v;
        glm::vec3 texCoord = w * v0 + u * v1 + v * v2;
        float textureScale = 0.1f;
        hit.u = fmod(texCoord.x * textureScale, 1.0f);
        if (hit.u < 0.0f) hit.u += 1.0f;
        hit.v = fmod(texCoord.z * textureScale, 1.0f);
        if (hit.v < 0.0f) hit.v += 1.0f;
        
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
    glm::vec3 center;
    glm::vec3 axis;
    float radius;
    float halfHeight;

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
        
        glm::vec3 oc = ray.origin - center;
        float rayAxisDot = glm::dot(ray.direction, axis);
        float ocAxisDot = glm::dot(oc, axis);
        glm::vec3 rayPerp = ray.direction - rayAxisDot * axis;
        glm::vec3 ocPerp = oc - ocAxisDot * axis;
        float a = glm::dot(rayPerp, rayPerp);
        float b = 2.0f * glm::dot(rayPerp, ocPerp);
        float c = glm::dot(ocPerp, ocPerp) - radius * radius;
        
        float discriminant = b * b - 4.0f * a * c;
        if (discriminant < 0.0f || a < 1e-6f) return hit;
        
        float sqrtD = sqrtf(discriminant);
        float t1 = (-b - sqrtD) / (2.0f * a);
        float t2 = (-b + sqrtD) / (2.0f * a);
        
        float t = (t1 > 0.0f && t1 < ray.tMax) ? t1 : t2;
        if (t <= 0.0f || t > ray.tMax) return hit;
        
        glm::vec3 intersection = ray.origin + t * ray.direction;
        glm::vec3 toIntersection = intersection - center;
        float projOnAxis = glm::dot(toIntersection, axis);
        
        if (fabs(projOnAxis) > halfHeight) return hit;
        glm::vec3 normal = toIntersection - projOnAxis * axis;
        float normalLen = glm::length(normal);
        if (normalLen > 1e-6f) {
            normal = normal / normalLen;
        } else {
            glm::vec3 up(0, 1, 0);
            if (fabs(glm::dot(axis, up)) > 0.9f) up = glm::vec3(1, 0, 0);
            normal = glm::normalize(glm::cross(axis, up));
        }
        
        glm::vec3 tangent = axis;
        glm::vec3 bitangent = glm::normalize(glm::cross(normal, tangent));
        tangent = glm::normalize(glm::cross(bitangent, normal));
        
        hit.hit = true;
        hit.distance = t;
        hit.intersection = intersection;
        hit.normal = normal;
        hit.tangent = tangent;
        hit.bitangent = bitangent;
        hit.object = this;
        
        return hit;
    }
};

class Sphere : public Object {
private:
    glm::vec3 center;
    float radius;

public:
    Sphere(const glm::vec3 &center, float radius) 
        : center(center), radius(radius) {
    }
    
    Sphere(const glm::vec3 &center, float radius, const Material &mat) 
        : center(center), radius(radius) {
        material = mat;
    }

    Hit intersect(Ray ray) {
        Hit hit;
        hit.hit = false;
        
        glm::vec3 oc = ray.origin - center;
        float a = glm::dot(ray.direction, ray.direction);
        float b = 2.0f * glm::dot(oc, ray.direction);
        float c = glm::dot(oc, oc) - radius * radius;
        float discriminant = b * b - 4.0f * a * c;
        
        if (discriminant < 0.0f) return hit;
        
        float sqrtD = sqrtf(discriminant);
        float t1 = (-b - sqrtD) / (2.0f * a);
        float t2 = (-b + sqrtD) / (2.0f * a);
        
        float t = (t1 > 0.0f && t1 < ray.tMax) ? t1 : t2;
        if (t <= 0.0f || t > ray.tMax) return hit;
        
        hit.hit = true;
        hit.distance = t;
        hit.intersection = ray.origin + t * ray.direction;
        hit.normal = glm::normalize(hit.intersection - center);
        
        glm::vec3 toPoint = hit.intersection - center;
        glm::vec3 up(0.0f, 1.0f, 0.0f);
        
        glm::vec3 normalProjY = glm::vec3(hit.normal.x, 0.0f, hit.normal.z);
        if (glm::length(normalProjY) > 0.001f) {
            normalProjY = glm::normalize(normalProjY);
            hit.tangent = glm::normalize(glm::cross(up, normalProjY));
        } else {
            hit.tangent = glm::vec3(1.0f, 0.0f, 0.0f);
        }
        
        hit.bitangent = glm::normalize(glm::cross(hit.normal, hit.tangent));
        hit.tangent = glm::normalize(glm::cross(hit.bitangent, hit.normal));
        float u = atan2(hit.normal.z, hit.normal.x) / (2.0f * M_PI) + 0.5f;
        float v = acos(glm::clamp(hit.normal.y, -1.0f, 1.0f)) / M_PI;
        hit.u = u;
        hit.v = v;
        
        hit.object = this;
        
        return hit;
    }
};

class WaterPlane : public Object {
private:
    glm::vec3 center;
    glm::vec3 normal;
    float size;
    float waveHeight;
    float waveFrequency;
    float time;
    int octaves;

public:
    WaterPlane(glm::vec3 center, glm::vec3 normal, float size, float waveHeight = 0.2f, float waveFrequency = 1.0f, float time = 0.0f)
        : center(center), normal(glm::normalize(normal)), size(size), 
          waveHeight(waveHeight), waveFrequency(waveFrequency), time(time), octaves(4) {
    }
    
    WaterPlane(glm::vec3 center, glm::vec3 normal, float size, Material material, 
               float waveHeight = 0.2f, float waveFrequency = 1.0f, float time = 0.0f)
        : center(center), normal(glm::normalize(normal)), size(size),
          waveHeight(waveHeight), waveFrequency(waveFrequency), time(time), octaves(4) {
        this->material = material;
    }
    
    void setTime(float t) {
        time = t;
    }
    
    void setWaveHeight(float h) {
        waveHeight = h;
    }
    
    void setWaveFrequency(float f) {
        waveFrequency = f;
    }
    
    float getHeight(float x, float z) const {
        float noise1 = PerlinNoise::fractalNoise3D(x * waveFrequency * 0.5f, z * waveFrequency * 0.5f, time * 0.3f, octaves, 0.5f);
        float noise2 = PerlinNoise::fractalNoise3D((x * 0.707f + z * 0.707f) * waveFrequency * 1.2f, (x * -0.707f + z * 0.707f) * waveFrequency * 1.2f, time * 0.4f, octaves - 1, 0.5f);
        float noise3 = PerlinNoise::fractalNoise3D(x * waveFrequency * 2.0f, z * waveFrequency * 2.0f, time * 0.6f, octaves - 2, 0.5f);
        float noise4 = PerlinNoise::noise3D(x * waveFrequency * 4.0f, z * waveFrequency * 4.0f, time * 0.8f);
        float regionNoise = PerlinNoise::noise2D(x * 0.1f, z * 0.1f);
        float combinedNoise = noise1 * 0.5f + noise2 * 0.25f + noise3 * 0.15f + noise4 * 0.05f + regionNoise * 0.05f;
        return center.y + combinedNoise * waveHeight;
    }
    
    glm::vec3 getNormal(float x, float z) const {
        float eps = 0.1f;
        float heightL = getHeight(x - eps, z);
        float heightR = getHeight(x + eps, z);
        float heightD = getHeight(x, z - eps);
        float heightU = getHeight(x, z + eps);
        glm::vec3 gradient((heightR - heightL) / (2.0f * eps), 1.0f, (heightU - heightD) / (2.0f * eps));
        return glm::normalize(gradient);
    }
    
    Hit intersect(Ray ray) {
        Hit hit;
        hit.hit = false;
        
        if (fabs(ray.direction.y) < 1e-6) return hit;
        
        float minY = center.y - waveHeight;
        float maxY = center.y + waveHeight;
        float tMin = (minY - ray.origin.y) / ray.direction.y;
        float tMax = (maxY - ray.origin.y) / ray.direction.y;
        if (tMin > tMax) {
            float temp = tMin;
            tMin = tMax;
            tMax = temp;
        }
        if (tMax < 0.0f || tMin > ray.tMax) return hit;
        
        float tStart = glm::max(0.0f, tMin);
        float tEnd = glm::min(ray.tMax, tMax);
        float t = (tStart + tEnd) * 0.5f;
        
        for (int iter = 0; iter < 10; iter++) {
            glm::vec3 pos = ray.origin + t * ray.direction;
            if (fabs(pos.x - center.x) > size || fabs(pos.z - center.z) > size) return hit;
            
            float height = getHeight(pos.x, pos.z);
            float diff = pos.y - height;
            
            if (fabs(diff) < 0.001f) {
                hit.hit = true;
                hit.distance = t;
                hit.intersection = pos;
                hit.intersection.y = height;
                hit.normal = getNormal(pos.x, pos.z);
                
                glm::vec3 up(0.0f, 1.0f, 0.0f);
                glm::vec3 tangent = glm::normalize(glm::cross(up, hit.normal));
                if (glm::length(tangent) < 0.1f) tangent = glm::vec3(1.0f, 0.0f, 0.0f);
                hit.bitangent = glm::normalize(glm::cross(hit.normal, tangent));
                hit.tangent = glm::normalize(glm::cross(hit.bitangent, hit.normal));
                
                float textureScale = 0.1f;
                hit.u = fmod(pos.x * textureScale, 1.0f);
                if (hit.u < 0.0f) hit.u += 1.0f;
                hit.v = fmod(pos.z * textureScale, 1.0f);
                if (hit.v < 0.0f) hit.v += 1.0f;
                hit.object = this;
                return hit;
            }
            
            if (diff > 0.0f) tEnd = t;
            else tStart = t;
            t = (tStart + tEnd) * 0.5f;
        }
        return hit;
    }
};

class Disc : public Object {
private:
    glm::vec3 center;
    glm::vec3 normal;
    float radius;

public:
    Disc(const glm::vec3 &center, const glm::vec3 &normal, float radius) 
        : center(center), normal(glm::normalize(normal)), radius(radius) {
    }
    
    Disc(const glm::vec3 &center, const glm::vec3 &normal, float radius, const Material &mat) 
        : center(center), normal(glm::normalize(normal)), radius(radius) {
        material = mat;
    }
    
    bool castsShadow() const override {
        return false;
    }

    Hit intersect(Ray ray) {
        Hit hit;
        hit.hit = false;
        
        float denom = glm::dot(normal, ray.direction);
        if (fabs(denom) < 1e-6) return hit;
        
        glm::vec3 toCenter = center - ray.origin;
        float t = glm::dot(toCenter, normal) / denom;
        
        if (t <= 0.0f || t > ray.tMax) return hit;
        
        glm::vec3 intersection = ray.origin + t * ray.direction;
        glm::vec3 toIntersection = intersection - center;
        float distSq = glm::dot(toIntersection, toIntersection);
        
        if (distSq > radius * radius) return hit;
        
        hit.hit = true;
        hit.distance = t;
        hit.intersection = intersection;
        hit.normal = normal;
        hit.tangent = glm::vec3(0.0f);
        hit.bitangent = glm::vec3(0.0f);
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
                    closest.object = this;
                }
            }
        }
        return closest;
    }
    
    int GetTriangleCount() const { return (int)triangles.size(); }
    int GetBVHNodeCount() const { return bvh ? bvh->GetTotalNodes() : 0; }
};

	

/**
 Light class (point light)
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

/**
 Area Light class (for soft shadows)
 */
class AreaLight {
public:
    glm::vec3 position;
    glm::vec3 color;
    glm::vec3 u;
    glm::vec3 v;
    float size;
    bool isSphere;
    float attenuationPower;
    
    AreaLight(glm::vec3 pos, glm::vec3 col, float s, bool sphere = false)
        : position(pos), color(col), size(s), isSphere(sphere), attenuationPower(2.0f) {
        u = glm::vec3(1.0f, 0.0f, 0.0f);
        v = glm::vec3(0.0f, 0.0f, 1.0f);
    }
    
    AreaLight(glm::vec3 pos, glm::vec3 col, glm::vec3 uDir, glm::vec3 vDir, float s)
        : position(pos), color(col), u(glm::normalize(uDir)), v(glm::normalize(vDir)), size(s), isSphere(false), attenuationPower(2.0f) {
    }
    
    AreaLight(glm::vec3 pos, glm::vec3 col, glm::vec3 uDir, glm::vec3 vDir, float s, float attenPower)
        : position(pos), color(col), u(glm::normalize(uDir)), v(glm::normalize(vDir)), size(s), isSphere(false), attenuationPower(attenPower) {
    }
    
    /**
     * Sample a point on area light
     */
    glm::vec3 samplePoint(int seed) const {
        if (isSphere) {
            float r1 = ((seed * 1103515245 + 12345) & 0x7fffffff) / 2147483648.0f;
            float r2 = (((seed + 1) * 1103515245 + 12345) & 0x7fffffff) / 2147483648.0f;
            
            float theta = r1 * 2.0f * M_PI;
            float phi = acosf(2.0f * r2 - 1.0f);
            
            glm::vec3 dir(
                sinf(phi) * cosf(theta),
                sinf(phi) * sinf(theta),
                cosf(phi)
            );
            
            return position + dir * size;
        } else {
            float r1 = ((seed * 1103515245 + 12345) & 0x7fffffff) / 2147483648.0f;
            float r2 = (((seed + 1) * 1103515245 + 12345) & 0x7fffffff) / 2147483648.0f;
            float uOffset = (r1 - 0.5f) * size;
            float vOffset = (r2 - 0.5f) * size;
            
            return position + u * uOffset + v * vOffset;
        }
    }
};

vector<Light *> lights; ///< A list of point lights in the scene
vector<AreaLight *> areaLights; ///< A list of area lights in the scene
//glm::vec3 ambient_light(0.1,0.1,0.1);
// new ambient light
glm::vec3 ambient_light(0.001,0.001,0.001);
vector<Object *> objects; ///< A list of all objects in the scene

struct SmokeVolume {
    glm::vec3 position;
    glm::vec3 size;
    float density;
    float age;
};

vector<SmokeVolume> smokeVolumes;

void loadTexturesToMesh(Mesh* mesh, const std::string& baseName) {
    static Texture baseTexture, normalMap, aoMap, roughnessMap;
    if (baseTexture.load(baseName + "_basecolor.bmp")) mesh->setTexture(&baseTexture);
    if (normalMap.load(baseName + "_normal.bmp")) mesh->setNormalMap(&normalMap);
    if (aoMap.load(baseName + "_ao.bmp")) mesh->setAOMap(&aoMap);
    if (roughnessMap.load(baseName + "_roughness.bmp")) mesh->setRoughnessMap(&roughnessMap);
}

Material createWardMaterial(glm::vec3 diffuse, float roughnessX, float roughnessY) {
    Material mat;
    mat.ambient = glm::vec3(0.1f);
    mat.diffuse = diffuse;
    mat.specular = glm::vec3(0.5f);
    mat.shininess = 64.0f;
    mat.useWard = true;
    mat.wardRoughnessX = roughnessX;
    mat.wardRoughnessY = roughnessY;
    mat.wardSpecular = glm::vec3(0.9f, 0.9f, 1.0f);
    return mat;
}

void transformVertices(std::vector<glm::vec3>& verts, float scale, float c, float s) {
    for (auto &v : verts) {
        v *= scale;
        float x = v.x, z = v.z;
        v.x = c * x + s * z;
        v.z = -s * x + c * z;
    }
}

float getSmokeDensity(glm::vec3 pos, const SmokeVolume& smoke) {
    glm::vec3 toCenter = pos - smoke.position;
    float maxSize = glm::max(glm::max(smoke.size.x, smoke.size.y), smoke.size.z);
    if (glm::length(toCenter) > maxSize * 1.5f) return 0.0f;
    
    glm::vec3 localPos = toCenter / smoke.size;
    float dist = glm::length(localPos);
    
    float noiseScale = 3.0f;
    float noiseStrength = 0.6f;
    float noiseTime = smoke.age * 0.4f;
    
    float noise = PerlinNoise::fractalNoise3D(
        pos.x * noiseScale + smoke.position.x * 0.1f,
        pos.y * noiseScale + smoke.position.y * 0.1f + noiseTime,
        pos.z * noiseScale + smoke.position.z * 0.1f,
        5, 0.5f
    );
    
    noise = (noise + 1.0f) * 0.5f;
    
    float noiseOffset = (noise - 0.5f) * noiseStrength;
    float adjustedDist = dist - noiseOffset;
    
    if (adjustedDist > 1.0f) return 0.0f;
    if (adjustedDist < 0.0f) adjustedDist = 0.0f;
    
    float baseDensity = smoke.density * (1.0f - adjustedDist * adjustedDist);
    baseDensity = baseDensity * baseDensity;
    baseDensity *= 1.0f / (1.0f + smoke.age * 0.5f);
    
    float density = baseDensity * (0.7f + noise * 0.3f);
    
    return glm::clamp(density, 0.0f, 1.0f);
}

glm::vec3 traceVolumeRay(Ray ray, float maxDistance, float& outTransmittance) {
    if (smokeVolumes.empty()) {
        outTransmittance = 1.0f;
        return glm::vec3(0.0f);
    }
    
    float stepSize = 0.2f;
    int maxSteps = (int)(maxDistance / stepSize);
    float minTransmittance = 0.05f;
    
    glm::vec3 color(0.0f);
    float transmittance = 1.0f;
    
    float t = 0.0f;
    for (int i = 0; i < maxSteps && transmittance > minTransmittance; i++) {
        glm::vec3 pos = ray.origin + ray.direction * t;
        float totalDensity = 0.0f;
        
        for (const auto& smoke : smokeVolumes) {
            glm::vec3 toCenter = pos - smoke.position;
            float maxSize = glm::max(glm::max(smoke.size.x, smoke.size.y), smoke.size.z);
            if (glm::length(toCenter) > maxSize * 1.2f) continue;
            totalDensity += getSmokeDensity(pos, smoke);
        }
        
        if (totalDensity > 0.001f) {
            float absorption = totalDensity * stepSize * 2.0f;
            float scattering = totalDensity * stepSize * 0.5f;
            
            transmittance *= exp(-absorption);
            
            glm::vec3 smokeColor(0.7f, 0.65f, 0.6f);
            glm::vec3 scatteredLight = smokeColor * scattering;
            color += scatteredLight * transmittance;
        }
        
        t += stepSize;
    }
    
    outTransmittance = transmittance;
    return color;
}

/**
 Function to check if a point is in shadow from a point light source
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
        if (!objects[k]->castsShadow()) {
            continue;
        }
        
        Hit hit = objects[k]->intersect(shadowRay);
        // Early exit: if we find any intersection before reaching the light, point is in shadow
        if(hit.hit && hit.distance < shadowRay.tMax && hit.distance > 0.0f) {
            return true; // Point is in shadow
        }
    }
    return false; // Point is not in shadow - no objects block the path to light
}

/**
 Function to compute soft shadow factor from an area light source
 Uses multiple shadow samples to create soft shadows
 @param point The point to check
 @param areaLight The area light source
 @param numSamples Number of samples for soft shadow (more samples = smoother shadows)
 @return Shadow factor [0, 1], where 0 = fully shadowed, 1 = fully lit
 */
float computeSoftShadow(glm::vec3 point, const AreaLight& areaLight, int numSamples = 8) {
    int unblockedSamples = 0;
    
    for (int i = 0; i < numSamples; i++) {
        glm::vec3 lightPoint = areaLight.samplePoint((int)(point.x * 1000 + point.y * 1000 + point.z * 1000) + i);
        
        glm::vec3 lightDirection = lightPoint - point;
        float lightDistance = glm::length(lightDirection);
        lightDirection = glm::normalize(lightDirection);
        
        // Offset the ray origin slightly along the light direction to avoid self-intersection
        glm::vec3 offsetPoint = point + lightDirection * 0.001f;
        Ray shadowRay(offsetPoint, lightDirection);
        shadowRay.tMax = lightDistance - 0.002f;
        
        // Check for intersection
        bool blocked = false;
        for(int k = 0; k < objects.size(); k++){
            if (!objects[k]->castsShadow()) {
                continue;
            }
            
            Hit hit = objects[k]->intersect(shadowRay);
            if(hit.hit && hit.distance < shadowRay.tMax && hit.distance > 0.0f) {
                blocked = true;
                break;
            }
        }
        
        if (!blocked) {
            unblockedSamples++;
        }
    }
    
    return (float)unblockedSamples / (float)numSamples;
}

/**
 * Ward anisotropic reflectance model - compute specular reflection
 * @param light_direction Light direction (from point to light)
 * @param view_direction View direction (from point to camera)
 * @param normal Normal vector
 * @param tangent Tangent vector (anisotropic direction X)
 * @param bitangent Bitangent vector (anisotropic direction Y)
 * @param material Material parameters
 * @return Ward specular reflection color
 */
glm::vec3 WardSpecular(glm::vec3 light_direction, glm::vec3 view_direction, 
                       glm::vec3 normal, glm::vec3 tangent, glm::vec3 bitangent, 
                       Material material) {
    light_direction = glm::normalize(light_direction);
    view_direction = glm::normalize(view_direction);
    normal = glm::normalize(normal);
    tangent = glm::normalize(tangent);
    bitangent = glm::normalize(bitangent);
    
    glm::vec3 halfVector = glm::normalize(light_direction + view_direction);
    float cosThetaI = glm::clamp(glm::dot(normal, light_direction), 0.0f, 1.0f);
    float cosThetaR = glm::clamp(glm::dot(normal, view_direction), 0.0f, 1.0f);
    if (cosThetaI <= 0.0f || cosThetaR <= 0.0f) return glm::vec3(0.0f);
    
    float cosThetaH = glm::clamp(glm::dot(normal, halfVector), 0.0f, 1.0f);
    float sinThetaH = sqrtf(1.0f - cosThetaH * cosThetaH);
    float tanThetaH = (cosThetaH > 0.001f) ? sinThetaH / cosThetaH : 0.0f;
    
    float dotHT = glm::dot(halfVector, tangent);
    float dotHB = glm::dot(halfVector, bitangent);
    
    float alphaX = material.wardRoughnessX;
    float alphaY = material.wardRoughnessY;
    float hDotT2 = dotHT * dotHT;
    float hDotB2 = dotHB * dotHB;
    float hDotTB2 = hDotT2 + hDotB2;
    
    float alpha2 = 0.0f;
    if (hDotTB2 > 0.001f) {
        alpha2 = (hDotT2 * alphaX * alphaX + hDotB2 * alphaY * alphaY) / hDotTB2;
    } else {
        alpha2 = (alphaX * alphaX + alphaY * alphaY) * 0.5f;
    }
    
    if (alpha2 < 0.001f) alpha2 = 0.001f;
    
    float tanThetaH2 = tanThetaH * tanThetaH;
    float expTerm = expf(-tanThetaH2 / alpha2);
    
    float denominator = 4.0f * M_PI * alphaX * alphaY * sqrtf(cosThetaI * cosThetaR);
    
    if (denominator < 0.001f) denominator = 0.001f;
    float wardSpec = material.wardSpecular.r * expTerm / denominator;
    
    return material.wardSpecular * glm::vec3(wardSpec);
}

// Force lighting model: -1=auto, 0=Phong, 1=Ward
int forceLightingModel = -1;

/** Function for computing color of an object according to the Phong Model (or Ward Model for anisotropic materials)
 @param point A point belonging to the object for which the color is computer
 @param normal A normal vector the the point
 @param view_direction A normalized direction from the point to the viewer/camera
 @param material A material structure representing the material of the object
 @param tangent Optional tangent vector for Ward model (default: zero vector)
 @param bitangent Optional bitangent vector for Ward model (default: zero vector)
 @param object Optional pointer to object for texture access (default: nullptr)
 @param u Texture coordinate U (default: 0.0)
 @param v Texture coordinate V (default: 0.0)
*/
glm::vec3 PhongModel(glm::vec3 point, glm::vec3 normal, glm::vec3 view_direction, Material material, 
                     glm::vec3 tangent = glm::vec3(0.0f), glm::vec3 bitangent = glm::vec3(0.0f),
                     Object* object = nullptr, float u = 0.0f, float v = 0.0f){

	glm::vec3 color(0.0);
	
	bool isEmissive = (material.specular == glm::vec3(0.0f) && 
	                   glm::length(material.ambient - material.diffuse) < 0.01f &&
	                   glm::length(material.ambient) > 0.1f);
	if (isEmissive) {
		return glm::clamp(material.ambient, glm::vec3(0.0), glm::vec3(1.0));
	}
	float finalShininess = material.shininess;
	if (object && object->roughnessMap && object->roughnessMap->isLoaded()) {
		glm::vec3 roughnessSample = object->roughnessMap->sample(u, v);
		float roughness = (roughnessSample.r + roughnessSample.g + roughnessSample.b) / 3.0f;
		roughness = glm::clamp(roughness, 0.001f, 1.0f);
		
		float roughness4 = roughness * roughness * roughness * roughness;
		finalShininess = 0.5f / roughness4 - 0.5f;
		finalShininess = glm::max(finalShininess, 1.0f);
	}
	
	float aoValue = 1.0f;
	if (object && object->aoMap && object->aoMap->isLoaded()) {
		glm::vec3 aoSample = object->aoMap->sample(u, v);
		aoValue = (aoSample.r + aoSample.g + aoSample.b) / 3.0f;
		aoValue = glm::clamp(aoValue, 0.0f, 1.0f);
	}
	
	glm::vec3 finalNormal = normal;
	if (object && object->normalMap && object->normalMap->isLoaded() && 
	    glm::length(tangent) > 0.001f && glm::length(bitangent) > 0.001f) {
		
		glm::vec3 normalMapSample = object->normalMap->sample(u, v);
		glm::vec3 tangentSpaceNormal = normalMapSample * 2.0f - 1.0f;
		tangentSpaceNormal = glm::normalize(tangentSpaceNormal);
		
		tangent = glm::normalize(tangent);
		bitangent = glm::normalize(bitangent);
		normal = glm::normalize(normal);
		
		tangent = glm::normalize(tangent - glm::dot(tangent, normal) * normal);
		bitangent = glm::normalize(bitangent - glm::dot(bitangent, normal) * normal);
		
		if (glm::dot(glm::cross(tangent, bitangent), normal) < 0.0f) {
			bitangent = -bitangent;
		}
		finalNormal = tangentSpaceNormal.x * tangent + 
		              tangentSpaceNormal.y * bitangent + 
		              tangentSpaceNormal.z * normal;
		finalNormal = glm::normalize(finalNormal);
	}
	
	for(int light_num = 0; light_num < lights.size(); light_num++){
		if(isInShadow(point, lights[light_num]->position)) continue;

		glm::vec3 light_direction = glm::normalize(lights[light_num]->position - point);
		float NdotL = glm::clamp(glm::dot(finalNormal, light_direction), 0.0f, 1.0f);

		glm::vec3 diffuse_color = material.diffuse;
		if (object && object->texture && object->texture->isLoaded()) {
			diffuse_color = object->texture->sample(u, v);
		}
		glm::vec3 diffuse = diffuse_color * glm::vec3(NdotL);
		
		glm::vec3 specular;
		bool useWard = false;
		if (forceLightingModel == 1) {
			useWard = (glm::length(tangent) > 0.001f && glm::length(bitangent) > 0.001f);
		} else if (forceLightingModel == 0) {
			useWard = false;
		} else {
			useWard = (material.useWard && glm::length(tangent) > 0.001f && glm::length(bitangent) > 0.001f);
		}
		
		if (useWard) {
			specular = WardSpecular(light_direction, view_direction, finalNormal, tangent, bitangent, material);
		} else {
			glm::vec3 reflected_direction = glm::reflect(-light_direction, finalNormal);
			float VdotR = glm::clamp(glm::dot(view_direction, reflected_direction), 0.0f, 1.0f);
			specular = material.specular * glm::vec3(pow(VdotR, finalShininess));
		}
		
        float r = glm::distance(point,lights[light_num]->position);
        r = max(r, 0.1f);
        color += lights[light_num]->color * (diffuse + specular) * 0.3f / (r * r);
	}
	
	for(int light_num = 0; light_num < areaLights.size(); light_num++){
		const AreaLight& areaLight = *areaLights[light_num];
		float shadowFactor = computeSoftShadow(point, areaLight, 8);
		
		if (shadowFactor > 0.0f) {
			glm::vec3 light_direction = glm::normalize(areaLight.position - point);
			float NdotL = glm::clamp(glm::dot(finalNormal, light_direction), 0.0f, 1.0f);

			glm::vec3 diffuse_color = material.diffuse;
			if (object && object->texture && object->texture->isLoaded()) {
				diffuse_color = object->texture->sample(u, v);
			}
			glm::vec3 diffuse = diffuse_color * glm::vec3(NdotL);
			
			glm::vec3 specular;
			bool useWard = false;
			if (forceLightingModel == 1) {
				useWard = (glm::length(tangent) > 0.001f && glm::length(bitangent) > 0.001f);
			} else if (forceLightingModel == 0) {
				useWard = false;
			} else {
				useWard = (material.useWard && glm::length(tangent) > 0.001f && glm::length(bitangent) > 0.001f);
			}
			
			if (useWard) {
				specular = WardSpecular(light_direction, view_direction, finalNormal, tangent, bitangent, material);
			} else {
				glm::vec3 reflected_direction = glm::reflect(-light_direction, finalNormal);
				float VdotR = glm::clamp(glm::dot(view_direction, reflected_direction), 0.0f, 1.0f);
				specular = material.specular * glm::vec3(pow(VdotR, finalShininess));
			}
			
			float r = glm::distance(point, areaLight.position);
			r = max(r, 0.1f);
			float areaFactor = areaLight.isSphere ? (4.0f * M_PI * areaLight.size * areaLight.size) : (areaLight.size * areaLight.size);
			float intensity = 2.0f;
			float attenuation = powf(r, areaLight.attenuationPower);
			color += areaLight.color * (diffuse + specular) * intensity * shadowFactor / attenuation;
		}
	}
	
	color += ambient_light * material.ambient * aoValue;
	color = glm::clamp(color, glm::vec3(0.0), glm::vec3(1.0));
	return color;
}

/**
 * Generate random sample point on aperture (for depth-of-field effect)
 * @param aperture Aperture radius
 * @param seed Random seed (based on pixel position)
 * @return Random point on aperture (near camera plane, Z=0)
 */
glm::vec3 sampleAperture(float aperture, int seed) {
    float r1 = ((seed * 1103515245 + 12345) & 0x7fffffff) / 2147483648.0f;
    float r2 = (((seed + 1) * 1103515245 + 12345) & 0x7fffffff) / 2147483648.0f;
    
    float r = sqrtf(r1) * aperture;
    float theta = r2 * 2.0f * M_PI;
    
    return glm::vec3(r * cosf(theta), r * sinf(theta), 0.0f);
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
	
	float volumeDistance = closest_hit.hit ? closest_hit.distance : 50.0f;
	float transmittance = 1.0f;
	glm::vec3 volumeColor = traceVolumeRay(ray, volumeDistance, transmittance);
	
	if(closest_hit.hit){
		glm::vec3 tangent = closest_hit.tangent;
		glm::vec3 bitangent = closest_hit.bitangent;
		glm::vec3 objectColor = PhongModel(closest_hit.intersection, closest_hit.normal, glm::normalize(-ray.direction), closest_hit.object->getMaterial(), tangent, bitangent, closest_hit.object, closest_hit.u, closest_hit.v);
		
		color = volumeColor + objectColor * transmittance;
	}else{
		color = volumeColor;
	}
	
	return color;
}
// Scene definition
void sceneDefinition (float elevationAngle = -20.0f, float recoilDistance = 0.0f, float smokeIntensity = 0.0f, 
                       float waterTime = 0.0f, float waveHeight = 0.15f, float waveFrequency = 0.5f){
	smokeVolumes.clear();
	areaLights.clear();
	lights.push_back(new Light(glm::vec3(0, 5, 10), glm::vec3(0.6, 0.6, 0.6)));
	
    Material gray_diffuse;
    gray_diffuse.ambient = glm::vec3(0.07f);
    gray_diffuse.diffuse = glm::vec3(0.7f);
    gray_diffuse.specular = glm::vec3(0.2f);
    gray_diffuse.shininess = 32.0f;

    Material waterMaterial;
    waterMaterial.ambient = glm::vec3(0.05f, 0.1f, 0.15f);
    waterMaterial.diffuse = glm::vec3(0.2f, 0.4f, 0.6f);
    waterMaterial.specular = glm::vec3(0.8f, 0.9f, 1.0f);
    waterMaterial.shininess = 128.0f;
    
    WaterPlane* water = new WaterPlane(
        glm::vec3(0.0f, -3.0f, 0.0f),
        glm::vec3(0.0f, 1.0f, 0.0f),
        50.0f,
        waterMaterial,
        waveHeight,
        waveFrequency,
        waterTime
    );
    
    static Texture waterTexture;
    if (waterTexture.load("./water2.bmp")) {
        water->setTexture(&waterTexture);
    }
    
    objects.push_back(water);

    const float meshScale = 0.4f; 
    const float angleDeg  = 135.0f;
    const float angleRad  = glm::radians(angleDeg);
    const float c = std::cos(angleRad);
    const float s = std::sin(angleRad);

    {
        std::vector<glm::vec3> verts; 
        std::vector<glm::ivec3> faces;
        loadOBJ("./meshes/barrel.obj", verts, faces);

        float barrelLength = 0.0f, maxZ = 0.0f;
        if (!verts.empty()) {
            float minZ = verts[0].z;
            maxZ = verts[0].z;
            for (const auto &v : verts) {
                if (v.z < minZ) minZ = v.z;
                if (v.z > maxZ) maxZ = v.z;
            }
            barrelLength = (maxZ - minZ) * meshScale;
        }

        glm::vec3 T_barrel_base(-0.2f, -2.0f, 15.4f);
        glm::vec3 forwardDir(s, 0.0f, c);
        forwardDir = glm::normalize(forwardDir);
        
        glm::vec3 cylinderAxis(-c, 0.0f, s);
        cylinderAxis = glm::normalize(cylinderAxis);
        glm::vec3 cylinderCenter = T_barrel_base + forwardDir * 3.0f - forwardDir * 0.3f;
        cylinderCenter.y += 0.4f;
        
        glm::vec3 rotationAxisPoint(cylinderCenter.x, T_barrel_base.y, cylinderCenter.z);
        glm::vec3 rotationAxis = cylinderAxis;
        float angleRad = glm::radians(elevationAngle);
        
        glm::vec3 axis = glm::normalize(rotationAxis);
        float cosA = cosf(angleRad);
        float sinA = sinf(angleRad);
        
        glm::mat3 K(0.0f, -axis.z, axis.y,
                    axis.z, 0.0f, -axis.x,
                    -axis.y, axis.x, 0.0f);
        glm::mat3 K2 = K * K;
        glm::mat3 rotationMatrix = glm::mat3(1.0f) + sinA * K + (1.0f - cosA) * K2;
        
        const float barrelOffset = 0.3f;
        glm::vec3 T_barrel = T_barrel_base + forwardDir * barrelOffset - forwardDir * recoilDistance;
        
        std::vector<glm::vec3> transformedVerts = verts;
        for (auto &v : transformedVerts) {
            v *= meshScale;
            float x = v.x, z = v.z;
            v.x = c * x + s * z;
            v.z = -s * x + c * z;
            glm::vec3 vWorld = T_barrel + v;
            glm::vec3 vRelative = vWorld - rotationAxisPoint;
            v = rotationAxisPoint + rotationMatrix * vRelative - T_barrel;
        }
        
        if (!transformedVerts.empty() && !faces.empty()) {
            Mesh* barrelMesh = new Mesh(transformedVerts, faces, gray_diffuse, T_barrel);
            loadTexturesToMesh(barrelMesh, "./Sci-fi_Metal_Walkway_001");
            objects.push_back(barrelMesh);
        }
        
        if (smokeIntensity > 0.0f) {
            glm::vec3 rotatedBarrelDirection = glm::normalize(rotationMatrix * forwardDir);
            glm::vec3 muzzleLocalPos(0.0f, 0.0f, maxZ * meshScale);
            glm::vec3 muzzleRotated(c * muzzleLocalPos.x + s * muzzleLocalPos.z, muzzleLocalPos.y, -s * muzzleLocalPos.x + c * muzzleLocalPos.z);
            glm::vec3 muzzlePosition = rotationAxisPoint + rotationMatrix * (T_barrel + muzzleRotated - rotationAxisPoint);
            
            glm::vec3 flameColor(1.0f, 0.6f, 0.1f);
            float flameIntensity = smokeIntensity;
            
            if (flameIntensity > 0.0f) {
                glm::vec3 flameNormal = rotatedBarrelDirection;
                glm::vec3 up(0.0f, 1.0f, 0.0f);
                glm::vec3 flameU = glm::normalize(glm::cross(flameNormal, up));
                if (glm::length(flameU) < 0.1f) {
                    flameU = glm::normalize(glm::cross(flameNormal, glm::vec3(1.0f, 0.0f, 0.0f)));
                }
                glm::vec3 flameV = glm::normalize(glm::cross(flameNormal, flameU));
                float flameSize = 0.3f + flameIntensity * 0.4f;
                
                glm::vec3 flameBrightness = flameColor * (2.0f + flameIntensity * 3.0f);
                areaLights.push_back(new AreaLight(muzzlePosition, flameBrightness, flameU, flameV, flameSize, 2.0f));
                
                Material flameMaterial;
                flameMaterial.ambient = flameBrightness;
                flameMaterial.diffuse = flameBrightness;
                flameMaterial.specular = glm::vec3(0.0f);
                flameMaterial.shininess = 1.0f;
                objects.push_back(new Disc(muzzlePosition + rotatedBarrelDirection * 0.05f, flameNormal, flameSize, flameMaterial));
            }
            
            int numSmokeVolumes = 1 + (int)(smokeIntensity * 2);
            for (int i = 0; i < numSmokeVolumes; i++) {
                float t = (float)i / (float)(numSmokeVolumes - 1);
                float angle = t * 4.0f * M_PI;
                float horizontalSpread = t * t * smokeIntensity * 0.6f;
                
                glm::vec3 smokePos = muzzlePosition + 
                    glm::vec3(0.0f, 1.0f, 0.0f) * (t * smokeIntensity * 1.5f) +
                    rotatedBarrelDirection * (t * smokeIntensity * 0.8f);
                smokePos.x += cosf(angle) * horizontalSpread;
                smokePos.z += sinf(angle) * horizontalSpread;
                
                SmokeVolume smoke;
                smoke.position = smokePos;
                smoke.size = glm::vec3(0.3f + t * smokeIntensity * 0.8f, 0.4f + t * smokeIntensity * 1.0f, 0.3f + t * smokeIntensity * 0.8f);
                smoke.density = smokeIntensity * (1.0f - t * 0.6f);
                smoke.age = t * 0.5f;
                smokeVolumes.push_back(smoke);
            }
        }
        
        
    }

    {
        std::vector<glm::vec3> verts; 
        std::vector<glm::ivec3> faces;
        loadOBJ("./meshes/turret.obj", verts, faces);

        std::vector<glm::vec3> transformedVerts = verts;
        transformVertices(transformedVerts, meshScale, c, s);
        glm::vec3 T_turret(0.0f, -2.0f, 15.0f);
        if (!transformedVerts.empty() && !faces.empty()) {
            Mesh* turretMesh = new Mesh(transformedVerts, faces, gray_diffuse, T_turret);
            loadTexturesToMesh(turretMesh, "./Sci-fi_Metal_Walkway_001");
            objects.push_back(turretMesh);
        }
    }
    
    Material wardMaterial1 = createWardMaterial(glm::vec3(0.3f, 0.3f, 0.4f), 0.05f, 0.25f);
    Material wardMaterial2 = createWardMaterial(glm::vec3(0.3f, 0.3f, 0.4f), 0.25f, 0.05f);
    
    glm::vec3 sphere1Pos(-3.0f, 2.0f, 22.0f);
    glm::vec3 sphere2Pos(3.0f, 2.0f, 22.0f);
    float sphereRadius = 0.8f;
    
    Sphere* sphere1 = new Sphere(sphere1Pos, sphereRadius, wardMaterial1);
    Sphere* sphere2 = new Sphere(sphere2Pos, sphereRadius, wardMaterial2);
    
    objects.push_back(sphere1);
    objects.push_back(sphere2);
    
    glm::vec3 moonPosition(-15.0f, sphere1Pos.y + 8.0f, sphere1Pos.z);
    glm::vec3 moonColor(1.0f, 0.95f, 0.8f);
    float moonIntensity = 0.23f;
    float moonSize = 1.5f;
    
    glm::vec3 moonNormal = glm::normalize(glm::vec3(0.0f) - moonPosition);
    glm::vec3 moonU = glm::normalize(glm::cross(moonNormal, glm::vec3(0.0f, 1.0f, 0.0f)));
    if (glm::length(moonU) < 0.1f) {
        moonU = glm::normalize(glm::cross(moonNormal, glm::vec3(1.0f, 0.0f, 0.0f)));
    }
    glm::vec3 moonV = glm::normalize(glm::cross(moonU, moonNormal));
    glm::vec3 moonBrightness = moonColor * moonIntensity * 1.4f;
    
    areaLights.push_back(new AreaLight(moonPosition, moonColor * moonIntensity, moonU, moonV, moonSize, 1.2f));
    
    Material moonMaterial;
    moonMaterial.ambient = moonBrightness;
    moonMaterial.diffuse = moonBrightness;
    moonMaterial.specular = glm::vec3(0.0f);
    moonMaterial.shininess = 1.0f;
    objects.push_back(new Disc(moonPosition, moonNormal, moonSize, moonMaterial));
}
glm::vec3 toneMapping(glm::vec3 intensity){
	float gamma = 1.0/2.0;
	float alpha = 12.0f;
	return glm::clamp(alpha * glm::pow(intensity, glm::vec3(gamma)), glm::vec3(0.0), glm::vec3(1.0));
}

// Render frame
void renderFrame(Image& image, int width, int height, float fov, bool useAntiAliasing = true, bool useDepthOfField = true, float apertureSize = 0.1f, float focalDist = 15.0f) {
    float s = 2*tan(0.5*fov/180*M_PI)/width;
    float X = -s * width / 2;
    float Y = s * height / 2;
    
    int samplesPerPixel = useAntiAliasing ? 4 : 1;
    const float invSamples = 1.0f / samplesPerPixel;
    const float focalDistance = focalDist;
    const float aperture = useDepthOfField ? apertureSize : 0.0f;
    
    for(int i = 0; i < width ; i++) {
        for(int j = 0; j < height ; j++){
            glm::vec3 color(0.0f);
            
            if (useAntiAliasing) {
                int sampleIndex = 0;
                for (int sy = 0; sy < 2; sy++) {
                    for (int sx = 0; sx < 2; sx++) {
                        float offsetX = (sx + 0.5f) * 0.5f - 0.5f;
                        float offsetY = (sy + 0.5f) * 0.5f - 0.5f;
                        
                        float dx = X + (i + offsetX) * s;
                        float dy = Y - (j + offsetY) * s;
                        glm::vec3 originalDirection = glm::normalize(glm::vec3(dx, dy, 1.0f));
                        
                        Ray ray;
                        if (useDepthOfField && aperture > 0.0f) {
                            glm::vec3 focalPoint = originalDirection * focalDistance;
                            int seed = (i * height + j) * samplesPerPixel + sampleIndex;
                            glm::vec3 apertureOffset = sampleAperture(aperture, seed);
                            glm::vec3 rayDirection = glm::normalize(focalPoint - apertureOffset);
                            ray = Ray(apertureOffset, rayDirection);
                        } else {
                            ray = Ray(glm::vec3(0.0f), originalDirection);
                        }
                        
                        color += trace_ray(ray);
                        sampleIndex++;
                    }
                }
                color *= invSamples;
            } else {
                float dx = X + i * s;
                float dy = Y - j * s;
                glm::vec3 originalDirection = glm::normalize(glm::vec3(dx, dy, 1.0f));
                
                Ray ray;
                if (useDepthOfField && aperture > 0.0f) {
                    glm::vec3 focalPoint = originalDirection * focalDistance;
                    int seed = i * height + j;
                    glm::vec3 apertureOffset = sampleAperture(aperture, seed);
                    glm::vec3 rayDirection = glm::normalize(focalPoint - apertureOffset);
                    ray = Ray(apertureOffset, rayDirection);
                } else {
                    ray = Ray(glm::vec3(0.0f), originalDirection);
                }
                
                color = trace_ray(ray);
            }
            image.setPixel(i, j, toneMapping(color));
        }
    }
}

int main(int argc, const char * argv[]) {
    
    // Check for --compare-aa mode
    if (argc > 1 && string(argv[1]) == "--compare-aa") {
        int width = 1920;
        int height = 1080;
        float fov = 90;
        
        sceneDefinition(-20.0f, 0.0f, 0.0f, 5.0f, 0.15f, 0.5f);
        
        clock_t t1 = clock();
        Image imageNoAA(width, height);
        renderFrame(imageNoAA, width, height, fov, false);
        t1 = clock() - t1;
        imageNoAA.writeImage("./result_no_aa.ppm");
        cout << "No AA rendering time: " << ((double)t1) / CLOCKS_PER_SEC << " seconds" << endl;
        
        clock_t t2 = clock();
        Image imageAA(width, height);
        renderFrame(imageAA, width, height, fov, true);
        t2 = clock() - t2;
        imageAA.writeImage("./result_with_aa.ppm");
        cout << "With AA rendering time: " << ((double)t2) / CLOCKS_PER_SEC << " seconds" << endl;
        
        return 0;
    }
    
    // Check for --compare-dof mode
    if (argc > 1 && string(argv[1]) == "--compare-dof") {
        int width = 1920, height = 1080;
        float fov = 90;
        sceneDefinition(-20.0f, 0.0f, 0.0f, 5.0f, 0.15f, 0.5f);
        
        clock_t t1 = clock();
        Image imageNoDOF(width, height);
        renderFrame(imageNoDOF, width, height, fov, true, false);
        t1 = clock() - t1;
        imageNoDOF.writeImage("./result_no_dof.ppm");
        cout << "No DOF rendering time: " << ((double)t1) / CLOCKS_PER_SEC << " seconds" << endl;
        
        const float strongAperture = 0.5f;
        const float focalDistance = 22.5f;
        clock_t t2 = clock();
        Image imageDOF(width, height);
        renderFrame(imageDOF, width, height, fov, true, true, strongAperture, focalDistance);
        t2 = clock() - t2;
        imageDOF.writeImage("./result_with_dof.ppm");
        cout << "With DOF rendering time: " << ((double)t2) / CLOCKS_PER_SEC << " seconds" << endl;
        
        return 0;
    }
    
    if (argc > 1 && string(argv[1]) == "--compare-phong-ward") {
        int width = 1920, height = 1080;
        float fov = 90;
        sceneDefinition(-20.0f, 0.0f, 0.0f, 5.0f, 0.15f, 0.5f);
        
        forceLightingModel = 0;
        Image imagePhong(width, height);
        renderFrame(imagePhong, width, height, fov, true, false);
        imagePhong.writeImage("./result_phong.ppm");
        
        forceLightingModel = 1;
        Image imageWard(width, height);
        renderFrame(imageWard, width, height, fov, true, false);
        imageWard.writeImage("./result_ward.ppm");
        forceLightingModel = -1;
        
        return 0;
    }
    
    // Check for --animation mode
    if (argc > 1 && string(argv[1]) == "--animation") {
        std::vector<glm::vec3> verts;
        std::vector<glm::ivec3> faces;
        loadOBJ("./meshes/barrel.obj", verts, faces);
        
        float barrelLength = 0.0f;
        const float meshScale = 0.4f;
        if (!verts.empty()) {
            float minZ = verts[0].z;
            float maxZ = verts[0].z;
            for (const auto &v : verts) {
                if (v.z < minZ) minZ = v.z;
                if (v.z > maxZ) maxZ = v.z;
            }
            barrelLength = (maxZ - minZ) * meshScale;
        }
        float maxRecoil = barrelLength / 20.0f;
        
        int width = 1280;
        int height = 720;
        float fov = 90;
        int numFrames = 30;
        int elevationStartFrame = 15;
        int firingFrame = 22;
        int elevationFrames = firingFrame - elevationStartFrame;
        int recoilFrames = numFrames - firingFrame;
        float maxAngle = -20.0f;
        
        clock_t animationStart = clock();
        
        for (int frame = 0; frame < numFrames; frame++) {
            clock_t frameStart = clock();
            float elevationAngle = 0.0f;
            float recoilDistance = 0.0f;
            float smokeIntensity = 0.0f;
            
            if (frame < elevationStartFrame) {
                elevationAngle = 0.0f;
                recoilDistance = 0.0f;
                smokeIntensity = 0.0f;
            } else if (frame < firingFrame) {
                int elevationFrameIndex = frame - elevationStartFrame;
                float t = (float)elevationFrameIndex / (float)(elevationFrames - 1);
                elevationAngle = maxAngle * t;
                recoilDistance = 0.0f;
                smokeIntensity = 0.0f;
            } else {
                elevationAngle = maxAngle;
                int recoilFrameIndex = frame - firingFrame;
                float recoilT = (float)recoilFrameIndex / (float)(recoilFrames - 1);
                
                float smokeT = (float)recoilFrameIndex / (float)(recoilFrames - 1);
                float smoothT = smokeT * smokeT * (3.0f - 2.0f * smokeT);
                smokeIntensity = pow(1.0f - smoothT, 1.5f);
                smokeIntensity = glm::clamp(smokeIntensity, 0.0f, 1.0f);
                
                if (recoilT < 0.3f) {
                    recoilDistance = maxRecoil * (recoilT / 0.3f);
                } else if (recoilT < 0.5f) {
                    recoilDistance = maxRecoil;
                } else {
                    float t = (recoilT - 0.5f) / 0.5f;
                    float easeOut = 1.0f - (1.0f - t) * (1.0f - t);
                    recoilDistance = maxRecoil * (1.0f - easeOut);
                }
            }
            
            for (size_t i = 0; i < objects.size(); i++) delete objects[i];
            for (size_t i = 0; i < lights.size(); i++) delete lights[i];
            objects.clear();
            lights.clear();
            
            float currentTime = (float)frame * 0.2f;
            float dynamicWaveHeight = 0.15f + 0.05f * sinf(currentTime * 0.3f);
            float dynamicWaveFrequency = 0.5f + 0.2f * cosf(currentTime * 0.4f);
            
            sceneDefinition(elevationAngle, recoilDistance, smokeIntensity, currentTime, 
                           dynamicWaveHeight, dynamicWaveFrequency);
            
            Image image(width, height);
            renderFrame(image, width, height, fov, true);
            char filename[256];
            snprintf(filename, sizeof(filename), "./frame_%03d.ppm", frame);
            image.writeImage(filename);
            
            clock_t frameEnd = clock();
            double frameTime = ((double)(frameEnd - frameStart)) / CLOCKS_PER_SEC;
            cout << "Frame " << frame + 1 << "/" << numFrames << " - " << frameTime << " seconds" << endl;
        }
        
        clock_t animationEnd = clock();
        double totalTime = ((double)(animationEnd - animationStart)) / CLOCKS_PER_SEC;
        cout << "Animation complete: " << numFrames << " frames in " << totalTime << " seconds" << endl;
        return 0;
    }

    clock_t t = clock(); // variable for keeping the time of the rendering

    int width = 2560; //width of the image (increase for better quality)
    int height = 1440; // height of the image (increase for better quality)
    float fov = 90; // field of view

	float waterTime = 5.0f;
	float waveHeight = 0.15f;
	float waveFrequency = 0.5f;
	sceneDefinition(-20.0f, 0.0f, 0.0f, waterTime, waveHeight, waveFrequency);

	Image image(width, height);
	renderFrame(image, width, height, fov, true);
	
    t = clock() - t;
    double elapsedTime = ((double)t) / CLOCKS_PER_SEC;
    cout << "Rendering time: " << elapsedTime << " seconds" << endl;

	// Writing the final results of the rendering
	if (argc == 2){
		image.writeImage(argv[1]);
	}else{
		image.writeImage("./result.ppm");
	}

	
    return 0;
}