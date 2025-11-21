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
#include "glm/glm.hpp"
#include "glm/gtx/transform.hpp"

#include "Image.h"
#include "Material.h"

using namespace std;

/**
 Class representing a single ray.
 */
class Ray{
public:
    glm::vec3 origin; ///< Origin of the ray
    glm::vec3 direction; ///< Direction of the ray
	/**
	 Contructor of the ray
	 @param origin Origin of the ray
	 @param direction Direction of the ray
	 */
    Ray(glm::vec3 origin, glm::vec3 direction) : origin(origin), direction(direction){
    }
};


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
    glm::vec3 n0; 
    glm::vec3 n1;
    glm::vec3 n2;
    bool hasVertexNormals = false;

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
    Triangle(const glm::vec3 &a, const glm::vec3 &b, const glm::vec3 &c,
             const glm::vec3 &na, const glm::vec3 &nb, const glm::vec3 &nc,
             const Material &mat){
        v0 = a; v1 = b; v2 = c;
        faceNormal = glm::normalize(glm::cross(v1 - v0, v2 - v0));
        n0 = glm::normalize(na);
        n1 = glm::normalize(nb);
        n2 = glm::normalize(nc);
        hasVertexNormals = true;
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
        if (t <= 0.0f) return hit;

        hit.hit = true;
        hit.distance = t;
        hit.intersection = ray.origin + t * ray.direction;
        if(hasVertexNormals){
            float w = 1.0f - u - v;
            glm::vec3 ns = glm::normalize(w*n0 + u*n1 + v*n2);
            hit.normal = ns;
        } else {
            hit.normal = faceNormal;
        }
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

static void loadOBJ_VN(const std::string &path,
                       std::vector<glm::vec3> &out_vertices,
                       std::vector<glm::vec3> &out_normals,
                       std::vector<glm::ivec3> &out_face_v,
                       std::vector<glm::ivec3> &out_face_n){
    out_vertices.clear();
    out_normals.clear();
    out_face_v.clear();
    out_face_n.clear();
    std::ifstream in(path);
    std::string line;
    while(std::getline(in, line)){
        if(line.empty() || line[0] == '#') continue;
        std::istringstream iss(line);
        std::string tag; iss >> tag;
        if(tag == "v"){
            float x, y, z; iss >> x >> y >> z;
            out_vertices.push_back(glm::vec3(x,y,z));
        } else if(tag == "vn"){
            float x, y, z; iss >> x >> y >> z;
            out_normals.push_back(glm::normalize(glm::vec3(x,y,z)));
        } else if(tag == "f"){
            std::vector<int> vind;
            std::vector<int> nind;
            std::string tok;
            while(iss >> tok){
                int vi = 0, ni = 0;
                // formats: v, v//vn, v/vt, v/vt/vn
                size_t p1 = tok.find('/');
                if(p1 == std::string::npos){
                    vi = std::stoi(tok);
                } else {
                    std::string a = tok.substr(0, p1);
                    vi = std::stoi(a);
                    size_t p2 = tok.find('/', p1+1);
                    if(p2 != std::string::npos){
                        std::string c = tok.substr(p2+1);
                        if(!c.empty()) ni = std::stoi(c);
                    } else {
                        // v/vt form, no normal index
                        ni = 0;
                    }
                }
                vind.push_back(vi);
                nind.push_back(ni);
            }
            if(vind.size() >= 3){
                for(size_t k=1; k+1<vind.size(); ++k){
                    out_face_v.push_back(glm::ivec3(vind[0]-1, vind[k]-1, vind[k+1]-1));
                    // if normal index is missing, store -1
                    int n0 = nind[0] ? (nind[0]-1) : -1;
                    int n1 = nind[k] ? (nind[k]-1) : -1;
                    int n2 = nind[k+1] ? (nind[k+1]-1) : -1;
                    out_face_n.push_back(glm::ivec3(n0, n1, n2));
                }
            }
        }
    }
}

/** Mesh object that holds triangles parsed from OBJ */
class Mesh : public Object{

private:
    std::vector<Triangle> triangles;

public:
    Mesh(const std::vector<glm::vec3> &vertices,
         const std::vector<glm::ivec3> &faces,
         const Material &mat,
         const glm::vec3 &translate){
        material = mat;
        triangles.reserve(faces.size());
        for(const auto &f : faces){
            glm::vec3 a = vertices[f.x] + translate;
            glm::vec3 b = vertices[f.y] + translate;
            glm::vec3 c = vertices[f.z] + translate;
            triangles.emplace_back(a,b,c, material);
        }
    }

    Mesh(const std::vector<glm::vec3> &vertices,
         const std::vector<glm::vec3> &normals,
         const std::vector<glm::ivec3> &faces_v,
         const std::vector<glm::ivec3> &faces_n,
         const Material &mat,
         const glm::vec3 &translate){
        material = mat;
        triangles.reserve(faces_v.size());
        for(size_t i=0;i<faces_v.size();++i){
            const glm::ivec3 &fv = faces_v[i];
            const glm::ivec3 &fn = faces_n[i];
            glm::vec3 a = vertices[fv.x] + translate;
            glm::vec3 b = vertices[fv.y] + translate;
            glm::vec3 c = vertices[fv.z] + translate;
            if(fn.x>=0 && fn.y>=0 && fn.z>=0 && fn.x < (int)normals.size() && fn.y < (int)normals.size() && fn.z < (int)normals.size()){
                triangles.emplace_back(a,b,c, normals[fn.x], normals[fn.y], normals[fn.z], material);
            } else {
                triangles.emplace_back(a,b,c, material);
            }
        }
    }

    Hit intersect(Ray ray){
        Hit closest; closest.hit = false; closest.distance = INFINITY; closest.object = this;
        for(auto &tri : triangles){
            Hit h = tri.intersect(ray);
            if(h.hit && h.distance < closest.distance){
                closest = h;
            }
        }
        return closest;
    }
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


/** Function for computing color of an object according to the Phong Model
 @param point A point belonging to the object for which the color is computer
 @param normal A normal vector the the point
 @param view_direction A normalized direction from the point to the viewer/camera
 @param material A material structure representing the material of the object
*/
glm::vec3 PhongModel(glm::vec3 point, glm::vec3 normal, glm::vec3 view_direction, Material material){

	glm::vec3 color(0.0);
	for(int light_num = 0; light_num < lights.size(); light_num++){

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
    objects.push_back(new Plane(glm::vec3(0,-3,0), glm::vec3(0.0,1,0)));
    objects.push_back(new Plane(glm::vec3(0,1,30), glm::vec3(0.0,0.0,-1.0), green_diffuse));
    objects.push_back(new Plane(glm::vec3(-15,1,0), glm::vec3(1.0,0.0,0.0), red_diffuse));
    objects.push_back(new Plane(glm::vec3(15,1,0), glm::vec3(-1.0,0.0,0.0), blue_diffuse));
    objects.push_back(new Plane(glm::vec3(0,27,0), glm::vec3(0.0,-1,0)));
    objects.push_back(new Plane(glm::vec3(0,1,-0.01), glm::vec3(0.0,0.0,1.0), green_diffuse));
    // Load a mesh and place it in the scene (translated away from the camera)
    std::vector<glm::vec3> verts; std::vector<glm::vec3> norms; std::vector<glm::ivec3> faces; std::vector<glm::ivec3> facesN;
    loadOBJ_VN("./meshes/bunny_with_normals.obj", verts, norms, faces, facesN);
    Material gray_diffuse;
    gray_diffuse.ambient = glm::vec3(0.07f);
    gray_diffuse.diffuse = glm::vec3(0.7f);
    gray_diffuse.specular = glm::vec3(0.2f);
    gray_diffuse.shininess = 32.0f;
    // Place bunny slightly above the floor (y ~ -2) and forward (z ~ 10)
    glm::vec3 meshOffset(0.0f, -2.0f, 10.0f);
    if(!verts.empty() && !faces.empty()){
        objects.push_back(new Mesh(verts, norms, faces, facesN, gray_diffuse, meshOffset));
    }
	// Load armadillo mesh
	std::vector<glm::vec3> verts2; std::vector<glm::vec3> norms2; std::vector<glm::ivec3> faces2; std::vector<glm::ivec3> facesN2;
	loadOBJ_VN("./meshes/armadillo_with_normals.obj", verts2, norms2, faces2, facesN2);
	glm::vec3 off2(-5.0f, -2.0f, 12.0f);
	if(!verts2.empty() && !faces2.empty()){
		objects.push_back(new Mesh(verts2, norms2, faces2, facesN2, gray_diffuse, off2));
	}
	// Load lucy mesh
	std::vector<glm::vec3> verts3; std::vector<glm::vec3> norms3; std::vector<glm::ivec3> faces3; std::vector<glm::ivec3> facesN3;
	loadOBJ_VN("./meshes/lucy_with_normals.obj", verts3, norms3, faces3, facesN3);
	glm::vec3 off3(5.0f, -2.0f, 12.0f);
	if(!verts3.empty() && !faces3.empty()){
		objects.push_back(new Mesh(verts3, norms3, faces3, facesN3, gray_diffuse, off3));
	}
	// Load horseshoe mesh (without normals)
	std::vector<glm::vec3> vertsHorse; std::vector<glm::ivec3> facesHorse;
	glm::vec3 offHorse(0.0f, -2.0f, 8.0f);
	loadOBJ("./meshes/horseshoe.obj", vertsHorse, facesHorse);
	if(!vertsHorse.empty() && !facesHorse.empty()){
		objects.push_back(new Mesh(vertsHorse, facesHorse, gray_diffuse, offHorse));
	}
}
glm::vec3 toneMapping(glm::vec3 intensity){
	float gamma = 1.0/2.0;
	float alpha = 12.0f;
	return glm::clamp(alpha * glm::pow(intensity, glm::vec3(gamma)), glm::vec3(0.0), glm::vec3(1.0));
}
int main(int argc, const char * argv[]) {

    clock_t t = clock(); // variable for keeping the time of the rendering

    int width = 640; //width of the image (increase for better quality)
    int height = 480; // height of the image (increase for better quality)
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
