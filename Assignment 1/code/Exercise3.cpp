/**
@file main.cpp
*/

#include <iostream>
#include <fstream>
#include <cmath>
#include <ctime>
#include <vector>
#include "glm/glm.hpp"
#include "glm/gtx/transform.hpp"

#include "Image.h"
#include "Material.h"

using namespace std;


//Author : Xuanlin Chen
//Work E-mail : chenxu@usi.ch
//Personal E-mail : kissofazshara@gmail.com

std::ostream& operator<<(std::ostream& os, const glm::vec3& vec) {
    os << "(" << vec.x << ", " << vec.y << ", " << vec.z << ")";
    return os;
}

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
};

/**
 Implementation of the class Object for sphere shape.
 */
class Sphere : public Object{
private:
    float radius; ///< Radius of the sphere
    glm::vec3 center; ///< Center of the sphere

public:
	/**
	 The constructor of the sphere
	 @param radius Radius of the sphere
	 @param center Center of the sphere
	 @param color Color of the sphere
	 */
    Sphere(float radius, glm::vec3 center, glm::vec3 color) : radius(radius), center(center){
		this->color = color;
    }
	Sphere(float radius, glm::vec3 center, Material material) : radius(radius), center(center){
		this->material = material;
	}
	/** Implementation of the intersection function*/
	Hit intersect(Ray ray){
		
		Hit hit;
		hit.hit = false;
		
		/* ------------------ Exercise 1 -------------------
		 
		Place for your code: ray-sphere intersection. Remember to set all the fields of the hit structure:

		 hit.intersection =
		 hit.normal =
		 hit.distance =
		 hit.object = this;
		 
		------------------------------------------------- */
		
		glm::vec3 OC = center - ray.origin; //Vector OC
		float a = glm::dot(OC, ray.direction);//Length of a
		float D = sqrt(std::pow(glm::length(OC), 2) - std::pow(a, 2)); //D^2 = OC^2 - a^2
		float b = sqrt(std::pow(radius, 2) - std::pow(D, 2));//b^2 = r^2 - D^2

		if (D <= radius){ //If D < r 2 intersections; if D = r 1 intersection; if D > r no intersecitons
			hit.hit = true;
			float t1 = a - b;
			float t2 = a + b;
			float t;
			if (t1 < t2 && t1 > 0){ //Judge the intersection, it should be cloest to the camera and be positvie
				t = t1;
			}else{
				t = t2;
			}
			hit.intersection = ray.origin + t * ray.direction; //γ(t) = td + o
			hit.distance = glm::distance(ray.origin, hit.intersection);
			hit.normal = glm::normalize(hit.intersection - center);
			hit.object = this;
		}

		return hit;
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
glm::vec3 ambient_light(0.1,0.1,0.1);
vector<Object *> objects; ///< A list of all objects in the scene


/** Function for computing color of an object according to the Phong Model
 @param point A point belonging to the object for which the color is computer
 @param normal A normal vector the the point
 @param view_direction A normalized direction from the point to the viewer/camera
 @param material A material structure representing the material of the object
*/
glm::vec3 PhongModel(glm::vec3 point, glm::vec3 normal, glm::vec3 view_direction, Material material){
    glm::vec3 color(0.0); // 
    glm::vec3 I_a = material.ambient * ambient_light;

    for(int i = 0; i < lights.size(); i++){
        Light* light = lights[i];
        glm::vec3 light_direction = glm::normalize(light->position - point); 
        glm::vec3 diffuse_component(0.0);
        glm::vec3 specular_component(0.0);
        //float att = 1 / glm::distance(point, light->position); 
        //std::cout << "att: " << att << std::endl;

        float diffuse_intensity = std::max(glm::dot(normal, light_direction), 0.0f); //To make sure <n,l> > 0;
        diffuse_component = material.diffuse * light->color * diffuse_intensity; 

        if (material.shininess > 0.0f) {
            glm::vec3 reflection_direction = glm::reflect(-light_direction, normal); 
            float specular_intensity = std::pow(std::max(glm::dot(view_direction, reflection_direction), 0.0f), material.shininess); // 镜面反射光照公式
            specular_component = material.specular * light->color * specular_intensity;
        }
        color += (diffuse_component + specular_component);
        //color += (diffuse_component + specular_component) * att; 
    }
    color += I_a; 
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
		/* ------------------Excercise 3--------------------
		 
		 Use the second line when you implement PhongModel function - Exercise 3
					 
		 ------------------------------------------------- */
		//color = closest_hit.object->color;
        //std::cout << "Lights: " << closest_hit.intersection << std::endl;
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

	
	
	/* ------------------Excercise 2--------------------
	 
	Place for your code: additional sphere
	 
	------------------------------------------------- */
	
	
	/* ------------------Excercise 3--------------------
	 
	 Add here all the objects to the scene. Remember to add them using the constructor for the sphere with material structure.
	 You will also need to define the materials.
	 Example of adding one sphere:
	 
	 Material red_specular;
	 red_specular.diffuse = glm::vec3(1.0f, 0.3f, 0.3f);
	 red_specular.ambient = glm::vec3(0.01f, 0.03f, 0.03f);
	 red_specular.specular = glm::vec3(0.5);
	 red_specular.shininess = 10.0;
	 
	 objects.push_back(new Sphere(0.5, glm::vec3(-1,-2.5,6), red_specular));
	 
	 Remember also about adding some lights. For example a white light of intensity 0.4 and position in (0,26,5):
	 
	 lights.push_back(new Light(glm::vec3(0, 26, 5), glm::vec3(0.4)));
	 
	------------------------------------------------- */

    Material blue_specular;
    blue_specular.ambient = glm::vec3(0.07f, 0.07f, 0.1f);
	blue_specular.diffuse = glm::vec3(0.7f, 0.7f, 1.0f);
	blue_specular.specular = glm::vec3(0.6);
	blue_specular.shininess = 100.0;

    Material red_specular;
    red_specular.ambient = glm::vec3(0.01f, 0.03f, 0.03f);
	red_specular.diffuse = glm::vec3(1.0f, 0.3f, 0.3f);
	red_specular.specular = glm::vec3(0.5);
	red_specular.shininess = 10.0;

	Material green_specular;
    green_specular.ambient = glm::vec3(0.07f, 0.09f, 0.07f);
	green_specular.diffuse = glm::vec3(0.7f, 0.9f, 0.7f);
	green_specular.specular = glm::vec3(0);
	green_specular.shininess = 0.0;

    objects.push_back(new Sphere(1.0, glm::vec3(1,-2,8), blue_specular));
	objects.push_back(new Sphere(0.5, glm::vec3(-1,-2.5,6), red_specular));
	objects.push_back(new Sphere(1.0, glm::vec3(2,-2,6), green_specular));
	 
	lights.push_back(new Light(glm::vec3(0, 26, 5), glm::vec3(0.4)));
	lights.push_back(new Light(glm::vec3(0, 1, 12), glm::vec3(0.4)));
	lights.push_back(new Light(glm::vec3(0, 5, 1), glm::vec3(0.4)));
    
    
}

int main(int argc, const char * argv[]) {

    clock_t t = clock(); // variable for keeping the time of the rendering

    int width = 1024; //width of the image
    int height = 768; // height of the image
    float fov = 90; // field of view

	sceneDefinition(); // Let's define a scene

	Image image(width,height); // Create an image where we will store the result

	/* ------------------ Exercise 1 -------------------
	 
	Place for your code: Loop over pixels to form and traverse the rays through the scene
	 
	------------------------------------------------- */

	float s = 2 * tan(fov / 2 / 180 * M_PI) / width; // s is the length of a pixel
	float X = (-width * s) / 2; // X is the X-axis coordinate in the world of the top-left corner of the image
	float Y = (height * s) / 2; // Y is the Y-axis coordinate in the world of the top-left corner of the image
	

    for(int i = 0; i < width ; i++)
        for(int j = 0; j < height ; j++){
			float d_x = X + s / 2 + i * s;
			float d_y = Y - s / 2 - j * s;
			float d_z = 1.0; //The（dx,dy,dz) coordinate of pixel P(i,j) in the world
			glm::vec3 origin = glm::vec3(0, 0, 0); //The camera
			glm::vec3 direction = glm::vec3(d_x, d_y, d_z); //The direction
			direction = glm::normalize(direction); //Normalization

			Ray ray(origin, direction);

			image.setPixel(i, j, trace_ray(ray)); 
			/* ------------------ Exercise 1 -------------------
			 
			Place for your code: ray definition for pixel (i,j), ray traversal
			 
			 ------------------------------------------------- */
			
			//Definition of the ray
			//glm::vec3 origin(0, 0, 0);
			//glm::vec3 direction(?, ?, ?);               // fill in the correct values
			//direction = glm::normalize(direction);
			
			//Ray ray(origin, direction);  // ray traversal
			
			//image.setPixel(i, j, trace_ray(ray));

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
