//
//  Material.h
//  Raytracer
//
//  Created by Piotr Didyk on 14.07.21.
//

#ifndef Material_h
#define Material_h

#include "glm/glm.hpp"

/**
 Structure describing a material of an object
 */
struct Material{
    glm::vec3 ambient = glm::vec3(0.0);
    glm::vec3 diffuse = glm::vec3(1.0);
    glm::vec3 specular = glm::vec3(0.0);
    float shininess = 0.0;
    
    // Ward anisotropic reflectance model parameters
    bool useWard = false;           // 是否使用Ward模型
    float wardRoughnessX = 0.1f;   // X方向粗糙度 (αx)
    float wardRoughnessY = 0.1f;   // Y方向粗糙度 (αy)
    glm::vec3 wardSpecular = glm::vec3(1.0);  // Ward镜面反射系数 (ρs)
};

#endif /* Material_h */
