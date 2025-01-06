#pragma once
#ifndef INCLUDE_SPHERE_H_
#define INCLUDE_SPHERE_H_

#include <glad/glad.h>
#include <string.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <vector>
#include <cassert>
#include <iostream>

using glm::cross;
using glm::dot;
using glm::inverse;
using glm::mat3;
using glm::mat4;
using glm::pi;
using glm::radians;
using glm::rotate;
using glm::scale;
using glm::translate;
using glm::transpose;
using glm::vec2;
using glm::vec3;
using glm::vec4;

struct Vertex {
	vec3 position;
	vec3 normal;
};

class Sphere {
public:
	explicit Sphere(const glm::vec3& center, float radius);
	void Initial();
	void draw() const;
	void generateSphereObjFile(const glm::vec3& center, float radius);
private:
	GLuint VAO;
	GLuint VBO;
	GLuint EBO;
	std::vector<Vertex> vertices_;
	std::vector<GLuint> indices_;
	void loadDataFromFile(const std::string& path);
	
};

#endif