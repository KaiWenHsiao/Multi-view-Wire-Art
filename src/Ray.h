#pragma once

#include <glm\vec3.hpp>
#include <glm\glm.hpp>

#include <vector>

class Ray
{
public:
	glm::vec3 m_orig;
	glm::vec3 m_dir;

	Ray(glm::vec3 orig, glm::vec3 dir);
	Ray();
	virtual ~Ray();
};

