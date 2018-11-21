#include "Ray.h"


Ray::Ray(glm::vec3 orig, glm::vec3 dir)
{
	m_orig = orig;
	m_dir = dir;
	glm::normalize(m_dir);
}

Ray::Ray()
{	
}

Ray::~Ray()
{
}
