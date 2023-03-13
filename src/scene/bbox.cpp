#include "bbox.h"

#include "GL/glew.h"

#include <algorithm>
#include <iostream>

namespace CGL {

bool BBox::intersect(const Ray& r, double& t0, double& t1) const {

  // TODO (Part 2.2):
  // Implement ray - bounding box intersection test
  // If the ray intersected the bouding box within the range given by
  // t0, t1, update t0 and t1 with the new intersection times.

	// Computing tmin and tmax
	double tx1 = (min.x - r.o.x) / r.d.x;
	double tx2 = (max.x - r.o.x) / r.d.x;
	double ty1 = (min.y - r.o.y) / r.d.y;
	double ty2 = (max.y - r.o.y) / r.d.y;
	double tz1 = (min.z - r.o.z) / r.d.z;
	double tz2 = (max.z - r.o.z) / r.d.z;

	Vector3D tmin = Vector3D(std::min(tx1, tx2), std::min(ty1, ty2), std::min(tz1, tz2));
	Vector3D tmax = Vector3D(std::max(tx1, tx2), std::max(ty1, ty2), std::max(tz1, tz2));

	// Max of min and min of max
	double max_of_min = std::max(tmin.x, std::max(tmin.y, tmin.z));
	double min_of_max = std::min(tmax.x, std::min(tmax.y, tmax.z));

	if (max_of_min > min_of_max) return false;

	// Update t0 and t1
	t0 = max_of_min;
	t1 = min_of_max;
	return true;

}

void BBox::draw(Color c, float alpha) const {

  glColor4f(c.r, c.g, c.b, alpha);

  // top
  glBegin(GL_LINE_STRIP);
  glVertex3d(max.x, max.y, max.z);
  glVertex3d(max.x, max.y, min.z);
  glVertex3d(min.x, max.y, min.z);
  glVertex3d(min.x, max.y, max.z);
  glVertex3d(max.x, max.y, max.z);
  glEnd();

  // bottom
  glBegin(GL_LINE_STRIP);
  glVertex3d(min.x, min.y, min.z);
  glVertex3d(min.x, min.y, max.z);
  glVertex3d(max.x, min.y, max.z);
  glVertex3d(max.x, min.y, min.z);
  glVertex3d(min.x, min.y, min.z);
  glEnd();

  // side
  glBegin(GL_LINES);
  glVertex3d(max.x, max.y, max.z);
  glVertex3d(max.x, min.y, max.z);
  glVertex3d(max.x, max.y, min.z);
  glVertex3d(max.x, min.y, min.z);
  glVertex3d(min.x, max.y, min.z);
  glVertex3d(min.x, min.y, min.z);
  glVertex3d(min.x, max.y, max.z);
  glVertex3d(min.x, min.y, max.z);
  glEnd();

}

std::ostream& operator<<(std::ostream& os, const BBox& b) {
  return os << "BBOX(" << b.min << ", " << b.max << ")";
}

} // namespace CGL
