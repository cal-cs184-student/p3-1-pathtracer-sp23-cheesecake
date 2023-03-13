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

	// Mins and maxes
	double txmin = std::min(tx1, tx2);
	double txmax = std::max(tx1, tx2);
	double tymin = std::min(ty1, ty2);
	double tymax = std::max(ty1, ty2);
	double tzmin = std::min(tz1, tz2);
	double tzmax = std::max(tz1, tz2);

	// Max of min and min of max
	double tmin = std::max(txmin, std::max(tymin, tzmin));
	double tmax = std::min(txmax, std::min(tymax, tzmax));

	if (tmin > tmax) return false;
	if (tmin < t0 || tmax > t1) return false;
	if (tmin < 0 || tmax < 0) return false;
	if (tmin < r.min_t || tmax < r.min_t || tmin > r.max_t || tmax > r.max_t) return false;

	// Update t0 and t1
	t0 = tmin;
	t1 = tmax;
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
