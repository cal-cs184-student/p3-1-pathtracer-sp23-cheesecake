#include "sphere.h"

#include <cmath>

#include "pathtracer/bsdf.h"
#include "util/sphere_drawing.h"

namespace CGL {
namespace SceneObjects {

bool Sphere::test(const Ray &r, double &t1, double &t2) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection test.
  // Return true if there are intersections and writing the
  // smaller of the two intersection times in t1 and the larger in t2.

	// Equation according to slides

	double a = dot(r.d, r.d);
	double b = dot(2 * (r.o - o), r.d);
	double c = dot((r.o - o), (r.o - o)) - r2;

	double s = (b * b) - (4 * a * c);
	if (s < 0) return false;
	t1 = ((-1.0 * b) + sqrt(s)) / (2.0 * a);
	t2 = ((-1.0 * b) - sqrt(s)) / (2.0 * a);

	// If behind, return false
	if (t1 < 0 || t2 < 0) return false;

	// Swap if t1 > t2
	if (t1 > t2) {
		double tmp = t1;
		t1 = t2;
		t1 = tmp;
	}

  return true;

}

bool Sphere::has_intersection(const Ray &r) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection.
  // Note that you might want to use the the Sphere::test helper here.

	double t1, t2;
	bool res = test(r, t1, t2);

	if (r.min_t > t1 || r.max_t < t2) return false;

	if (res) {
		r.max_t = t1;
	}

  return res;
}

bool Sphere::intersect(const Ray &r, Intersection *i) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection.
  // Note again that you might want to use the the Sphere::test helper here.
  // When an intersection takes place, the Intersection data should be updated
  // correspondingly.

	double t1, t2;
	bool res = test(r, t1, t2);

	if (r.min_t > t1 || r.max_t < t2) return false;

	if (res) {
		r.max_t = t1;
		i->t = t1;
		Vector3D n = ((r.o + t1 * r.d) - o);
		n = n / n.norm();
		i->n = n;
		i->primitive = this;
		i->bsdf = get_bsdf();
	}

	return res;
}

void Sphere::draw(const Color &c, float alpha) const {
  Misc::draw_sphere_opengl(o, r, c);
}

void Sphere::drawOutline(const Color &c, float alpha) const {
  // Misc::draw_sphere_opengl(o, r, c);
}

} // namespace SceneObjects
} // namespace CGL
