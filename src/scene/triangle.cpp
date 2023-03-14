#include "triangle.h"

#include "CGL/CGL.h"
#include "GL/glew.h"

namespace CGL {
namespace SceneObjects {

Triangle::Triangle(const Mesh *mesh, size_t v1, size_t v2, size_t v3) {
  p1 = mesh->positions[v1];
  p2 = mesh->positions[v2];
  p3 = mesh->positions[v3];
  n1 = mesh->normals[v1];
  n2 = mesh->normals[v2];
  n3 = mesh->normals[v3];
  bbox = BBox(p1);
  bbox.expand(p2);
  bbox.expand(p3);

  bsdf = mesh->get_bsdf();
}

BBox Triangle::get_bbox() const { return bbox; }

bool Triangle::has_intersection(const Ray &r) const {
  // Part 1, Task 3: implement ray-triangle intersection
  // The difference between this function and the next function is that the next
  // function records the "intersection" while this function only tests whether
  // there is a intersection.


    // Definitions
    Vector3D e1 = p2 - p1;
    Vector3D e2 = p3 - p1;
    Vector3D d = r.o - p1; // Vector from ray or

    // Check for parallelism
    Vector3D h = cross(r.d, e2);
    double a = dot(h, e1);
    if (a == 0) {
        return false;
    }

    // Compute coordinates
    Vector3D s = r.o - p1;
    double u = dot(s, h) / a;

    Vector3D q = cross(s, e1);
    double v = dot(r.d, q) / a;

    // Check intersection boundaries
    double t = dot(e2, q) / a;
    if (u >= 0 && v >= 0 && u + v <= 1 && t > r.min_t && t < r.max_t) {
        r.max_t = t;
        return true;
    }

  return false;

}




bool Triangle::intersect(const Ray &r, Intersection *isect) const {
  // Part 1, Task 3:
  // implement ray-triangle intersection. When an intersection takes
  // place, the Intersection data should be updated accordingly


    // Definitions
    Vector3D e1 = p2 - p1;
    Vector3D e2 = p3 - p1;
    Vector3D d = r.o - p1; // Vector from ray or

    // Check for parallelism
    Vector3D h = cross(r.d, e2);
    double a = dot(h, e1);
    if (a == 0) {
        return false;
    }

    // Compute coordinates
    Vector3D s = r.o - p1;
    double u = dot(s, h) / a;

    Vector3D q = cross(s, e1);
    double v = dot(r.d, q) / a;

    // Check intersection boundaries
    double t = dot(e2, q) / a;
    if (u >= 0 && v >= 0 && u + v <= 1 && t > r.min_t && t < r.max_t) {
        r.max_t = t;

        isect->t = t;
        Vector3D n = (1 - u - v) * n1 + u * n2 + v * n3;
        n.normalize();
        isect->n = n;
        isect->primitive = this;
        isect->bsdf = get_bsdf();
        return true;
    }

    return false;

}

void Triangle::draw(const Color &c, float alpha) const {
  glColor4f(c.r, c.g, c.b, alpha);
  glBegin(GL_TRIANGLES);
  glVertex3d(p1.x, p1.y, p1.z);
  glVertex3d(p2.x, p2.y, p2.z);
  glVertex3d(p3.x, p3.y, p3.z);
  glEnd();
}

void Triangle::drawOutline(const Color &c, float alpha) const {
  glColor4f(c.r, c.g, c.b, alpha);
  glBegin(GL_LINE_LOOP);
  glVertex3d(p1.x, p1.y, p1.z);
  glVertex3d(p2.x, p2.y, p2.z);
  glVertex3d(p3.x, p3.y, p3.z);
  glEnd();
}

} // namespace SceneObjects
} // namespace CGL
