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

    float eps = 0.001;

    // Compute normal
    Vector3D N = cross(p2 - p1, p3 - p2);

    // If ray and plane are parallel, return false
    if (abs(dot(N, r.d)) < eps) {
        return false;
    }

    // Compute t, check if it is beyond range
    float t = (dot(N, p1) - dot(N, r.o)) / dot(N, r.d);
    if (t < r.min_t || t > r.max_t) return false;

    // If triangle is behind ray, return false
    if (t < 0) return false;

    // Intersection point
    Vector3D p = r.o + t * r.d;

    // Test each edge to see if point in triangle
    Vector3D C, ep;
    Vector3D e1 = p2 - p1;
    ep = p - p1;
    C = cross(e1, ep);
    if (dot(N, C) < 0) return false; 

    Vector3D e2 = p3 - p2;
    ep = p - p2;
    C = cross(e2, ep);
    if (dot(N, C) < 0) return false;

    Vector3D e3 = p1 - p3;
    ep = p - p3;
    C = cross(e3, ep);
    if (dot(N, C) < 0) return false;

  return true;

}




bool Triangle::intersect(const Ray &r, Intersection *isect) const {
  // Part 1, Task 3:
  // implement ray-triangle intersection. When an intersection takes
  // place, the Intersection data should be updated accordingly


    float eps = 0.001;

    // Compute normal
    Vector3D N = cross(p2 - p1, p3 - p2);
    float normalize = dot(N, N);

    // If ray and plane are parallel, return false
    if (abs(dot(N, r.d)) < eps) {
        return false;
    }

    // Compute t, check if it is beyond range
    float t = (dot(N, p1) - dot(N, r.o)) / dot(N, r.d);
    if (t < r.min_t || t > r.max_t) return false;

    // If triangle is behind ray, return false
    if (t < 0) return false;

    // Intersection point
    Vector3D p = r.o + t * r.d;

    // Test each edge to see if point in triangle
    Vector3D C, ep;
    Vector3D e1 = p2 - p1;
    ep = p - p1;
    C = cross(e1, ep);
    if (dot(N, C) < 0) return false;

    Vector3D e2 = p3 - p2;
    ep = p - p2;
    C = cross(e2, ep);
    float alpha = dot(N, C) / normalize;
    if (alpha < 0) return false;

    Vector3D e3 = p1 - p3;
    ep = p - p3;
    C = cross(e3, ep);
    float beta = dot(N, C) / normalize;
    if (beta < 0) return false;

    isect->t = t;
    isect->n = alpha * n1 + beta * n2 + (1 - alpha - beta) * n3;
    isect->primitive = this;
    isect->bsdf = get_bsdf();

    return true;


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
