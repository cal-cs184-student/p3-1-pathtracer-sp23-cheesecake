#include "pathtracer.h"

#include "scene/light.h"
#include "scene/sphere.h"
#include "scene/triangle.h"


using namespace CGL::SceneObjects;

namespace CGL {

PathTracer::PathTracer() {
  gridSampler = new UniformGridSampler2D();
  hemisphereSampler = new UniformHemisphereSampler3D();

  tm_gamma = 2.2f;
  tm_level = 1.0f;
  tm_key = 0.18;
  tm_wht = 5.0f;
}

PathTracer::~PathTracer() {
  delete gridSampler;
  delete hemisphereSampler;
}

void PathTracer::set_frame_size(size_t width, size_t height) {
  sampleBuffer.resize(width, height);
  sampleCountBuffer.resize(width * height);
}

void PathTracer::clear() {
  bvh = NULL;
  scene = NULL;
  camera = NULL;
  sampleBuffer.clear();
  sampleCountBuffer.clear();
  sampleBuffer.resize(0, 0);
  sampleCountBuffer.resize(0, 0);
}

void PathTracer::write_to_framebuffer(ImageBuffer &framebuffer, size_t x0,
                                      size_t y0, size_t x1, size_t y1) {
  sampleBuffer.toColor(framebuffer, x0, y0, x1, y1);
}

Vector3D
PathTracer::estimate_direct_lighting_hemisphere(const Ray &r,
                                                const Intersection &isect) {
  // Estimate the lighting from this intersection coming directly from a light.
  // For this function, sample uniformly in a hemisphere.

  // Note: When comparing Cornel Box (CBxxx.dae) results to importance sampling, you may find the "glow" around the light source is gone.
  // This is totally fine: the area lights in importance sampling has directionality, however in hemisphere sampling we don't model this behaviour.

  // make a coordinate system for a hit point
  // with N aligned with the Z direction.
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  // w_out points towards the source of the ray (e.g.,
  // toward the camera if this is a primary ray)
  const Vector3D hit_p = r.o + r.d * isect.t;
  const Vector3D w_out = w2o * (-r.d); // w_r

  // This is the same number of total samples as
  // estimate_direct_lighting_importance (outside of delta lights). We keep the
  // same number of samples for clarity of comparison.
  int num_samples = scene->lights.size() * ns_area_light;
  Vector3D L_out;

  // TODO (Part 3): Write your sampling loop here
  for (int i = 0; i < num_samples; i++) {
      Vector3D wi = hemisphereSampler->get_sample();
      Vector3D wiw = o2w * wi;
      Ray r = Ray(wiw * EPS_D + hit_p, wiw);
      Intersection new_isect;

      if (bvh->intersect(r, &new_isect)) {
          Vector3D em = new_isect.bsdf->get_emission();
          Vector3D sample = isect.bsdf->f(w_out, wi) * em * cos_theta(wi) * PI * 2.0;
          L_out += sample;
      }
  }
  // TODO BEFORE YOU BEGIN
  // UPDATE `est_radiance_global_illumination` to return direct lighting instead of normal shading
  L_out = L_out / (1.0 * num_samples);
  return L_out;
}

Vector3D
PathTracer::estimate_direct_lighting_importance(const Ray &r,
                                                const Intersection &isect) {
  // Estimate the lighting from this intersection coming directly from a light.
  // To implement importance sampling, sample only from lights, not uniformly in
  // a hemisphere.

  // make a coordinate system for a hit point
  // with N aligned with the Z direction.
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  // w_out points towards the source of the ray (e.g.,
  // toward the camera if this is a primary ray)
  const Vector3D hit_p = r.o + r.d * isect.t;
  const Vector3D w_out = w2o * (-r.d);
  Vector3D L_out;

  // TODO: Part 3, Task 4
  int num_lights = scene->lights.size();

  for (int i = 0; i < num_lights; i++){
    Vector3D wi;
    double distToLight;
    double pdf;

    if (scene->lights[i]->is_delta_light()){
        Vector3D sample = scene->lights[i]->sample_L(hit_p, &wi, &distToLight, &pdf);
        Vector3D w_in = w2o * wi;

        if (w_in.z >= 0) {
            Intersection new_isect;
            Ray r = Ray(hit_p + EPS_D * wi, wi);
            r.max_t = distToLight;
            bool intersected = bvh->intersect(r, &new_isect);

            if (!intersected) {
                L_out += (sample * cos_theta(w_in) * isect.bsdf->f(w_out, w_in)) / pdf;
            }
        }
    } else {
        Vector3D sample;
        int num_samples = ns_area_light;
        Intersection new_isect;

        for (int j = 0; j < num_samples; j++){
            Vector3D sample = scene->lights[i]->sample_L(hit_p, &wi, &distToLight, &pdf);
            Vector3D w_in = w2o * wi;

            if (w_in.z >= 0){

                Ray r = Ray(hit_p + EPS_D * wi, wi);
                r.max_t = distToLight;

                bool intersected = bvh->intersect(r, &new_isect);

                if (!intersected) {
                    sample += (sample * cos_theta(w_in) * isect.bsdf->f(w_out, w_in)) / 1.0 * pdf;
                }
            }

        }
//        L_out += runningSample / num_samples * 1.0;
        L_out += sample * 1.0;
    }

  }


  return L_out;
}

Vector3D PathTracer::zero_bounce_radiance(const Ray &r,
                                          const Intersection &isect) {
  // TODO: Part 3, Task 2
  // Returns the light that results from no bounces of light
  Vector3D emission = isect.bsdf->get_emission();
  return emission;
}

Vector3D PathTracer::one_bounce_radiance(const Ray &r,
                                         const Intersection &isect) {
  // TODO: Part 3, Task 3
  // Returns either the direct illumination by hemisphere or importance sampling
  // depending on `direct_hemisphere_sample`
  if (direct_hemisphere_sample) {
      return estimate_direct_lighting_hemisphere(r, isect);
  } else {
      return estimate_direct_lighting_importance(r, isect);
  }

}

Vector3D PathTracer::at_least_one_bounce_radiance(const Ray &r,
                                                  const Intersection &isect) {
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  Vector3D hit_p = r.o + r.d * isect.t;
  Vector3D w_out = w2o * (-r.d);

  Vector3D L_out(0, 0, 0);


  // TODO: Part 4, Task 2
  // Returns the one bounce radiance + radiance from extra bounces at this point.
  // Should be called recursively to simulate extra bounces.
  L_out = one_bounce_radiance(r, isect);

  double p = coin_flip(.5) ? 0.6 : 0.7;
  double pdf;

  Vector3D w_in;

  Vector3D sample = isect.bsdf->sample_f(w_out, &w_in, &pdf);
  bool flag = (r.depth != max_ray_depth || max_ray_depth <= 1) || (r.depth <= 1) && (coin_flip(1 - p));

  if (!flag) {
      Vector3D wwi = o2w * w_in;
      Intersection isect;

      Ray ray = Ray(hit_p + EPS_D * wwi, wwi, INF_D, r.depth - 1);

      if (bvh->intersect(ray, &isect)) {
          Vector3D b = at_least_one_bounce_radiance(ray, isect);

          if (r.depth == this->max_ray_depth) {
              L_out += (b *w_in.z * sample);
              L_out /= pdf;
          }
          else {
              L_out += (b * w_in.z * sample);
              L_out /= pdf;
              L_out /= p;
          }
      }
  }


  return L_out;
}

Vector3D PathTracer::est_radiance_global_illumination(const Ray &r) {
  Intersection isect;
  Vector3D L_out;

  // You will extend this in assignment 3-2.
  // If no intersection occurs, we simply return black.
  // This changes if you implement hemispherical lighting for extra credit.

  // The following line of code returns a debug color depending
  // on whether ray intersection with triangles or spheres has
  // been implemented.
  //
  // REMOVE THIS LINE when you are ready to begin Part 3.
  
  if (!bvh->intersect(r, &isect))
    return L_out;


//  L_out = (isect.t == INF_D) ? debug_shading(r.d) : normal_shading(isect.n);
//  L_out = (isect.t == INF_D) ? debug_shading(r.d) : zero_bounce_radiance(r, isect) + one_bounce_radiance(r, isect);
  // TODO (Part 3): Return the direct illumination.
//  L_out = estimate_direct_lighting_importance(r, isect);

//    L_out = zero_bounce_radiance(r, isect) + one_bounce_radiance(r, isect);
  // TODO (Part 4): Accumulate the "direct" and "indirect"
  L_out = zero_bounce_radiance(r, isect) + at_least_one_bounce_radiance(r, isect);
  // parts of global illumination into L_out rather than just direct

  return L_out;
}

void PathTracer::raytrace_pixel(size_t x, size_t y) {
  // TODO (Part 1.2):
  // Make a loop that generates num_samples camera rays and traces them
  // through the scene. Return the average Vector3D.
  // You should call est_radiance_global_illumination in this function.

  // TODO (Part 5):
  // Modify your implementation to include adaptive sampling.
  // Use the command line parameters "samplesPerBatch" and "maxTolerance"

  int num_samples = ns_aa;          // total samples to evaluate
  Vector2D origin = Vector2D(x, y); // bottom left corner of the pixel

  // Part 5 variables
  double s1 = 0;
  double s2 = 0;

  Vector3D total = Vector3D(0, 0, 0);
  int i;
  for (i = 0; i < num_samples; i++) {
      Vector2D s = gridSampler->get_sample();
      Ray r = camera->generate_ray((x + s.x) / (double)sampleBuffer.w, (y + s.y) / (double)sampleBuffer.h);
      Vector3D curr_sample = est_radiance_global_illumination(r);
      total += curr_sample;

      // Part 5
      s1 += curr_sample.illum();
      s2 += (curr_sample.illum() * curr_sample.illum());
      int n = i + 1;
      if (n % samplesPerBatch == 0) {
          double mean = s1 / n;
          double var = (1.0 / (n - 1.0)) * (s2 - (s1 * s1 / n));
          double I = 1.96 * sqrt(var) / sqrt(i);
          if (I <= maxTolerance * mean) break;
      }

  }

  Vector3D average = total / (double)num_samples;

  sampleBuffer.update_pixel(average, x, y);
  sampleCountBuffer[x + y * sampleBuffer.w] = i;
}

void PathTracer::autofocus(Vector2D loc) {
  Ray r = camera->generate_ray(loc.x / sampleBuffer.w, loc.y / sampleBuffer.h);
  Intersection isect;

  bvh->intersect(r, &isect);

  camera->focalDistance = isect.t;
}

} // namespace CGL
