#include "pathtracer.h"

#include "misc.h"
#include "pathtracer/intersection.h"
#include "pathtracer/ray.h"
#include "scene/light.h"
#include "scene/sphere.h"
#include "scene/triangle.h"
#include "util/random_util.h"
#include "vector3D.h"


using namespace CGL::SceneObjects;

namespace CGL {

PathTracer::PathTracer() {
	gridSampler = new UniformGridSampler2D();
	hemisphereSampler = new UniformHemisphereSampler3D();
	sphereSampler = new UniformSphereSampler3D();
	interactionSampler = new InteractionSampler();

	tm_gamma = 2.2f;
	tm_level = 1.0f;
	tm_key = 0.18;
	tm_wht = 5.0f;
}

PathTracer::~PathTracer() {
	delete gridSampler;
	delete hemisphereSampler;
	delete sphereSampler;
	delete interactionSampler;
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

void PathTracer::write_to_framebuffer(ImageBuffer &framebuffer, size_t x0, size_t y0, size_t x1, size_t y1) {
	sampleBuffer.toColor(framebuffer, x0, y0, x1, y1);
}

Vector3D
PathTracer::estimate_direct_lighting_hemisphere(const Ray &r, const Intersection &isect) {
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
	const Vector3D w_out = w2o * (-r.d);

	// This is the same number of total samples as
	// estimate_direct_lighting_importance (outside of delta lights). We keep the
	// same number of samples for clarity of comparison.
	int num_samples = scene->lights.size() * ns_area_light;
	Vector3D L_out;

	// TODO (Part 3): Write your sampling loop here
	for (int i = 0; i < num_samples; i++) {
		// Random direction using hemisphereSampler in local coordinates
		Vector3D w_in = hemisphereSampler->get_sample();
		double pdf = 1 / (2.0 * PI);
		double cos_theta = w_in.z;
		Vector3D f = isect.bsdf->f(w_out, w_in);

		// Create the "bounced" ray
		Ray shadow_ray(hit_p, o2w * w_in);
		shadow_ray.min_t = EPS_F;
		
		// If shadow ray hits an object, update light out with that object's emission
		Intersection i_next;
		if (cos_theta > 0 && bvh->intersect(shadow_ray, &i_next)) {
			Vector3D L = i_next.bsdf->get_emission();
			L_out += L * f * cos_theta / pdf;
		}
	}
	L_out /= num_samples;
	return L_out;
}





Vector3D PathTracer::estimate_sphere_ray_marching(const Ray &r, const Intersection &isect) {
	// make a coordinate system for a hit point
	// with N aligned with the Z direction.
	Matrix3x3 o2w;
	make_coord_space(o2w, isect.n);
	Matrix3x3 w2o = o2w.T();

	// w_out points towards the source of the ray (e.g.,
	// toward the camera if this is a primary ray)
	const Vector3D hit_p = r.o + r.d * isect.t;
	const Vector3D w_out = w2o * (-r.d);

	// variables controlling falloff
	double k = 0.1;

	// Number of total light samples
	int num_samples = scene->lights.size() * ns_area_light;
	Vector3D L_out;
	for (int i = 0; i < num_samples; i++) {
		// Random direction using hemisphereSampler in local coordinates
		Vector3D w_in = sphereSampler->get_sample();
		double pdf = 1 / (4.0 * PI);
		double cos_theta = w_in.z;
		Vector3D f = isect.bsdf->f(w_out, w_in);

		// Determine the distance of
		double s_pdf;
		double s = interactionSampler->get_sample(k, &s_pdf);

		///
		/// create shadow ray
		/// create bounced ray
		/// return 0.1 * atleastonebounce(shadow) + 0.9 * atleastonebounce(bounced)
		///

		// Create the "shadow" ray
		Ray shadow_ray(hit_p, o2w * w_in);
		shadow_ray.min_t = EPS_F;

		// If shadow ray hits an object, update light out with that object's emission
		Intersection i_next;
		if (cos_theta > 0 && bvh->intersect(shadow_ray, &i_next)) {
			Vector3D L = i_next.bsdf->get_emission();
			L_out += L * f * cos_theta / pdf;

			const Vector3D next_hit_p = shadow_ray.o + shadow_ray.d * i_next.t;
			if (s < (next_hit_p - hit_p).norm()) {
				L_out *= 0.5;
			}
		}

	}
	L_out /= num_samples;
	return L_out;
}

Vector3D PathTracer::zero_bounce_radiance(const Ray &r, const Intersection &isect) {
	// TODO: Part 3, Task 2
	// Returns the light that results from no bounces of light

	return isect.bsdf->get_emission();
}

Vector3D PathTracer::one_bounce_radiance(const Ray &r, const Intersection &isect) {
	// TODO: Part 3, Task 3
	// Returns either the direct illumination by hemisphere or importance sampling
	// depending on `direct_hemisphere_sample`

	return estimate_sphere_ray_marching(r, isect);

	if (direct_hemisphere_sample) {
		return estimate_direct_lighting_hemisphere(r, isect);
	} else {
		return estimate_direct_lighting_importance(r, isect);
	}
}

Vector3D PathTracer::at_least_one_bounce_radiance(const Ray &r, const Intersection &isect) {
	Matrix3x3 o2w;
	make_coord_space(o2w, isect.n);
	Matrix3x3 w2o = o2w.T();

	Vector3D hit_p = r.o + r.d * isect.t;
	Vector3D w_out = w2o * (-r.d);

	Vector3D L_out = Vector3D(0,0,0);

	// TODO: Part 4, Task 2
	// Returns the one bounce radiance + radiance from extra bounces at this point.
	// Should be called recursively to simulate extra bounces.

	if (r.depth <= 0) return Vector3D();

	// 
	if ((isAccumBounces || r.depth == 1)) L_out += one_bounce_radiance(r, isect);

	double prob = 1;
	if (coin_flip(prob)) {
		// 
		Vector3D w_in;
		double pdf;
		Vector3D f = isect.bsdf->sample_f(w_out, &w_in, &pdf);
		double cos_theta = w_in.z;

		// Create the next bounced ray
		Ray next_ray(hit_p, o2w * w_in);
		next_ray.min_t = EPS_F;
		next_ray.depth = r.depth - 1;

		// 
		Intersection i_next;
		if (next_ray.depth > 0 && cos_theta > 0 && bvh->intersect(next_ray, &i_next)) {
			double T = 4;
			double isect_distance = ((next_ray.o + i_next.t * next_ray.d) - next_ray.o).norm();
			double extinct_dist = -(std::log10(1 - random_uniform())) / T;
			if (isect_distance <= extinct_dist) {
				L_out += at_least_one_bounce_radiance(next_ray, i_next) * f * cos_theta / pdf;
			}
			else {
				//Intersection i_air = Intersection();
				static CGL::DiffuseBSDF zero_bsdf(Vector3D(1, 1, 1));
				i_next.bsdf = &zero_bsdf;
				i_next.t = extinct_dist;
				L_out = at_least_one_bounce_radiance(next_ray, i_next) * (1 / (4 *PI)) / pdf;
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
	
	if (!bvh->intersect(r, &isect))
		return envLight ? envLight->sample_dir(r) : L_out;

	// prev code
	// return (isect.t == INF_D) ? debug_shading(r.d) : normal_shading(isect.n);

	// TODO (Part 3): Return the direct illumination.

	L_out = zero_bounce_radiance(r, isect);

	// TODO (Part 4): Accumulate the "direct" and "indirect"
	// parts of global illumination into L_out rather than just direct

	L_out += at_least_one_bounce_radiance(r, isect);

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

	double s1 = 0;
	double s2 = 0;
	int count = 1;
	Vector3D radiance = Vector3D();
	for (int i = 0; i < num_samples; i++, count++) {
		// Get sample within unit pixel and add origin for offset
		Vector2D sample_pos = origin + gridSampler->get_sample();
		Ray r = camera->generate_ray(sample_pos.x / sampleBuffer.w, sample_pos.y / sampleBuffer.h);
		r.depth = max_ray_depth;
		Vector3D sample_radiance = est_radiance_global_illumination(r);
		radiance += sample_radiance;

		// Update s1 and s2 for adaptive sampling
		s1 += sample_radiance.illum();
		s2 += sample_radiance.illum() * sample_radiance.illum();

		// Check if our samples so far is enough
		if (count % samplesPerBatch == 0) {
			double mu = s1 / (double) count;
			double var = (s2 - s1 * s1 / (double) count) / ((double) count - 1);
			double I2 = 1.96 * 1.96 * var / (double) count;
			if (I2 <= maxTolerance * maxTolerance * mu * mu) {
				break;
			}
		}
	}
	// Note: in my implementation count can equal num_saples + 1
	radiance /= min(count, num_samples);
	sampleBuffer.update_pixel(radiance, x, y);
	sampleCountBuffer[x + y * sampleBuffer.w] = min(count, num_samples);
}

void PathTracer::autofocus(Vector2D loc) {
	Ray r = camera->generate_ray(loc.x / sampleBuffer.w, loc.y / sampleBuffer.h);
	Intersection isect;

	bvh->intersect(r, &isect);

	camera->focalDistance = isect.t;
}

} // namespace CGL
