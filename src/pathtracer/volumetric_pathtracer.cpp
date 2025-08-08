#include "volumetric_pathtracer.h"

#include "matrix3x3.h"
#include "misc.h"
#include "pathtracer/bsdf.h"
#include "pathtracer/intersection.h"
#include "pathtracer/medium.h"
#include "pathtracer/pathtracer.h"
#include "pathtracer/phase_function.h"
#include "pathtracer/ray.h"
#include "scene/light.h"
#include "scene/sphere.h"
#include "scene/triangle.h"
#include "util/random_util.h"
#include "vector3D.h"
#include <iostream>


using namespace CGL::SceneObjects;

namespace CGL {

VolumetricPathTracer::VolumetricPathTracer() : PathTracer() {
	PhaseFunction* phase = new HenyeyGreensteinPhaseFunction(0.7);
	medium = new Medium(0.3, 0.01, phase);
	max_medium_ray_depth = 4;
	cout << "TEST" << endl;
}

VolumetricPathTracer::~VolumetricPathTracer() {
	delete medium;
}

Vector3D VolumetricPathTracer::zero_bounce_radiance(const Ray &r, const Intersection &isect) {
	double d = (r.d * isect.t).norm(); // just in case
	double transmittance = medium->transmittance(d);
	return isect.bsdf->get_emission() * transmittance;
}

Vector3D VolumetricPathTracer::estimate_direct_lighting_hemisphere(const Ray &r, const Intersection &isect) {
	Matrix3x3 o2w;
	make_coord_space(o2w, isect.n);
	Matrix3x3 w2o = o2w.T();

	// w_out points towards the source of the ray
	const Vector3D hit_p = r.o + r.d * isect.t;
	const Vector3D w_out = w2o * (-r.d);

	// num_samples = num_lights * sample per area light source
	int num_samples = scene->lights.size() * ns_area_light;
	Vector3D L_out;

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
			double d = (shadow_ray.d * i_next.t).norm(); // just in case
			double transmittance = medium->transmittance(d);
			L_out += L * f * cos_theta / pdf * transmittance;
		}
	}
	L_out /= num_samples;
	return L_out;
}

Vector3D VolumetricPathTracer::estimate_direct_lighting_importance(const Ray &r, const Intersection &isect) {
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

	// TODO (Part 3): Write your sampling loop here
	for (auto light = scene->lights.begin(); light != scene->lights.end(); light++) {
		Vector3D w_in;
		double distToLight;
		double pdf;
		Vector3D L_light;

		// Only sample once for point light sources
		int num_samples = (*light)->is_delta_light() ? 1 : ns_area_light;
		for (int i = 0; i < num_samples; i++) {
			// 
			Vector3D L = (*light)->sample_L(hit_p, &w_in, &distToLight, &pdf);
			double cos_theta = dot(w_in, isect.n);
			Vector3D f = isect.bsdf->f(w_out, w2o * w_in);

			// Create the "bounced" ray
			Ray shadow_ray(hit_p, w_in);
			shadow_ray.min_t = EPS_F;
			shadow_ray.max_t = distToLight - EPS_F;
			
			// If shadow ray is not obstructed, update light out
			Intersection i_next;
			if (cos_theta > 0 && !bvh->intersect(shadow_ray, &i_next)) {
				double transmittance = medium->transmittance(distToLight);
				L_light += L * f * cos_theta / pdf * transmittance;
			}
		}
		L_out += L_light / num_samples;
	}
	
	return L_out;
}

Vector3D VolumetricPathTracer::surface_one_bounce_radiance(const Ray &r, const Intersection &isect) {
	double d = (r.d * isect.t).norm(); // just in case
	double transmittance = medium->transmittance(d);
	if (direct_hemisphere_sample) {
		return estimate_direct_lighting_hemisphere(r, isect) * transmittance;
	} else {
		return estimate_direct_lighting_importance(r, isect) * transmittance;
	}
}

Vector3D VolumetricPathTracer::estimate_medium_direct_lighting_default(const Ray &r, double d) {
	// w_out points towards the source of the ray
	const Vector3D hit_p = r.o + r.d * d;
	const Vector3D w_out = -r.d;

	// num_samples = num_lights * sample per area light source
	int num_samples = scene->lights.size() * ns_area_light;
	Vector3D L_out;

	for (int i = 0; i < num_samples; i++) {
		// Random direction using hemisphereSampler in local coordinates
		Matrix3x3 o2w;
		make_coord_space(o2w, w_out);
		Vector3D w_in = medium->phase_function()->get_sample(w_out);

		// Create the "bounced" ray
		Ray shadow_ray(hit_p, o2w * w_in);
		shadow_ray.min_t = EPS_F;
		
		// If shadow ray hits an object, update light out with that object's emission
		Intersection i_next;
		if (bvh->intersect(shadow_ray, &i_next)) {
			Vector3D L = i_next.bsdf->get_emission();
			double d = (shadow_ray.d * i_next.t).norm(); // just in case
			double transmittance = medium->transmittance(d);
			L_out += L * transmittance;
		}
	}
	L_out /= num_samples;
	return L_out;
}

Vector3D VolumetricPathTracer::estimate_medium_direct_lighting_importance(const Ray &r, double d) {
	// w_out points towards the source of the ray (e.g.,
	// toward the camera if this is a primary ray)
	const Vector3D hit_p = r.o + r.d * d;
	const Vector3D w_out = -r.d;

	Vector3D L_out;

	// TODO (Part 3): Write your sampling loop here
	for (auto light = scene->lights.begin(); light != scene->lights.end(); light++) {
		Vector3D w_in;
		double distToLight;
		double pdf;
		Vector3D L_light;

		// Only sample once for point light sources
		int num_samples = (*light)->is_delta_light() ? 1 : ns_area_light;
		for (int i = 0; i < num_samples; i++) {
			// 
			Vector3D L = (*light)->sample_L(hit_p, &w_in, &distToLight, &pdf);
			Vector3D phase_val = medium->phase_function()->p(w_out, w_in);

			// Create the "bounced" ray
			Ray shadow_ray(hit_p, w_in);
			shadow_ray.min_t = EPS_F;
			shadow_ray.max_t = distToLight - EPS_F;
			
			// If shadow ray is not obstructed, update light out
			Intersection i_next;
			if (!bvh->intersect(shadow_ray, &i_next)) {
				double transmittance = medium->transmittance(distToLight);
				L_light += L * phase_val / pdf * transmittance;
			}
		}
		L_out += L_light / num_samples;
	}

	return L_out;
}

Vector3D VolumetricPathTracer::medium_one_bounce_radiance(const Ray &r, double d) {
	double transmittance = medium->transmittance(d);
	int select = 1;
	switch (select) {
		case 0:
			return estimate_medium_direct_lighting_default(r, d) * transmittance;

		case 1:
			return estimate_medium_direct_lighting_importance(r, d) * transmittance;

		return Vector3D();
	}
}

Vector3D VolumetricPathTracer::at_least_one_bounce_radiance(const Ray &r) {
	Intersection isect;
	double scatter_distance_pdf;
	double scatter_distance = medium->sample_distance(&scatter_distance_pdf);
	if (!bvh->intersect(r, &isect) || scatter_distance < r.min_t) {
		return Vector3D();
	}

	Vector3D L_out(0, 0, 0);

	// scattering happens before intersection
	if (scatter_distance < isect.t) {
		if (r.medium_depth <= 0) return Vector3D();

		L_out += medium_one_bounce_radiance(r, scatter_distance);

		double prob = 0.75;
		if (coin_flip(prob)) {
			Vector3D hit_p = r.o + r.d * scatter_distance;
			Vector3D w_out = -r.d;

			Matrix3x3 o2w;
			make_coord_space(o2w, w_out);
			Vector3D w_in = medium->phase_function()->get_sample(w_out);

			Ray next_ray(hit_p, o2w * w_in);
			next_ray.min_t = EPS_F;
			next_ray.medium_depth = r.medium_depth - 1;

			if (coin_flip(medium->scatter_proba())) {
				double transmittance = medium->transmittance(scatter_distance);
				L_out += at_least_one_bounce_radiance(next_ray) * transmittance / prob;
			}
		}
	} else {
		Matrix3x3 o2w;
		make_coord_space(o2w, isect.n);
		Matrix3x3 w2o = o2w.T();

		Vector3D hit_p = r.o + r.d * isect.t;
		Vector3D w_out = w2o * (-r.d);

		if (r.depth <= 0) return Vector3D();

		L_out += surface_one_bounce_radiance(r, isect);

		double prob = 0.65;
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
			if (cos_theta > 0) {
				double transmittance = medium->transmittance(isect.t);
				L_out += at_least_one_bounce_radiance(next_ray) * f * cos_theta / pdf * transmittance / prob;
			}
		}
	}

	return L_out;
}

Vector3D VolumetricPathTracer::est_radiance_global_illumination(const Ray &r) {
	Intersection isect;
	Vector3D L_out;

	// If no intersection occurs, we simply return black / environment map.
	
	if (!bvh->intersect(r, &isect))
		return envLight ? envLight->sample_dir(r) : L_out;

	// prev code
	// return (isect.t == INF_D) ? debug_shading(r.d) : normal_shading(isect.n);

	// TODO (Part 3): Return the direct illumination.

	L_out = zero_bounce_radiance(r, isect);

	// TODO (Part 4): Accumulate the "direct" and "indirect"
	// parts of global illumination into L_out rather than just direct

	L_out += at_least_one_bounce_radiance(r);

	return L_out;
}

void VolumetricPathTracer::raytrace_pixel(size_t x, size_t y) {
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
		r.medium_depth = max_medium_ray_depth; // <- added this
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

} // namespace CGL