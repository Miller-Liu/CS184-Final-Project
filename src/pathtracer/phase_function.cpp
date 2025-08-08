#include "phase_function.h"

#include "application/visual_debugger.h"
#include "misc.h"
#include "pathtracer/sampler.h"
#include "util/random_util.h"
#include "vector3D.h"

#include <algorithm>
#include <iostream>
#include <utility>


using std::max;
using std::min;
using std::swap;

namespace CGL {

double IsotropicPhaseFunction::p(const Vector3D wo, const Vector3D wi) const {
	return 1.0 / (4.0 * PI);
}

Vector3D IsotropicPhaseFunction::get_sample(const Vector3D &w_out, double *pdf) const {
	// Copied from sampler.cpp
	double z = random_uniform() * 2 - 1;
	double sinTheta = sqrt(std::max(0.0, 1.0f - z * z));

	double phi = 2.0f * PI * random_uniform();

	Vector3D w_in = Vector3D(cos(phi) * sinTheta, sin(phi) * sinTheta, z);
	(*pdf) = p(w_in, w_out);

	return w_in;
}

double HenyeyGreensteinPhaseFunction::p(const Vector3D wo, const Vector3D wi) const {
	double cos_theta = dot(wi, wo) / (wi.norm() * wo.norm());
    double denom = 1 + g * g - 2 * g * cos_theta;
    return (1.0 / (4 * M_PI)) * (1 - g * g) / (denom * sqrt(denom));
}

Vector3D HenyeyGreensteinPhaseFunction::get_sample(const Vector3D &w_out, double *pdf) const {
    double u1 = random_uniform();
    double u2 = random_uniform();

    double cos_theta;
    if (fabs(g) < 1e-3) {
      	cos_theta = 1 - 2 * u1;  // isotropic scattering
    } else {
		double sqr_term = (1 - g * g) / (1 - g + 2 * g * u1);
		cos_theta = (1 + g * g - sqr_term * sqr_term) / (2 * g);
    }
    double sin_theta = sqrt(max(0.0, 1 - cos_theta * cos_theta));
    double phi = 2 * M_PI * u2;

	Vector3D w_in = Vector3D(sin_theta * cos(phi), sin_theta * sin(phi), cos_theta);
	(*pdf) = p(w_in, w_out);

    return w_in;
}

} // namespace CGL
