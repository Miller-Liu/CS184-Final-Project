#include "medium.h"
#include "util/random_util.h"
#include "vector3D.h"

namespace CGL {

double Medium::scatter_proba() const {
	return sigma_s / sigma_t;
}

double Medium::get_extinction() const {
	return sigma_t;
}

double Medium::transmittance(double d) const {
	// Beer-Lambert law: T(t) = exp(-sigma_t * t)
    return std::exp(-sigma_t * d);
}

double Medium::sample_distance(double* pdf) const {
	// Sample random distance based on 
	double s = -log(1 - random_uniform()) / sigma_t;
	*pdf = sigma_t * exp(-sigma_t * s);
  	return s;
}

} // namespace CGL
