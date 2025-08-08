#ifndef CGL_MEDIUM_H
#define CGL_MEDIUM_H

#include "CGL/vector2D.h"
#include "CGL/vector3D.h"
#include "CGL/misc.h"
#include "pathtracer/phase_function.h"
#include "util/random_util.h"

namespace CGL {

class Medium {
	public:
		Medium(double sigma_s, double sigma_a, PhaseFunction* pf) 
			: sigma_s(sigma_s), sigma_a(sigma_a), sigma_t(sigma_s + sigma_a), pf(pf) {};

		virtual ~Medium() {}

		// Get scattering probability
		double scatter_proba() const;

		// Get sigma_t
		double get_extinction() const;

		// Transmittance using Beer-Lambert law
		double transmittance(double d) const;

		// Samples distance to next interaction
		double sample_distance(double* pdf) const;

		// Get phase function
		PhaseFunction* phase_function() const { return pf; }

	protected:
		double sigma_s; // scattering
		double sigma_a; // absorption
		double sigma_t; // extinction = sigma a + sigma s
		PhaseFunction* pf;
};

} // namespace CGL

#endif // CGL_MEDIUM_H