#ifndef CGL_PHASE_FUNCTION_H
#define CGL_PHASE_FUNCTION_H

#include "CGL/CGL.h"
#include "CGL/vector3D.h"
#include "CGL/matrix3x3.h"

#include "pathtracer/sampler.h"
#include "util/image.h"

#include <algorithm>

namespace CGL {
	
/**
 * PhaseFunction
 */
class PhaseFunction {
	public:
		virtual ~PhaseFunction() {}

		// Evaluate phase function
		virtual double p(const Vector3D wo, const Vector3D wi) const = 0;

		//
		virtual Vector3D get_sample(const Vector3D& w_out, double *pdf) const = 0;

		//
		virtual Vector3D get_sample(const Vector3D& w_out) const {
			double dummy_pdf;
			return get_sample(w_out, &dummy_pdf);
		};

}; // class PhaseFunction

class IsotropicPhaseFunction : public PhaseFunction {
	public:
		// Probability
		double p(const Vector3D wo, const Vector3D wi) const;
		Vector3D get_sample(const Vector3D& w_out, double *pdf) const;
};

class HenyeyGreensteinPhaseFunction : public PhaseFunction {
	public:
		HenyeyGreensteinPhaseFunction(double asymmetry) : g(asymmetry) {};
		double p(const Vector3D wo, const Vector3D wi) const;
		Vector3D get_sample(const Vector3D& w_out, double *pdf) const;

	protected:
		double g;
};

}  // namespace CGL

#endif  // CGL_PHASE_FUNCTION_H
