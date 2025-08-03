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

	return true;
}

bool Sphere::has_intersection(const Ray &r) const {

	// TODO (Part 1.4):
	// Implement ray - sphere intersection.
	// Note that you might want to use the the Sphere::test helper here.
	
	/*
	 o = origin of ray
	 d = direction of ray
	 c = center of circle
	 R = radius of circle
	 || (o + d * t) - c || ^ 2 = R ^ 2
	 (td)^2 + 2 * td * (o - c) + (o - c)^2 = R^2
	 t^2 + 2 * td * (o - c) + (o - c)^2 - R^2 = 0
	*/
	double b = 2.0 * dot(r.d, r.o - o);
	double c = dot(r.o - o, r.o - o) - r2;
	double discriminant = b * b - 4 * c;

	if (discriminant < 0) return false;

	double t_minus = (-b - sqrt(discriminant)) / 2.0;
	double t_plus = (-b + sqrt(discriminant)) / 2.0;

	return (r.min_t <= t_minus && t_minus <= r.max_t) || (r.min_t <= t_plus && t_plus <= r.max_t);
}

bool Sphere::intersect(const Ray &r, Intersection *isect) const {

	// TODO (Part 1.4):
	// Implement ray - sphere intersection.
	// Note again that you might want to use the the Sphere::test helper here.
	// When an intersection takes place, the Intersection data should be updated
	// correspondingly.

	// Check if there is an intersection
	if (!has_intersection(r)) {
		return false;
	}
	
	/*
	 o = origin of ray
	 d = direction of ray
	 c = center of circle
	 R = radius of circle
	 || (o + d * t) - c || ^ 2 = R ^ 2
	 (td)^2 + 2 * td * (o - c) + (o - c)^2 = R^2
	 t^2 + 2 * td * (o - c) + (o - c)^2 - R^2 = 0
	*/
	double b = 2.0 * dot(r.d, r.o - o);
	double c = dot(r.o - o, r.o - o) - r2;
	double discriminant = b * b - 4 * c;
	double t_minus = (-b - sqrt(discriminant)) / 2.0;
	double t_plus = (-b + sqrt(discriminant)) / 2.0;

	// Update intersection structure
	isect->t = t_minus >= r.min_t ? t_minus : t_plus;
	isect->n = (r.o + isect->t * r.d - o).unit();
	isect->primitive = this;
	isect->bsdf = get_bsdf();

	// Update ray max_t (essentially telling ray to ignoring items after this)
	r.max_t = isect->t;

	return true;
}

void Sphere::draw(const Color &c, float alpha) const {
	Misc::draw_sphere_opengl(o, r, c);
}

void Sphere::drawOutline(const Color &c, float alpha) const {
	// Misc::draw_sphere_opengl(o, r, c);
}

} // namespace SceneObjects
} // namespace CGL
