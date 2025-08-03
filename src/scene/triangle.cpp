#include "triangle.h"

#include "CGL/CGL.h"
#include "GL/glew.h"
#include "pathtracer/intersection.h"
#include "vector2D.h"
#include "vector3D.h"

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

	// Triangle edges 
	Vector3D e1 = p2 - p1;
	Vector3D e2 = p3 - p1;

	// check for if rayis parallel to triangle face
	if (dot(r.d, cross(e1, e2)) == 0){
		return false;
	}

	/*
	 Möller-Trumbore Algorithm
	 o + d * t = p1 + u * e1 + v * e2
	 o - p1 = [-d, e1, e2] @ [t, u, v]
	 We have Ax = s. so we can use Cramer's Rule to solve for t, u and v
	 t = det(A0) / det(A)
	 u = det(A2) / det(A)
	 v = det(A3) / det(A)
	 Where Ai is the matrix A with column replaced by s
	*/ 
	Vector3D s = r.o - p1;
	double det = dot(-r.d, cross(e1, e2));

	// Test u coordinate
	double u = dot(-r.d, cross(s, e2)) / det;
	if (u < 0.0 || u > 1.0) return false;

	// Test v coordinate
	double v = dot(-r.d, cross(e1, s)) / det;
	if (v < 0.0 || v > 1.0) return false;

	// Test w coordinate
	double w = 1 - u - v;
	if (w < 0.0 || w > 1.0) return false;

	// Test t
	double t = dot(s, cross(e1, e2)) / det;
	if (t < r.min_t || t > r.max_t) return false;

	return true;
}

bool Triangle::intersect(const Ray &r, Intersection *isect) const {
	// Part 1, Task 3:
	// implement ray-triangle intersection. When an intersection takes
	// place, the Intersection data should be updated accordingly

	// Check if there is an intersection
	if (!has_intersection(r)) {
		return false;
	}

	/*
	 Möller-Trumbore Algorithm
	 o + d * t = p1 + u * e1 + v * e2
	 o - p1 = [-d, e1, e2] @ [t, u, v]
	 We have Ax = s. so we can use Cramer's Rule to solve for t, u and v
	 t = det(A0) / det(A)
	 u = det(A2) / det(A)
	 v = det(A3) / det(A)
	 Where Ai is the matrix A with column replaced by s
	*/ 
	Vector3D e1 = p2 - p1;
	Vector3D e2 = p3 - p1;
	Vector3D s = r.o - p1;
	double det = dot(-r.d, cross(e1, e2));

	// Calculate u, v, w, and t # pay attention to which vertex corresponds to what
	double u = dot(-r.d, cross(s, e2)) / det;
	double v = dot(-r.d, cross(e1, s)) / det;
	double w = 1 - u - v;
	double t = dot(s, cross(e1, e2)) / det;

	// Update intersection structure
	isect->t = t;
	isect->n = (w * n1 + u * n2 + v * n3).unit();
	isect->primitive = this;
	isect->bsdf = get_bsdf();

	// Update ray max_t (essentially telling ray to ignoring items after this)
	r.max_t = isect->t;

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
