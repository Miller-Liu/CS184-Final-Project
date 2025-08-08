#ifndef CGL_VOLUMETRIC_PATHTRACER_H
#define CGL_VOLUMETRIC_PATHTRACER_H

#include "pathtracer/pathtracer.h"

#include "CGL/timer.h"

#include "pathtracer/medium.h"
#include "scene/bvh.h"
#include "pathtracer/sampler.h"
#include "pathtracer/intersection.h"

#include "application/renderer.h"

#include "scene/scene.h"
#include "vector3D.h"
using CGL::SceneObjects::Scene;

#include "scene/environment_light.h"
using CGL::SceneObjects::EnvironmentLight;

using CGL::SceneObjects::BVHNode;
using CGL::SceneObjects::BVHAccel;

namespace CGL {

    class VolumetricPathTracer : public PathTracer {
    public:
        VolumetricPathTracer();
        ~VolumetricPathTracer();

        /**
         * Normal surface pathtracing
         */
        Vector3D estimate_direct_lighting_hemisphere(const Ray& r, const SceneObjects::Intersection& isect) override;
        Vector3D estimate_direct_lighting_importance(const Ray& r, const SceneObjects::Intersection& isect) override;
        Vector3D surface_one_bounce_radiance(const Ray& r, const SceneObjects::Intersection& isect);

        /**
         * Medium pathtracing
         */
        Vector3D estimate_medium_direct_lighting_default(const Ray& r, double d);
        Vector3D estimate_medium_direct_lighting_importance(const Ray& r, double d);
        Vector3D medium_one_bounce_radiance(const Ray& r, double d);

        Vector3D est_radiance_global_illumination(const Ray& r) override;
        Vector3D zero_bounce_radiance(const Ray& r, const SceneObjects::Intersection& isect) override;
        using PathTracer::at_least_one_bounce_radiance;
        Vector3D at_least_one_bounce_radiance(const Ray& r);
    
        size_t max_medium_ray_depth;
        void raytrace_pixel(size_t x, size_t y) override;

        Medium* medium;
    };

}  // namespace CGL

#endif  // CGL_VOLUMETRIC_PATHTRACER_H
