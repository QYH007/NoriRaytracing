// Develop based on the provided normals.cpp
#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/warp.h>

NORI_NAMESPACE_BEGIN

class AverageVisibility : public Integrator {
public:
    AverageVisibility(const PropertyList &props) {
        /* parameter here is length */
        m_length = (size_t) props.getFloat("length", 1.f);
    }

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
        /* Find the surface that is visible in the requested direction */
        Intersection its;
        /* When there is no intersection, return Color3f(1.0f) */
        if (!scene->rayIntersect(ray, its))
            return Color3f(1.0f);

        Point3f intersect_point = its.p;
        // Normal3f n = its.shFrame.n;
        // Feedback: for now it probably doesn't affect the performance but it might at a later stage
        //generate a point on the hemisphere
        Vector3f direction = Warp::sampleUniformHemisphere(sampler,its.shFrame.n);

        //TRay (const PointType &o, const VectorType &d, Scalar mint, Scalar maxt)
        // https://gitlab.inf.ethz.ch/OU-CG-TEACHING/nori-base-23/-/issues/11
        Ray3f ray_av = Ray3f(intersect_point,direction,Epsilon,m_length);

        // Feedback: rayIntersect can also be called without an Intersection object 
        if (!scene->rayIntersect(ray_av))
            return Color3f(1.0f);
        else
            return Color3f(0.0f);

    }

    std::string toString() const {
        return tfm::format(
            "AverageVisibility[\n"
            "  length = \"%s\"\n"
            "]",
            m_length
        );
    }
    protected:
    float m_length;
};

NORI_REGISTER_CLASS(AverageVisibility, "av");
NORI_NAMESPACE_END