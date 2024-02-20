#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/bsdf.h>
#include <nori/sampler.h>
 
NORI_NAMESPACE_BEGIN
 
class Direct : public Integrator {
public:
    Direct(const PropertyList &props) {
        /* No parameters this time */
    }
 
    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
        /* Find the surface that is visible in the requested direction */
        Intersection its;
        if (!scene->rayIntersect(ray, its))
            return Color3f(0.0f);

        /* if intersect */
        // init the color in this pixel
        Color3f color_this_pixel = 0.f;
        // Feedback: for now it probably doesn't affect the performance but it might at a later stage.
        // Normal3f n = its.shFrame.n;

        // Get all lights
        // scene.h#L64-65
        // const std::vector<Emitter *> &getLights() const { return m_emitters; }
        const std::vector<Emitter *> &lights = scene->getLights();

        for (const Emitter *light : lights) {

            EmitterQueryRecord lRec;
            //  Point3f intersect_point = its.p;
            // lRec.ref is the hit point
            lRec.ref = its.p;

            // Sample the light source to get radiance
            // Here sample attribute is not used in my pointlight.cpp, set it to Point2f(0.f,0.f)
            // Feedback: ok, but better use `sampler->next2D()` instead of `Point2f(0, 0)` as the sample parameter to make it compatible with area lights in PA3.
            Color3f radiance = light->sample(lRec,sampler->next2D());

            // check if the light is occluded
            // rayIntersect can also be called without an Intersection object
            // Intersection its_shadow;
            // you can safely assume that there will be no emission at the first intersection
            if(!scene->rayIntersect(lRec.shadowRay)){

                // https://cgl.ethz.ch/teaching/cg23/www-nori/docs/structBSDFQueryRecord.html#details
                // BSDFQueryRecord (const Vector3f &wi, const Vector3f &wo, EMeasure measure)
                BSDFQueryRecord bRec = BSDFQueryRecord(its.shFrame.toLocal(lRec.wi), its.shFrame.toLocal(-ray.d), ESolidAngle);
                bRec.uv = its.uv;
                bRec.p = its.p;

                Color3f bsdfclolor = its.mesh->getBSDF()->eval(bRec);

                // the cosine term between the shading normal and the direction towards the light source
                // face normal and wi are all normalized, use the dot product to compute cosine term
                float cosine_term = its.shFrame.n.dot(lRec.wi);

                color_this_pixel += radiance * bsdfclolor * cosine_term;
            }
        }        
        return color_this_pixel;
    }
 
    /// Return a human-readable description for debugging purposes
    std::string toString() const {
        return "DirectIntegrator[]";
    }
};
 
NORI_REGISTER_CLASS(Direct, "direct");
NORI_NAMESPACE_END