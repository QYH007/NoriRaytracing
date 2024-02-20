#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/bsdf.h>
#include <nori/sampler.h>
 
NORI_NAMESPACE_BEGIN
 
class DirectMATS : public Integrator {
public:
    DirectMATS(const PropertyList &props) {
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
        // const std::vector<Emitter *> &lights = scene->getLights();
        // at each integrator call you should sample only a single emitter
        // const Emitter *light = scene->getRandomEmitter(sampler->next1D());
        // At first I try use random emitter, however it did not pass the pointlight test
        // Look at the result of sphere pointlight, it looks darker than reference
        // Hence I use all lights back and get expected result.

        // if the ray hits the emitter
        if(its.mesh->isEmitter()){
            // Create a query record that can be used to query the sampling density after having intersected an area emitter
            // Intersect point ref is the location of the camera
            // p is sample point on the emiter, n is the normal at the emitter point
            EmitterQueryRecord lRec(ray.o,its.p,its.shFrame.n);
            Color3f emitterclolor = its.mesh->getEmitter()->eval(lRec);
            color_this_pixel += emitterclolor;
        }

        // for (const Emitter *light: lights) {
            // Since this part are related to microfacet.cpp, which inherit from bsdf
            // So BSDFQueryRecord bRec is used here
            // BSDFQueryRecord (const Vector3f &wi)
 	        // Create a new record for sampling the BSDF.
            // wi: Incident direction (in the local frame)
            // Remember to normalized since I assume wi is a normalized vector
            BSDFQueryRecord bRec(its.shFrame.toLocal(-ray.d).normalized());

            bRec.uv = its.uv;

            // Different to previous code, here use its.mesh to use getBSDF function
            Color3f radiance = its.mesh->getBSDF()->sample(bRec,sampler->next2D());

            // check if the light is occluded
            // Intersection its_shadow;
            // TRay (const PointType &o, const VectorType &d, Scalar mint, Scalar maxt)
            // bRec.wo: Outgoing direction (in the local frame)
            // since the Incident direction has been given the ray.o in previous code
            // here the ray needed to be check is the outgoing one
            // No intersect test here, so the maxt is set to infinity
            // Remember to change the wo back to world coord since it is local one in bRec
            Ray3f shadowRay = Ray3f(its.p,its.shFrame.toWorld(bRec.wo),Epsilon,INFINITY);
            Intersection its_shadow;

            // if intersect
            if(scene->rayIntersect(shadowRay,its_shadow)){

                // check whether the hit point is emitter
                if(its_shadow.mesh->isEmitter()){
                // Create a query record that can be used to query the sampling density after having intersected an area emitter
                // Intersect point ref is the location of the camera
                // p is sample point on the emiter, n is the normal at the emitter point
                EmitterQueryRecord lRec(shadowRay.o,its_shadow.p,its_shadow.shFrame.n);
                Color3f emitterclolor = its_shadow.mesh->getEmitter()->eval(lRec);
                color_this_pixel += emitterclolor * radiance;
                }
            }
            else{
                
                
            }
        // }        
        return color_this_pixel;
    }
 
    /// Return a human-readable description for debugging purposes
    std::string toString() const {
        return "DirectIntegrator[]";
    }
};
 
NORI_REGISTER_CLASS(DirectMATS, "direct_mats");
NORI_NAMESPACE_END