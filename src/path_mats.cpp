#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/bsdf.h>
#include <nori/sampler.h>
 
NORI_NAMESPACE_BEGIN
 
class PathMATS : public Integrator {
public:
    PathMATS(const PropertyList &props) {
        /* No parameters this time */
    }
 
    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
        Ray3f traceRay = ray;

        // Initial radiance and throughput
        Color3f color_this_pixel = 0.f;
        // Initial throughput
        // At first I use float, but t*= radiance part has problem float * array.
        Color3f t(1.f);
        
        int bounce = 0;
        
        while(true){
            Intersection its;
            /* if intersect */
            if (!scene->rayIntersect(traceRay, its))
                break;
            // if the ray hits the emitter
            if(its.mesh->isEmitter()){
                // Create a query record that can be used to query the sampling density after having intersected an area emitter
                // Intersect point ref is the location of the camera
                // p is sample point on the emiter, n is the normal at the emitter point
                EmitterQueryRecord lRec(traceRay.o,its.p,its.shFrame.n);
                Color3f emitterclolor = its.mesh->getEmitter()->eval(lRec);
                // L += t * L_e(x_o); // Contrib from material sampling
                color_this_pixel += t * emitterclolor;
            }
            
            // Russian Roulettes
            if(bounce>3)
            {float successProbability = std::min<float>(t.maxCoeff(), 0.99);
            if (sampler->next1D() > successProbability) break;
                else t /= successProbability;}

            // w_i = sample from BSDF at x_o;
            // BSDFQueryRecord (const Vector3f &wi)
 	        // Create a new record for sampling the BSDF.
            // wi: Incident direction (in the local frame)
            // Remember to normalized since I assume wi is a normalized vector
            BSDFQueryRecord bRec(its.shFrame.toLocal(- traceRay.d).normalized());

            bRec.uv = its.uv;

            // Different to previous code, here use its.mesh to use getBSDF function
            Color3f radiance = its.mesh->getBSDF()->sample(bRec,sampler->next2D());

            t *= radiance;
            traceRay = Ray3f(its.p,its.shFrame.toWorld(bRec.wo),Epsilon,INFINITY);
            bounce++;
        }
      
        return color_this_pixel;
    }    
 
    /// Return a human-readable description for debugging purposes
    std::string toString() const {
        return "PathIntegrator[]";
    }
};
 
NORI_REGISTER_CLASS(PathMATS, "path_mats");
NORI_NAMESPACE_END