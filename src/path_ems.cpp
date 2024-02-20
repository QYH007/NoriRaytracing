#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/bsdf.h>
#include <nori/sampler.h>
 
NORI_NAMESPACE_BEGIN
 
class PathEMS : public Integrator {
public:
    PathEMS(const PropertyList &props) {
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
            const Emitter *light = scene->getRandomEmitter(sampler->next1D());

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
                color_this_pixel += t* emitterclolor;
            }
            
            // Russian Roulettes
            if(bounce>3)
            {float successProbability = std::min<float>(t.maxCoeff(), 0.99);
            if (sampler->next1D() > successProbability) break;
                else t /= successProbability;}

            EmitterQueryRecord lRec;
            //  Point3f intersect_point = its.p;
            // lRec.ref is the hit point
            lRec.ref = its.p;
            Color3f radiance = light->sample(lRec,sampler->next2D());

            if(!scene->rayIntersect(lRec.shadowRay)){

                // https://cgl.ethz.ch/teaching/cg23/www-nori/docs/structBSDFQueryRecord.html#details
                // BSDFQueryRecord (const Vector3f &wi, const Vector3f &wo, EMeasure measure)
                BSDFQueryRecord bRec = BSDFQueryRecord(its.shFrame.toLocal(lRec.wi), its.shFrame.toLocal(-traceRay.d), ESolidAngle);
                bRec.uv = its.uv;
                bRec.p = its.p;

                Color3f bsdfclolor = its.mesh->getBSDF()->eval(bRec);

                // the cosine term between the shading normal and the direction towards the light source
                // face normal and wi are all normalized, use the dot product to compute cosine term
                float cosine_term = its.shFrame.n.dot(lRec.wi);

                color_this_pixel += radiance * bsdfclolor * cosine_term * t;
                
            }
            BSDFQueryRecord bRec(its.shFrame.toLocal(-traceRay.d).normalized());
            Color3f BSDF_radiance = its.mesh->getBSDF()->sample(bRec,sampler->next2D());
            
            t *= BSDF_radiance;
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
 
NORI_REGISTER_CLASS(PathEMS, "path_ems");
NORI_NAMESPACE_END