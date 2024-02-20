#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/bsdf.h>
#include <nori/sampler.h>
 
NORI_NAMESPACE_BEGIN
 
class PathMIS : public Integrator {
public:
    PathMIS(const PropertyList &props) {
        /* No parameters this time */
    }
 
    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
        Ray3f traceRay = ray;

        // Initial radiance and throughput
        Color3f color_this_pixel = 0.f;
        // Initial throughput
        // At first I use float, but t*= radiance part has problem float * array.
        Color3f t(1.f);

        // Get number of light from the scene
        // Could be reused so extract out
        Emitter *envEmitter = nullptr;
        
        const std::vector<Emitter *> &lights = scene->getLights();
        const int light_num = lights.size();
        for (Emitter *currentLight : lights) {
            if (currentLight->isEnvEmitter()){
                envEmitter = currentLight;
                break;
            }
        }

        // Init wmat
        float w_mat = 1.f;
        float w_em = 1.f;

        int bounce = 0;        
        
        while(true){
            Intersection its;
            /* if intersect */
            if (!scene->rayIntersect(traceRay, its)){
                if (envEmitter != nullptr) {
                    EmitterQueryRecord eqr(traceRay.o, -traceRay.d);
                    color_this_pixel += w_mat * t * envEmitter->Le(eqr) * light_num;
                }
                break;
            }
            // if the ray hits the emitter
            if(its.mesh->isEmitter()){
                // Create a query record that can be used to query the sampling density after having intersected an area emitter
                // Intersect point ref is the location of the camera
                // p is sample point on the emiter, n is the normal at the emitter point
                EmitterQueryRecord lRec(traceRay.o,its.p,its.shFrame.n);
                Color3f emitterclolor = its.mesh->getEmitter()->eval(lRec) * light_num;
                // L += w_mat * t * L_e(x_o); // Contrib from mats
                color_this_pixel += w_mat * t * emitterclolor;
            }

            // Russian Roulette
            if(bounce>0)
            {float successProbability = std::min<float>(t.maxCoeff(), 0.99);
            if (sampler->next1D() > successProbability) break;
                else t /= successProbability;}

            // Contribution from emitter sampling

            // Get random light from the scene
            const Emitter *light = scene->getRandomEmitter(sampler->next1D());        

            /***
            Light Sampling part
            ***/
                EmitterQueryRecord lRec;
                //  Point3f intersect_point = its.p;
                // lRec.ref is the hit point
                lRec.ref = its.p;

                // Sample the light source to get radiance
                Color3f Emitter_radiance = light->sample(lRec, sampler->next2D());
                // std::cout<< Emitter_radiance[0] <<","<< Emitter_radiance[1] <<","<< Emitter_radiance[2] <<std::endl;

                // https://gitlab.inf.ethz.ch/OU-CG-TEACHING/nori-base-23/-/issues/81
                // float ems_pdf_em = light->pdf(lRec) * (1/light_num);
                float ems_pdf_em = light->pdf(lRec);
                // std::cout<< ems_pdf_em <<std::endl;

                // check if the light is occluded
                // rayIntersect can also be called without an Intersection object
                // Intersection its_shadow;
                if(!scene->rayIntersect(lRec.shadowRay)){

                    // https://cgl.ethz.ch/teaching/cg23/www-nori/docs/structBSDFQueryRecord.html#details
                    // BSDFQueryRecord (const Vector3f &wi, const Vector3f &wo, EMeasure measure)
                    BSDFQueryRecord bRec = BSDFQueryRecord(its.shFrame.toLocal(lRec.wi), its.shFrame.toLocal(-traceRay.d),  ESolidAngle);
                    bRec.uv = its.uv;
                    bRec.p = its.p;
                    Color3f bsdfclolor = its.mesh->getBSDF()->eval(bRec);
                    float ems_pdf_mat = its.mesh->getBSDF()->pdf(bRec);

                    // the cosine term between the shading normal and the direction towards the light source
                    // face normal and wi are all normalized, use the dot product to compute cosine term
                    float cosine_term = its.shFrame.n.dot(lRec.wi);
                    if((ems_pdf_em + ems_pdf_mat)>Epsilon)
                        w_em = ems_pdf_em / (ems_pdf_em + ems_pdf_mat);
                    else
                        w_em = ems_pdf_em;
                    
                    color_this_pixel += Emitter_radiance * bsdfclolor * cosine_term * light_num * w_em * t;
                    // std::cout<< color_this_pixel[0] <<","<< color_this_pixel[1] <<","<< color_this_pixel[2] <<std::endl;
                }
        
            /***
            BRDF Sampling part
            ***/
                BSDFQueryRecord bRec(its.shFrame.toLocal(-traceRay.d).normalized());

                bRec.uv = its.uv;

                // Different to previous code, here use its.mesh to use getBSDF function
                Color3f BSDF_radiance = its.mesh->getBSDF()->sample(bRec,sampler->next2D());
                float mats_pdf_mat = its.mesh->getBSDF()->pdf(bRec);

                // check if the light is occluded
                // Intersection its_shadow;
                // TRay (const PointType &o, const VectorType &d, Scalar mint, Scalar maxt)
                // bRec.wo: Outgoing direction (in the local frame)
                // since the Incident direction has been given the ray.o in previous code
                // here the ray needed to be check is the outgoing one
                // No intersect test here, so the maxt is set to infinity
                // Remember to change the wo back to world coord since it is local one in bRec
                // it used to be shadowRay here, now it is the same to the traceRay
                traceRay = Ray3f(its.p,its.shFrame.toWorld(bRec.wo),Epsilon,INFINITY);
                Intersection its_shadow;

                // if intersect
                if(scene->rayIntersect(traceRay, its_shadow)){

                    // check whether the hit point is emitter
                    if(its_shadow.mesh->isEmitter()){
                        // Create a query record that can be used to query the sampling density after having intersected an area emitter
                        // Intersect point ref is the location of the camera
                        // p is sample point on the emiter, n is the normal at the emitter point
                        EmitterQueryRecord lRec(traceRay.o,its_shadow.p,its_shadow.shFrame.n);
                        // Color3f emitterclolor = its_shadow.mesh->getEmitter()->eval(lRec);
                        float mats_pdf_em = its_shadow.mesh->getEmitter()->pdf(lRec);
                        if((mats_pdf_em + mats_pdf_mat)>Epsilon)
                            w_mat = mats_pdf_mat / (mats_pdf_em + mats_pdf_mat);
                        else
                            w_mat = mats_pdf_mat;
                    }
                
                } else {
                    // ray escaped, give it a random infinite light.
                    EmitterQueryRecord lRec(traceRay.o, traceRay.d);
                    float mats_pdf_em = envEmitter->pdf(lRec);
                    if((mats_pdf_em + mats_pdf_mat)>Epsilon)
                        w_mat = mats_pdf_mat / (mats_pdf_em + mats_pdf_mat);
                    else
                        w_mat = mats_pdf_mat;
                    // std::cout<< mats_pdf_em <<","<< mats_pdf_mat<<std::endl;
                }

                // Found some issue: emitter shown on ball is off. No focus on ground.
                // For delta BSDFs the MIS weights are
                // w_mat = 1, w_em = 0
                // You need to check whether the BSDF consists of delta functions and, if so, manually set the MIS weights
                if (bRec.measure == EDiscrete) {
                    w_mat = 1.0f;
                    w_em = 0.f;
                }
                t *= BSDF_radiance;  
                bounce++;          
        }
        return color_this_pixel;
    }
 
    /// Return a human-readable description for debugging purposes
    std::string toString() const {
        return "Path_mis[]";
    }
};
 
NORI_REGISTER_CLASS(PathMIS, "path_mis");
NORI_NAMESPACE_END