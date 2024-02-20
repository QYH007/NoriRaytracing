#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/bsdf.h>
#include <nori/sampler.h>
 
NORI_NAMESPACE_BEGIN
 
class PathMISMip : public Integrator {
public:
    PathMISMip(const PropertyList &props) {
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
        const int light_num = scene->getLights().size();

        // Init wmat
        float w_mat = 1.f;
        float w_em = 1.f;

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
                // L += w_mat * t * L_e(x_o); // Contrib from mats
                color_this_pixel += w_mat * t * emitterclolor;
            }

            //Russian Roulette
            if(bounce>3)
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
                Color3f Emitter_radiance = light->sample(lRec,sampler->next2D());

                // https://gitlab.inf.ethz.ch/OU-CG-TEACHING/nori-base-23/-/issues/81
                // float ems_pdf_em = light->pdf(lRec) * (1/light_num);
                // Feedback: Your pdf_em does not take into account the number of lights in computing w_em and w_mat. 
                // w_em and w_mat still sum to 1 for every direction, so the rendering is unbiased, but they don't 
                // correspond to the balance heuristics. Same for your direct_mis implementation in PA3. 
                // In your case, both ems_pdf_em and mats_pdf_em should divide by the number of lights.
                float ems_pdf_em = light->pdf(lRec) / light_num;

                // check if the light is occluded
                // rayIntersect can also be called without an Intersection object
                // Intersection its_shadow;
                if(!scene->rayIntersect(lRec.shadowRay)){

                    // https://cgl.ethz.ch/teaching/cg23/www-nori/docs/structBSDFQueryRecord.html#details
                    // BSDFQueryRecord (const Vector3f &wi, const Vector3f &wo, EMeasure measure)
                    BSDFQueryRecord bRec = BSDFQueryRecord(its.shFrame.toLocal(-traceRay.d),its.shFrame.toLocal(lRec.wi),  ESolidAngle);
                    bRec.uv = its.uv;
                    bRec.p = its.p;
                    //Color3f bsdfclolor = its.mesh->getBSDF()->eval(bRec);
                    Color3f bsdfclolor = its.mesh->getBSDF()->evalMipmap(bRec, its.L);
                    if(its.L<1){
                        return Color3f(1.0, 0.9, 0.9);
                    }
                    if(its.L<2 && its.L>=1){
                        return Color3f(1.0, 0.8, 0.8);
                    }
                    if(its.L<3 && its.L>=2){
                        return Color3f(1.0, 0.7, 0.7);
                    }
                    if(its.L<4 && its.L>=3){
                        return Color3f(1.0, 0.6, 0.6);
                    }
                    if(its.L<5 && its.L>=4){
                        return Color3f(1.0, 0.5, 0.5);
                    }
                    if(its.L<6 && its.L>=5){
                        return Color3f(1.0, 0.4, 0.4);
                    }
                    if(its.L<7 && its.L>=6){
                        return Color3f(1.0, 0.3, 0.3);
                    }
                    float ems_pdf_mat = its.mesh->getBSDF()->pdf(bRec);

                    // the cosine term between the shading normal and the direction towards the light source
                    // face normal and wi are all normalized, use the dot product to compute cosine term
                    float cosine_term = its.shFrame.n.dot(lRec.wi);
                    if((ems_pdf_em + ems_pdf_mat)>Epsilon)
                        w_em = ems_pdf_em / (ems_pdf_em + ems_pdf_mat);
                    else
                        w_em = ems_pdf_em;

                    color_this_pixel += Emitter_radiance * bsdfclolor * cosine_term * light_num * w_em * t;
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
                if(scene->rayIntersect(traceRay,its_shadow)){

                    // check whether the hit point is emitter
                    if(its_shadow.mesh->isEmitter()){
                    // Create a query record that can be used to query the sampling density after having intersected an area emitter
                    // Intersect point ref is the location of the camera
                    // p is sample point on the emiter, n is the normal at the emitter point
                    EmitterQueryRecord lRec(traceRay.o,its_shadow.p,its_shadow.shFrame.n);
                    // Color3f emitterclolor = its_shadow.mesh->getEmitter()->eval(lRec);
                    // Feedback: Your pdf_em does not take into account the number of lights in computing w_em and w_mat. 
                    // w_em and w_mat still sum to 1 for every direction, so the rendering is unbiased, but they don't 
                    // correspond to the balance heuristics. Same for your direct_mis implementation in PA3. 
                    // In your case, both ems_pdf_em and mats_pdf_em should divide by the number of lights.
                    float mats_pdf_em = its_shadow.mesh->getEmitter()->pdf(lRec)/light_num;
                    if((mats_pdf_em + mats_pdf_mat)>Epsilon)
                        w_mat = mats_pdf_mat / (mats_pdf_em + mats_pdf_mat);
                    else
                        w_mat = mats_pdf_mat;
                    }
                
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
        return "PathIntegrator[]";
    }
};
 
NORI_REGISTER_CLASS(PathMISMip, "path_mis_mip");
NORI_NAMESPACE_END