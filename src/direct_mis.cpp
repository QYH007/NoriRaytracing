#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/bsdf.h>
#include <nori/sampler.h>
 
NORI_NAMESPACE_BEGIN
 
class DirectMIS : public Integrator {
public:
    DirectMIS(const PropertyList &props) {
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
        const Emitter *light = scene->getRandomEmitter(sampler->next1D());
        // At first I try use random emitter, however it did not pass the pointlight test
        // Look at the result of sphere pointlight, it looks darker than reference
        // Hence I use all lights back and get expected result.
        // Update: Try to multipy it with light numbers. Hope it could work.
        const int light_num = scene->getLights().size();

        // if the ray hits the emitter
        if(its.mesh->isEmitter()){
            // Create a query record that can be used to query the sampling density after having intersected an area emitter
            // Intersect point ref is the location of the camera
            // p is sample point on the emiter, n is the normal at the emitter point
            EmitterQueryRecord lRec(ray.o,its.p,its.shFrame.n);
            Color3f emitterclolor = its.mesh->getEmitter()->eval(lRec);
            color_this_pixel += emitterclolor;
        }

        /***
        Light Sampling part
        ***/
            EmitterQueryRecord lRec;
            //  Point3f intersect_point = its.p;
            // lRec.ref is the hit point
            lRec.ref = its.p;

            // Sample the light source to get radiance
            // Here sample attribute is not used in my pointlight.cpp, set it to Point2f(0.f,0.f)
            // Feedback: ok, but better use `sampler->next2D()` instead of `Point2f(0, 0)` as the sample parameter to make it compatible with area lights in PA3.
            Color3f Emitter_radiance = light->sample(lRec,sampler->next2D());

            // Feedback: Your pdf_em does not take into account the number of lights in computing w_em and w_mat. 
            // w_em and w_mat still sum to 1 for every direction, so the rendering is unbiased, but they don't 
            // correspond to the balance heuristics. Same for your direct_mis implementation in PA3. 
            // In your case, both ems_pdf_em and mats_pdf_em should divide by the number of lights.
            float ems_pdf_em = light->pdf(lRec) / light_num;

            // check if the light is occluded
            // rayIntersect can also be called without an Intersection object
            // Intersection its_shadow;
            // you can safely assume that there will be no emission at the first intersection
            if(!scene->rayIntersect(lRec.shadowRay)){

                // https://cgl.ethz.ch/teaching/cg23/www-nori/docs/structBSDFQueryRecord.html#details
                // BSDFQueryRecord (const Vector3f &wi, const Vector3f &wo, EMeasure measure)
                BSDFQueryRecord bRec = BSDFQueryRecord(its.shFrame.toLocal(-ray.d),its.shFrame.toLocal(lRec.wi),  ESolidAngle);
                bRec.uv = its.uv;
                bRec.p = its.p;
                Color3f bsdfclolor = its.mesh->getBSDF()->eval(bRec);
                float ems_pdf_mat = its.mesh->getBSDF()->pdf(bRec);

                // the cosine term between the shading normal and the direction towards the light source
                // face normal and wi are all normalized, use the dot product to compute cosine term
                float cosine_term = its.shFrame.n.dot(lRec.wi);

                // if((ems_pdf_em + ems_pdf_mat)>1e-8)
                //     color_this_pixel += Emitter_radiance * bsdfclolor * cosine_term * light_num * ems_pdf_em / (ems_pdf_em + ems_pdf_mat);
                // else
                //     color_this_pixel += Emitter_radiance * bsdfclolor * cosine_term * light_num;
                color_this_pixel += Emitter_radiance * bsdfclolor * cosine_term * light_num * ems_pdf_em / (ems_pdf_em + ems_pdf_mat);
            }
        
        /***
        BRDF Sampling part
        ***/
            BSDFQueryRecord bRec(its.shFrame.toLocal(-ray.d).normalized());

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
                // Feedback: Your pdf_em does not take into account the number of lights in computing w_em and w_mat. 
                // w_em and w_mat still sum to 1 for every direction, so the rendering is unbiased, but they don't 
                // correspond to the balance heuristics. Same for your direct_mis implementation in PA3. 
                // In your case, both ems_pdf_em and mats_pdf_em should divide by the number of lights.
                float mats_pdf_em = its_shadow.mesh->getEmitter()->pdf(lRec) / light_num;
                // if((mats_pdf_em + mats_pdf_mat)>1e-8)
                //     color_this_pixel += emitterclolor * BSDF_radiance * mats_pdf_mat / (mats_pdf_em + mats_pdf_mat);
                // else
                //     color_this_pixel += emitterclolor * BSDF_radiance;
                color_this_pixel += emitterclolor * BSDF_radiance * mats_pdf_mat / (mats_pdf_em + mats_pdf_mat);
                }
                
            }
            
     
        return color_this_pixel;
    }
 
    /// Return a human-readable description for debugging purposes
    std::string toString() const {
        return "DirectIntegrator[]";
    }
};
 
NORI_REGISTER_CLASS(DirectMIS, "direct_mis");
NORI_NAMESPACE_END