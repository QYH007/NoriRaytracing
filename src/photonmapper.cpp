/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob

    Nori is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License Version 3
    as published by the Free Software Foundation.

    Nori is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include <nori/integrator.h>
#include <nori/sampler.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>
#include <nori/scene.h>
#include <nori/photon.h>

NORI_NAMESPACE_BEGIN

class PhotonMapper : public Integrator {
public:
    /// Photon map data structure
    typedef PointKDTree<Photon> PhotonMap;

    PhotonMapper(const PropertyList &props) {
        /* Lookup parameters */
        m_photonCount  = props.getInteger("photonCount", 1000000);
        m_photonRadius = props.getFloat("photonRadius", 0.0f /* Default: automatic */);
    }

    virtual void preprocess(const Scene *scene) override {
        cout << "Gathering " << m_photonCount << " photons .. ";
        cout.flush();

        /* Create a sample generator for the preprocess step */
        Sampler *sampler = static_cast<Sampler *>(
            NoriObjectFactory::createInstance("independent", PropertyList()));

        /* Allocate memory for the photon map */
        m_photonMap = std::unique_ptr<PhotonMap>(new PhotonMap());
        m_photonMap->reserve(m_photonCount);

		/* Estimate a default photon radius */
		if (m_photonRadius == 0)
			m_photonRadius = scene->getBoundingBox().getExtents().norm() / 500.0f;

	

		/* How to add a photon?
		 * m_photonMap->push_back(Photon(
		 *	Point3f(0, 0, 0),  // Position
		 *	Vector3f(0, 0, 1), // Direction
		 *	Color3f(1, 2, 3)   // Power
		 * ));
		 */

		// put your code to trace photons here
        // Get number of light from the scene
        // Could be reused so extract out
        const int light_num = scene->getLights().size();
        int count_deposited = 0;
        count_emitted = 0;

        // Loop to emit photons
        while(count_deposited<m_photonCount){
            const Emitter *light = scene->getRandomEmitter(sampler->next1D());
            Ray3f x_i;
            // multipy light_num to avoid the scene being to dark(random emitter)
            Color3f W = light_num * light->samplePhoton(x_i, sampler->next2D(), sampler->next2D());
            count_emitted++;
            while(true){
                Intersection its;
                /* if no intersect, then break */
                if (!scene->rayIntersect(x_i, its))
                    break;
                /* if hit diffuse surface */
                // if (isDiffuseSurface(x_i)) 
                //  depositPhoton(x_i,w_o,W);
                if (its.mesh->getBSDF()->isDiffuse()){
                    // ray points to the photon point, the direction on photon point should point out
                    m_photonMap->push_back(Photon(its.p, -x_i.d, W));
                    count_deposited++;
                }

                // Russian Roulette
                float successProbability = std::min<float>(W.maxCoeff(), 0.99);
                if (sampler->next1D() > successProbability) break;
                    else W /= successProbability;
                
                // w_i = sample from BSDF at x_o;
                // BSDFQueryRecord (const Vector3f &wi)
                // Create a new record for sampling the BSDF.
                // wi: Incident direction (in the local frame)
                // Remember to normalized since I assume wi is a normalized vector
                BSDFQueryRecord bRec(its.shFrame.toLocal(-x_i.d).normalized());
                bRec.uv = its.uv;

                // Different to previous code, here use its.mesh to use getBSDF function
                Color3f radiance = its.mesh->getBSDF()->sample(bRec,sampler->next2D());

                W *= radiance;
                x_i = Ray3f(its.p,its.shFrame.toWorld(bRec.wo),Epsilon,INFINITY);
            }
        }

		/* Build the photon map */
        m_photonMap->build();
    }

    virtual Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &_ray) const override {
    	
		/* How to find photons?
		 * std::vector<uint32_t> results;
		 * m_photonMap->search(Point3f(0, 0, 0), // lookup position
		 *                     m_photonRadius,   // search radius
		 *                     results);
		 *
		 * for (uint32_t i : results) {
		 *    const Photon &photon = (*m_photonMap)[i];
		 *    cout << "Found photon!" << endl;
		 *    cout << " Position  : " << photon.getPosition().toString() << endl;
		 *    cout << " Power     : " << photon.getPower().toString() << endl;
		 *    cout << " Direction : " << photon.getDirection().toString() << endl;
		 * }
		 */

		// put your code for path tracing with photon gathering here
        Ray3f traceRay = _ray;

        // Initial radiance and throughput
        Color3f Li = 0.f;
        // Initial throughput
        // At first I use float, but t*= radiance part has problem float * array.
        Color3f t(1.f);

        int bounce = 0;
        
        
        while(true){
            Intersection its;
            /* if not intersect */
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
                Li += t * emitterclolor;
            }

            if (its.mesh->getBSDF()->isDiffuse()){
                Color3f photonDensityEstimation(0.f);
                std::vector<uint32_t> results;
                m_photonMap->search(its.p, // lookup position
                                    m_photonRadius,   // search radius
                                    results);
                for (uint32_t i : results) {
                    const Photon &photon = (*m_photonMap)[i];
                    // cout << "Found photon!" << endl;
                    // cout << " Position  : " << photon.getPosition().toString() << endl;
                    // cout << " Power     : " << photon.getPower().toString() << endl;
                    // cout << " Direction : " << photon.getDirection().toString() << endl;
                    // summing the product of each photon's power and the BSDF value given the photon's direction
                    BSDFQueryRecord bRec = BSDFQueryRecord(its.shFrame.toLocal(-traceRay.d),its.shFrame.toLocal(photon.getDirection()),  ESolidAngle);
                    bRec.uv = its.uv;
                    bRec.p = its.p;
                    Color3f fr = its.mesh->getBSDF()->eval(bRec);
                    photonDensityEstimation += fr * photon.getPower();
                }
                // and then dividing by the product of the area in which photons were queried from (i.e. πr^2 where r is the photon radius) 
                // and the total amount of photons emitted in the preprocessing step
                Li += t * photonDensityEstimation / (M_PI * m_photonRadius * m_photonRadius * count_emitted);
                break;
            }
            
            if(bounce>3){// Russian Roulette
            float successProbability = std::min<float>(t.maxCoeff(), 0.99);
            if (sampler->next1D() > successProbability) break;
                else t /= successProbability;
            }
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
		return Li;
    }

    virtual std::string toString() const override {
        return tfm::format(
            "PhotonMapper[\n"
            "  photonCount = %i,\n"
            "  photonRadius = %f\n"
            "]",
            m_photonCount,
            m_photonRadius
        );
    }
private:
    /* 
     * Important: m_photonCount is the total number of photons deposited in the photon map,
     * NOT the number of emitted photons. You will need to keep track of those yourself.
     */ 
    int m_photonCount;
    float m_photonRadius;
    std::unique_ptr<PhotonMap> m_photonMap;
    int count_emitted;
};

NORI_REGISTER_CLASS(PhotonMapper, "photonmapper");
NORI_NAMESPACE_END
