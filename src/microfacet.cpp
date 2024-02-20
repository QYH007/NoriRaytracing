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

#include <nori/bsdf.h>
#include <nori/frame.h>
#include <nori/warp.h>

NORI_NAMESPACE_BEGIN

class Microfacet : public BSDF {
public:
    Microfacet(const PropertyList &propList) {
        /* RMS surface roughness */
        m_alpha = propList.getFloat("alpha", 0.1f);

        /* Interior IOR (default: BK7 borosilicate optical glass) */
        m_intIOR = propList.getFloat("intIOR", 1.5046f);

        /* Exterior IOR (default: air) */
        m_extIOR = propList.getFloat("extIOR", 1.000277f);

        /* Albedo of the diffuse base material (a.k.a "kd") */
        m_kd = propList.getColor("kd", Color3f(0.5f));

        /* To ensure energy conservation, we must scale the 
           specular component by 1-kd. 

           While that is not a particularly realistic model of what 
           happens in reality, this will greatly simplify the 
           implementation. Please see the course staff if you're 
           interested in implementing a more realistic version 
           of this BRDF. */
        m_ks = 1 - m_kd.maxCoeff();
    }

    /// Evaluate the microfacet normal distribution D
    float evalBeckmann(const Normal3f &m) const {
        float temp = Frame::tanTheta(m) / m_alpha,
              ct = Frame::cosTheta(m), ct2 = ct*ct;

        return std::exp(-temp*temp) 
            / (M_PI * m_alpha * m_alpha * ct2 * ct2);
    }

    /// Evaluate Smith's shadowing-masking function G1 
    float smithBeckmannG1(const Vector3f &v, const Normal3f &m) const {
        float tanTheta = Frame::tanTheta(v);

        /* Perpendicular incidence -- no shadowing/masking */
        if (tanTheta == 0.0f)
            return 1.0f;

        /* Can't see the back side from the front and vice versa */
        if (m.dot(v) * Frame::cosTheta(v) <= 0)
            return 0.0f;

        float a = 1.0f / (m_alpha * tanTheta);
        if (a >= 1.6f)
            return 1.0f;
        float a2 = a * a;

        /* Use a fast and accurate (<0.35% rel. error) rational
           approximation to the shadowing-masking function */
        return (3.535f * a + 2.181f * a2) 
             / (1.0f + 2.276f * a + 2.577f * a2);
    }

    /// Evaluate the BRDF for the given pair of directions
    virtual Color3f eval(const BSDFQueryRecord &bRec) const override {
        /* This is a smooth BRDF -- return zero if the measure
           is wrong, or when queried for illumination on the backside */
        if (bRec.measure != ESolidAngle
            || Frame::cosTheta(bRec.wi) <= 0
            || Frame::cosTheta(bRec.wo) <= 0)
            return Color3f(0.0f);

        Normal3f wh = (bRec.wi+bRec.wo).normalized();
        float D = evalBeckmann(wh);
        // get the function from common.cpp
        float F = fresnel(wh.dot(bRec.wi),m_extIOR,m_intIOR);
        float G = smithBeckmannG1(bRec.wi,wh) * smithBeckmannG1(bRec.wo,wh);
        float Denominator = 4 * Frame::cosTheta(bRec.wi) * Frame::cosTheta(bRec.wo);
        return m_kd / M_PI + m_ks * (D * F * G) / Denominator;
    }

    /// Evaluate the sampling density of \ref sample() wrt. solid angles
    virtual float pdf(const BSDFQueryRecord &bRec) const override {
        // Adapt from diffuse.cpp, when I realize that it should have some prediction of the orientation
        /* This is a smooth BRDF -- return zero if the measure
           is wrong, or when queried for illumination on the backside */
        if (bRec.measure != ESolidAngle
            || Frame::cosTheta(bRec.wi) <= 0
            || Frame::cosTheta(bRec.wo) <= 0)
            return 0.0f;

        Normal3f wh = (bRec.wi+bRec.wo).normalized();
        float D = evalBeckmann(wh);
        float Jh = 1/(4*wh.dot(bRec.wo));
        return m_ks * D * Frame::cosTheta(wh) * Jh + (1 - m_ks) * Frame::cosTheta(bRec.wo) / M_PI;
    }

    /// Sample the BRDF
    virtual Color3f sample(BSDFQueryRecord &bRec, const Point2f &_sample) const override {

        // Adapt from diffuse.cpp
        if (Frame::cosTheta(bRec.wi) <= 0)
                    return Color3f(0.0f);

        bRec.measure = ESolidAngle;

        /* Relative index of refraction: no change */
        bRec.eta = 1.0f;

        // whether to sample the specular or the diffuse component based on the value of k_s
        if(_sample.x() < m_ks){     // The Specular part
            // According to https://gitlab.inf.ethz.ch/OU-CG-TEACHING/nori-base-23/-/issues/51
            // need to do some rescaling
            Point2f sample(_sample.x()/m_ks,_sample.y());
            // The distribution function D is the Beckmann distribution
            // randomly generate a ωh, with PDF proportional to D
            Vector3f wh = Warp::squareToBeckmann(sample,m_alpha);
            // reflect incident direction ωi about ωh to obtain ωo
            // cos(θi) = ωi · wh
            // cos(θo) = cos(θi)
            // ωo = 2 * cos(θi) * wh - ωi
            // Isosceles triangle base vector minus one of its leg vectors
            bRec.wo = 2 * wh.dot(bRec.wi) * wh - bRec.wi;
        }
        else{      // The diffuse part
            // Adapt from src/diffuse.cpp
            // According to https://gitlab.inf.ethz.ch/OU-CG-TEACHING/nori-base-23/-/issues/51
            // need to do some rescaling
            // _sample.x() - m_ks since _sample.x() itself is larger than m_ks and may cause >1 in new sample.x
            // use 1-m_ks as Denominator in case m_ks is too little
            Point2f sample((_sample.x() - m_ks)/(1 - m_ks),_sample.y());
            bRec.wo = Warp::squareToCosineHemisphere(sample);
        }

        if(pdf(bRec)>0.0f)
        // F = fr * Le * cos θi
        // According to https://gitlab.inf.ethz.ch/OU-CG-TEACHING/nori-base-23/-/issues/57
        // your sample method returns eval * cos(theta_o) / pdf , which is the convention of all BSDFs, note the foreshortening term here.
            return eval(bRec)*Frame::cosTheta(bRec.wo)/pdf(bRec);
        else
            return Color3f(0.f);
    }

    virtual std::string toString() const override {
        return tfm::format(
            "Microfacet[\n"
            "  alpha = %f,\n"
            "  intIOR = %f,\n"
            "  extIOR = %f,\n"
            "  kd = %s,\n"
            "  ks = %f\n"
            "]",
            m_alpha,
            m_intIOR,
            m_extIOR,
            m_kd.toString(),
            m_ks
        );
    }
private:
    float m_alpha;
    float m_intIOR, m_extIOR;
    float m_ks;
    Color3f m_kd;
};

NORI_REGISTER_CLASS(Microfacet, "microfacet");
NORI_NAMESPACE_END
