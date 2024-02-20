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

NORI_NAMESPACE_BEGIN

/// Ideal dielectric BSDF
class Dielectric : public BSDF {
public:
    Dielectric(const PropertyList &propList) {
        /* Interior IOR (default: BK7 borosilicate optical glass) */
        m_intIOR = propList.getFloat("intIOR", 1.5046f);

        /* Exterior IOR (default: air) */
        m_extIOR = propList.getFloat("extIOR", 1.000277f);
    }

    virtual Color3f eval(const BSDFQueryRecord &) const override {
        /* Discrete BRDFs always evaluate to zero in Nori */
        return Color3f(0.0f);
    }

    virtual float pdf(const BSDFQueryRecord &) const override {
        /* Discrete BRDFs always evaluate to zero in Nori */
        return 0.0f;
    }

    virtual Color3f sample(BSDFQueryRecord &bRec, const Point2f &sample) const override {
        // throw NoriException("Unimplemented!");
        // reference: https://www.pbr-book.org/3ed-2018/Reflection_Models/Specular_Reflection_and_Transmission#FresnelReflectance
        // No need to predict whether costheta >0 or <0 since it has been implemented
        // in the fresnel function(swap)
        float F = fresnel(Frame::cosTheta(bRec.wi),m_extIOR,m_intIOR);

        // if reflect
        if(F > sample.x())  // proportional 
        {
        // The same as in mirror.cpp
        // Reflection in local coordinates
        bRec.wo = Vector3f(
            -bRec.wi.x(),
            -bRec.wi.y(),
             bRec.wi.z()
        );
        bRec.measure = EDiscrete;

        /* Relative index of refraction: no change */
        bRec.eta = 1.0f;

        return Color3f(1.f);
        }
        // if refract
        else{
            // https://www.pbr-book.org/3ed-2018/Reflection_Models/Specular_Reflection_and_Transmission#SpecularTransmission
            // Spectrum SpecularTransmission::Sample_f
            // if (!Refract(wo, Faceforward(Normal3f(0, 0, 1), wo), etaI / etaT, wi))
            // eta = etaI / etaT
            // here etaI = extIOR, etaT = intIOR
            // the same predict as in common.cpp
            float etaI = m_extIOR, etaT = m_intIOR;
            float cosThetaI = Frame::cosTheta(bRec.wi);
            /* The same as in common.cpp,
            Swap the indices of refraction if the interaction starts
            at the inside of the object */
            if(cosThetaI<0.0f){
                std::swap(etaI, etaT);
                cosThetaI = -cosThetaI;
            }
            // Feedback:  -- bRec.eta should be inverted.
            // bRec.eta = etaT / etaI;
            bRec.eta = etaI / etaT;
            float sinThetaTSqr = bRec.eta * bRec.eta * (1-cosThetaI*cosThetaI);
            float cosThetaT = std::sqrt(1.0f - sinThetaTSqr);
            // The above function from pbrtv3, dealing with the Faceforward(Normal3f(0, 0, 1), wo) part
            Normal3f n(0.f,0.f,1.f);
            if(n.dot(bRec.wi)<0)
                n = -n;
            // As calculated from Refract function in above pbrtv3
            /* <<Compute  using Snell’s law>> 
                Float cosThetaI = Dot(n, wi);
                Float sin2ThetaI = std::max(0.f, 1.f - cosThetaI * cosThetaI);
                Float sin2ThetaT = eta * eta * sin2ThetaI;
                <<Handle total internal reflection for transmission>> 
                   if (sin2ThetaT >= 1) return false;

                Float cosThetaT = std::sqrt(1 - sin2ThetaT);

                 *wt = eta * -wi + (eta * cosThetaI - cosThetaT) * Vector3f(n);*/
            bRec.wo = bRec.eta * -bRec.wi + (bRec.eta * cosThetaI - cosThetaT) * n;
            // The same as in common.cpp
            // Feedback:  -- bRec.eta should be inverted.
            bRec.eta = etaT / etaI;
            bRec.measure = EDiscrete;

            
            // float ft = 1 - F;
            // ft *= (m_extIOR * m_extIOR) / (m_intIOR * m_intIOR);
            
            // return Color3f((bRec.wo.x()+bRec.wi.x()) * ft / abs(cosThetaI),
            //    (bRec.wo.y()+bRec.wi.y()) * ft / abs(cosThetaI),
            //    (bRec.wo.z()-bRec.wi.z()) * ft / abs(cosThetaI));
            // https://gitlab.inf.ethz.ch/OU-CG-TEACHING/nori-base-23/-/issues/65
            // At first I thought we need to calculate the fr in the slides
            // However, based on the issue and the previous specular term
            // I think that what need to be returned here is the coefficient of the function
            // i.e. the radiometric quantities or power
            return Color3f((etaI * etaI) / (etaT * etaT));
        }        
    }

    virtual std::string toString() const override {
        return tfm::format(
            "Dielectric[\n"
            "  intIOR = %f,\n"
            "  extIOR = %f\n"
            "]",
            m_intIOR, m_extIOR);
    }
private:
    float m_intIOR, m_extIOR;
};

NORI_REGISTER_CLASS(Dielectric, "dielectric");
NORI_NAMESPACE_END
