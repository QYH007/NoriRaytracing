/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Romain Prévost

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

#include <nori/emitter.h>
#include <nori/warp.h>
#include <nori/shape.h>

NORI_NAMESPACE_BEGIN

class AreaEmitter : public Emitter {
public:
    AreaEmitter(const PropertyList &props) {
        m_radiance = props.getColor("radiance");
    }

    virtual std::string toString() const override {
        return tfm::format(
                "AreaLight[\n"
                "  radiance = %s,\n"
                "]",
                m_radiance.toString());
    }

    virtual Color3f eval(const EmitterQueryRecord & lRec) const override {
        if(!m_shape)
            throw NoriException("There is no shape attached to this Area light!");

        // At first I thought n is the normal at the intersection point, however it cause black at the whole scene
        // After tracing back to emitter.h, I realize that n is the normal at emitter point
        // wi is the direction pointed to light. If the dot product is below 0, means that the light face to the 
        // direction of the intersection point
        if(lRec.n.dot(lRec.wi) < 0){
            return m_radiance;
        }else{
            // zero otherwise
            return Color3f(0.0f);
        }
    }

    virtual Color3f sample(EmitterQueryRecord & lRec, const Point2f & sample) const override {
        if(!m_shape)
            throw NoriException("There is no shape attached to this Area light!");

        // shape.h
        /// Data structure with ref to call sampleSurface()
        ShapeQueryRecord sRec(lRec.ref);
        // sampleSurface is a function from Shape class, only m_shape is a shape here
        m_shape->sampleSurface(sRec,sample);
        // This method should set sRec.p, sRec.n and sRec.pdf
        lRec.p = sRec.p;
        lRec.n = sRec.n;
        // compute wi
        lRec.wi = (lRec.p-lRec.ref).normalized();        
        // TRay (const PointType &o, const VectorType &d, Scalar mint, Scalar maxt)
        // lRec.wi: Direction between the hit point and the emitter point
        // mint Epsilon as in av.cpp, maxt should be length hence (point-point).norm()
        // Feedback: Better set `maxt` to `distance - Epsilon` to avoid intersection with the light source itself
        lRec.shadowRay = Ray3f(lRec.ref,lRec.wi,Epsilon,(lRec.p-lRec.ref).norm()-Epsilon);
        lRec.pdf = pdf(lRec);
        if(pdf(lRec)>0.0f)
            return eval(lRec)/pdf(lRec);
        else
            return Color3f(0.f);
    }

    virtual float pdf(const EmitterQueryRecord &lRec) const override {
        if(!m_shape)
            throw NoriException("There is no shape attached to this Area light!");

        // shape.h
        /// Data structure with ref and p to call pdfSurface()
        ShapeQueryRecord sRec(lRec.ref,lRec.p);

        // Check whether the surface is back-facing
        // Feedback: Should set pdf(...) to 0 if incident ray hits the back of the area light. Your current version of pdf(...) invalidates the if condition in sample(...).
        if (lRec.n.dot(-lRec.wi) <= 0) {
            return 0.0f;
        }
        // Deleted since n is the normal at emitter not at intersection surface

        float ShapePdf = m_shape->pdfSurface(sRec);
        // https://www.pbr-book.org/3ed-2018/Light_Transport_I_Surface_Reflection/Sampling_Light_Sources#AreaLights
        // Float pdf = DistanceSquared(ref.p, isectLight.p) / (AbsDot(isectLight.n, -wi) * Area());
        float pdf = ShapePdf * (lRec.p-lRec.ref).squaredNorm()/abs(lRec.n.dot(-lRec.wi));

        return pdf;
    }


    virtual Color3f samplePhoton(Ray3f &ray, const Point2f &sample1, const Point2f &sample2) const override {
        if(!m_shape)
            throw NoriException("There is no shape attached to this Area light!");

        // Implementation: first, uniformly choose a location on the surface
        // shape.h
        /// Empty constructor
        ShapeQueryRecord sRec;
        // sampleSurface is a function from Shape class, only m_shape is a shape here
        m_shape->sampleSurface(sRec,sample1);

        // then choose a cosine-weighted random direction over the hemisphere around the surface normal. 
        Vector3f dir = Warp::squareToCosineHemisphere(sample2);

        // Issue: wired looking
        // Transform the sampled direction to the local coordinate system defined by the surface normal
        Frame frame(sRec.n);
        dir = frame.toWorld(dir);
        
        // Feedback:  -- l.124 Note: Ray3f(o, d) will automatically set mint and maxt correctly
        ray = Ray3f(sRec.p,dir);
        // The power of the photon is π⋅A⋅Le , where Le is the emitted radiance, and A is the total surface area of the shape light.
        // pdf = 1/area, area = 1/pdf
        return M_PI * m_radiance * 1 / (m_shape->pdfSurface(sRec));
    }


protected:
    Color3f m_radiance;
};

NORI_REGISTER_CLASS(AreaEmitter, "area")
NORI_NAMESPACE_END