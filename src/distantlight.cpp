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

class DistantEmitter : public Emitter {
public:
    DistantEmitter(const PropertyList &props) {
        m_radiance = props.getColor("radiance");
        m_direction = props.getVector3("direction");
    }

    virtual std::string toString() const override {
        return tfm::format(
                "AreaLight[\n"
                "  radiance = %s,\n"
                "]",
                m_radiance.toString());
    }

    virtual Color3f eval(const EmitterQueryRecord & lRec) const override {
        return m_radiance;
    }

    virtual Color3f sample(EmitterQueryRecord & lRec, const Point2f & sample) const override {
        lRec.pdf = 1.0;
        lRec.wi = -m_direction.normalized();
        lRec.n = m_direction.normalized();
        lRec.shadowRay = Ray3f(lRec.ref,lRec.wi,Epsilon,INFINITY);
        return m_radiance;
    }

    virtual float pdf(const EmitterQueryRecord &lRec) const override {
        return 1.f;
    }


    virtual Color3f samplePhoton(Ray3f &ray, const Point2f &sample1, const Point2f &sample2) const override {

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
        
        ray = Ray3f(sRec.p,dir,Epsilon,INFINITY);
        // The power of the photon is π⋅A⋅Le , where Le is the emitted radiance, and A is the total surface area of the shape light.
        // pdf = 1/area, area = 1/pdf
        return M_PI * m_radiance * 1 / (m_shape->pdfSurface(sRec));
    }


protected:
    Color3f m_radiance;
    Vector3f m_direction;
};

NORI_REGISTER_CLASS(DistantEmitter, "distant")
NORI_NAMESPACE_END