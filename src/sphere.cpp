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

#include <nori/shape.h>
#include <nori/bsdf.h>
#include <nori/emitter.h>
#include <nori/warp.h>

NORI_NAMESPACE_BEGIN

class Sphere : public Shape {
public:
    Sphere(const PropertyList & propList) {
        m_position = propList.getPoint3("center", Point3f());
        m_radius = propList.getFloat("radius", 1.f);

        m_bbox.expandBy(m_position - Vector3f(m_radius));
        m_bbox.expandBy(m_position + Vector3f(m_radius));
    }

    virtual BoundingBox3f getBoundingBox(uint32_t index) const override { return m_bbox; }

    virtual Point3f getCentroid(uint32_t index) const override { return m_position; }

    virtual bool rayIntersect(uint32_t index, const Ray3f &ray, float &u, float &v, float &t) const override {

	/* to be implemented */
        Point3f center = m_position;
        Point3f origin = ray.o;
        Vector3f direction = ray.d;
        
        // 04-raytracing.pdf#P43-46
        // 1.compute distance2 from ray origin to sphere center
        float square_distance = (center-origin).squaredNorm();
        // 2.compute ray distance which is closest to sphere center
        float projection = (center-origin).dot(direction);
        //  - reject if ray is outside and points away from sphere
        if(square_distance > pow(m_radius,2))
            if(projection<0)
                return false;
        
        // 3.compute shortest distance2 from sphere center to ray
        float square_shortest_distance = square_distance - pow(projection,2);
        // - reject if greater than r2
        if(square_shortest_distance > pow(m_radius,2))
            return false;

        // 4.compute half-chord distance2
        float half_chord_distance = pow(m_radius,2)-square_shortest_distance;
        // 5.t=(c-o)·d+-sqrt(r2-D2)
        float temp_t = projection - sqrt(half_chord_distance);
        if(temp_t >= ray.mint && temp_t <= ray.maxt){
            t = temp_t;
            return true;
        }
        temp_t = projection + sqrt(half_chord_distance);
        if(temp_t >= ray.mint && temp_t <= ray.maxt){
            t = temp_t;
            return true;
        }
        return false;
    }

    virtual void setHitInformation(uint32_t index, const Ray3f &ray, Intersection & its) const override {
        /* to be implemented */
        //if there is any intersect point
        its.p = ray.o + its.t * ray.d;
        //adapt form mesh.cpp#L54,143,157
        Normal3f n = (its.p-m_position).normalized();
        // https://gitlab.inf.ethz.ch/OU-CG-TEACHING/nori-base-23/-/issues/14
        its.geoFrame = Frame(n);
        its.shFrame = its.geoFrame;
        //common.cpp
        //another reference: https://en.wikipedia.org/wiki/UV_mapping#Finding_UV_on_a_sphere
        its.uv = sphericalCoordinates(n);
        its.uv(0) = 0.5+its.uv(0)/(2*M_PI);
        its.uv(1) = its.uv(1)/(M_PI);
    }

    virtual void sampleSurface(ShapeQueryRecord & sRec, const Point2f & sample) const override {
        Vector3f q = Warp::squareToUniformSphere(sample);
        sRec.p = m_position + m_radius * q;
        sRec.n = q;
        sRec.pdf = std::pow(1.f/m_radius,2) * Warp::squareToUniformSpherePdf(Vector3f(0.0f,0.0f,1.0f));
    }
    virtual float pdfSurface(const ShapeQueryRecord & sRec) const override {
        return std::pow(1.f/m_radius,2) * Warp::squareToUniformSpherePdf(Vector3f(0.0f,0.0f,1.0f));
    }


    virtual std::string toString() const override {
        return tfm::format(
                "Sphere[\n"
                "  center = %s,\n"
                "  radius = %f,\n"
                "  bsdf = %s,\n"
                "  emitter = %s\n"
                "]",
                m_position.toString(),
                m_radius,
                m_bsdf ? indent(m_bsdf->toString()) : std::string("null"),
                m_emitter ? indent(m_emitter->toString()) : std::string("null"));
    }

protected:
    Point3f m_position;
    float m_radius;
};

NORI_REGISTER_CLASS(Sphere, "sphere");
NORI_NAMESPACE_END
