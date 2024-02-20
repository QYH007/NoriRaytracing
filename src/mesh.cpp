/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob, Romain Prévost

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

#include <nori/mesh.h>
#include <nori/bbox.h>
#include <nori/bsdf.h>
#include <nori/emitter.h>
#include <nori/warp.h>
#include <Eigen/Geometry>

NORI_NAMESPACE_BEGIN

Mesh::Mesh() { }

void Mesh::activate() {
    Shape::activate();

    m_pdf.reserve(getPrimitiveCount());
    for(uint32_t i = 0 ; i < getPrimitiveCount() ; ++i) {
        m_pdf.append(surfaceArea(i));
    }
    m_pdf.normalize();
}

void Mesh::sampleSurface(ShapeQueryRecord & sRec, const Point2f & sample) const {
    Point2f s = sample;
    size_t idT = m_pdf.sampleReuse(s.x());

    Vector3f bc = Warp::squareToUniformTriangle(s);

    sRec.p = getInterpolatedVertex(idT,bc);
    if (m_N.size() > 0) {
        sRec.n = getInterpolatedNormal(idT, bc);
    }
    else {
        Point3f p0 = m_V.col(m_F(0, idT));
        Point3f p1 = m_V.col(m_F(1, idT));
        Point3f p2 = m_V.col(m_F(2, idT));
        Normal3f n = (p1-p0).cross(p2-p0).normalized();
        sRec.n = n;
    }
    sRec.pdf = m_pdf.getNormalization();
}
float Mesh::pdfSurface(const ShapeQueryRecord & sRec) const {
    return m_pdf.getNormalization();
}

Point3f Mesh::getInterpolatedVertex(uint32_t index, const Vector3f &bc) const {
    return (bc.x() * m_V.col(m_F(0, index)) +
            bc.y() * m_V.col(m_F(1, index)) +
            bc.z() * m_V.col(m_F(2, index)));
}

Normal3f Mesh::getInterpolatedNormal(uint32_t index, const Vector3f &bc) const {
    return (bc.x() * m_N.col(m_F(0, index)) +
            bc.y() * m_N.col(m_F(1, index)) +
            bc.z() * m_N.col(m_F(2, index))).normalized();
}

float Mesh::surfaceArea(uint32_t index) const {
    uint32_t i0 = m_F(0, index), i1 = m_F(1, index), i2 = m_F(2, index);

    const Point3f p0 = m_V.col(i0), p1 = m_V.col(i1), p2 = m_V.col(i2);

    return 0.5f * Vector3f((p1 - p0).cross(p2 - p0)).norm();
}

bool Mesh::rayIntersect(uint32_t index, const Ray3f &ray, float &u, float &v, float &t) const {
    uint32_t i0 = m_F(0, index), i1 = m_F(1, index), i2 = m_F(2, index);
    const Point3f p0 = m_V.col(i0), p1 = m_V.col(i1), p2 = m_V.col(i2);

    /* Find vectors for two edges sharing v[0] */
    Vector3f edge1 = p1 - p0, edge2 = p2 - p0;

    /* Begin calculating determinant - also used to calculate U parameter */
    Vector3f pvec = ray.d.cross(edge2);

    /* If determinant is near zero, ray lies in plane of triangle */
    float det = edge1.dot(pvec);

    if (det > -1e-8f && det < 1e-8f)
        return false;
    float inv_det = 1.0f / det;

    /* Calculate distance from v[0] to ray origin */
    Vector3f tvec = ray.o - p0;

    /* Calculate U parameter and test bounds */
    u = tvec.dot(pvec) * inv_det;
    if (u < 0.0 || u > 1.0)
        return false;

    /* Prepare to test V parameter */
    Vector3f qvec = tvec.cross(edge1);

    /* Calculate V parameter and test bounds */
    v = ray.d.dot(qvec) * inv_det;
    if (v < 0.0 || u + v > 1.0)
        return false;

    /* Ray intersects triangle -> compute t */
    t = edge2.dot(qvec) * inv_det;

    return t >= ray.mint && t <= ray.maxt;
}

float DifferenceOfProducts(float a, float b, float c, float d) {
    float cd = c * d;
    float differenceOfProducts = std::fma(a, b, -cd);
    float error = std::fma(-c, d, cd);
    return differenceOfProducts + error;
}
float Clamp(float val, float low, float high) {
    if (val < low)       return low;
    else if (val > high) return high;
    else                 return val;
}


void Mesh::setHitInformation(uint32_t index, const Ray3f &ray, Intersection & its) const {
    /* Find the barycentric coordinates */
    Vector3f bary;
    bary << 1-its.uv.sum(), its.uv;

    /* Vertex indices of the triangle */
    uint32_t idx0 = m_F(0, index), idx1 = m_F(1, index), idx2 = m_F(2, index);

    Point3f p0 = m_V.col(idx0), p1 = m_V.col(idx1), p2 = m_V.col(idx2);

    Point2f uv0 = m_UV.col(idx0), uv1 = m_UV.col(idx1), uv2 = m_UV.col(idx2);

    /* Compute the intersection positon accurately
       using barycentric coordinates */
    its.p = bary.x() * p0 + bary.y() * p1 + bary.z() * p2;

    /* Compute proper texture coordinates if provided by the mesh */
    if (m_UV.size() > 0)
        its.uv = bary.x() * m_UV.col(idx0) +
                 bary.y() * m_UV.col(idx1) +
                 bary.z() * m_UV.col(idx2);
    
    /* Compute the geometry frame */
    its.geoFrame = Frame((p1-p0).cross(p2-p0).normalized());

    if (m_N.size() > 0) {
        /* Compute the shading frame. Note that for simplicity,
           the current implementation doesn't attempt to provide
           tangents that are continuous across the surface. That
           means that this code will need to be modified to be able
           use anisotropic BRDFs, which need tangent continuity */

        its.shFrame = Frame(
                (bary.x() * m_N.col(idx0) +
                 bary.y() * m_N.col(idx1) +
                 bary.z() * m_N.col(idx2)).normalized());
    } else {
        its.shFrame = its.geoFrame;
    }

    // -------------------------------------Work for MipMapping------------------------------------
    // Compute dpdu, dpdv
    Vector2f duv02 = uv0 - uv2, duv12 = uv1 - uv2;
    Vector3f dp02 = p0 - p2, dp12 = p1 - p2;

    float determinant = duv02[0] * duv12[1] - duv02[1] * duv12[0];
    float invdet = 1 / determinant;

    Vector3f dpdu = ( duv12[1] * dp02 - duv02[1] * dp12) * invdet;
    Vector3f dpdv = (-duv12[0] * dp02 + duv02[0] * dp12) * invdet;



    float dudx=0,dvdx=0,dudy=0,dvdy=0;


    if (ray.hasDifferentials){
        //std::cout<< "p: " << p << std::endl;
        //std::cout<<"n: "<< shFrame.n<<std::endl;
        float d = -its.shFrame.n.dot(its.p);
        //std::cout<< "main dir: " << ray.d << std::endl;
        //std::cout<< "x dir: " << ray.rxDirection << std::endl;

        float tx = (-its.shFrame.n.dot(ray.rxOrigin) - d) / its.shFrame.n.dot(ray.rxDirection);
        Point3f px = ray.rxOrigin + tx * ray.rxDirection;

        float ty = (-its.shFrame.n.dot(ray.ryOrigin) - d) / its.shFrame.n.dot(ray.ryDirection);
        Point3f py = ray.ryOrigin + ty * ray.ryDirection;
        //std::cout<< "px: " << p << std::endl;
        //std::cout<< "dpdy: " << dpdy << std::endl;
        Vector3f dpdx = px - its.p;
        Vector3f dpdy = py - its.p;


        // Compute ATA and its determinant
        float ata00 = dpdu.dot(dpdu), ata01 = dpdu.dot(dpdv);
        float ata11 = dpdv.dot(dpdv);
        float invDet = 1 / DifferenceOfProducts(ata00, ata11, ata01, ata01);

        invDet = std::isfinite(invDet) ? invDet : 0.f;

        // Compute ATb for x and y
        float atb0x = dpdu.dot(dpdx), atb1x = dpdv.dot(dpdx);
        float atb0y = dpdu.dot(dpdy), atb1y = dpdv.dot(dpdy);

        dudx = DifferenceOfProducts(ata11, atb0x, ata01, atb1x) * invDet;
        dvdx = DifferenceOfProducts(ata00, atb1x, ata01, atb0x) * invDet;
        dudy = DifferenceOfProducts(ata11, atb0y, ata01, atb1y) * invDet;
        dvdy = DifferenceOfProducts(ata00, atb1y, ata01, atb0y) * invDet;

        dudx = std::isfinite(dudx) ? Clamp(dudx, -1e8f, 1e8f) : 0.f;
        dvdx = std::isfinite(dvdx) ? Clamp(dvdx, -1e8f, 1e8f) : 0.f;
        dudy = std::isfinite(dudy) ? Clamp(dudy, -1e8f, 1e8f) : 0.f;
        dvdy = std::isfinite(dvdy) ? Clamp(dvdy, -1e8f, 1e8f) : 0.f;

        float duvx = sqrt(dudx * dudx + dvdx * dvdx); 
        float duvy = sqrt(dudy * dudy + dvdy * dvdy);
        float max_diff = std::max(duvx, duvy);

        its.dpdx = dpdx;
        its.dpdy = dpdy;
        its.dudx = dudx;
        its.dvdx = dvdx;
        its.dudy = dudy;
        its.dvdy = dvdy;

        its.L = std::max(log2(max_diff), 0.0f);

        // std::cout<< "max_diff: " <<its.L << endl;
    }
}


BoundingBox3f Mesh::getBoundingBox(uint32_t index) const {
    BoundingBox3f result(m_V.col(m_F(0, index)));
    result.expandBy(m_V.col(m_F(1, index)));
    result.expandBy(m_V.col(m_F(2, index)));
    return result;
}

Point3f Mesh::getCentroid(uint32_t index) const {
    return (1.0f / 3.0f) *
        (m_V.col(m_F(0, index)) +
         m_V.col(m_F(1, index)) +
         m_V.col(m_F(2, index)));
}


std::string Mesh::toString() const {
    return tfm::format(
        "Mesh[\n"
        "  name = \"%s\",\n"
        "  vertexCount = %i,\n"
        "  triangleCount = %i,\n"
        "  bsdf = %s,\n"
        "  emitter = %s\n"
        "]",
        m_name,
        m_V.cols(),
        m_F.cols(),
        m_bsdf ? indent(m_bsdf->toString()) : std::string("null"),
        m_emitter ? indent(m_emitter->toString()) : std::string("null")
    );
}

NORI_NAMESPACE_END
