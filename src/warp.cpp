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

#include <nori/warp.h>
#include <nori/vector.h>
#include <nori/frame.h>

NORI_NAMESPACE_BEGIN

Vector3f Warp::sampleUniformHemisphere(Sampler *sampler, const Normal3f &pole) {
    // Naive implementation using rejection sampling
    Vector3f v;
    do {
        v.x() = 1.f - 2.f * sampler->next1D();
        v.y() = 1.f - 2.f * sampler->next1D();
        v.z() = 1.f - 2.f * sampler->next1D();
    } while (v.squaredNorm() > 1.f);

    if (v.dot(pole) < 0.f)
        v = -v;
    v /= v.norm();

    return v;
}

Point2f Warp::squareToUniformSquare(const Point2f &sample) {
    return sample;
}

float Warp::squareToUniformSquarePdf(const Point2f &sample) {
    return ((sample.array() >= 0).all() && (sample.array() <= 1).all()) ? 1.0f : 0.0f;
}

// https://www.pbr-book.org/3ed-2018/Monte_Carlo_Integration/2D_Sampling_with_Multidimensional_Transformations#SamplingaUnitDisk
Point2f Warp::squareToUniformDisk(const Point2f &sample) {
    float r = std::sqrt(sample[0]);
    float theta = 2 * M_PI * sample[1];
    return Point2f(r*std::cos(theta),r*std::sin(theta));
}

// coding similar to squareToUniformSquarePdf
// within the range, area of the disk is Pi * 1 ^ 2, pdf should be 1/area
float Warp::squareToUniformDiskPdf(const Point2f &p) {
    return (p.norm() <=1) ? 1.0f / M_PI : 0.0f;
}

// 08-monte-carlo.pdf#P79
// wz:[-1,1], capz[cosThetaMax,1], range of capz is (1-cosThetaMax)
Vector3f Warp::squareToUniformSphereCap(const Point2f &sample, float cosThetaMax) {
    // float wz = 2 * sample[0] - 1;
    float capz = 1 - sample[0] * (1 - cosThetaMax);
    float r = std::sqrt(std::max((float) 0, (float) 1-capz*capz));
    float phi = 2 * M_PI * sample[1];
    return Vector3f(r*std::cos(phi),r*std::sin(phi),capz);
}

// Scap = 2 Pi R h
float Warp::squareToUniformSphereCapPdf(const Vector3f &v, float cosThetaMax) {
    return (v.z() >= cosThetaMax) ? 1.0f / (2 * M_PI * 1 * (1.0f - cosThetaMax)) : 0.0f;
}

// 08-monte-carlo.pdf#P79
// https://www.pbr-book.org/3ed-2018/Monte_Carlo_Integration/2D_Sampling_with_Multidimensional_Transformations#UniformlySamplingaHemisphere
Vector3f Warp::squareToUniformSphere(const Point2f &sample) {
    float wz = 2 * sample[0] - 1;
    float r = std::sqrt(std::max((float) 0, (float) 1-wz*wz));
    float phi = 2 * M_PI * sample[1];
    return Vector3f(r*std::cos(phi),r*std::sin(phi),wz);
}

// coding similar to squareToUniformSquarePdf
// within the range, area of the disk is 4 * Pi * 1 ^ 2, pdf should be 1/area
float Warp::squareToUniformSpherePdf(const Vector3f &v) {
    return (v.norm() <=1) ? 1.0f / (4 * M_PI) : 0.0f;
}

// 08-monte-carlo.pdf#P79
// https://www.pbr-book.org/3ed-2018/Monte_Carlo_Integration/2D_Sampling_with_Multidimensional_Transformations#UniformlySamplingaHemisphere
// wz:[-1,1], hem[0,1], range of hemz is 1
Vector3f Warp::squareToUniformHemisphere(const Point2f &sample) {
    float hemz = sample[0];
    float r = std::sqrt(std::max((float) 0, (float) 1-hemz*hemz));
    float phi = 2 * M_PI * sample[1];
    return Vector3f(r*std::cos(phi),r*std::sin(phi),hemz);
}

// Shem = 2 * Pi * r^2
float Warp::squareToUniformHemispherePdf(const Vector3f &v) {
    return (v.z() >= 0.f) ? 1.0f / (2 * M_PI * 1 * 1) : 0.0f;
}

// PDF = cos(theta) / pi
// CDF = 1/pi integral (theta) (0) cos (x) dx = 1/pi sin(theta)
// sample[0] = 1/pi sin(theta)
// sin(theta) = pi sample[0]
// cos(theta) = sqrt(1-sin2(theta)) = sqrt(1-(pi*sample[0])^2)
// however, this cause problem since pi*sample[0]^2 not always < 1
// if using max(0,cos(theta)) will cause samples cluster at hemz=0
// an alternative solution is to apply the square root to the cosine square value
// let cos2(theta) = 1 - sample[0]
// in this way, cos(theta) = sqrt(1-sample[0])
Vector3f Warp::squareToCosineHemisphere(const Point2f &sample) {
    float hemz = std::sqrt(1-sample[0]);
    float r = std::sqrt(std::max((float) 0, (float) 1-hemz*hemz));
    float phi = 2 * M_PI * sample[1];
    return Vector3f(r*std::cos(phi),r*std::sin(phi),hemz);
}

// pdf = cos(theta)/pi
float Warp::squareToCosineHemispherePdf(const Vector3f &v) {
    return (v.z() >= 0.f) ? v.z() / M_PI : 0.0f;
}

Vector3f Warp::squareToBeckmann(const Point2f &sample, float alpha) {
    float tan2theta = -alpha * alpha * log(1.0f - sample[0]);
    float theta = atan(sqrt(tan2theta));
    float r = std::sqrt(std::max((float) 0, (float) 1-cos(theta)*cos(theta)));
    float phi = 2 * M_PI * sample[1];
    return Vector3f(r*std::cos(phi),r*std::sin(phi),cos(theta));
}

// p(theta,phi)=D(theta)cos(theta)sin(theta)= exp(-tan^2(theta)/(alpha^2))/(pi*alpha^2 * cos^4(theta))*cos(theta)sin(theta)
float Warp::squareToBeckmannPdf(const Vector3f &m, float alpha) {
    float costheta = m.z();
    float theta = acos(costheta);
    float part1 = exp(-pow(tan(theta),2)/pow(alpha,2));
    float part2 = M_PI * pow(alpha,2) * pow(cos(theta),4);
    // return (m.norm()<=1) && (m.z() >= 0.f) ? part1/part2*cos(theta) : 0.0f;
    // feedback: pdf test should be z>0 (avoid /0) and m.norm() could be greater than one due to slight numerical issues
    return (m.z() > 0.f) ? part1/part2*cos(theta) : 0.0f;
}

Vector3f Warp::squareToUniformTriangle(const Point2f &sample) {
    float su1 = sqrtf(sample.x());
    float u = 1.f - su1, v = sample.y() * su1;
    return Vector3f(u,v,1.f-u-v);
}

NORI_NAMESPACE_END
