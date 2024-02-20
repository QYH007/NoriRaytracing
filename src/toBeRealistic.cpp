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

#include <nori/camera.h>
#include <nori/rfilter.h>
#include <nori/warp.h>
#include <Eigen/Geometry>

NORI_NAMESPACE_BEGIN

/**
 * \brief Perspective camera with depth of field
 *
 * This class implements a simple perspective camera model. It uses an
 * infinitesimally small aperture, creating an infinite depth of field.
 */
class RealisticCamera : public Camera {
public:
    float zoom(float data) const {
        return data * 0.001;
    }
    RealisticCamera(const PropertyList &propList) {
        /* Width and height in pixels. Default: 720p */
        m_outputSize.x() = propList.getInteger("width", 1280);
        m_outputSize.y() = propList.getInteger("height", 720);

        m_invOutputSize = m_outputSize.cast<float>().cwiseInverse();
        if (propList.has("shutterOpen"))
            m_shutterOpen = propList.getFloat("shutterOpen",0);
        if (propList.has("shutterClose"))
            m_shutterClose =  propList.getFloat("shutterClose",0);
        if (propList.has("apertureDiameter"))
            m_apertureDiameter = propList.getFloat("apertureDiameter",0);
        if (propList.has("focusDistance"))
            m_focusDistance = propList.getFloat("focusDistance",0);
        if (propList.has("simpleWeighting"))
            m_simpleWeighting = propList.getFloat("simpleWeighting",0);

        if (propList.has("lensRadius"))
            m_lensRadius = propList.getFloat("lensRadius", 0.2);
        if (propList.has("focalDistance"))
            m_focalDistance = propList.getFloat("focalDistance", 10.f);
        if (propList.has("focalDistance"))
            m_focalDistance = propList.getFloat("focalDistance", 10.f);


        std::vector<float> lensData;
        // init lens data
        elementInterfaces.push_back((LensElementInterface){zoom(35.98738),  zoom(1.21638),  1.54,   zoom(23.716) * 0.5f});
        elementInterfaces.push_back((LensElementInterface){zoom(11.69718),  zoom(9.9957),   1.f,    zoom(17.996) * 0.5f});
        elementInterfaces.push_back((LensElementInterface){zoom(13.08714),  zoom(5.12622),  1.772,  zoom(12.364) * 0.5f});
        elementInterfaces.push_back((LensElementInterface){zoom(-22.63294), zoom(1.76924),  1.617,  zoom(9.812) * 0.5f});
        elementInterfaces.push_back((LensElementInterface){zoom(71.05802),  zoom(0.8184),   1.f,    zoom(9.152) * 0.5f});
        elementInterfaces.push_back((LensElementInterface){zoom(0),         zoom(2.27766),  0.f,    zoom(8.756) * 0.5f});
        elementInterfaces.push_back((LensElementInterface){zoom(-9.58584),  zoom(2.43254),  1.617,  zoom(8.184) * 0.5f});
        elementInterfaces.push_back((LensElementInterface){zoom(-11.28864), zoom(0.11506),  1.f,    zoom(9.152) * 0.5f});
        elementInterfaces.push_back((LensElementInterface){zoom(-166.7765), zoom(3.09606),  1.713,  zoom(10.648) * 0.5f});
        elementInterfaces.push_back((LensElementInterface){zoom(-7.5911),   zoom(1.32682),  1.805,  zoom(11.44) * 0.5f});
        elementInterfaces.push_back((LensElementInterface){zoom(-16.7662),  zoom(3.98068),  1.f,    zoom(12.276) * 0.5f});
        elementInterfaces.push_back((LensElementInterface){zoom(-7.70286),  zoom(1.21638),  1.617,  zoom(13.42) * 0.5f});
        elementInterfaces.push_back((LensElementInterface){zoom(-11.97328), zoom(1.f),      1.f,    zoom(17.996) * 0.5f});


        std::cout<<"apertureRadius"<<elementInterfaces[0].apertureRadius<<std::endl;


        /* Specifies an optional camera-to-world transformation. Default: none */
        m_cameraToWorld = propList.getTransform("toWorld", Transform());

        /* Horizontal field of view in degrees */
        m_fov = propList.getFloat("fov", 30.0f);

        /* Near and far clipping planes in world-space units */
        m_nearClip = propList.getFloat("nearClip", 1e-4f);
        m_farClip = propList.getFloat("farClip", 1e4f);

        m_rfilter = NULL;
    }

    // Helper func
    float LensRearZ() const {
        return elementInterfaces.back().thickness;
    }

    float LensFrontZ() const {
        float zSum = 0;
        for (const LensElementInterface &element : elementInterfaces)
            zSum += element.thickness;
        return zSum;
    }

    float RearElementRadius() const {
        return elementInterfaces.back().apertureRadius;
    }
    bool Quadratic(float a, float b, float c, float *t0, float *t1) const {
        double discrim = (double)b * (double)b - 4 * (double)a * (double)c;
        if (discrim < 0) return false;
        double rootDiscrim = std::sqrt(discrim);
        double q;
        if (b < 0) q = -.5 * (b - rootDiscrim);
        else       q = -.5 * (b + rootDiscrim);
        *t0 = q / a;
        *t1 = c / q;
        if (*t0 > *t1) std::swap(*t0, *t1);
        return true;
     }

    Point2f ConcentricSampleDisk(const Point2f &u) const {
        // <<Map uniform random numbers to [-1, 1]^2>> 
        Point2f uOffset = 2.f * u - Vector2f(1, 1);

        //<<Handle degeneracy at the origin>> 
        if (uOffset[0] == 0 && uOffset[1] == 0)
            return Point2f(0, 0);

        //<<Apply concentric mapping to point>> 
        float theta, r;
        static const float PiOver2 = 1.57079632679489661923;
        static const float PiOver4 = 0.78539816339744830961;
        if (std::abs(uOffset[0]) > std::abs(uOffset[1])) {
            r = uOffset[0];
            theta = PiOver4 * (uOffset[0] / uOffset[1]);
        } else {
            r = uOffset[1];
            theta = PiOver2 - PiOver4 * (uOffset[0] / uOffset[1]);
        }
        return r * Point2f(std::cos(theta), std::sin(theta));

    }
    bool IntersectSphericalElement(float radius, float zCenter, const Ray3f &ray, float *t, Normal3f *n) const {
        // Compute t0 and t1 for ray–element intersection
        Point3f o = ray.o - Vector3f(0, 0, zCenter);
        float A = ray.d.x()*ray.d.x() + ray.d.y()*ray.d.y() + ray.d.z()*ray.d.z();
        float B = 2 * (ray.d.x()*o.x() + ray.d.y()*o.y() + ray.d.z()*o.z());
        float C = o.x()*o.x() + o.y()*o.y() + o.z()*o.z() - radius*radius;
        float t0, t1;
        if (!Quadratic(A, B, C, &t0, &t1))
            return false;
        // Select intersection  based on ray direction and element curvature
        bool useCloserT = (ray.d.z() > 0) ^ (radius < 0);
        *t = useCloserT ? std::min(t0, t1) : std::max(t0, t1);
        if (*t < 0)
           return false;
        // Compute surface normal of element at ray intersection point
        *n = Normal3f(Vector3f(o + *t * ray.d));
        *n = n->dot(-ray.d) < 0 ? -*n : *n; 
        return true;
    }
    bool Refract(const Vector3f &wi, const Normal3f &n, float eta, Vector3f *wt) const {
        // Compute  using Snell’s law
        float cosThetaI = n.dot(wi);
        float sin2ThetaI = std::max(0.f, 1.f - cosThetaI * cosThetaI);
        float sin2ThetaT = eta * eta * sin2ThetaI;
        // Handle total internal reflection for transmission
        float cosThetaT = std::sqrt(1 - sin2ThetaT);

        *wt = eta * -wi + (eta * cosThetaI - cosThetaT) * Vector3f(n);
        return true;
    }

    void ComputeCardinalPoints(const Ray3f &rIn, const Ray3f &rOut, float *pz, float *fz) const {
        float tf = -rOut.o.x() / rOut.d.x();
        *fz = -rOut(tf).z();
        float tp = (rIn.o.x() - rOut.o.x()) / rOut.d.x();
        *pz = -rOut(tp).z();
    }

    float FocusThickLens(float focusDistance) {
        Point2f pz, fz;
        ComputeThickLensApproximation(pz, fz);
        // Compute translation of lens, delta, to focus at focusDistance
        float f = fz[0] - pz[0];
        float z = -focusDistance;
        float delta = 0.5f * (pz[1] - z + pz[0] -
            std::sqrt((pz[1] - z - pz[0]) * (pz[1] - z - 4 * f - pz[0])));

        return elementInterfaces.back().thickness + delta;
    }

    virtual void activate() override {
        float aspect = m_outputSize.x() / (float) m_outputSize.y();

        /* Project vectors in camera space onto a plane at z=1:
         *
         *  xProj = cot * x / z
         *  yProj = cot * y / z
         *  zProj = (far * (z - near)) / (z * (far-near))
         *  The cotangent factor ensures that the field of view is 
         *  mapped to the interval [-1, 1].
         */
        float recip = 1.0f / (m_farClip - m_nearClip),
              cot = 1.0f / std::tan(degToRad(m_fov / 2.0f));

        Eigen::Matrix4f perspective;
        perspective <<
            cot, 0,   0,   0,
            0, cot,   0,   0,
            0,   0,   m_farClip * recip, -m_nearClip * m_farClip * recip,
            0,   0,   1,   0;

        /**
         * Translation and scaling to shift the clip coordinates into the
         * range from zero to one. Also takes the aspect ratio into account.
         */
        m_sampleToCamera = Transform( 
            Eigen::DiagonalMatrix<float, 3>(Vector3f(0.5f, -0.5f * aspect, 1.0f)) *
            Eigen::Translation<float, 3>(1.0f, -1.0f/aspect, 0.0f) * perspective).inverse();

        /* If no reconstruction filter was assigned, instantiate a Gaussian filter */
        if (!m_rfilter) {
            m_rfilter = static_cast<ReconstructionFilter *>(
                    NoriObjectFactory::createInstance("gaussian", PropertyList()));
            m_rfilter->activate();
        }
    }

    bool TraceLensesFromFilm(const Ray3f &rCamera, Ray3f *rOut) const {

        float elementZ = 0;
        // Transform rCamera from camera to lens system space 
        Eigen::Matrix4f m;  
        m << 1.f, 0.f, 0.f, 0.f,
            0.f, 1.f, 0.f, 0.f,
            0.f, 0.f, -1.f, 0.f,
            0.f, 0.f, 0.f, 1.f;
        static const Transform CameraToLens = Transform(m,m);
        
        Ray3f rLens = CameraToLens*rCamera;

        for (int i = elementInterfaces.size() - 1; i >= 0; --i) {
            const LensElementInterface &element = elementInterfaces[i];
            // Update ray from film accounting for interaction with element
            elementZ -= element.thickness;
            // Compute intersection of ray with lens element
            float t;
            Normal3f n;
            bool isStop = (element.curvatureRadius == 0);
            if (isStop)
                t = (elementZ - rLens.o.z()) / rLens.d.z();
            else {
                float radius = element.curvatureRadius;
                float zCenter = elementZ + element.curvatureRadius;
                if (!IntersectSphericalElement(radius, zCenter, rLens, &t, &n))
                    return false;
            }
            // Test intersection point against element aperture
            Point3f pHit = rLens(t);
            float r2 = pHit.x() * pHit.x() + pHit.y() * pHit.y();
            if (r2 > element.apertureRadius * element.apertureRadius)
                  return false;
                rLens.o = pHit;
            // Update ray path for element interface interaction
            if (!isStop) {
                Vector3f w;
                float etaI = element.eta;
                float etaT = (i > 0 && elementInterfaces[i - 1].eta != 0) ?
                    elementInterfaces[i - 1].eta : 1;
                if (!Refract(-rLens.d.normalized(), n, etaI / etaT, &w))
                    return false;
                rLens.d = w;
            }

        }

        // Transform rLens from lens system space back to camera space
        if (rOut != nullptr) {
           static const Transform LensToCamera = CameraToLens;
           *rOut = LensToCamera*rLens;
        }

        return true;
    }

    void ComputeThickLensApproximation(Point2f pz, Point2f fz) const {
        // Find height from optical axis for parallel rays
        float x = .001 *  m_outputSize.x();

        // Compute cardinal points for film side of lens system
        Ray3f rScene(Point3f(x, 0, LensFrontZ() + 1), Vector3f(0, 0, -1));
        Ray3f rFilm;
        //TraceLensesFromScene(rScene, &rFilm);
        ComputeCardinalPoints(rScene, rFilm, &pz[0], &fz[0]);

        // Compute cardinal points for scene side of lens system
        rFilm = Ray3f(Point3f(x, 0, LensRearZ() - 1), Vector3f(0, 0, 1));
        TraceLensesFromFilm(rFilm, &rScene);
        ComputeCardinalPoints(rFilm, rScene, &pz[1], &fz[1]);

    }

    Color3f sampleRay(Ray3f &ray,
            const Point2f &samplePosition,
            const Point2f &apertureSample) const {
        /* Compute the corresponding position on the 
           near plane (in local camera space) */
        Point3f nearP = m_sampleToCamera * Point3f(
            samplePosition.x() * m_invOutputSize.x(),
            samplePosition.y() * m_invOutputSize.y(), 0.0f);

        /* Turn into a normalized ray direction, and
           adjust the ray interval accordingly */
        Vector3f d = nearP.normalized();
        float invZ = 1.0f / d.z();

	    Point2f lens_sample = m_lensRadius * Warp::squareToUniformDisk(apertureSample);
        Point3f pFocus = d * m_focalDistance * invZ;

        Point3f origin(lens_sample.x(), lens_sample.y(), 0);
        d = (pFocus - origin).normalized();

        ray = Ray3f(m_cameraToWorld * origin, m_cameraToWorld * d, m_nearClip * invZ, m_farClip * invZ);

        return Color3f(1.0f);
    }

    virtual void addChild(NoriObject *obj) override {
        switch (obj->getClassType()) {
            case EReconstructionFilter:
                if (m_rfilter)
                    throw NoriException("Camera: tried to register multiple reconstruction filters!");
                m_rfilter = static_cast<ReconstructionFilter *>(obj);
                break;

            default:
                throw NoriException("Camera::addChild(<%s>) is not supported!",
                    classTypeName(obj->getClassType()));
        }
    }

    /// Return a human-readable summary
    virtual std::string toString() const override {
        return tfm::format(
            "Realistic[\n"
            "  cameraToWorld = %s,\n"
            "  outputSize = %s,\n"
            "  fov = %f,\n"
            "  clip = [%f, %f],\n"
            "  rfilter = %s\n"
            "]",
            indent(m_cameraToWorld.toString(), 18),
            m_outputSize.toString(),
            m_fov,
            m_nearClip,
            m_farClip,
            indent(m_rfilter->toString())
        );
    }
private:
    Vector2f m_invOutputSize;
    Transform m_sampleToCamera;
    Transform m_cameraToWorld;
    float m_fov;
    float m_nearClip;
    float m_farClip;

    float m_shutterOpen;
    float m_shutterClose;
    float m_apertureDiameter;
    float m_focusDistance;

    bool m_simpleWeighting;
    

    // Thin Lens
    float m_lensRadius, m_focalDistance;

    // realistic
    std::string m_lensdata;

    struct LensElementInterface {
        float curvatureRadius;
        float thickness;
        float eta;
        float apertureRadius;
    };
    struct Bounds2f {
        Point2f pMin;
        Point2f pMax;
    };

    std::vector<LensElementInterface> elementInterfaces;
    std::vector<Bounds2f> exitPupilBounds;
};

NORI_REGISTER_CLASS(RealisticCamera, "realistic");
NORI_NAMESPACE_END
