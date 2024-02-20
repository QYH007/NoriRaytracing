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
        
        // for (int i = 0; i < (int)lensData.size(); i += 4) {
        //     if (lensData[i] == 0) {
        //         if (m_apertureDiameter > lensData[i + 3]) {
        //             std::cout<<"warning"<<std::endl;
        //         } else {
        //             lensData[i + 3] = m_apertureDiameter;
        //         }
        //     }
        //     elementInterfaces.push_back(LensElementInterface(
        //         {lensData[i] * (float).001, lensData[i + 1] * (float).001,
        //         lensData[i + 2], lensData[i + 3] * float(.001) / float(2.)}));
        // }

        /* Specifies an optional camera-to-world transformation. Default: none */
        m_cameraToWorld = propList.getTransform("toWorld", Transform());

        /* Horizontal field of view in degrees */
        m_fov = propList.getFloat("fov", 30.0f);

        /* Near and far clipping planes in world-space units */
        m_nearClip = propList.getFloat("nearClip", 1e-4f);
        m_farClip = propList.getFloat("farClip", 1e4f);

        m_rfilter = NULL;
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
    std::vector<float> lensData;

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
