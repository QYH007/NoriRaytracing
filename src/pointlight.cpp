#include <nori/emitter.h>
 
NORI_NAMESPACE_BEGIN
 
class PointLight : public Emitter {
public:
    PointLight(const PropertyList &props) {
        m_position = props.getPoint3("position",Point3f());
        m_power = props.getColor("power",Color3f());
        std::cout << "Position value was : " << m_position << std::endl;
        std::cout << "power value was : " << m_power << std::endl;
    }
 
    // pure virtual function "nori::Emitter::sample" has no overrider
    // "For this PA, you only need to consider Emitter's sample method, which is supposed to
    //  take a shading point called lRec.ref and a 2D sample (∈[0,1]2\in [0, 1]^2∈[0,1]2) as 
    // input, return importance emissive radiance, and fill in other fields in lRec."
    Color3f sample(EmitterQueryRecord &lRec, const Point2f &sample) const {
        // https://gitlab.inf.ethz.ch/OU-CG-TEACHING/nori-base-23/-/issues/24
        // "it should just set the sampled point on the light surface to the location of light"
        lRec.p = m_position;

        // https://github.com/mmp/pbrt-v3/blob/master/src/lights/point.cpp#L48-L49
        //     *wi = Normalize(pLight - ref.p);
        lRec.wi = (m_position-lRec.ref).normalized();
        // *pdf = 1.f;
        lRec.pdf = 1.f;

        // TRay (const PointType &o, const VectorType &d, Scalar mint, Scalar maxt)
        // lRec.wi: Direction between the hit point and the emitter point
        // mint Epsilon as in av.cpp, maxt should be length hence (point-point).norm()
        // Feedback: Better set `maxt` to `distance - Epsilon` to avoid intersection with the light source itself
        lRec.shadowRay = Ray3f(lRec.ref,lRec.wi,Epsilon,(m_position-lRec.ref).norm()-Epsilon);
        
        // *nLight = (Normal3f)ray->d;

        return eval(lRec)/pdf(lRec);
    }

    // pure virtual function "nori::Emitter::eval" has no overrider
    Color3f eval(const EmitterQueryRecord &lRec) const {
        // https://github.com/mmp/pbrt-v3/blob/master/src/lights/point.cpp#L52
        // return I / DistanceSquared(pLight, ref.p);
        // https://github.com/mmp/pbrt-v3/blob/master/src/lights/point.cpp#L55
        // Spectrum PointLight::Power() const { return 4 * Pi * I; }
        // hence I = power / (4 * Pi)
        // https://github.com/mmp/pbrt-v3/blob/master/src/core/geometry.h#L1072
        // DistanceSquared: return (p1 - p2).LengthSquared()
        // https://github.com/mmp/pbrt-v3/blob/master/src/core/geometry.h#L147
        // Float LengthSquared() const { return x * x + y * y; }
        return m_power / ((4 * M_PI) * (m_position - lRec.ref).squaredNorm());
    }

    // pure virtual function "nori::Emitter::pdf" has no overrider
    float pdf(const EmitterQueryRecord &lRec) const {
        // *pdf = 1.f;
        // Should it be lRec.pdf or just 1.f?
        // access to the probability density that is realized by the \ref sample() method.
        // return lRec.pdf;
        // Feedback: The pdf method should return 1 instead of copying the value from lRec
        return 1.f;
    }
 
    /// Return a human-readable description for debugging purposes
    std::string toString() const {
        return tfm::format(
            "PointLight[\n"
            "  m_position = \"%s\"\n"
            "]",
            m_position,
            "  m_power = \"%s\"\n"
            "]",
            m_power
        );
    }
protected:
    Point3f m_position;
    Color3f m_power;
};
 
NORI_REGISTER_CLASS(PointLight, "point");
NORI_NAMESPACE_END