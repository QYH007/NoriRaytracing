//
// Created by Alessia Paccagnella on 28/11/2019.
//

#include <nori/bitmap.h>
#include <nori/emitter.h>
#include <nori/frame.h>


NORI_NAMESPACE_BEGIN

typedef Eigen::Array<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> matrix;

class EnvLight: public Emitter {
public:

        EnvLight(const PropertyList &propList) {

            if (propList.has("strength")) {
                m_strength = propList.getFloat("strength");
            }

            m_lightToWorld = propList.getTransform("toWorld", Transform());
            m_worldToLight = m_lightToWorld.getInverseMatrix();

            // env file loading
            std::string m_path = propList.getString("filename");
            m_image = Bitmap(m_path);    
            m_rows = m_image.rows(); 
            m_cols = m_image.cols();
            

            // Precompute pdf
            m_pPhi = Eigen::MatrixXf(m_rows, m_cols);
            for (int x = 0; x < m_rows; x++)
            {
                float sinTheta = sin(M_PI * float(x + 0.5f) / float(m_rows));
                for (int y = 0; y < m_cols; y++)
                {
                    m_pPhi(x, y) = m_image(x, y).getLuminance() * sinTheta;
                }
            }

            m_pTheta = Eigen::VectorXf(m_pPhi.rows());
            m_pTheta = m_pPhi.rowwise().sum();
            m_pTheta = m_pTheta / m_pTheta.sum();
            for (int i = 0; i < m_pPhi.rows(); i++)
            {
                if (m_pTheta(i) > 1e-5)
                    m_pPhi.row(i) /= m_pTheta(i);
                else
                    m_pPhi.row(i) *= 0;
            }

        }

        virtual std::string toString() const override {
                return tfm::format(
                        "EnvironmentLight[\n"
                        "  file = %s,\n"
                        "  lightToWorld = %s\n"
                        "]",
                        m_path,
                        indent(m_lightToWorld.toString(), 18));
            }

        Color3f eval(const EmitterQueryRecord & lRec) const override {
            // Nearest neighbor interpolation
            Vector3f w = (m_worldToLight * lRec.wi).normalized();
            float theta = std::acos(clamp(w.z(), -1.f, 1.f));
            float p = std::atan2(w.y(), w.x());
            float phi = (p < 0) ? (p + 2 * M_PI) : p; 
            int x = round(theta * INV_PI * (m_rows - 1));
            int y = round(phi * INV_TWOPI * (m_cols - 1));
            x = clamp(x, 0, m_rows-1);
            y = clamp(y, 0, m_cols-1);
            return m_strength*m_image(x, m_cols -1 - y);
        }


        virtual Color3f sample(EmitterQueryRecord & lRec, const Point2f & sample) const override {
            float theta = M_PI * sample.x(), phi = 2 * M_PI * sample.y();
            float cosTheta = std::cos(theta), sinTheta = std::sin(theta);
            float sinPhi = std::sin(phi), cosPhi = std::cos(phi);
            lRec.wi = m_lightToWorld * Vector3f(sinTheta * cosPhi, sinTheta * sinPhi, cosTheta);
            lRec.shadowRay = Ray3f(lRec.ref, lRec.wi, Epsilon, INFINITY);
            if (pdf(lRec) > 1e-9)
                return eval(lRec)/pdf(lRec);
            else 
                return Color3f(0.f);
        }


        virtual float pdf(const EmitterQueryRecord &lRec) const override {
            Vector3f w = (m_worldToLight * lRec.wi).normalized();
            float theta = std::acos(clamp(w.z(), -1.f, 1.f));
            float p = std::atan2(w.y(), w.x());
            float phi = (p < 0) ? (p + 2 * M_PI) : p; 
            float sinTheta = std::sin(theta);
            if (sinTheta == 0) return 0;
            
            int x = round(theta * INV_PI * (m_rows - 1));
            int y = round(phi * INV_TWOPI * (m_cols -1));
            x = clamp(x, 0, m_rows-1);
            y = clamp(y, 0, m_cols-1);
            float pdf = m_pTheta(x) * m_pPhi(x, m_cols -1 - y);
            
            return pdf / (2 * sinTheta) * INV_PI * INV_PI;
        }

        virtual Color3f Le(const EmitterQueryRecord &lRec) const override{
            return eval(lRec);
        }

        virtual bool isEnvEmitter() const override{
            return true;
        } 


protected:
    float m_strength = 1;
    Transform m_worldToLight;
    Transform m_lightToWorld;

    // img
    std::string m_path;
    Bitmap m_image;
    // height and width of the image
    int m_rows;
    int m_cols;

    Eigen::VectorXf m_pTheta;
    Eigen::MatrixXf m_pPhi;
    //luminance matrix
    matrix luminance;
    //pdf matrix
    matrix mpdf;
    //cdf matrix
    matrix mcdf;
    //pdf marginal matrix
    matrix pmarginal;
    //cdf marginal matrix
    matrix cmarginal;
 

};

NORI_REGISTER_CLASS(EnvLight, "envlight")
NORI_NAMESPACE_END