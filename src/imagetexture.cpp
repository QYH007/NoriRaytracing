// modified from checkerboard.cpp
// reference: https://github.com/mitsuba-renderer/mitsuba3/blob/master/src/textures/bitmap.cpp

#include <nori/object.h>
#include <nori/texture.h>
// // import existing bitmap to read the texture picture
// #include <nori/bitmap.h>
// Modify: bitmap could only handle exr files
// include std_image to handle different file types
// #define STB_IMAGE_IMPLEMENTATION
// Comment out the above line due to the conflict with nanogui
#include <stb_image.h>
// can also include libpng/libjpeg/lodepng to handle png files

// the same as onj.cpp, use filesystem to load the path
#include <filesystem/resolver.h>

NORI_NAMESPACE_BEGIN

//We expanded the ImageTexture in image_texture.cpp to compute the mip pyramid 
//when a texture is loaded and perform trilinear interpolation when sampled.

/**
 * Image as texture
 * parameters:
 * * - filename
 *   - |string|
 *   - Filename of the texture to be loaded
 * 
 * * - scale
 *   - |vector2f|
 *   - The scale of the loaded texture
 * 
 * * - interpolation
 *   - |string|
 *   - Specifies how pixel values are interpolated and filtered when queried over
 *     large UV regions. The following options are available:
 *        - ``Linear`` (default) : perform bilinear interpolation, but no filtering.
 *        - ``Nearest``: disable filtering and interpolation. In this mode, the plugin
 *            performs nearest neighbor lookups of texture values.
 * 
 * * - extension
 *   - |string|
 *   - Controls the behavior of texture evaluations that fall outside of the
 *      :math:`[0, 1]` range. The following options are currently available:
 *        - ``REPEAT`` (default): tile the texture infinitely.
 *        - ``MIRROR``: MIRROR the texture along its boundaries.
 *        - ``CLAMP``: CLAMP coordinates to the edge of the texture.
*/
template <typename T>
class ImageTexture : public Texture<T> {
public: 
    ImageTexture(const PropertyList &props) {
        m_file = props.getString("filename","");
        m_scale = props.getVector2("scale", Vector2f(1));
        m_lerp = props.getString("interpolation","Linear");
        m_wrap = props.getString("extension","REPEAT");

        filesystem::path filePath = 
            getFileResolver()->resolve(m_file);
        m_image = stbi_load(filePath.str().c_str(), &m_img_width, &m_img_height, &m_img_channels, STBI_default);
        
        if(m_lerp != "Linear" && m_lerp != "Nearest" )
            throw NoriException("Invalid filter type \"%s\", must be one of: \"Nearest\", or "
                    "\"Linear\"!", m_lerp);

        if(m_wrap != "REPEAT" && m_wrap != "MIRROR" && m_wrap != "CLAMP")
            throw NoriException("Invalid wrap mode \"%s\", must be one of: \"REPEAT\", "
                    "\"MIRROR\", or \"CLAMP\"!", m_wrap);
    }

    virtual T eval(const Point2f & uv) override {
        /* to be implemented */
        // scale the origin uv
        float u = uv.x()/m_scale.x();
        float v = uv.y()/m_scale.y();

        // wrap the uv coordinates back into [0-1] range
        if(u<0 || u>=1)
            u = wrapCoordinate(u);
        if(v<0 || v>=1)
            v = wrapCoordinate(v);

        // nearest mode
        if(m_lerp == "Nearest"){
            int x = static_cast<int>(u * m_img_width);
            int y = static_cast<int>(v * m_img_height);
            return getPixelColor(x,y);
        }else{
        // get the left-up pixel of recent uv
        int fu = floor(u * m_img_width);
        int fv = floor(v * m_img_height);
        int cu = fu + 1;
        int cv = fv + 1;

        if(cu>m_img_width)
            cu = fu;
        if(cv>m_img_height)
            cv = fv;

        T color_fu_fv = getPixelColor(fu, fv);
        T color_fu_cv = getPixelColor(fu, cv);
        T color_cu_fv = getPixelColor(cu, fv);
        T color_cu_cv = getPixelColor(cu, cv);

        float tu = u * m_img_width - fu;
        float tv = v * m_img_height - fv;

        // bilerp
        T result = (1 - tu) * ((1 - tv) * color_fu_fv + tv * color_fu_cv) +
                tu * ((1 - tv) * color_cu_fv + tv * color_cu_cv);
        return result;
        }
    }

    std::string toString() const override {
        return tfm::format(
            "ImageTexture[\n"
                    "  filename = %s,\n"
                    "  scale = %s,\n"
                    "  interpolation = %s,\n"
                    "  extension = %s,\n"
                    "  width = %i,\n"
                    "  height = %i,\n"
                    "  channels = %i,\n"
                    "]",
            m_file,
            m_scale.toString(),
            m_lerp,
            m_wrap,
            m_img_width,
            m_img_height,
            m_img_channels
        );
    }

protected:
    // the scale of texture
    Vector2f m_scale;

    // the file path
    std::string m_file;
    // the loaded file
    unsigned char *m_image;

    int m_img_width, m_img_height, m_img_channels;

    // the filter type(see above)
    std::string m_lerp;
    // the wrap mode(see above)
    std::string m_wrap;

    static float GammaCorrect(float value) {
        if (value <= 0.0031308f) return 12.92f * value;
        return 1.055f * std::pow(value, 1.f/2.4f) - 0.055f;
    }

    float wrapCoordinate(float coord) const {
        switch (m_wrap[0]) {
            // In C++, you cannot use string literals in switch statements directly. 
            case 'r':
                return std::fmod(coord, 1.0f);
            case 'm':
                return std::fmod(coord, 2.0f) < 1.0f ? std::fmod(coord, 1.0f) : 1.0f - std::fmod(coord, 1.0f);
            case 'c':
                return std::clamp(coord, 0.0f, 1.0f);
            default:
                return coord;
        }
    }

    T getPixelColor(int x,int y) const {
        if (m_image != NULL) {
            if (m_img_channels == 3 || m_img_channels == 4) {
                // RGB format pic
                int pixel_index = (y * m_img_width + x) * m_img_channels;
                float red = m_image[pixel_index]/255.0f;
                float green = m_image[pixel_index + 1]/255.0f;
                float blue = m_image[pixel_index + 2]/255.0f;
                return Color3f(red,green,blue);
                // return Color3f(GammaCorrect(red),GammaCorrect(green),GammaCorrect(blue));
            }
            else if (m_img_channels == 1) {
                // grey scale pic
                int pixel_index = y * m_img_width + x;
                float gray = m_image[pixel_index]/255.0f;
                return gray;
                // return GammaCorrect(gray);
            }
            else {
                // other number of channels           
                throw NoriException("Unsupported number of channels: %d\n", m_img_channels);
            }
        }
        else {
            // no pic loaded
            throw NoriException("Failed to load image: %s\n", stbi_failure_reason());
        }
    }
};

NORI_REGISTER_TEMPLATED_CLASS(ImageTexture,Color3f,"image_texture")
NORI_NAMESPACE_END