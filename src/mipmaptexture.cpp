// modified from checkerboard.cpp
// reference: https://github.com/mitsuba-renderer/mitsuba3/blob/master/src/textures/bitmap.cpp

#include <nori/object.h>
#include <nori/texture.h>
#include <nori/shape.h>
// // import existing bitmap to read the texture picture
// #include <nori/bitmap.h>
// Modify: bitmap could only handle exr files
// include std_image to handle different file types
// #define STB_IMAGE_IMPLEMENTATION
// Comment out the above line due to the conflict with nanogui
#include <stb_image.h>
#include <stb_image_write.h>
// can also include libpng/libjpeg/lodepng to handle png files

// the same as onj.cpp, use filesystem to load the path
#include <filesystem/resolver.h>
#include <filesystem>
namespace fs = std::filesystem;

NORI_NAMESPACE_BEGIN

template <typename T>
class MipmapTexture : public Texture<T> {
public: 
    MipmapTexture(const PropertyList &props) {
        m_file = props.getString("filename","");
        m_scale = props.getVector2("scale", Vector2f(1));
        m_lerp = props.getString("interpolation", "Linear");
        m_wrap = props.getString("extension","REPEAT");

        filesystem::path filePath = 
            getFileResolver()->resolve(m_file);
        m_image = stbi_load(filePath.str().c_str(), &m_img_width, &m_img_height, &m_img_channels, STBI_default);

        // mipmap_init    
        m_levels = std::log2(std::max(m_img_width, m_img_height)) + 1;
        m_mipmaps = std::vector<unsigned char*>(m_levels);

        generateMipmap();

        if(m_lerp != "Linear" && m_lerp != "Nearest" )
            throw NoriException("Invalid filter type \"%s\", must be one of: \"Nearest\", or "
                    "\"Linear\"!", m_lerp);

        if(m_wrap != "REPEAT" && m_wrap != "MIRROR" && m_wrap != "CLAMP")
            throw NoriException("Invalid wrap mode \"%s\", must be one of: \"REPEAT\", "
                    "\"MIRROR\", or \"CLAMP\"!", m_wrap);
    }

    void generateMipmap() {

        m_mipmaps[0] = m_image;
        int levelWidth = m_img_width;
        int levelHeight = m_img_height;
        int oldLevelWidth = 0;
        if (!fs::exists("Level_0.jpg")) {
            stbi_write_png("Level_0.jpg", m_img_width, m_img_height, m_img_channels, m_mipmaps[0], m_img_width * m_img_channels);
        }
        std::cout << "Level 0 - Width: " << m_img_width << ", Height: " << m_img_height << std::endl;

        for (int i = 1; i < m_levels; ++i) {
            oldLevelWidth = levelWidth;
            levelWidth = std::max(1, m_img_width >> i);
            levelHeight = std::max(1, m_img_height >> i);

            m_mipmaps[i] = new unsigned char[levelWidth * levelHeight * m_img_channels];

            // count for every texel
            for (int y = 0; y < levelHeight; ++y) {
                for (int x = 0; x < levelWidth; ++x) {
                    for (int c = 0; c < m_img_channels; ++c) {
                        float sum = 0.0f;
                        for (int dy = 0; dy < 2; ++dy) {
                            for (int dx = 0; dx < 2; ++dx) {
                                int px = x * 2 + dx;
                                int py = y * 2 + dy;
                                sum += 255.0 * getMipmapColor(m_mipmaps[i-1], oldLevelWidth,  px, py)[c];
                            }
                        }
  
                        sum /= 4.0f;  // average
                        m_mipmaps[i][(y * levelWidth + x) * m_img_channels + c] = sum;
                    }              
                }


            }
            char fn[20]; // for file name
            sprintf(fn, "Level_%d.jpg", i);
            if (!fs::exists(fn)) {
                stbi_write_png(fn, levelWidth, levelHeight, m_img_channels, m_mipmaps[i], levelWidth * m_img_channels);
            }
            //std::cout << "Level " << i << " - Width: " << levelWidth << ", Height: " << levelHeight << std::endl;
        }
    }

    virtual T evalMipmap(const Point2f & uv, float mipmapLevel) override {

        // compute for levels by the max surface directional differential
        int level1 = std::floor(mipmapLevel);
        int level2 = std::min(level1 + 1, m_levels - 1);
        // compute weight
        float t = mipmapLevel - level1;

        // Bilinear in both level
        T texel1 = sampleBilinear(uv, level1);
        T texel2 = sampleBilinear(uv, level2);

        // do Trillinear above the two levels with weight
        return ((1.0f - t) * texel1 + t * texel2);

    }

    T sampleBilinear(const Point2f &uv, int level) {
        int level_width =  std::max(1, m_img_width >> level);
        int level_height =  std::max(1, m_img_height >> level);
        // compute uv
        float u = uv.x()/m_scale.x();
        float v = uv.y()/m_scale.y();
        if(u<0 || u>=1)
            u = wrapCoordinate(u);
        if(v<0 || v>=1)
            v = wrapCoordinate(v);

        // get neighbor texels
        int x1 = floor(u * level_width);
        int y1 = floor(v * level_height);
        int x2 = x1 + 1;
        int y2 = y1 + 1;

        if(x2>level_width)
            x2 = x1;
        if(y2>level_width)
            y2 = y1;

        // binlinear computing
        float tx = u * level_width - x1;
        float ty = v * level_height - y1;

        // look up color in mipmap within correct level
        T texel1 = getMipmapColor(m_mipmaps[level],level_width, x1, y1);
        T texel2 = getMipmapColor(m_mipmaps[level],level_width, x2, y1);
        T texel3 = getMipmapColor(m_mipmaps[level],level_width, x1, y2);
        T texel4 = getMipmapColor(m_mipmaps[level],level_width, x2, y2);

        T value1 = (1.0f - tx) * texel1 + tx * texel2;
        T value2 = (1.0f - tx) * texel3 + tx * texel4;
        return (1.0f - ty) * value1 + ty * value2;
    }


    virtual T eval(const Point2f & uv) override {
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
    std::vector<unsigned char*> m_mipmaps;
    int m_levels;

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
            case 'REPEAT':
                return std::fmod(coord, 1.0f);
            case 'MIRROR':
                return std::fmod(coord, 2.0f) < 1.0f ? std::fmod(coord, 1.0f) : 1.0f - std::fmod(coord, 1.0f);
            case 'CLAMP':
                return std::clamp(coord, 0.0f, 1.0f);
            default:
                return coord;
        }
    }

    T getMipmapColor( unsigned char* mipmap, int levelWidth, int x, int y) const {
        if (mipmap != NULL) {
            if (m_img_channels == 3 || m_img_channels == 4) {
                // RGB format pic
                int pixel_index = (y * levelWidth + x) * m_img_channels;
                float red = mipmap[pixel_index]/255.0f;
                float green = mipmap[pixel_index + 1]/255.0f;
                float blue = mipmap[pixel_index + 2]/255.0f;
                return Color3f(red,green,blue);
            }
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

NORI_REGISTER_TEMPLATED_CLASS(MipmapTexture, Color3f, "mipmap_texture")
NORI_NAMESPACE_END