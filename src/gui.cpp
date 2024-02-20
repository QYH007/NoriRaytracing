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

#include <nori/gui.h>
#include <nori/block.h>
#include <nori/parser.h>
#include <nori/bitmap.h>
#include <nanogui/shader.h>
#include <nanogui/label.h>
#include <nanogui/slider.h>
#include <nanogui/button.h>
#include <nanogui/layout.h>
#include <nanogui/icons.h>
#include <nanogui/renderpass.h>
#include <nanogui/progressbar.h>
#include <nanogui/texture.h>

#include <nanogui/opengl.h>

#include <filesystem/resolver.h>
#include <fstream>

NORI_NAMESPACE_BEGIN

#define PANEL_HEIGHT 50

NoriCanvas::NoriCanvas(nanogui::Widget* parent, const ImageBlock& block) : nanogui::Canvas(parent, 1), m_block(block) {
    using namespace nanogui;

    m_shader = new Shader(
        render_pass(),
        /* An identifying name */
        "Tonemapper",
        /* Vertex shader */
        R"(#version 330
        uniform ivec2 size;
        uniform int borderSize;

        in vec2 position;
        out vec2 uv;
        void main() {
            gl_Position = vec4(position.x * 2 - 1, position.y * 2 - 1, 0.0, 1.0);

            // Crop away image border (due to pixel filter)
            vec2 total_size = size + 2 * borderSize;
            vec2 scale = size / total_size;
            uv = vec2(position.x * scale.x + borderSize / total_size.x,
                      1 - (position.y * scale.y + borderSize / total_size.y));
        })",
        /* Fragment shader */
        R"(#version 330
        uniform sampler2D source;
        uniform float scale;
        in vec2 uv;
        out vec4 out_color;
        float toSRGB(float value) {
            if (value < 0.0031308)
                return 12.92 * value;
            return 1.055 * pow(value, 0.41666) - 0.055;
        }
        void main() {
            vec4 color = texture(source, uv);
            color *= scale / color.w;
            out_color = vec4(toSRGB(color.r), toSRGB(color.g), toSRGB(color.b), 1);
        })"
    );

    // Draw 2 triangles
    uint32_t indices[3 * 2] = {
        0, 1, 2,
        2, 3, 0
    };
    float positions[2 * 4] = {
        0.f, 0.f,
        1.f, 0.f,
        1.f, 1.f,
        0.f, 1.f
    };

    m_shader->set_buffer("indices", VariableType::UInt32, { 3 * 2 }, indices);
    m_shader->set_buffer("position", VariableType::Float32, { 4, 2 }, positions);

    const Vector2i& size = m_block.getSize();
    m_shader->set_uniform("size", nanogui::Vector2i(size.x(), size.y()));
    m_shader->set_uniform("borderSize", m_block.getBorderSize());

    update();
}


void NoriCanvas::draw_contents() {
    // Reload the partially rendered image onto the GPU
    m_block.lock();
    const Vector2i& size = m_block.getSize();
    m_shader->set_uniform("scale", m_scale);
    m_texture->upload((uint8_t*)m_block.data());
    m_shader->set_texture("source", m_texture);
    m_shader->begin();
    m_shader->draw_array(nanogui::Shader::PrimitiveType::Triangle, 0, 6, true);
    m_shader->end();
    m_block.unlock();
}


void NoriCanvas::update() {
    using namespace nanogui;
    // Assumes that m_block stays static
    const Vector2i& size = m_block.getSize();
    const int borderSize = m_block.getBorderSize();

    // Allocate texture memory for the rendered image
    m_texture = new Texture(
        Texture::PixelFormat::RGBA,
        Texture::ComponentFormat::Float32,
        nanogui::Vector2i(size.x() + 2 * borderSize,
            size.y() + 2 * borderSize),
        Texture::InterpolationMode::Nearest,
        Texture::InterpolationMode::Nearest);
}

NoriScreen::NoriScreen(ImageBlock& block)
    : nanogui::Screen(nanogui::Vector2i(block.getSize().x(), block.getSize().y() + PANEL_HEIGHT), "Nori", true),
    m_block(block), m_renderThread(m_block)
{
    using namespace nanogui;
    
    set_size(nanogui::Vector2i(block.getSize().x(), block.getSize().y() + PANEL_HEIGHT));

    /* Add some UI elements to adjust the exposure value */
    panel = new Widget(this);
    panel->set_width(m_block.getSize().x());
    panel->set_layout(new BoxLayout(Orientation::Horizontal, Alignment::Middle, 10, 10));
    Button* buttonOpen = new Button(panel, "", FA_FOLDER);

    buttonOpen->set_callback(
        [this]() {
        using FileType = std::pair<std::string, std::string>;
        std::vector<FileType> filetypes = { FileType("xml", "Nori Scene File"), FileType("exr", "EXR Image File") };
        std::string filename = nanogui::file_dialog(filetypes, false);
        drop_event({ filename });
    }
    );
    buttonOpen->set_tooltip("Open XML or EXR file (Ctrl+O)");

    m_progressBar = new ProgressBar(panel);
    m_progressBar->set_fixed_size(nanogui::Vector2i(200, 10));
    Button* buttonStop = new Button(panel, "", FA_STOP);
    buttonStop->set_callback(
        [this]() {
        m_renderThread.stopRendering();
    }
    );
    buttonStop->set_tooltip("Abort rendering (Ctrl+Z)");

    new Label(panel, "Exposure: ", "sans-bold");
    m_slider = new Slider(panel);
    m_slider->set_value(0.5f);
    m_slider->set_fixed_width(120);
    m_slider->set_callback(
        [&](float value) {
        m_scale = std::pow(2.f, (value - 0.5f) * 20);
        m_render_canvas->set_scale(m_scale);
    }
    );

    Button* buttonSave = new Button(panel, "", FA_SAVE);
    buttonSave->set_callback(
        [this]() {
        using FileType = std::pair<std::string, std::string>;
        std::vector<FileType> filetypes = { FileType("", "Image File Name") };
        std::string filename;
        // Keep asking for a filename until the user cancels or the file does not exist
        auto file_exists = [](const std::string& name) {
            std::ifstream f(name.c_str());
            return f.good();
        };
        while (true) {
            filename = nanogui::file_dialog(filetypes, true);
            if (filename.empty() || (!file_exists(filename + ".png") && !file_exists(filename + ".exr")))
                break;
            cerr << "Error: file \"" << filename << "\" already exists!" << endl;
        }
        if (!filename.empty()) {
            m_block.lock();
            std::unique_ptr<Bitmap> bitmap(m_block.toBitmap());
            m_block.unlock();
            bitmap->array() *= m_scale;   // apply exposure
            bitmap->save(filename);
        }
    }
    );
    buttonSave->set_tooltip("Save rendering with the selected exposure to disk");

    set_resize_callback([this](nanogui::Vector2i) { requestLayoutUpdate(); });

    m_render_canvas = new NoriCanvas(this, m_block);
    m_render_canvas->set_background_color({ 100, 100, 100, 255 });

    updateLayout();

    set_visible(true);
    draw_all();
}


void NoriScreen::draw_contents() {
    if (m_requiresLayoutUpdate) {
        updateLayout();
        m_requiresLayoutUpdate = false;
    }

    if (m_progressBar) {
        m_progressBar->set_value(m_renderThread.getProgress());
    }
    nanogui::Screen::draw_contents();
}


void NoriScreen::updateLayout() {
    perform_layout();

    {
        panel->set_position(nanogui::Vector2i((m_size.x() - panel->size().x()) / 2, 0));
    }

    nanogui::Vector2i contentOffset(0, PANEL_HEIGHT);

    {  
        nanogui::Vector2i contentWindow = m_size - contentOffset;
        nanogui::Vector2f blockSize(m_block.getSize().x(), m_block.getSize().y());
        nanogui::Vector2f ratio = (blockSize / nanogui::Vector2f(contentWindow));
        float maxRatio = std::max(ratio.x(), ratio.y());
        nanogui::Vector2i canvasSize = blockSize / maxRatio;

        m_render_canvas->set_fixed_size(canvasSize);
        m_render_canvas->set_position({ ((contentWindow - canvasSize) / 2 + contentOffset).x(), PANEL_HEIGHT });
    }

    perform_layout();
}


bool NoriScreen::drop_event(const std::vector<std::string>& filenames) {
    if (filenames.size() > 0) {
        std::string filename = filenames[0];
        if (filename.size() == 0) {
            return true;
        }
        filesystem::path path(filename);

        if (path.extension() == "xml") {
            /* Render the XML scene file */
            openXML(filename);
        } else if (path.extension() == "exr") {
            /* Alternatively, provide a basic OpenEXR image viewer */
            openEXR(filename);
        } else {
            cerr << "Error: unknown file \"" << filename
            << "\", expected an extension of type .xml or .exr" << endl;
        }

    }
    return true;
}

bool NoriScreen::keyboard_event(int key, int scancode, int action, int modifiers) {
    const bool press = (action == GLFW_PRESS);
    if (press && key == GLFW_KEY_O && modifiers & GLFW_MOD_CONTROL) {
        using FileType = std::pair<std::string, std::string>;
        std::vector<FileType> filetypes = { FileType("xml", "Nori Scene File"), FileType("exr", "EXR Image File") };
        std::string filename = nanogui::file_dialog(filetypes, false);
        drop_event({ filename });
        return true;
    }
    if (press && key == GLFW_KEY_Z && modifiers & GLFW_MOD_CONTROL) {
        m_renderThread.stopRendering();
        return true;
    }

    return nanogui::Screen::keyboard_event(key, scancode, action, modifiers);
}

void NoriScreen::openXML(const std::string& filename) {

    if(m_renderThread.isBusy()) {
        cerr << "Error: rendering in progress, you need to wait until it's done" << endl;
        return;
    }

    try {

        m_renderThread.renderScene(filename);

        m_block.lock();
        Vector2i bsize = m_block.getSize();
        m_render_canvas->update();
        m_block.unlock();

        requestLayoutUpdate();

    } catch (const std::exception &e) {
        cerr << "Fatal error: " << e.what() << endl;
    }

}

void NoriScreen::openEXR(const std::string& filename) {

    if(m_renderThread.isBusy()) {
        cerr << "Error: rendering in progress, you need to wait until it's done" << endl;
        return;
    }

    Bitmap bitmap(filename);
    m_block.lock();
    m_block.init(Vector2i(bitmap.cols(), bitmap.rows()), nullptr);
    m_block.fromBitmap(bitmap);
    Vector2i bsize = m_block.getSize();
    m_render_canvas->update();
    m_block.unlock();

    requestLayoutUpdate();
}

NORI_NAMESPACE_END
