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

#if !defined(__NORI_GUI_H)
#define __NORI_GUI_H

#include <nori/common.h>
#include <nanogui/screen.h>
#include <nanogui/renderpass.h>
#include <nanogui/shader.h>
#include <nanogui/canvas.h>
#include <nori/render.h>

NORI_NAMESPACE_BEGIN

class NoriCanvas : public nanogui::Canvas {
public:
    NoriCanvas(nanogui::Widget* parent, const ImageBlock& block);
    void draw_contents() override;
    void update();
    void set_scale(float scale) {
        m_scale = scale;
    }
private:
    const ImageBlock& m_block;
    nanogui::ref<nanogui::Shader> m_shader;
    nanogui::ref<nanogui::Texture> m_texture;
    nanogui::ref<nanogui::RenderPass> m_renderPass;
    float m_scale = 1.f;
};

class NoriScreen : public nanogui::Screen {
public:
    NoriScreen(ImageBlock& block);
     void draw_contents() override;

    virtual bool keyboard_event(int key, int scancode, int action, int modifiers) override;
    virtual bool drop_event(const std::vector<std::string>& filenames) override;

    void updateLayout();

    void requestLayoutUpdate() {
        m_requiresLayoutUpdate = true;
    }

    void openXML(const std::string& filename);
    void openEXR(const std::string& filename);

private:
    ImageBlock& m_block;
    nanogui::ref<NoriCanvas> m_render_canvas;
    nanogui::ref<nanogui::Shader> m_shader;
    nanogui::ref<nanogui::RenderPass> m_renderPass;
    nanogui::Slider* m_slider = nullptr;
    nanogui::ProgressBar* m_progressBar = nullptr;
    nanogui::ref<nanogui::Texture> m_texture;
    bool m_requiresLayoutUpdate = false;
    float m_scale = 1.f;
    Widget* panel = nullptr;

    RenderThread m_renderThread;
};

NORI_NAMESPACE_END

#endif /* __NORI_GUI_H */
