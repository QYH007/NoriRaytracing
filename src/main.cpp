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

#include <nori/block.h>
#include <nori/gui.h>
#include <filesystem/path.h>
#include <indicators/progress_bar.hpp>

using namespace nori;


bool run_gui(std::string filename, bool is_xml) {
    try {
        nanogui::init();

        // Open the UI with a dummy image
        ImageBlock block(Vector2i(720, 720), nullptr);
        {
            // Scoped
            nanogui::ref<NoriScreen> screen = new NoriScreen(block);

            if (filename.length() > 0) {
                if (is_xml) {
                    /* Render the XML scene file */
                    screen->openXML(filename);
                } else {
                    /* Alternatively, provide a basic OpenEXR image viewer */
                    screen->openEXR(filename);
                }
            }

            nanogui::mainloop(50.f);
        }

        nanogui::shutdown();

    } catch (const std::exception &e) {
        cerr << "Fatal error: " << e.what() << endl;
        return 1;
    }

    return 0;
}


bool render_headless(std::string filename, bool is_xml) {
    // TODOs - proper handling of an ctrl+z, progress bar, CL argument -b for headless
	ImageBlock block(Vector2i(720, 720), nullptr);
	RenderThread renderer(block);

    if (!filename.length()) {
        cerr << "Need to provide an input XML file to render in headless mode" << endl;
        return 1;
    }

    if (!is_xml) {
        cerr << "Only XML files supported in the headless mode" << endl;
        return 1;
    }

	try {
		// StringBar m_cliBar;
		renderer.renderScene(filename);

#ifndef NORI_HEADLESS
        indicators::ProgressBar bar{
            indicators::option::PrefixText{"Rendering... "},
            indicators::option::BarWidth{50},
            indicators::option::ShowPercentage{true},
            indicators::option::ShowElapsedTime{true},
            indicators::option::ShowRemainingTime{true}
        };
		while (renderer.isBusy()) {
            bar.set_progress(renderer.getProgress() * 100);
			std::this_thread::sleep_for(std::chrono::seconds(1));
		}
#else
        while (renderer.isBusy()) {
			std::this_thread::sleep_for(std::chrono::seconds(1));
		}
#endif

	}
	catch (const std::exception &e) {
		cerr << "Failed to render in the headless mode " << e.what() << endl;
        return 1;
	}

    return 0;
}


int main(int argc, char **argv) {
    std::string filename = "";
    bool headless = false;

    for (int i = 1; i < argc; ++i) {
        std::string token(argv[i]);
        if (token == "--help") {
            cout << "Syntax: " << argv[0] << "[-b] <scene.[xml|exr]>" <<  endl;
            return 0;
        }
        
        if (token == "-b" || token == "--background") {
            headless = true;
            continue;
        }

        if (!filename.length()) {
            filename = token;
            continue;
        } else {
            cerr << "Syntax: " << argv[0] << "[-b] <scene.[xml|exr]>" <<  endl;
            return -1;
        }
    }

    bool is_xml = false;
    if (filename.length()) {
        filesystem::path path(filename);

        if (path.extension() == "xml") {
            is_xml = true;
        } else if (path.extension() == "exr") {
            /* Alternatively, provide a basic OpenEXR image viewer */
            is_xml = false;
        } else {
            cerr << "Error: unknown file \"" << filename
            << "\", expected an extension of type .xml or .exr" << endl;
        }
    }

#ifdef NORI_HEADLESS
    if (!headless) {
        cout << "Forcing background mode" << endl;
        headless = true;
    }
#endif

    if (headless) {
        return render_headless(filename, is_xml);
    } else {
        return run_gui(filename, is_xml);
    }

}
