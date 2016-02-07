#include <SFML/Graphics.hpp>

#include <osgViewer/Viewer>
#include <osgGA/TrackballManipulator>
#include <osgGA/StateSetManipulator>

#include "geom/polyhedron.h"
#include "geom/io.h"
#include "throw_if.h"

#include "rs274_backplot.h"
#include "rs274ngc_return.hh"

#include <boost/program_options.hpp>
#include "print_exception.h"

#include <iostream>
#include <fstream>
#include <thread>
#include <atomic>
#include <mutex>

namespace po = boost::program_options;

int convert(sf::Keyboard::Key k) {
    using namespace osgGA;
    switch(k) {
        case sf::Keyboard::Unknown:
	        break;
        case sf::Keyboard::A:
        case sf::Keyboard::B:
        case sf::Keyboard::C:
        case sf::Keyboard::D:
        case sf::Keyboard::E:
        case sf::Keyboard::F:
        case sf::Keyboard::G:
        case sf::Keyboard::H:
        case sf::Keyboard::I:
        case sf::Keyboard::J:
        case sf::Keyboard::K:
        case sf::Keyboard::L:
        case sf::Keyboard::M:
        case sf::Keyboard::N:
        case sf::Keyboard::O:
        case sf::Keyboard::P:
        case sf::Keyboard::Q:
        case sf::Keyboard::R:
        case sf::Keyboard::S:
        case sf::Keyboard::T:
        case sf::Keyboard::U:
        case sf::Keyboard::V:
        case sf::Keyboard::W:
        case sf::Keyboard::X:
        case sf::Keyboard::Y:
        case sf::Keyboard::Z:
            return GUIEventAdapter::KEY_A + (k - sf::Keyboard::A);
        case sf::Keyboard::Num0:
        case sf::Keyboard::Num1:
        case sf::Keyboard::Num2:
        case sf::Keyboard::Num3:
        case sf::Keyboard::Num4:
        case sf::Keyboard::Num5:
        case sf::Keyboard::Num6:
        case sf::Keyboard::Num7:
        case sf::Keyboard::Num8:
        case sf::Keyboard::Num9:
            return GUIEventAdapter::KEY_0 + (k - sf::Keyboard::Num0);
        case sf::Keyboard::Escape:
            return GUIEventAdapter::KEY_Escape;
        case sf::Keyboard::LControl:
            return GUIEventAdapter::KEY_Control_L;
        case sf::Keyboard::LShift:
            return GUIEventAdapter::KEY_Shift_L;
        case sf::Keyboard::LAlt:
            return GUIEventAdapter::KEY_Alt_L;
        case sf::Keyboard::LSystem:
            return GUIEventAdapter::KEY_Meta_L;
        case sf::Keyboard::RControl:
            return GUIEventAdapter::KEY_Control_R;
        case sf::Keyboard::RShift:
            return GUIEventAdapter::KEY_Shift_R;
        case sf::Keyboard::RAlt:
            return GUIEventAdapter::KEY_Alt_L;
        case sf::Keyboard::RSystem:
            return GUIEventAdapter::KEY_Meta_R;
        case sf::Keyboard::Menu:
	        break;
        case sf::Keyboard::LBracket:
	        break;
        case sf::Keyboard::RBracket:
	        break;
        case sf::Keyboard::SemiColon:
	        break;
        case sf::Keyboard::Comma:
	        break;
        case sf::Keyboard::Period:
	        break;
        case sf::Keyboard::Quote:
	        break;
        case sf::Keyboard::Slash:
	        break;
        case sf::Keyboard::BackSlash:
	        break;
        case sf::Keyboard::Tilde:
	        break;
        case sf::Keyboard::Equal:
	        break;
        case sf::Keyboard::Dash:
	        break;
        case sf::Keyboard::Space:
	        break;
        case sf::Keyboard::Return:
	        break;
        case sf::Keyboard::BackSpace:
	        break;
        case sf::Keyboard::Tab:
	        break;
        case sf::Keyboard::PageUp:
	        break;
        case sf::Keyboard::PageDown:
	        break;
        case sf::Keyboard::End:
	        break;
        case sf::Keyboard::Home:
	        break;
        case sf::Keyboard::Insert:
	        break;
        case sf::Keyboard::Delete:
	        break;
        case sf::Keyboard::Add:
	        break;
        case sf::Keyboard::Subtract:
	        break;
        case sf::Keyboard::Multiply:
	        break;
        case sf::Keyboard::Divide:
	        break;
        case sf::Keyboard::Left:
	        break;
        case sf::Keyboard::Right:
	        break;
        case sf::Keyboard::Up:
	        break;
        case sf::Keyboard::Down:
	        break;
        case sf::Keyboard::Numpad0:
	        break;
        case sf::Keyboard::Numpad1:
	        break;
        case sf::Keyboard::Numpad2:
	        break;
        case sf::Keyboard::Numpad3:
	        break;
        case sf::Keyboard::Numpad4:
	        break;
        case sf::Keyboard::Numpad5:
	        break;
        case sf::Keyboard::Numpad6:
	        break;
        case sf::Keyboard::Numpad7:
	        break;
        case sf::Keyboard::Numpad8:
	        break;
        case sf::Keyboard::Numpad9:
	        break;
        case sf::Keyboard::F1:
	        break;
        case sf::Keyboard::F2:
	        break;
        case sf::Keyboard::F3:
	        break;
        case sf::Keyboard::F4:
	        break;
        case sf::Keyboard::F5:
	        break;
        case sf::Keyboard::F6:
	        break;
        case sf::Keyboard::F7:
	        break;
        case sf::Keyboard::F8:
	        break;
        case sf::Keyboard::F9:
	        break;
        case sf::Keyboard::F10:
	        break;
        case sf::Keyboard::F11:
	        break;
        case sf::Keyboard::F12:
	        break;
        case sf::Keyboard::F13:
	        break;
        case sf::Keyboard::F14:
	        break;
        case sf::Keyboard::F15:
	        break;
        case sf::Keyboard::Pause:
	        break;
        case sf::Keyboard::KeyCount:
	        break;
    }
    return -1;
}

void pushModel(osg::Geode* geode, const geom::object_t& object) {
    auto geom = new osg::Geometry();

    auto vertices = new osg::Vec3Array;
    auto normals = new osg::Vec3Array;
    vertices->reserve(object.vertices.size() * 3);
    normals->reserve(object.faces.size());
    for(auto& face : object.faces) {
        for(auto& vi : face.vertices) {
            auto& v = object.vertices[vi];
            vertices->push_back({static_cast<float>(v.x), static_cast<float>(v.y), static_cast<float>(v.z)});
        }
        normals->push_back({static_cast<float>(face.normal.x), static_cast<float>(face.normal.y), static_cast<float>(face.normal.z)});
    }
    geom->setVertexArray(vertices);
    geom->setNormalArray(normals, osg::Array::BIND_PER_PRIMITIVE_SET);

    auto colors = new osg::Vec4Array;
    colors->push_back({0.3f,0.3f,0.3f,0.8f});
    geom->setColorArray(colors, osg::Array::BIND_OVERALL);

    geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::TRIANGLES, 0, vertices->size()));

    geode->addDrawable(geom);
}

int main(int argc, char* argv[]) {
    po::options_description options("nc_model");
    std::vector<std::string> args(argv, argv + argc);
    args.erase(begin(args));

    options.add_options()
        ("help,h", "display this help and exit")
        ("model", po::value<std::string>(), "Model file")
    ;

    try {
        po::variables_map vm;
        store(po::command_line_parser(args).options(options).run(), vm);

        if(vm.count("help")) {
            std::cout << options << "\n";
            return 0;
        }
        notify(vm);

        sf::VideoMode mode = sf::VideoMode::getDesktopMode();
        mode.width = 800;
        mode.height = 600;

        sf::Window window(mode, "Backplot");
        window.setVerticalSyncEnabled(true);

    //    osg::setNotifyLevel(osg::DEBUG_INFO);

        osgViewer::Viewer viewer;
        sf::Vector2u size = window.getSize();
        auto gw = viewer.setUpViewerAsEmbeddedInWindow(0, 0, size.x, size.y);

        viewer.setCameraManipulator(new osgGA::TrackballManipulator);
        viewer.getCameraManipulator()->setHomePosition({0, 0, 100}, {0,0,0}, {0,0,1}, false);
        osg::ref_ptr<osgGA::StateSetManipulator> statesetManipulator = new osgGA::StateSetManipulator(viewer.getCamera()->getStateSet());
        viewer.addEventHandler(statesetManipulator.get());

        auto geode = new osg::Geode();
        viewer.setSceneData(geode);

        viewer.realize();

        std::thread model_thread([&]{
            if(vm.count("model")) {
                geom::object_t model;
                std::ifstream is(vm["model"].as<std::string>());
                throw_if(!(is >> model), "Unable to read model from file");

                pushModel(geode, model);
            }
        });

        rs274_backplot backplotter{geode};

        auto adapt = [gw](const sf::Event& event) {
            auto eq = gw->getEventQueue();
            switch(event.type) {
                case sf::Event::Resized:
                    eq->windowResize(0, 0, event.size.width, event.size.height);
                    break;
                case sf::Event::MouseWheelMoved:
                    eq->mouseWarped(event.mouseWheel.x, event.mouseWheel.y);
                    eq->mouseScroll(event.mouseWheel.delta > 0 ? osgGA::GUIEventAdapter::SCROLL_UP : osgGA::GUIEventAdapter::SCROLL_DOWN);
                    break;
                case sf::Event::MouseMoved:
                    eq->mouseMotion(event.mouseMove.x, event.mouseMove.y);
                    break;
                case sf::Event::MouseButtonPressed:
                    if (event.mouseButton.button == sf::Mouse::Left)
                        eq->mouseButtonPress(event.mouseButton.x, event.mouseButton.y, 1);
                    if (event.mouseButton.button == sf::Mouse::Right)
                        eq->mouseButtonPress(event.mouseButton.x, event.mouseButton.y, 3);
                    if (event.mouseButton.button == sf::Mouse::Middle)
                        eq->mouseButtonPress(event.mouseButton.x, event.mouseButton.y, 2);
                    break;
                case sf::Event::MouseButtonReleased:
                    if (event.mouseButton.button == sf::Mouse::Left)
                        eq->mouseButtonRelease(event.mouseButton.x, event.mouseButton.y, 1);
                    if (event.mouseButton.button == sf::Mouse::Right)
                        eq->mouseButtonRelease(event.mouseButton.x, event.mouseButton.y, 3);
                    if (event.mouseButton.button == sf::Mouse::Middle)
                        eq->mouseButtonRelease(event.mouseButton.x, event.mouseButton.y, 2);
                    break;
                case sf::Event::KeyPressed: {
                    auto key = convert(event.key.code);
                    if(key != -1)
                        eq->keyPress(key, event.key.code);
                    break;
                }
                case sf::Event::KeyReleased: {
                    auto key = convert(event.key.code);
                    if(key != -1)
                        eq->keyPress(key, event.key.code);
                    break;
                }
                default:
                    break;
            }
        };

        std::atomic<bool> running{true};
        std::mutex draw_mt;

        std::thread rs274_thread([&] {

            std::string line;
            while(running && std::getline(std::cin, line)) {
                int status;
                
                status = backplotter.read(line.c_str());
                if(status != RS274NGC_OK) {
                    if(status != RS274NGC_EXECUTE_FINISH) {
                        std::cerr << "Error reading line!: \n";
                        std::cerr << line <<"\n";
                        return status;
                    }
                }
                
                status = backplotter.execute();
                if(status != RS274NGC_OK)
                    return status;
                std::cerr << line << "\n";
            }
            return 0;
        });

        while(running) {
            sf::Event event;
            while (window.pollEvent(event)) {
                adapt(event);
                switch(event.type) {
                    case sf::Event::Closed:
                        running = false;
                        break;
                    case sf::Event::Resized:
                        gw->resized(0, 0, event.size.width, event.size.height);
                        break;
                    case sf::Event::KeyPressed:
                        if(event.key.code == sf::Keyboard::Escape)
                            running = false;
                        break;
                    default:
                        break;
                }
            }

            {
                std::lock_guard<std::mutex> lock(draw_mt);
                viewer.frame();
                window.display();
            }
        }

        model_thread.join();
        rs274_thread.join();

    } catch(const po::error& e) {
        print_exception(e);
        std::cout << options << "\n";
        return 1;
    } catch(const std::exception& e) {
        print_exception(e);
        return 1;
    }

    return 0;
}
