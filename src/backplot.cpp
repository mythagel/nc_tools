#include <SFML/Graphics.hpp>

#include <osgViewer/Viewer>
#include <osgGA/TrackballManipulator>
#include <osgGA/StateSetManipulator>

#include <iostream>

osg::Node* createBackplot()
{
    auto geode = new osg::Geode();

    auto geom = new osg::Geometry();

    auto vertices = new osg::Vec3Array;
    vertices->reserve(8);
    vertices->push_back({-1.13704, -2.15188e-09, 0.40373});
    vertices->push_back({-0.856897, -2.15188e-09, 0.531441});
    vertices->push_back({-0.889855, -2.15188e-09, 0.444927});
    vertices->push_back({-0.568518, -2.15188e-09, 0.40373});
    vertices->push_back({-1.00933, -2.15188e-09, 0.370773});
    vertices->push_back({-0.716827, -2.15188e-09, 0.292498});
    vertices->push_back({-1.07936, 9.18133e-09, 0.317217});
    vertices->push_back({-0.700348, 9.18133e-09, 0.362533});
    geom->setVertexArray(vertices);

    auto colors = new osg::Vec4Array;
    colors->push_back({1.0f,0.0f,0.0f,1.0f});
    geom->setColorArray(colors, osg::Array::BIND_OVERALL);

    auto normals = new osg::Vec3Array;
    normals->push_back({0.0f,-1.0f,0.0f});
    geom->setNormalArray(normals, osg::Array::BIND_OVERALL);

    //geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES,0,vertices->size()));
    geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP,0,vertices->size()));

    geode->addDrawable(geom);
    return geode;
}

int main() {
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
    osg::ref_ptr<osgGA::StateSetManipulator> statesetManipulator = new osgGA::StateSetManipulator(viewer.getCamera()->getStateSet());
    viewer.addEventHandler(statesetManipulator.get());

    viewer.setSceneData(createBackplot());

    viewer.realize();

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
        }
    };
 
    for(bool running = true; running; ) {
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
                case sf::Event::KeyPressed: {
                    std::cout << "key: " << event.key.code << "\n";
                    std::cout << "control:" << event.key.control << "\n";
                    std::cout << "alt:" << event.key.alt << "\n";
                    //if (event.key.code == sf::Keyboard::Escape)
                    break;
                }
            }
        }

        viewer.frame();
        window.display();
    }

    return 0;
}
