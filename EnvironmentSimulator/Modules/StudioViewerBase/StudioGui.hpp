#pragma once

#include <osgViewer/ViewerEventHandlers>
#include "StudioViewer.hpp"
#include "StudioDataModel.hpp"

class StudioGui : public osgGA::GUIEventHandler
{
public:
    StudioGui(viewer::StudioViewer* viewer);
    ~StudioGui();

    bool             handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa) override;
    void             newFrame(osg::RenderInfo& renderInfo);
    void             render(osg::RenderInfo& renderInfo);
    StudioDataModel* GetModel()
    {
        return &data_model_;
    }

private:
    void Exit();

    double time_;
    bool   left_mouse_pressed_   = false;
    bool   right_mouse_pressed_  = false;
    bool   middle_mouse_pressed_ = false;
    float  mouse_wheel_;
    bool   initialized_;

    int                   viewport_width_  = -1;
    int                   viewport_height_ = -1;
    viewer::StudioViewer* viewer_          = nullptr;
    StudioDataModel       data_model_;
};
