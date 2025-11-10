/*
 * esmini - Environment Simulator Minimalistic
 * https://github.com/esmini/esmini
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright (c) partners of Simulation Scenarios
 * https://sites.google.com/view/simulationscenarios
 */

#ifndef TRAFFICLIGHT_HPP_
#define TRAFFICLIGHT_HPP_

#include <osg/PositionAttitudeTransform>
#include <osg/Switch>
#include <osg/Geode>
#include <osg/Texture2D>
#include <string>

class TrafficLightModel
{
public:
    TrafficLightModel(unsigned int n_lights, std::string texture_filename);
    osg::ref_ptr<osg::Geode>                     CreateOpenBox();
    osg::ref_ptr<osg::Switch>                    CreateFrontSwitchNode(int index, osg::ref_ptr<osg::Texture2D> texture);
    osg::ref_ptr<osg::PositionAttitudeTransform> GetTx()
    {
        return tx_.get();
    }
    void         SetState(unsigned int light_index, bool state);
    bool         GetState(unsigned int light_index) const;
    unsigned int GetNrLamps() const
    {
        return n_lights_;
    }

private:
    std::vector<osg::ref_ptr<osg::Switch>>       switches_;
    osg::ref_ptr<osg::PositionAttitudeTransform> tx_;
    unsigned int                                 n_lights_;
    float                                        box_height_ = 0.4f;
    float                                        box_width_  = 0.4f;
    float                                        box_depth_  = 0.1f;
    bool                                         dirty_      = false;  // indicates whether graphics needs to update
};

#endif  // TRAFFICLIGHT_HPP_
