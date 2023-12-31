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

#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <math.h>
#include "OSCCommon.hpp"
#include "CommonMini.hpp"
#include "RoadManager.hpp"
#include "Entities.hpp"

namespace scenarioengine
{
    class OSCOrientation
    {
    public:
        OSCOrientation() : type_(roadmanager::Position::OrientationType::ORIENTATION_ABSOLUTE), h_(std::nan("")), p_(std::nan("")), r_(std::nan(""))
        {
        }
        OSCOrientation(roadmanager::Position::OrientationType type, double h, double p, double r) : type_(type), h_(h), p_(p), r_(r)
        {
        }

        roadmanager::Position::OrientationType type_;
        double                                 h_;
        double                                 p_;
        double                                 r_;
    };

    class OSCPosition
    {
    public:
        typedef enum
        {
            WORLD,
            RELATIVE_WORLD,
            RELATIVE_OBJECT,
            ROAD,
            RELATIVE_ROAD,
            LANE,
            RELATIVE_LANE,
            ROUTE,
            TRAJECTORY,
            UNDEFINED
        } PositionType;

        OSCPosition() : type_(PositionType::UNDEFINED)
        {
        }
        OSCPosition(PositionType type) : type_(type)
        {
        }
        virtual ~OSCPosition() = default;

        PositionType type_;

        virtual double GetX()
        {
            return position_.GetX();
        }
        virtual double GetY()
        {
            return position_.GetY();
        }
        virtual double GetZ()
        {
            return position_.GetZ();
        }
        virtual double GetH()
        {
            return position_.GetH();
        }
        virtual double GetP()
        {
            return position_.GetP();
        }
        virtual double GetR()
        {
            return position_.GetR();
        }
        virtual void                   Print() = 0;
        virtual roadmanager::Position *GetRMPos()
        {
            return &position_;
        }

    protected:
        roadmanager::Position position_;
    };

    class OSCPositionWorld : OSCPosition
    {
    public:
        OSCPositionWorld() : OSCPosition(PositionType::WORLD)
        {
        }

        // base_on_pos: Provide position from which to base XY to road coord mapping from (optimize search)
        OSCPositionWorld(double x, double y, double z, double h, double p, double r, OSCPosition *base_on_pos);

        void Print()
        {
            position_.Print();
        }
    };

    class OSCPositionLane : OSCPosition
    {
    public:
        OSCPositionLane() : OSCPosition(PositionType::LANE)
        {
        }
        OSCPositionLane(int roadId, int laneId, double s, double offset, OSCOrientation orientation);

        void Print()
        {
            position_.Print();
        }
    };

    class OSCPositionRoad : OSCPosition
    {
    public:
        OSCPositionRoad() : OSCPosition(PositionType::LANE)
        {
        }
        OSCPositionRoad(int roadId, double s, double t, OSCOrientation orientation);

        void Print()
        {
            position_.Print();
        }
    };

    class OSCPositionRelativeObject : OSCPosition
    {
    public:
        Object *object_;

        OSCPositionRelativeObject(Object *object, double dx, double dy, double dz, OSCOrientation orientation);

        void Print();

        roadmanager::Position *GetRMPos()
        {
            return &position_;
        }
    };

    class OSCPositionRelativeWorld : OSCPosition
    {
    public:
        Object *object_;

        OSCPositionRelativeWorld(Object *object, double dx, double dy, double dz, OSCOrientation orientation);

        void                   Print();
        roadmanager::Position *GetRMPos()
        {
            return &position_;
        }
    };

    class OSCPositionRelativeLane : OSCPosition
    {
    public:
        Object        *object_;
        OSCOrientation o_;

        OSCPositionRelativeLane(Object                              *object,
                                int                                  dLane,
                                double                               ds,
                                double                               offset,
                                OSCOrientation                       orientation,
                                roadmanager::Position::DirectionMode direction_mode);

        void                   Print();
        roadmanager::Position *GetRMPos()
        {
            return &position_;
        }
    };

    class OSCPositionRelativeRoad : OSCPosition
    {
    public:
        Object *object_;

        OSCOrientation o_;

        OSCPositionRelativeRoad(Object *object, double ds, double dt, OSCOrientation orientation);

        void                   Print();
        roadmanager::Position *GetRMPos()
        {
            return &position_;
        }
    };

    class OSCPositionRoute : OSCPosition
    {
    public:
        OSCPositionRoute() : OSCPosition(PositionType::ROUTE)
        {
        }
        OSCPositionRoute(std::shared_ptr<roadmanager::Route> route, double s, int laneId, double laneOffset);

        void SetRoute(std::shared_ptr<roadmanager::Route> route)
        {
            position_.SetRoute(route);
        }
        void SetRouteRefLaneCoord(roadmanager::Route *route, double pathS, int laneId, double laneOffset, OSCOrientation *orientation);
        void SetRouteRefLaneCoord(roadmanager::Route *route, double pathS, int laneId, double laneOffset);
        void SetRouteRelativeHeading(double h_relative)
        {
            position_.SetHeadingRelative(h_relative);
        }

        void Print()
        {
            position_.Print();
        }
    };

    class OSCPositionTrajectory : OSCPosition
    {
    public:
        OSCPositionTrajectory() : OSCPosition(PositionType::TRAJECTORY)
        {
        }
        OSCPositionTrajectory(roadmanager::RMTrajectory *traj, double s, double t, OSCOrientation orientation);

        void SetTrajectory(roadmanager::RMTrajectory *traj)
        {
            position_.SetTrajectory(traj);
        }

        void Print()
        {
            position_.Print();
        }
    };

}  // namespace scenarioengine
