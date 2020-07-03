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
//#include "Entities.hpp"

namespace scenarioengine
{

  /*
  Class Center
  Represents the geometrical center of the bounding box expressed in coordinates that
  refer to the coordinate system of the entity (e.g. the vehicle coordinate system).

  x 	double 	 	Center offset in x direction.
  y 	double 	 	Center offset in y direction.
  z 	double 	 	Center offset in z direction.
  */
  class Center {
  public:

    Center() : x_(0),y_(0),z_(0) {}
    double x_;
    double y_;
    double z_;
  };

/*
Class Dimension
Dimensions for a three dimensional box. Width, length and height are the absolute extensions
in the (y,x,z) coordinate system of the entity's local coordinate system.

width 	double 	Width of the entity's bounding box. Unit: m; Range: [0..inf[.
length 	double 	Length of the entity's bounding box. Unit: m; Range: [0..inf[.
height 	double 	Height of the entity's bounding box. Unit: m; Range: [0..inf[.
*/
  class Dimensions {
  public:
    Dimensions() : width_(0),length_(0),height_() {}

    double width_;
    double length_;
    double height_;

  };

  /*
  Class BoundingBox
  Defines geometric properties of the entities as a simplified three dimensional bounding box.

  center 	Represents the geometrical center of the bounding box expressed in coordinates that refer to the coordinate system of the entity (e.g. the vehicle coordinate system).
  dimensions 	Width, length and height of the bounding box.
  */
	class OSCBoundingBox
	{
	public:

		Center center_;
    Dimensions dimensions_;
	};



}
