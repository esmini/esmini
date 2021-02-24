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

#include "Entities.hpp"
#include "Controller.hpp"

using namespace scenarioengine;

#define ELEVATION_DIFF_THRESHOLD 2.5

void Object::SetEndOfRoad(bool state, double time)
{
	if (state == true)
	{
		end_of_road_timestamp_ = time;
	}
	else
	{
		end_of_road_timestamp_ = 0.0;
	}
}

void Object::SetOffRoad(bool state, double time)
{
	if (state == true)
	{
		off_road_timestamp_ = time;
	}
	else
	{
		off_road_timestamp_ = 0.0;
	}
}

void  Object::SetStandStill(bool state, double time)
{
	if (state == true)
	{
		stand_still_timestamp_ = time;
	}
	else
	{
		stand_still_timestamp_ = 0.0;
	}
}

int Object::GetAssignedControllerType()
{
	if (controller_)
	{
		return controller_->GetType();
	}
	else
	{
		// Report 0 if not assigned or not activated on any domain
		return 0;
	}
}

int Object::GetActivatedControllerType()
{
	if (controller_ && controller_->GetDomain())
	{
		return controller_->GetType();
	}
	else
	{
		// Report 0 if not assigned or not activated on any domain
		if (IsGhost())
		{
			return Controller::Type::GHOST_RESERVED_TYPE;
		}
		else
		{ 
			return 0;
		}
	}
}

bool Object::IsControllerActiveOnDomains(int domainMask)
{
	if (controller_)
	{
		return (controller_->GetDomain() & domainMask) == domainMask;
	}
	else
	{
		return false;
	}
}

bool Object::IsControllerActiveOnAnyOfDomains(int domainMask)
{
	if (controller_)
	{
		return controller_->GetDomain() & domainMask;
	}
	else
	{
		return false;
	}
}

int Object::GetControllerMode()
{
	if (controller_)
	{
		return controller_->GetMode();
	}
	else
	{
		return 0;  // default
	}
}

void Object::SetVisibilityMask(int mask)
{
	visibilityMask_ = mask;
	SetDirtyBits(dirty_ | DirtyBit::VISIBILITY);
}

void Object::SetVel(double x_vel, double y_vel, double z_vel)
{
	pos_.SetVel(x_vel, y_vel, z_vel);
	SetDirtyBits(dirty_ | DirtyBit::VELOCITY);
}

void Object::SetAcc(double x_acc, double y_acc, double z_acc)
{
	pos_.SetAcc(x_acc, y_acc, z_acc);
	SetDirtyBits(dirty_ | DirtyBit::ACCELERATION);
}

void Object::SetAngularVel(double h_vel, double p_vel, double r_vel)
{
	pos_.SetAngularVel(h_vel, p_vel, r_vel);
	SetDirtyBits(dirty_ | DirtyBit::ANGULAR_RATE);
}

void Object::SetAngularAcc(double h_acc, double p_acc, double r_acc)
{
	pos_.SetAngularAcc(h_acc, p_acc, r_acc);
	SetDirtyBits(dirty_ | DirtyBit::ANGULAR_ACC);
}

bool Object::Collision(Object* target)
{
	// Apply method Separating Axis Theorem (SAT)
	// http://www.euclideanspace.com/threed/games/examples/cars/collisions/
	// https://www.sevenson.com.au/actionscript/sat/

	// Idea:
	// For each side of the bounding boxes:
	//   The normal of that edge will be the projection axis
	//   Project all points of the two bounding boxes onto that axis
	//   If we find ONE side wich a gap between point clusters from BB1 and BB2,
	//   it's enough to conclude they are not overlapping/colliding
	// 
	// Optimization: Since the bounding boxes are boxes with parallel
	// sides, we only need to check half of the sides

	if (target == 0)
	{
		return false;
	}

	Object* obj0 = this;
	Object* obj1 = target;

	// First do a rough check to rule out potential overlap/collision
	// Compare radial/euclidean distance with sum of the diagonal dimension of the bounding boxes
	double x = 0, y = 0;
	double dist = fabs(this->pos_.getRelativeDistance(target->pos_, x, y));
	double max_length = this->boundingbox_.dimensions_.length_ + target->boundingbox_.dimensions_.length_;
	double max_width = this->boundingbox_.dimensions_.width_ + target->boundingbox_.dimensions_.width_;
	double dist_threshold = sqrt(max_length * max_length + max_width * max_width);

	if (dist > dist_threshold)
	{
		return false;
	}

	// Also do a Z sanity check, to rule out on different road elevations
	if (fabs(obj0->pos_.GetZ() - obj1->pos_.GetZ()) > ELEVATION_DIFF_THRESHOLD)
	{
		return false;
	}

	for (int i = 0; i < 2; i++)  // for each of the two BBs
	{
		if (i == 1)
		{ 
			// swap order, now check all edges of target object
			obj0 = target;
			obj1 = this;
		}

		double n0[2] = { 0.0, 0.0 };
		for (int j = 0; j < 2; j++)  // for longitudinal and lateral sides
		{
			if (j == 0)
			{
				// Normal for longitudinal side (sides) points along lateral side
				n0[0] = 0.0;
				n0[1] = 1.0;
			}
			else
			{
				// Normal for lateral side (front/rear) points along lateral side
				n0[0] = 1.0;
				n0[1] = 0.0;
			}
		
			// Rotate the normal/projection axis to align with bounding box/vehicle
			double n1[2] = { 0.0, 0.0 };
			RotateVec2D(n0[0], n0[1], obj0->pos_.GetH(), n1[0], n1[1]);

			// Now, project each point of each BB onto the rotated normal
			// And register min and max for point cluster of each BB
			double min[2] = { 0.0, 0.0 }, max[2] = { 0.0, 0.0 };
			for (int k = 0; k < 2; k++)
			{ 
				Object* obj = (k == 0 ? obj0 : obj1);

				// Specify bounding box corner vertices, starting at first quadrant
				double vertices[4][2] =
				{
					{ obj->boundingbox_.center_.x_ + obj->boundingbox_.dimensions_.length_ / 2.0, obj->boundingbox_.center_.y_ + obj->boundingbox_.dimensions_.width_ / 2.0 },
					{ obj->boundingbox_.center_.x_ - obj->boundingbox_.dimensions_.length_ / 2.0, obj->boundingbox_.center_.y_ + obj->boundingbox_.dimensions_.width_ / 2.0 },
					{ obj->boundingbox_.center_.x_ - obj->boundingbox_.dimensions_.length_ / 2.0, obj->boundingbox_.center_.y_ - obj->boundingbox_.dimensions_.width_ / 2.0 },
					{ obj->boundingbox_.center_.x_ + obj->boundingbox_.dimensions_.length_ / 2.0, obj->boundingbox_.center_.y_ - obj->boundingbox_.dimensions_.width_ / 2.0 }
				};

				for (int l = 0; l < 4; l++)
				{
					double point_to_project[2];

					// Align projection points to object heading
					RotateVec2D(vertices[l][0], vertices[l][1], obj->pos_.GetH(), point_to_project[0], point_to_project[1]);

					double dot_p = GetDotProduct2D(
						obj->pos_.GetX() + point_to_project[0],
						obj->pos_.GetY() + point_to_project[1],
						n1[0], n1[1]);

					if (l == 0)
					{
						min[k] = max[k] = dot_p;
					}
					else 
					{
						min[k] = MIN(dot_p, min[k]);
						max[k] = MAX(dot_p, max[k]);
					}
				}
			}

			if (min[0] < min[1] - SMALL_NUMBER && max[0] < min[1] - SMALL_NUMBER ||
				max[0] > max[1] + SMALL_NUMBER && min[0] > max[1] + SMALL_NUMBER)
			{
				// gap found
				return false;
			}
		}
	}

	return true;
}

int Entities::addObject(Object* obj)
{
	obj->id_ = getNewId();
	object_.push_back(obj);
	return obj->id_;
}

void Entities::removeObject(int id) 
{
	for (size_t i = 0; i < object_.size(); i++) 
	{
		if (object_[i]->id_ == id) 
		{
			object_.erase(object_.begin() + i);
		}
	}
}

void Entities::removeObject(std::string name)
{
	for (size_t i = 0; i < object_.size(); i++) 
	{
		if (object_[i]->name_ == name) 
		{
			object_.erase(object_.begin() + i);
		}
	}
}

bool Entities::nameExists(std::string name)
{
	for (size_t i = 0; i < object_.size(); i++) 
	{
		if (object_[i]->name_ == name) 
		{
			return true;
		}
	}
	return false;
}

bool Entities::indexExists(int id) 
{
	for (size_t i = 0; i < object_.size(); i++) 
	{
		if (object_[i]->id_ == id) 
		{
			return false;
		}
	}
	return true;
}

int Entities::getNewId()
{
	int retint = 0;
	while (!indexExists(retint))
	{
		retint += 1;
	}
	return retint;
}


Object* Entities::GetObjectByName(std::string name)
{
	for (size_t i = 0; i < object_.size(); i++)
	{
		if (name == object_[i]->name_)
		{
			return object_[i];
		}
	}

	LOG("Failed to find object %s", name.c_str());
	
	return 0;
}

Object* Entities::GetObjectById(int id)
{
	for (size_t i = 0; i < object_.size(); i++)
	{
		if (id == object_[i]->id_)
		{
			return object_[i];
		}
	}

	LOG("Failed to find object with id %d", id);

	return 0;
}
