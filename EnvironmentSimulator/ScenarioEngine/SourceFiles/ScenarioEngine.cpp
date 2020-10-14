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

#include "ScenarioEngine.hpp"
#include "CommonMini.hpp"

#define WHEEL_RADIUS 0.35

using namespace scenarioengine;

ScenarioEngine::ScenarioEngine(std::string oscFilename, bool disable_controllers)
{
	InitScenario(oscFilename, disable_controllers);
}

ScenarioEngine::ScenarioEngine(const pugi::xml_document &xml_doc, bool disable_controllers)
{
	InitScenario(xml_doc, disable_controllers);
}

void ScenarioEngine::InitScenario(std::string oscFilename, bool disable_controllers)
{
	// Load and parse data
	LOG("Init %s", oscFilename.c_str());
	quit_flag = false;
	disable_controllers_ = disable_controllers;
	headstart_time_ = 0;
	simulationTime_ = 0;
	scenarioReader = new ScenarioReader(&entities, &catalogs, disable_controllers);
	if (scenarioReader->loadOSCFile(oscFilename.c_str()) != 0)
	{
		throw std::invalid_argument(std::string("Failed to load OpenSCENARIO file ") + oscFilename);
	}

	parseScenario();
}

void ScenarioEngine::InitScenario(const pugi::xml_document &xml_doc, bool disable_controllers)
{
	LOG("Init %s", xml_doc.name());
	quit_flag = false;
	disable_controllers_ = disable_controllers;
	headstart_time_ = 0;
	simulationTime_ = 0;
	scenarioReader->loadOSCMem(xml_doc);
	parseScenario();
}

ScenarioEngine::~ScenarioEngine()
{
	LOG("Closing");
}


void ScenarioEngine::step(double deltaSimTime, bool initial)
{
	simulationTime_ += deltaSimTime;

	if (entities.object_.size() == 0)
	{
		return;
	}

	if (initial)
	{
		if (!disable_controllers_)
		{ 
			// Initialize controllers
			for (size_t i = 0; i < scenarioReader->controller_.size(); i++)
			{
				scenarioReader->controller_[i]->Init();
			}
		}

		// Set initial values for speed and acceleration derivation
		for (size_t i = 0; i < entities.object_.size(); i++)
		{
			Object* obj = entities.object_[i];

			obj->state_old.pos_x = obj->pos_.GetX();
			obj->state_old.pos_y = obj->pos_.GetY();
			obj->state_old.vel_x = obj->pos_.GetVelX();
			obj->state_old.vel_y = obj->pos_.GetVelY();
			obj->state_old.h = obj->pos_.GetH();
			obj->state_old.h_rate = obj->pos_.GetHRate();
			obj->reset_ = true;
		}

		// kick off init actions
		for (size_t i = 0; i < init.private_action_.size(); i++)
		{
			init.private_action_[i]->Start();
			init.private_action_[i]->UpdateState();
		}
	}
	else
	{
		// reset indicators of applied control
		for (size_t i = 0; i < entities.object_.size(); i++)
		{
			entities.object_[i]->dirty_lat_ = false;
			entities.object_[i]->dirty_long_ = false;
			entities.object_[i]->reset_ = false;
		}

		// Run controllers
		if (!disable_controllers_)
		{
			for (size_t i = 0; i < scenarioReader->controller_.size(); i++)
			{
				if (scenarioReader->controller_[i]->Active())
				{
					if (simulationTime_ > 0 || scenarioReader->controller_[i]->GetType() == Controller::Type::CONTROLLER_TYPE_GHOST)
					{
						scenarioReader->controller_[i]->Step(deltaSimTime);
					}
				}
			}
		}

		// Fetch external states from gateway, except the initial run where scenario engine sets all positions
		for (size_t i = 0; i < entities.object_.size(); i++)
		{
			Object* obj = entities.object_[i];
			ObjectState o;

			if (scenarioGateway.getObjectStateById(entities.object_[i]->id_, o) != 0)
			{
				LOG("Gateway did not provide state for external car %d", entities.object_[i]->id_);
			}
			else
			{
				entities.object_[i]->pos_ = o.state_.pos;
				entities.object_[i]->speed_ = o.state_.speed;
				entities.object_[i]->wheel_angle_ = o.state_.wheel_angle;
				entities.object_[i]->wheel_rot_ = o.state_.wheel_rot;
			}
		}
	}

	// Step inital actions - might be extened in time (more than one step)
	for (size_t i = 0; i < init.private_action_.size(); i++)
	{
		if (init.private_action_[i]->IsActive())
		{
			//LOG("Stepping action of type %d", init.private_action_[i]->action_[j]->type_)
			init.private_action_[i]->Step(deltaSimTime, getSimulationTime());
			init.private_action_[i]->UpdateState();
		}
	}

	// Story
	// First evaluate StoryBoard stopTrigger
	if (storyBoard.stop_trigger_ && storyBoard.stop_trigger_->Evaluate(&storyBoard, simulationTime_) == true)
	{
		quit_flag = true;
		return;
	}

	// Then evaluate all stories
	bool all_done = true;  // This flag will indicate whether all acts are done or not
	for (size_t i = 0; i < storyBoard.story_.size(); i++)
	{
		Story *story = storyBoard.story_[i];

		for (size_t j = 0; j < story->act_.size(); j++)
		{
			Act *act = story->act_[j];

			if (act->IsTriggable())
			{
				// Check start conditions
				if (!act->start_trigger_)
				{
					// Start act even if there's no trigger
					act->Start();
				}
				else if (act->start_trigger_->Evaluate(&storyBoard, simulationTime_) == true)
				{
					act->Start();
				}
			}

			if (act->IsActive() && act->stop_trigger_)
			{
				if (act->stop_trigger_->Evaluate(&storyBoard, simulationTime_) == true)
				{
					act->End();
				}
			}

			act->UpdateState();

			// Check whether this act is done - and update flag for all acts
			all_done = all_done && act->state_ == Act::State::COMPLETE;

			// Update state of sub elements - moving from transitions to stable states
			for (size_t k = 0; k < act->maneuverGroup_.size(); k++)
			{
				for (size_t l = 0; l < act->maneuverGroup_[k]->maneuver_.size(); l++)
				{
					OSCManeuver *maneuver = act->maneuverGroup_[k]->maneuver_[l];

					for (size_t m = 0; m < maneuver->event_.size(); m++)
					{
						for (size_t n = 0; n < maneuver->event_[m]->action_.size(); n++)
						{
							maneuver->event_[m]->action_[n]->UpdateState();
						}
						maneuver->event_[m]->UpdateState();
					}
				}
			}

			// Maneuvers
			if (act->IsActive())
			{
				for (size_t k = 0; k < act->maneuverGroup_.size(); k++)
				{
					for (size_t l = 0; l < act->maneuverGroup_[k]->maneuver_.size(); l++)
					{
						OSCManeuver *maneuver = act->maneuverGroup_[k]->maneuver_[l];

						for (size_t m = 0; m < maneuver->event_.size(); m++)
						{
							Event *event = maneuver->event_[m];

							if (event->IsTriggable())
							{
								// Check event conditions
								if (event->start_trigger_->Evaluate(&storyBoard, simulationTime_) == true)
								{
									// Check priority
									if (event->priority_ == Event::Priority::OVERWRITE)
									{
										// Activate trigged event
										if (event->IsActive())
										{
											LOG("Can't overwrite own running event (%s) - skip trig", event->name_.c_str());
										}
										else
										{
											// Deactivate any currently active event
											for (size_t n = 0; n < maneuver->event_.size(); n++)
											{
												if (maneuver->event_[n]->IsActive())
												{
													maneuver->event_[n]->End();
													LOG("Event %s ended, overwritten by event %s",
														maneuver->event_[n]->name_.c_str(), event->name_.c_str());
												}
											}

											event->Start();
										}
									}
									else if (event->priority_ == Event::Priority::SKIP)
									{
										if (maneuver->IsAnyEventActive())
										{
											LOG("Event is running, skipping trigged %s", event->name_.c_str());
										}
										else
										{
											event->Start();
										}
									}
									else if (event->priority_ == Event::Priority::PARALLEL)
									{
										// Don't care if any other action is ongoing, launch anyway
										if (event->IsActive())
										{
											LOG("Event %s already running, trigger ignored", event->name_.c_str());
										}
										else if (maneuver->IsAnyEventActive())
										{
											LOG("Event(s) ongoing, %s will run in parallel", event->name_.c_str());
										}
										event->Start();
									}
									else
									{
										LOG("Unknown event priority: %d", event->priority_);
									}
								}
							}

							// Update (step) all active actions, for all objects connected to the action
							if (event->IsActive())
							{
								bool active = false;

								for (size_t n = 0; n < event->action_.size(); n++)
								{
									if (event->action_[n]->IsActive())
									{
										event->action_[n]->Step(deltaSimTime, getSimulationTime());

										active = active || (event->action_[n]->IsActive());
									}
								}
								if (!active)
								{
									// Actions done -> Set event done
									event->End();
								}
							}
						}
					}
				}
			}
		}
	}

	stepObjects(deltaSimTime);

	// Report resulting states to the gateway
	for (size_t i = 0; i < entities.object_.size(); i++)
	{
		Object *obj = entities.object_[i];

		scenarioGateway.reportObject(obj->id_, obj->name_, static_cast<int>(obj->type_), obj->category_holder_,obj->model_id_,
			obj->GetControllerType(), obj->boundingbox_, simulationTime_, obj->speed_, obj->wheel_angle_, obj->wheel_rot_, &obj->pos_);
	}

	if (!disable_controllers_)
	{
		for (size_t i = 0; i < scenarioReader->controller_.size(); i++)
		{
			if (scenarioReader->controller_[i]->Active())
			{
				scenarioReader->controller_[i]->PostFrame();
			}
		}
	}
	
	if (all_done)
	{
		LOG("All acts are done, quit now");
		quit_flag = true;
	}
}

void ScenarioEngine::printSimulationTime()
{
	LOG("simulationTime = %.2f", simulationTime_);
}

ScenarioGateway *ScenarioEngine::getScenarioGateway()
{
	return &scenarioGateway;
}

void ScenarioEngine::parseScenario()
{
	bool hybrid_objects = false;

	scenarioReader->LoadControllers();

	scenarioReader->SetGateway(&scenarioGateway);
	scenarioReader->parseGlobalParameterDeclarations();

	// Init road manager
	scenarioReader->parseRoadNetwork(roadNetwork);

	if (getOdrFilename().empty())
	{
		LOG("No OpenDRIVE file specified, continue without");
	}
	else
	{
		// First assume absolute path or relative to current directory
		std::string filename = getOdrFilename();
		if (!FileExists(filename.c_str()) || !roadmanager::Position::LoadOpenDrive(filename.c_str()))
		{
			// Then try relative path to scenario directory
			filename = CombineDirectoryPathAndFilepath(DirNameOf(scenarioReader->getScenarioFilename()), getOdrFilename());
			if (!FileExists(filename.c_str()) || !roadmanager::Position::LoadOpenDrive(filename.c_str()))
			{
				// Finally look for the file in current directory
				std::string path = std::string(getOdrFilename().c_str());
				std::string base_filename = path.substr(path.find_last_of("/\\") + 1);
				LOG("Failed to load %s - looking for file %s in current folder", getOdrFilename().c_str(), base_filename.c_str());
				if (!roadmanager::Position::LoadOpenDrive(base_filename.c_str()))
				{
					throw std::invalid_argument(std::string("Failed to load OpenDRIVE file ") + std::string(getOdrFilename().c_str()));
				}
			}
		}
	}

	odrManager = roadmanager::Position::GetOpenDrive();

	scenarioReader->parseCatalogs();
	scenarioReader->parseEntities();

	scenarioReader->parseInit(init);
	scenarioReader->parseStoryBoard(storyBoard);
	storyBoard.entities_ = &entities; 

	SetSimulationTime(0);
	for (size_t i = 0; i < entities.object_.size(); i++)
	{
		Object* obj = entities.object_[i];

		if (!disable_controllers_ && obj->GetControllerType() == Controller::Type::CONTROLLER_TYPE_GHOST)
		{
			if (obj->controller_->GetHeadstartTime() > GetHeadstartTime())
			{
				SetHeadstartTime(obj->controller_->GetHeadstartTime());
				SetSimulationTime(-obj->controller_->GetHeadstartTime());
			}
		}
	}

	// Print loaded data
	entities.Print();

	storyBoard.Print();
}

void ScenarioEngine::stepObjects(double dt)
{
	for (size_t i = 0; i < entities.object_.size(); i++)
	{
		Object *obj = entities.object_[i];

		if (simulationTime_ < 0 && obj->GetControllerType() != Controller::Type::CONTROLLER_TYPE_GHOST)
		{
			continue;  // only ghosts allowed to execute before time == 0
		}

		if (fabs(obj->speed_) > SMALL_NUMBER)  // do not move objects when speed is zero
		{
			int retvalue = 0;
			double steplen = obj->speed_ * dt;

			// Add or subtract stepsize according to curvature and offset, in order to keep constant speed
			double curvature = obj->pos_.GetCurvature();
			double offset = obj->pos_.GetT();
			if (abs(curvature) > SMALL_NUMBER)
			{
				// Approximate delta length by sampling curvature in current position
				steplen += steplen * curvature * offset;
			}

			if (obj->pos_.GetRoute() && !obj->dirty_lat_)
			{
				int retvalue = obj->pos_.MoveRouteDS(steplen);

				if (retvalue == roadmanager::Position::ErrorCode::ERROR_END_OF_ROUTE)
				{
					if (!obj->IsEndOfRoad())
					{
						obj->SetEndOfRoad(true, simulationTime_);
					}
				}
				else
				{
					obj->SetEndOfRoad(false);
				}
			}
			else if (!obj->dirty_long_)  // No action has updated longitudinal dimension
			{
				if (!obj->dirty_lat_)  // No action has updated lateral dimension
				{
					// make sure entity is aligned to the road 
					if (obj->pos_.GetHRelative() > M_PI_2 && obj->pos_.GetHRelative() < 3 * M_PI_2)
					{
						obj->pos_.SetHeadingRelative(M_PI);
					}
					else
					{
						obj->pos_.SetHeadingRelative(0);
					}
				}

				// Adjustment movement to heading and road direction
				if (GetAbsAngleDifference(obj->pos_.GetH(), obj->pos_.GetDrivingDirection()) > M_PI_2)
				{
					// If pointing in other direction
					steplen *= -1;
				}

				retvalue = obj->pos_.MoveAlongS(steplen);

				if (retvalue == roadmanager::Position::ErrorCode::ERROR_END_OF_ROAD)
				{
					if (!obj->IsEndOfRoad())
					{
						obj->SetEndOfRoad(true, simulationTime_);
					}
				}
				else
				{
					obj->SetEndOfRoad(false);
				}
			}
		}

		// Calculate resulting updated velocity, acceleration and heading rate (rad/s) NOTE: in global coordinate sys
		double dx = obj->pos_.GetX() - obj->state_old.pos_x;
		double dy = obj->pos_.GetY() - obj->state_old.pos_y;

		if (dt > SMALL_NUMBER)
		{
			obj->pos_.SetVelX(dx / dt);
			obj->pos_.SetVelY(dy / dt);
			obj->pos_.SetAccX((obj->pos_.GetVelX() - obj->state_old.vel_x) / dt);
			obj->pos_.SetAccY((obj->pos_.GetVelY() - obj->state_old.vel_y) / dt);
			double heading_rate_new = GetAngleDifference(obj->pos_.GetH(), obj->state_old.h) / dt;
			obj->pos_.SetHRate(heading_rate_new);
			obj->pos_.SetHAcc(GetAngleDifference(heading_rate_new, obj->state_old.h_rate) / dt);
			
			// Update wheel rotations of internal scenario objects
			if (obj->GetControllerType() == Controller::Type::CONTROLLER_TYPE_DEFAULT && !obj->dirty_lat_)
			{
				obj->wheel_angle_ = heading_rate_new / 2;
			}
			if (obj->GetControllerType() == Controller::Type::CONTROLLER_TYPE_DEFAULT && !obj->dirty_long_)
			{
				obj->wheel_rot_ = fmod(obj->wheel_rot_ + obj->speed_ * dt / WHEEL_RADIUS, 2 * M_PI);
			}

		}
		else
		{
			// calculate approximated velocity vector based on current heading
			obj->pos_.SetVelX(obj->speed_ * cos(obj->pos_.GetH()));
			obj->pos_.SetVelY(obj->speed_ * sin(obj->pos_.GetH()));
		}

		// store current values for next loop
		obj->state_old.pos_x = obj->pos_.GetX();
		obj->state_old.pos_y = obj->pos_.GetY();
		obj->state_old.vel_x = obj->pos_.GetVelX();
		obj->state_old.vel_y = obj->pos_.GetVelY();
		obj->state_old.h = obj->pos_.GetH();
		obj->state_old.h_rate = obj->pos_.GetHRate();

		if (!obj->reset_)
		{
			obj->odometer_ += abs(sqrt(dx * dx + dy * dy));  // odometer always measure all movements as positive, I guess...
		}

		obj->trail_.AddState((float)simulationTime_, (float)obj->pos_.GetX(), (float)obj->pos_.GetY(), (float)obj->pos_.GetZ(), (float)obj->speed_);
	}
}
