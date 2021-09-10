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
#include "ControllerFollowGhost.hpp"
#include "ControllerExternal.hpp"
#include "ControllerRel2Abs.hpp"

#define WHEEL_RADIUS 0.35
#define STAND_STILL_THRESHOLD 1e-3  // meter per second
#define TRAJECTORY_SAMPLE_TIME 0.2

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
	quit_flag = false;
	disable_controllers_ = disable_controllers;
	headstart_time_ = 0;
	simulationTime_ = 0;
	initialized_ = false;
	scenarioReader = new ScenarioReader(&entities, &catalogs, disable_controllers);


	std::vector<std::string> file_name_candidates;
	// absolute path or relative to current directory
	file_name_candidates.push_back(oscFilename);
	// Remove all directories from path and look in current directory
	file_name_candidates.push_back(FileNameOf(oscFilename));
	// Finally check registered paths
	for (size_t i = 0; i < SE_Env::Inst().GetPaths().size(); i++)
	{
		file_name_candidates.push_back(CombineDirectoryPathAndFilepath(SE_Env::Inst().GetPaths()[i], FileNameOf(oscFilename)));
	}
	size_t i;
	for (i = 0; i < file_name_candidates.size(); i++)
	{
		if (FileExists(file_name_candidates[i].c_str()))
		{
			if (scenarioReader->loadOSCFile(file_name_candidates[i].c_str()) != 0)
			{
				throw std::invalid_argument(std::string("Failed to load OpenSCENARIO file ") + oscFilename);
			}
			else
			{
				break;
			}
		}
	}

	if (i == file_name_candidates.size())
	{
		throw std::invalid_argument(std::string("Couldn't locate OpenSCENARIO file ") + oscFilename);
	}

	if (!scenarioReader->IsLoaded())
	{
		throw std::invalid_argument(std::string("Couldn't load OpenSCENARIO file ") + oscFilename);
	}

	parseScenario();
}

void ScenarioEngine::InitScenario(const pugi::xml_document &xml_doc, bool disable_controllers)
{
	quit_flag = false;
	disable_controllers_ = disable_controllers;
	headstart_time_ = 0;
	simulationTime_ = 0;
	initialized_ = false;
	scenarioReader = new ScenarioReader(&entities, &catalogs, disable_controllers);
	if (scenarioReader->loadOSCMem(xml_doc) != 0)
	{
		throw std::invalid_argument("Failed to load OpenSCENARIO from XML string");
	}
	parseScenario();
}

ScenarioEngine::~ScenarioEngine()
{
	scenarioReader->UnloadControllers();
	delete scenarioReader;
	scenarioReader = 0;
	LOG("Closing");
}

int ScenarioEngine::step(double deltaSimTime)
{
	simulationTime_ += deltaSimTime;

	if (entities.object_.size() == 0)
	{
		return -1;
	}

	if (!initialized_)
	{
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
			init.private_action_[i]->Start(simulationTime_, deltaSimTime);
			init.private_action_[i]->UpdateState();
		}
		initialized_ = true;
	}
	else
	{
		// reset update bits and indicators of applied control
		for (size_t i = 0; i < entities.object_.size(); i++)
		{
			Object* obj = entities.object_[i];
			ObjectState o;

			obj->ClearDirtyBits(
				Object::DirtyBit::LATERAL |
				Object::DirtyBit::LONGITUDINAL |
				Object::DirtyBit::SPEED |
				Object::DirtyBit::WHEEL_ANGLE |
				Object::DirtyBit::WHEEL_ROTATION
			);
			obj->reset_ = false;

			// Fetch states from gateway, in case external reports
			if (scenarioGateway.getObjectStateById(obj->id_, o) != 0)
			{
				LOG("Gateway did not provide state for external car %d", obj->id_);
			}
			else
			{
				obj->pos_ = o.state_.pos;
				obj->speed_ = o.state_.info.speed;
				obj->wheel_angle_ = o.state_.info.wheel_angle;
				obj->wheel_rot_ = o.state_.info.wheel_rot;

				if (obj->pos_.GetStatusBitMask() & static_cast<int>(roadmanager::Position::PositionStatusMode::POS_STATUS_END_OF_ROAD))
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
	}

	// Step inital actions - might be extened in time (more than one step)
	for (size_t i = 0; i < init.private_action_.size(); i++)
	{
		if (init.private_action_[i]->IsActive())
		{
			//Add action to object initActions vector if it doesn't contain the action
			if (std::find(init.private_action_[i]->object_->initActions_.begin(), init.private_action_[i]->object_->initActions_.end(), init.private_action_[i]) == init.private_action_[i]->object_->initActions_.end())
			{
				init.private_action_[i]->object_->initActions_.push_back(init.private_action_[i]);
			}
			//LOG("Stepping action of type %d", init.private_action_[i]->action_[j]->type_)
			init.private_action_[i]->Step(getSimulationTime(), deltaSimTime);
			init.private_action_[i]->UpdateState();
		}
	}

	// Story
	// First evaluate StoryBoard stopTrigger
	if (storyBoard.stop_trigger_ && storyBoard.stop_trigger_->Evaluate(&storyBoard, simulationTime_) == true)
	{
		quit_flag = true;
		return -1;
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
					act->Start(simulationTime_, deltaSimTime);
				}
				else if (act->start_trigger_->Evaluate(&storyBoard, simulationTime_) == true)
				{
					act->Start(simulationTime_, deltaSimTime);
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

							//add event to objectEvents vector
							if (event->IsTriggable() || event->IsActive())
							{
								for (size_t n = 0; n < event->action_.size(); n++)
								{
									OSCAction* action = event->action_[n];
									if (action->base_type_ == OSCAction::BaseType::PRIVATE)
									{
										OSCPrivateAction* pa = (OSCPrivateAction*)action;
										if (!pa->object_->containsEvent(event))
										{
											pa->object_->addEvent(event);
											break;
										}

									}
								}
							}

							if (event->IsTriggable())
							{
								// Check event conditions
								if (event->start_trigger_->Evaluate(&storyBoard, simulationTime_) == true)
								{
									bool startEvent = false;

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
													//remove event from objectEvents vector
													for (size_t o = 0; o < maneuver->event_[n]->action_.size(); o++)
													{
														OSCAction* action = maneuver->event_[n]->action_[o];
														if (action->base_type_ == OSCAction::BaseType::PRIVATE)
														{
															OSCPrivateAction* pa = (OSCPrivateAction*)action;
															pa->object_->removeEvent(event);

															break;
														}
													}

													maneuver->event_[n]->End();
													LOG("Event %s ended, overwritten by event %s",
														maneuver->event_[n]->name_.c_str(), event->name_.c_str());
												}
											}

											startEvent = true;
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
											startEvent = true;
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

										startEvent = true;
									}
									else
									{
										LOG("Unknown event priority: %d", event->priority_);
									}

									if (startEvent)
									{
										event->Start(simulationTime_, deltaSimTime);
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
										event->action_[n]->Step(simulationTime_, deltaSimTime);

										active = active || (event->action_[n]->IsActive());
									}
								}
								if (!active)
								{
									//remove event from objectEvents vector
									for (size_t n = 0; n < event->action_.size(); n++)
									{
										OSCAction* action = event->action_[n];
										if (action->base_type_ == OSCAction::BaseType::PRIVATE)
										{
											OSCPrivateAction* pa = (OSCPrivateAction*)action;
											pa->object_->removeEvent(event);
											break;
										}
									}

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

	for (size_t i = 0; i < entities.object_.size(); i++)
	{
		Object* obj = entities.object_[i];
		// Do not move objects when speed is zero,
		// and only ghosts allowed to execute before time == 0
		if (!(obj->IsControllerActiveOnDomains(ControlDomains::DOMAIN_BOTH) && obj->GetControllerMode() == Controller::Mode::MODE_OVERRIDE) &&
			fabs(obj->speed_) > SMALL_NUMBER &&
			(simulationTime_ > 0 || obj->IsGhost()))
		{
			defaultController(obj, deltaSimTime);
		}

		// Report state to the gateway
		scenarioGateway.reportObject(obj->id_, obj->name_, static_cast<int>(obj->type_), obj->category_, obj->model_id_,
			obj->GetActivatedControllerType(), obj->boundingbox_, simulationTime_, obj->speed_, obj->wheel_angle_, obj->wheel_rot_, &obj->pos_);
	}

	// Apply controllers
	if (!disable_controllers_)
	{
		for (size_t i = 0; i < scenarioReader->controller_.size(); i++)
		{
			if (scenarioReader->controller_[i]->Active())
			{
				if (simulationTime_ >= 0)
				{
					scenarioReader->controller_[i]->Step(deltaSimTime);
				}
			}
		}
	}

	// Check some states
	for (size_t i = 0; i < entities.object_.size(); i++)
	{
		Object* obj = entities.object_[i];

		// Off road?
		if (obj->pos_.IsOffRoad())
		{
			if (!obj->IsOffRoad())
			{
				obj->SetOffRoad(true, simulationTime_);
			}
		}
		else
		{
			obj->SetOffRoad(false);
		}

		// Stand still?
		if (obj->GetSpeed() > -STAND_STILL_THRESHOLD && obj->GetSpeed() < STAND_STILL_THRESHOLD)
		{
			if (!obj->IsStandStill())
			{
				obj->SetStandStill(true, simulationTime_);
			}
		}
		else
		{
			obj->SetStandStill(false);
		}
	}

	if (all_done)
	{
		LOG("All acts are done, quit now");
		quit_flag = true;
	}

	return 0;
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
	SetSimulationTime(0);

	if (!disable_controllers_)
	{
		scenarioReader->LoadControllers();
	}

	scenarioReader->SetGateway(&scenarioGateway);

	scenarioReader->parseOSCHeader();
	if (scenarioReader->GetVersionMajor() < 1)
	{
		LOG_AND_QUIT("OpenSCENARIO v%d.%d not supported. Please migrate scenario to v1.0 or v1.1 and try again.",
			scenarioReader->GetVersionMajor(), scenarioReader->GetVersionMinor());
	}
	LOG("Loading %s (v%d.%d)", FileNameOf(scenarioReader->getScenarioFilename()).c_str(), scenarioReader->GetVersionMajor(), scenarioReader->GetVersionMinor());

	scenarioReader->parseGlobalParameterDeclarations();

	// Init road manager
	scenarioReader->parseRoadNetwork(roadNetwork);

	if (getOdrFilename().empty())
	{
		LOG("No OpenDRIVE file specified, continue without");
	}
	else
	{
		std::vector<std::string> file_name_candidates;
		// absolute path or relative to current directory
		file_name_candidates.push_back(getOdrFilename());
		// relative path to scenario directory
		file_name_candidates.push_back(CombineDirectoryPathAndFilepath(DirNameOf(scenarioReader->getScenarioFilename()), getOdrFilename()));
		// Remove all directories from path and look in current directory
		file_name_candidates.push_back(FileNameOf(getOdrFilename()));
		// Finally check registered paths
		for (size_t i = 0; i < SE_Env::Inst().GetPaths().size(); i++)
		{
			file_name_candidates.push_back(CombineDirectoryPathAndFilepath(SE_Env::Inst().GetPaths()[i], FileNameOf(getOdrFilename())));
		}
		size_t i;
		bool located = false;
		for (i = 0; i < file_name_candidates.size(); i++)
		{
			if (FileExists(file_name_candidates[i].c_str()))
			{
				located = true;
				if (roadmanager::Position::LoadOpenDrive(file_name_candidates[i].c_str()) == true)
				{
					break;
				}
				else
				{
					LOG("Failed to load OpenDRIVE file: %s", file_name_candidates[i].c_str());
					if (i < file_name_candidates.size() - 1)
					{
						LOG("  -> trying: %s", file_name_candidates[i + 1].c_str());
					}
				}
			}
		}

		if (i == file_name_candidates.size())
		{
			throw std::invalid_argument(std::string("Failed to ") + (located ? "load" : "find") + \
				" OpenDRIVE file " + std::string(getOdrFilename().c_str()));
		}
	}

	odrManager = roadmanager::Position::GetOpenDrive();

	scenarioReader->parseCatalogs();
	scenarioReader->parseEntities();

	scenarioReader->parseInit(init);
	scenarioReader->parseStoryBoard(storyBoard);
	storyBoard.entities_ = &entities;

	// Finally, now when all entities have been loaded, initialize the controllers
	if (!disable_controllers_)
	{
		// Initialize controllers
		for (size_t i = 0; i < scenarioReader->controller_.size(); i++)
		{
			scenarioReader->controller_[i]->Init();
			if (scenarioReader->controller_[i]->GetType() == Controller::Type::CONTROLLER_TYPE_REL2ABS)
			{
				((ControllerRel2Abs*)(scenarioReader->controller_[i]))->SetScenarioEngine(this);
			}
		}

		// find out maximum headstart time for ghosts
		for (size_t i = 0; i < entities.object_.size(); i++)
		{
			Object* obj = entities.object_[i];

			if (obj->GetAssignedControllerType() == Controller::Type::CONTROLLER_TYPE_FOLLOW_GHOST ||
				(obj->GetAssignedControllerType() == Controller::Type::CONTROLLER_TYPE_EXTERNAL &&
					((ControllerExternal*)(obj->controller_))->UseGhost()))
			{
				SetupGhost(obj);

				if (obj->ghost_)
				{
					if (obj->ghost_->GetHeadstartTime() > GetHeadstartTime())
					{
						SetHeadstartTime(obj->ghost_->GetHeadstartTime());
						SetSimulationTime(-obj->ghost_->GetHeadstartTime());
					}
				}
			}
		}
	}
}

int ScenarioEngine::defaultController(Object* obj, double dt)
{
	int retval = 0;
	double steplen = obj->speed_ * dt;

	// Add or subtract stepsize according to curvature and offset, in order to keep constant speed
	double curvature = obj->pos_.GetCurvature();
	double offset = obj->pos_.GetT();
	if (abs(curvature) > SMALL_NUMBER)
	{
		// Approximate delta length by sampling curvature in current position
		steplen += steplen * curvature * offset;
	}

	if (!obj->CheckDirtyBits(Object::DirtyBit::LONGITUDINAL)) // No action has updated longitudinal dimension
	{
		if (obj->GetControllerMode() == Controller::Mode::MODE_ADDITIVE ||
			!obj->IsControllerActiveOnDomains(ControlDomains::DOMAIN_LONG))
		{
			if (obj->pos_.GetRoute())
			{
				retval = static_cast<int>(obj->pos_.MoveRouteDS(steplen));
			}
			else
			{
				// Adjustment movement to heading and road direction
				if (GetAbsAngleDifference(obj->pos_.GetH(), obj->pos_.GetDrivingDirection()) > M_PI_2)
				{
					// If pointing in other direction
					steplen *= -1;
				}
				retval = static_cast<int>(obj->pos_.MoveAlongS(steplen, 0.0, obj->GetJunctionSelectorAngle()));

				if (obj->GetJunctionSelectorStrategy() == roadmanager::Junction::JunctionStrategyType::RANDOM &&
					obj->pos_.IsInJunction() && obj->GetJunctionSelectorAngle() >= 0)
				{
					// Set junction selector angle as undefined during junction
					obj->SetJunctionSelectorAngle(std::nan(""));
				}
				else if (obj->GetJunctionSelectorStrategy() == roadmanager::Junction::JunctionStrategyType::RANDOM &&
					!obj->pos_.IsInJunction() && std::isnan(obj->GetJunctionSelectorAngle()))
				{
					// Set new random junction selector after coming out of junction
					obj->SetJunctionSelectorAngleRandom();
				}
			}

			if (obj->pos_.GetStatusBitMask() & static_cast<int>(roadmanager::Position::PositionStatusMode::POS_STATUS_END_OF_ROAD) ||
				obj->pos_.GetStatusBitMask() & static_cast<int>(roadmanager::Position::PositionStatusMode::POS_STATUS_END_OF_ROUTE))
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

	if (retval == 0)
	{
		return 0;
	}
	else
	{
		return -1;
	}
}

void ScenarioEngine::prepareOSIGroundTruth(double dt)
{

	for (size_t i = 0; i < entities.object_.size(); i++)
	{
		Object *obj = entities.object_[i];
		ObjectState o;

		// Fetch external states from gateway
		if (scenarioGateway.getObjectStateById(entities.object_[i]->id_, o) != 0)
		{
			LOG("Gateway did not provide state for external car %d", entities.object_[i]->id_);
		}
		else
		{
			obj->pos_ = o.state_.pos;
			obj->speed_ = o.state_.info.speed;
			obj->wheel_angle_ = o.state_.info.wheel_angle;
			obj->wheel_rot_ = o.state_.info.wheel_rot;
		}
		// Calculate resulting updated velocity, acceleration and heading rate (rad/s) NOTE: in global coordinate sys
		double dx = obj->pos_.GetX() - obj->state_old.pos_x;
		double dy = obj->pos_.GetY() - obj->state_old.pos_y;

		if (dt > SMALL_NUMBER)
		{
			if (!obj->CheckDirtyBits(Object::DirtyBit::VELOCITY))
			{
				// If not already reported, calculate linear velocity
				obj->SetVel(dx / dt, dy / dt, 0.0);
			}

			if (!obj->CheckDirtyBits(Object::DirtyBit::ACCELERATION))
			{
				// If not already reported, calculate linear acceleration
				obj->SetAcc((obj->pos_.GetVelX() - obj->state_old.vel_x) / dt, (obj->pos_.GetVelY() - obj->state_old.vel_y) / dt, 0.0);
			}

			double heading_rate_new = GetAngleDifference(obj->pos_.GetH(), obj->state_old.h) / dt;
			if (!obj->CheckDirtyBits(Object::DirtyBit::ANGULAR_RATE))
			{
				// If not already reported, calculate angular velocity/rate
				obj->SetAngularVel(heading_rate_new, 0.0, 0.0);
			}

			if (!obj->CheckDirtyBits(Object::DirtyBit::ANGULAR_ACC))
			{
				// If not already reported, calculate angular acceleration
				obj->SetAngularAcc(GetAngleDifference(heading_rate_new, obj->state_old.h_rate) / dt, 0.0, 0.0);
			}

			// Update wheel rotations of internal scenario objects
			if (!obj->CheckDirtyBits(Object::DirtyBit::WHEEL_ANGLE))
			{
				// An improvised calculation of a steering angle based on yaw rate and enitity speed
				obj->wheel_angle_ = SIGN(obj->GetSpeed()) * M_PI * heading_rate_new / MAX(fabs(obj->GetSpeed()), SMALL_NUMBER);
			}
			if (!obj->CheckDirtyBits(Object::DirtyBit::WHEEL_ROTATION))
			{
				obj->wheel_rot_ = fmod(obj->wheel_rot_ + obj->speed_ * dt / WHEEL_RADIUS, 2 * M_PI);
			}

		}
		else
		{
			// calculate approximated velocity vector based on current heading
			if (!obj->CheckDirtyBits(Object::DirtyBit::VELOCITY))
			{
				// If not already reported, calculate approximated velocity vector based on current heading
				obj->SetVel(obj->speed_ * cos(obj->pos_.GetH()), obj->speed_ * sin(obj->pos_.GetH()), 0.0);
			}
		}

		// Report updated pos values to the gateway
		scenarioGateway.reportObject(obj->id_, obj->name_, static_cast<int>(obj->type_), obj->category_, obj->model_id_,
			obj->GetActivatedControllerType(), obj->boundingbox_, simulationTime_, obj->speed_, obj->wheel_angle_, obj->wheel_rot_, &obj->pos_);

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

		if (obj->trail_.GetNumberOfVertices() == 0 || simulationTime_ - obj->trail_.GetVertex(-1)->time > TRAJECTORY_SAMPLE_TIME)
		{
			// Only add trail vertex when speed is not stable at 0
			if (obj->trail_.GetNumberOfVertices() == 0 || fabs(obj->trail_.GetVertex(-1)->speed) > SMALL_NUMBER || fabs(obj->GetSpeed()) > SMALL_NUMBER)
			{
				obj->trail_.AddVertex({ 0.0, obj->pos_.GetX(), obj->pos_.GetY(), obj->pos_.GetZ(), obj->pos_.GetH(), simulationTime_, obj->GetSpeed(), 0.0, false });
			}
		}

		// Clear dirty/update bits for any reported velocity and acceleration values
		obj->ClearDirtyBits(
			Object::DirtyBit::VELOCITY |
			Object::DirtyBit::ANGULAR_RATE |
			Object::DirtyBit::ACCELERATION |
			Object::DirtyBit::ANGULAR_ACC
		);
	}
}

void ScenarioEngine::ReplaceObjectInTrigger(Trigger* trigger, Object* obj1, Object* obj2, double timeOffset)
{
	if (trigger == 0)
	{
		return;
	}
	for (size_t i = 0; i < trigger->conditionGroup_.size(); i++)
	{
		for (size_t j = 0; j < trigger->conditionGroup_[i]->condition_.size(); j++)
		{
			OSCCondition* cond = trigger->conditionGroup_[i]->condition_[j];
			if (cond->base_type_ == OSCCondition::ConditionType::BY_ENTITY)
			{
				TrigByEntity* trig = (TrigByEntity*)cond;
				for (size_t k = 0; k < trig->triggering_entities_.entity_.size(); k++)
				{
					if (trig->triggering_entities_.entity_[k].object_ == obj1)
					{
						trig->triggering_entities_.entity_[k].object_ = obj2;
					}
				}
			}
			else if (cond->base_type_ == OSCCondition::ConditionType::BY_VALUE)
			{
				TrigByValue* trig = (TrigByValue*)cond;
				if (trig->type_ == TrigByValue::Type::SIMULATION_TIME)
				{
					((TrigBySimulationTime*)(trig))->value_ += timeOffset;
				}
			}
		}
	}
}

void ScenarioEngine::SetupGhost(Object* object)
{
	// FollowGhostController special treatment:
	// Create a new (ghost) vehicle and copy all actions from base object

	Vehicle* ghost = new Vehicle(*(Vehicle*)object);
	object->SetGhost(ghost);
	ghost->name_ += "_ghost";
	ghost->ghost_ = 0;
	ghost->controller_ = 0;
	ghost->isGhost_ = true;
	ghost->SetHeadstartTime(object->headstart_time_);
	entities.addObject(ghost);
	object->SetHeadstartTime(0);

	int numberOfInitActions = (int)init.private_action_.size();
	for (int i = 0; i < numberOfInitActions; i++)
	{
		OSCPrivateAction* action = init.private_action_[i];
		if (action->object_ == object)
		{
			// Copy all actions except ActivateController
			if (action->type_ != OSCPrivateAction::ACTIVATE_CONTROLLER)
			{
				OSCPrivateAction* newAction = action->Copy();
				newAction->object_ = ghost;
				init.private_action_.push_back(newAction);
			}
		}
	}

	for (size_t i = 0; i < storyBoard.story_.size(); i++)
	{
		Story* story = storyBoard.story_[i];

		for (size_t j = 0; j < story->act_.size(); j++)
		{
			Act* act = story->act_[j];
			ReplaceObjectInTrigger(act->start_trigger_, object, ghost, -ghost->GetHeadstartTime());
			for (size_t k = 0; k < act->maneuverGroup_.size(); k++)
			{
				ManeuverGroup* mg = act->maneuverGroup_[k];
				for (size_t l = 0; l < mg->actor_.size();l++)
				{
					if (mg->actor_[l]->object_ == object)
					{
						// Replace actor
						mg->actor_[l]->object_ = ghost;
					}
				}
				for (size_t l = 0; l < act->maneuverGroup_[k]->maneuver_.size(); l++)
				{
					OSCManeuver* maneuver = act->maneuverGroup_[k]->maneuver_[l];
					for (size_t m = 0; m < maneuver->event_.size(); m++)
					{
						Event* event = maneuver->event_[m];
						bool ghostIsActor = false;
						for (size_t n = 0; n < event->action_.size(); n++)
						{
							OSCAction* action = event->action_[n];
							if (action->base_type_ == OSCAction::BaseType::PRIVATE)
							{
								OSCPrivateAction* pa = (OSCPrivateAction*)action;
								if (pa->object_ == object)
								{
									// If at least one of the event actions is of relevant subset of action types
									// then move the action to the ghost object instance, and also make needed
									// changes to the event trigger
									if (pa->type_ == OSCPrivateAction::ActionType::LONG_SPEED ||
										pa->type_ == OSCPrivateAction::ActionType::LAT_LANE_CHANGE ||
										pa->type_ == OSCPrivateAction::ActionType::LAT_LANE_OFFSET ||
										pa->type_ == OSCPrivateAction::ActionType::SYNCHRONIZE ||
										pa->type_ == OSCPrivateAction::ActionType::FOLLOW_TRAJECTORY ||
										pa->type_ == OSCPrivateAction::ActionType::TELEPORT)
									{
										// Replace object
										pa->ReplaceObjectRefs(object, ghost);
										ghostIsActor = true;
									}
								}
							}
						}
						if (ghostIsActor)
						{
							ReplaceObjectInTrigger(event->start_trigger_, object, ghost, -ghost->GetHeadstartTime());
						}
					}
				}
			}
		}
	}
}
