#include "ScenarioEngine.hpp"
#include "CommonMini.hpp"


ScenarioEngine::ScenarioEngine(std::string oscFilename, double startTime)
{
	simulationTime = 0;
	InitScenario(oscFilename, startTime);
}

void ScenarioEngine::InitScenario(std::string oscFilename, double startTime)
{
	// Load and parse data
	LOG("Init %s", oscFilename.c_str());
	if (scenarioReader.loadOSCFile(oscFilename.c_str()) != 0)
	{
		throw std::invalid_argument(std::string("Failed to load OpenSCENARIO file ") + oscFilename);
	}

	// Init road manager
	scenarioReader.parseRoadNetwork(roadNetwork);
	if (!roadmanager::Position::LoadOpenDrive(getOdrFilename().c_str()))
	{
		throw std::invalid_argument(std::string("Failed to load OpenDRIVE file ") + getOdrFilename().c_str());
	}
	odrManager = roadmanager::Position::GetOpenDrive();

	scenarioReader.parseParameterDeclaration();
	scenarioReader.parseCatalogs(catalogs);
	scenarioReader.parseEntities(entities);
	scenarioReader.parseInit(init, &entities);
	scenarioReader.parseStory(story, &entities);

	this->startTime = startTime;

	// Print loaded data
	entities.Print();
	init.Print();
	for (size_t i = 0; i < story.size(); i++)
	{
		story[i]->Print();
	}
}

ScenarioEngine::~ScenarioEngine()
{
	LOG("Closing");
}

void ScenarioEngine::step(double deltaSimTime, bool initial)	
{
	if (entities.object_.size() == 0)
	{
		return;
	}	

	// Fetch external states from gateway, except the initial run where scenario engine sets all positions
	if (!initial)
	{
		for (int i = 0; i < entities.object_.size(); i++)
		{
			if (entities.object_[i]->extern_control_)
			{
				ObjectState o;

				if (scenarioGateway.getObjectStateById(entities.object_[i]->id_, o) != 0)
				{
					LOG("Gateway did not provide state for external car %d", entities.object_[i]->id_);
				}
				else
				{
					entities.object_[i]->pos_ = o.state_.pos;
					entities.object_[i]->speed_ = o.state_.speed;
				}
			}
		}
	}


	// Kick off initial actions
	if (initial)
	{
		// kick off init actions
		for (size_t i = 0; i < init.private_action_.size(); i++)
		{
			init.private_action_[i]->Trig();
		}
	}

	// Step inital actions
	for (size_t i = 0; i < init.private_action_.size(); i++)
	{
		if (init.private_action_[i]->state_ == OSCAction::State::ACTIVE)
		{
			//LOG("Stepping action of type %d", init.private_action_[i]->action_[j]->type_)
			init.private_action_[i]->Step(deltaSimTime);
		}
	}

	// Story 
	for (size_t i=0; i< story.size(); i++)
	{
		for (size_t j=0; j < story[i]->act_.size(); j++)
		{
			// Act conditions
			if (!story[i]->act_[j]->active_)
			{
				// Check start conditions
				for (size_t k = 0; k < story[i]->act_[j]->start_condition_group_.size(); k++)
				{
					for (size_t l = 0; l < story[i]->act_[j]->start_condition_group_[k]->condition_.size(); l++)
					{
						if (story[i]->act_[j]->start_condition_group_[k]->condition_[l]->Evaluate(story[i]->act_[j], simulationTime))
						{
							story[i]->act_[j]->active_ = true;
						}
					}
				}
			}

			if (story[i]->act_[j]->active_)
			{
				// Check end conditions
				for (size_t k = 0; k < story[i]->act_[j]->end_condition_group_.size(); k++)
				{
					for (size_t l = 0; l < story[i]->act_[j]->end_condition_group_[k]->condition_.size(); l++)
					{
						if (story[i]->act_[j]->end_condition_group_[k]->condition_[l]->Evaluate(story[i]->act_[j], simulationTime))
						{
							story[i]->act_[j]->active_ = false;
						}
					}
				}

				// Check cancel conditions
				for (size_t k = 0; k < story[i]->act_[j]->cancel_condition_group_.size(); k++)
				{
					for (size_t l = 0; l < story[i]->act_[j]->cancel_condition_group_[k]->condition_.size(); l++)
					{
						if (story[i]->act_[j]->cancel_condition_group_[k]->condition_[l]->Evaluate(story[i]->act_[j], simulationTime))
						{
							story[i]->act_[j]->active_ = false;
						}
					}
				}
			}

			// Maneuvers
			if (story[i]->act_[j]->active_)
			{
				for (size_t k = 0; k < story[i]->act_[j]->sequence_.size(); k++)
				{
					for (size_t l = 0; l < story[i]->act_[j]->sequence_[k]->maneuver_.size(); l++)
					{
						// Events
						for (size_t m = 0; m < story[i]->act_[j]->sequence_[k]->maneuver_[l]->event_.size(); m++)
						{
							Event *event = story[i]->act_[j]->sequence_[k]->maneuver_[l]->event_[m];

							if (!event->active_)
							{
								// Check event conditions
								for (size_t n = 0; n < event->start_condition_group_.size(); n++)
								{
									for (size_t o = 0; o < event->start_condition_group_[n]->condition_.size(); o++)
									{
										if (event->start_condition_group_[n]->condition_[o]->Evaluate(story[i]->act_[j], simulationTime))
										{
											event->active_ = true;
											LOG("event active %s", event->name_.c_str());

											// Start all actions in group
											for (size_t p = 0; p < event->action_.size(); p++)
											{
												event->action_[p]->Trig();
											}
										}
									}
								}
							}

							// Update (step) all active actions, for all objects connected to the action
							if (event->active_)
							{
								for (size_t n = 0; n < event->action_.size(); n++)
								{
									if (event->action_[n]->state_ == OSCAction::State::ACTIVE)
									{
										for (size_t o = 0; o < story[i]->act_[j]->sequence_[k]->actor_.size(); o++)
										{
											event->action_[n]->Step(deltaSimTime);
										}
									}
								}
							}
						}
					}
				}
			}
		}
	}

	// Report resulting states to the gateway
	for (int i = 0; i < entities.object_.size(); i++)
	{
		Object *obj = entities.object_[i];
		
		if (initial)
		{
			// Report all scenario objects the initial run, to establish initial positions and speed = 0
			scenarioGateway.reportObject(ObjectState(obj->id_, obj->name_, simulationTime, &obj->pos_, 0.0));
		}
		else if (!obj->extern_control_)
		{
			// Then report all except externally controlled objects
			scenarioGateway.reportObject(ObjectState(obj->id_, obj->name_, simulationTime, &obj->pos_, obj->speed_));
		}
	}

	stepObjects(deltaSimTime);
}

void ScenarioEngine::setTimeStep(double timeStep)
{
	this->timeStep = timeStep;
}

void ScenarioEngine::setSimulationTime(double simulationTime)
{
	this->simulationTime = simulationTime;
}

void ScenarioEngine::printSimulationTime()
{
	LOG("simulationTime = %.2f", simulationTime);
}

ScenarioGateway *ScenarioEngine::getScenarioGateway()
{
	return &scenarioGateway;
}

void ScenarioEngine::initRoutes()
{
	for (size_t i=0; i<catalogs.RouteCatalog.Route.size(); i++)
	{
		if (!catalogs.RouteCatalog.Route[i].Waypoint.empty())
		{
			roadmanager::Route rm_route;
			rm_route.setName(catalogs.RouteCatalog.Route[i].name);
			for (size_t j = 0; j < catalogs.RouteCatalog.Route[i].Waypoint.size(); j++)
			{
				roadmanager::Position * position = new roadmanager::Position();

				// Lane position
				int roadId = catalogs.RouteCatalog.Route[i].Waypoint[j].Position->GetTrackId();
				int lane_id = catalogs.RouteCatalog.Route[i].Waypoint[j].Position->GetLaneId();
				double s = catalogs.RouteCatalog.Route[i].Waypoint[j].Position->GetS();
				double offset = catalogs.RouteCatalog.Route[i].Waypoint[j].Position->GetOffset();

				position->SetLanePos(roadId, lane_id, s, offset);

				rm_route.AddWaypoint(position);
			}
//			cars.route.push_back(rm_route);
		}
	}
}

//void ScenarioEngine::initInit()
//{
//	LOG("initStoryboard started");
//
//	for (int i = 0; i < init.private_action_.size(); i++)
//	{
//		std::vector<Object*> actionEntities;
//		actionEntities.push_back(init.private_action_[i]->object_);
//	}
//
//	LOG("initStoryboard finished");
//}

void ScenarioEngine::stepObjects(double dt)
{
	for (size_t i = 0; i < entities.object_.size(); i++)
	{
		Object *obj = entities.object_[i];

		if (obj->follow_route_)
		{
//			obj->route.MoveDS(speed * dt);
//			obj->route.GetPosition(&position);
		}
		else
		{
			obj->pos_.MoveAlongS(obj->speed_ * dt);
		}
	}
}




