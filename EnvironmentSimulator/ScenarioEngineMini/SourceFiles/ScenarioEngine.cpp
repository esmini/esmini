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
	scenarioReader.parseEntities(entities, &catalogs);
	scenarioReader.parseInit(init, &entities, &catalogs);
	scenarioReader.parseStory(story, &entities, &catalogs);

	this->startTime = startTime;

	// Print loaded data
	entities.Print();
//	init.Print();
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

	// Step inital actions - might be extened in time (more than one step)
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
						if (story[i]->act_[j]->start_condition_group_[k]->condition_[l]->Evaluate(story[i], simulationTime))
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
						if (story[i]->act_[j]->end_condition_group_[k]->condition_[l]->Evaluate(story[i], simulationTime))
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
						if (story[i]->act_[j]->cancel_condition_group_[k]->condition_[l]->Evaluate(story[i], simulationTime))
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
						// Events - may only execute, at most, one at a time
						int active_event_id = -1;
						int waiting_event_id = -1;
						for (size_t m = 0; m < story[i]->act_[j]->sequence_[k]->maneuver_[l]->event_.size(); m++)
						{
							Event *event = story[i]->act_[j]->sequence_[k]->maneuver_[l]->event_[m];

							if (event->state_ == Event::State::ACTIVE)
							{
								if (active_event_id != -1)
								{
									LOG("Error: only one event, within a maneuver, may be active at a time");
								}
								active_event_id = (int)m;
							}
							else if (event->state_ == Event::State::WAITING)
							{
								waiting_event_id = (int)m;
							}
						}
						if (active_event_id == -1)
						{
							if (waiting_event_id != -1)
							{
								Event *event = story[i]->act_[j]->sequence_[k]->maneuver_[l]->event_[waiting_event_id];
								// OK to trig waiting event
								event->Activate();
								LOG("Activated waiting event %s", event->name_.c_str());
							}
						}


						for (size_t m = 0; m < story[i]->act_[j]->sequence_[k]->maneuver_[l]->event_.size(); m++)
						{
							Event *event = story[i]->act_[j]->sequence_[k]->maneuver_[l]->event_[m];
							
							if (event->state_ == Event::State::NOT_TRIGGED)
							{
								// Check event conditions
								for (size_t n = 0; n < event->start_condition_group_.size(); n++)
								{
									for (size_t o = 0; o < event->start_condition_group_[n]->condition_.size(); o++)
									{
										if (event->start_condition_group_[n]->condition_[o]->Evaluate(story[i], simulationTime))
										{
											// Check priority
											if (event->priority_ == Event::Priority::OVERWRITE)
											{
												// Deactivate any currently active event
												if (active_event_id >= 0)
												{
													story[i]->act_[j]->sequence_[k]->maneuver_[l]->event_[active_event_id]->Deactivate();
													LOG("Event %s cancelled", story[i]->act_[j]->sequence_[k]->maneuver_[l]->event_[active_event_id]->name_.c_str());
												}

												// Activate trigged event
												event->Activate();
												LOG("Event %s activated", event->name_.c_str());
											}
											else if (event->priority_ == Event::Priority::FOLLOWING)
											{
												// Tag event as waiting
												if (active_event_id >= 0)
												{
													event->state_ = Event::State::WAITING;
													LOG("Event %s is running, trigged event %s is waiting",
														story[i]->act_[j]->sequence_[k]->maneuver_[l]->event_[active_event_id]->name_.c_str(),
														event->name_.c_str());
												}
												else
												{
													event->Activate();
													LOG("No active event, event %s activated immediatelly", event->name_.c_str());
												}
											}
											else if(event->priority_ == Event::Priority::SKIP)
											{
												LOG("Event %s is running, skipping trigged %s", story[i]->act_[j]->sequence_[k]->maneuver_[l]->event_[m]->name_.c_str(), event->name_.c_str());
											}
											else
											{
												LOG("Unknown event priority: %d", event->priority_);
											}
										}
									}
								}
							}

							// Update (step) all active actions, for all objects connected to the action
							if (event->state_ == Event::State::ACTIVE)
							{
								bool active = false;
								for (size_t n = 0; n < event->action_.size(); n++)
								{
									if (event->action_[n]->state_ == OSCAction::State::ACTIVE)
									{
										event->action_[n]->Step(deltaSimTime);
										active = active || (event->action_[n]->state_ == OSCAction::State::ACTIVE);
									}
								}
								if (!active)
								{
									// Actions done -> Set event done
									event->Deactivate();
									LOG("Event %s done", event->name_.c_str());
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

void ScenarioEngine::stepObjects(double dt)
{
	for (size_t i = 0; i < entities.object_.size(); i++)
	{
		Object *obj = entities.object_[i];

		if (obj->pos_.GetRoute())
		{
			obj->pos_.MoveRouteDS(obj->speed_ * dt);
		}
		else
		{
			obj->pos_.MoveAlongS(obj->speed_ * dt);
		}
	}
}




