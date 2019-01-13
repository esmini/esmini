#include "ScenarioEngine.hpp"
#include "CommonMini.hpp"

using namespace scenarioengine;

ScenarioEngine::ScenarioEngine(std::string oscFilename, double startTime, ExternalControlMode ext_control)
{
	simulationTime = 0;
	req_ext_control_ = ext_control;
	InitScenario(oscFilename, startTime, ext_control);
}

void ScenarioEngine::InitScenario(std::string oscFilename, double startTime, ExternalControlMode ext_control)
{
	// Load and parse data
	LOG("Init %s", oscFilename.c_str());
	if (scenarioReader.loadOSCFile(oscFilename.c_str(), ext_control) != 0)
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

	if (req_ext_control_ > 0 && entities.object_.size() > 0)
	{
		LOG("Override external control flag from OSC file, new value: %s", req_ext_control_ == ExternalControlMode::EXT_CONTROL_OFF ? "Off" : "On");
		entities.object_[0]->extern_control_ = req_ext_control_ == ExternalControlMode::EXT_CONTROL_OFF ? false : true;
	}

	LOG("Requested external control: %d - %s, actual: %s", ext_control, scenarioReader.ExtControlMode2Str(ext_control).c_str(), GetExtControl()?"on":"off");


	this->startTime = startTime;

	// Print loaded data
	entities.Print();

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
	simulationTime += deltaSimTime;

	if (entities.object_.size() == 0)
	{
		return;
	}	

	// Fetch external states from gateway, except the initial run where scenario engine sets all positions
	if (!initial)
	{
		for (size_t i = 0; i < entities.object_.size(); i++)
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
		if (init.private_action_[i]->IsActive())
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
			// Update deactivated elements' state to inactive - This could probably be done in some other way...
			for (size_t k = 0; k < story[i]->act_[j]->sequence_.size(); k++)
			{
				for (size_t l = 0; l < story[i]->act_[j]->sequence_[k]->maneuver_.size(); l++)
				{
					OSCManeuver *maneuver = story[i]->act_[j]->sequence_[k]->maneuver_[l];

					for (size_t m = 0; m < maneuver->event_.size(); m++)
					{
						for (size_t n = 0; n < maneuver->event_[m]->action_.size(); n++)
						{
							if (maneuver->event_[m]->action_[n]->state_ == OSCAction::State::DEACTIVATED)
							{
								maneuver->event_[m]->action_[n]->state_ = OSCAction::State::INACTIVE;
							}
						}
						if (maneuver->event_[m]->state_ == Event::State::DEACTIVATED)
						{
							maneuver->event_[m]->state_ = Event::State::INACTIVE;
						}
					}
				}
			}
			if (story[i]->act_[j]->state_ == Act::State::DEACTIVATED)
			{
				story[i]->act_[j]->state_ = Act::State::INACTIVE;
			}

			// Check Act conditions
			if (!story[i]->act_[j]->IsActive())
			{
				// Check start conditions
				for (size_t k = 0; k < story[i]->act_[j]->start_condition_group_.size(); k++)
				{
					for (size_t l = 0; l < story[i]->act_[j]->start_condition_group_[k]->condition_.size(); l++)
					{
						if (story[i]->act_[j]->start_condition_group_[k]->condition_[l]->Evaluate(story[i], simulationTime))
						{
							story[i]->act_[j]->Trig();
						}
					}
				}
			}
			else
			{
				// If activated last step, make transition to activated
				if (story[i]->act_[j]->state_ == Act::State::ACTIVATED)
				{
					story[i]->act_[j]->state_ = Act::State::ACTIVE;
				}
			}

			if (story[i]->act_[j]->IsActive())
			{
				// Check end conditions
				for (size_t k = 0; k < story[i]->act_[j]->end_condition_group_.size(); k++)
				{
					for (size_t l = 0; l < story[i]->act_[j]->end_condition_group_[k]->condition_.size(); l++)
					{
						if (story[i]->act_[j]->end_condition_group_[k]->condition_[l]->Evaluate(story[i], simulationTime))
						{
							story[i]->act_[j]->Stop();
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
							story[i]->act_[j]->Stop();
						}
					}
				}
			}

			// Maneuvers
			if (story[i]->act_[j]->IsActive())
			{
				for (size_t k = 0; k < story[i]->act_[j]->sequence_.size(); k++)
				{
					for (size_t l = 0; l < story[i]->act_[j]->sequence_[k]->maneuver_.size(); l++)
					{
						OSCManeuver *maneuver = story[i]->act_[j]->sequence_[k]->maneuver_[l];

						// Events - may only execute one at a time
						for (size_t m = 0; m < maneuver->event_.size(); m++)
						{
							Event *event = story[i]->act_[j]->sequence_[k]->maneuver_[l]->event_[m];

							if (event->IsActive())
							{
								// If just activated, make transition to active
								if (event->state_ == Act::State::ACTIVATED)
								{
									event->state_ = Event::State::ACTIVE;
								}
							}
							else if(event->state_ == Act::State::DEACTIVATED)
							{
								// If just deactivated, make transition to inactive
								event->state_ = Event::State::INACTIVE;
							}
						}
						if(maneuver->GetActiveEventIdx() == -1 && maneuver->GetWaitingEventIdx() >= 0)
						{
							// When no active event, it's OK to trig waiting event
							maneuver->event_[maneuver->GetWaitingEventIdx()]->Trig();
						}


						for (size_t m = 0; m < maneuver->event_.size(); m++)
						{
							Event *event = maneuver->event_[m];
							
							if (event->Triggable())
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
												if (maneuver->GetActiveEventIdx() >= 0)
												{
													LOG("Event %s cancelled", maneuver->event_[maneuver->GetActiveEventIdx()]->name_.c_str());
													maneuver->event_[maneuver->GetActiveEventIdx()]->Stop();
												}

												// Activate trigged event
												event->Trig();
											}
											else if (event->priority_ == Event::Priority::FOLLOWING)
											{
												// If already an active event, this event will wait
												if (maneuver->GetActiveEventIdx() >= 0)
												{
													event->state_ = Event::State::WAITING;
													LOG("Event %s is running, trigged event %s is waiting",
														maneuver->event_[maneuver->GetActiveEventIdx()]->name_.c_str(), event->name_.c_str());
												}
												else
												{
													event->Trig();
												}
											}
											else if(event->priority_ == Event::Priority::SKIP)
											{
												if (maneuver->GetActiveEventIdx() >= 0)
												{
													LOG("Event %s is running, skipping trigged %s", 
														maneuver->event_[maneuver->GetActiveEventIdx()]->name_.c_str(), event->name_.c_str());
												}
												else
												{
													event->Trig();
												}
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
							if (event->IsActive())
							{
								bool active = false;

								for (size_t n = 0; n < event->action_.size(); n++)
								{
									if (event->action_[n]->IsActive())
									{
										event->action_[n]->Step(deltaSimTime);
										active = active || (event->action_[n]->IsActive());
									}
								}
								if (!active)
								{
									// Actions done -> Set event done
									event->Stop();
								}
							}
						}
					}
				}
			}
		}
	}

	// Report resulting states to the gateway
	for (size_t i = 0; i < entities.object_.size(); i++)
	{
		Object *obj = entities.object_[i];
		
		if (initial)
		{
			// Report all scenario objects the initial run, to establish initial positions and speed = 0
			scenarioGateway.reportObject(ObjectState(obj->id_, obj->name_, obj->model_id_, obj->extern_control_, simulationTime, &obj->pos_, 0.0));
		}
		else if (!obj->extern_control_)
		{
			// Then report all except externally controlled objects
			scenarioGateway.reportObject(ObjectState(obj->id_, obj->name_, obj->model_id_, obj->extern_control_, simulationTime, &obj->pos_, obj->speed_));
		}
	}

	stepObjects(deltaSimTime);
}

void ScenarioEngine::printSimulationTime()
{
	LOG("simulationTime = %.2f", simulationTime);
}

ScenarioGateway *ScenarioEngine::getScenarioGateway()
{
	return &scenarioGateway;
}

bool ScenarioEngine::GetExtControl()
{
	if (entities.object_.size() > 0)
	{
		return entities.object_[0]->extern_control_;
	}

	LOG("No objects initialized yet - ask later");

	return false;  // Hmm, what is a good default value...?
}

void ScenarioEngine::stepObjects(double dt)
{
	for (size_t i = 0; i < entities.object_.size(); i++)
	{
		Object *obj = entities.object_[i];

		if (obj->extern_control_ == false)
		{
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
}

