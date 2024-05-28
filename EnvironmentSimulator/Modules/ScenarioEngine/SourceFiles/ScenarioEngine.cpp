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
#include "ControllerFollowRoute.hpp"
#include "OSCParameterDistribution.hpp"

#define WHEEL_RADIUS          0.35
#define STAND_STILL_THRESHOLD 1e-3  // meter per second

using namespace scenarioengine;

static CallBack paramDeclCallback = {0, 0};

namespace scenarioengine
{
    void RegisterParameterDeclarationCallback(ParamDeclCallbackFunc func, void* data)
    {
        paramDeclCallback.func = func;
        paramDeclCallback.data = data;
    }
}  // namespace scenarioengine

ScenarioEngine::ScenarioEngine(std::string oscFilename, bool disable_controllers)
{
    init_status_ = InitScenario(oscFilename, disable_controllers);
}

ScenarioEngine::ScenarioEngine(const pugi::xml_document& xml_doc, bool disable_controllers)
{
    init_status_ = InitScenario(xml_doc, disable_controllers);
}

void ScenarioEngine::InitScenarioCommon(bool disable_controllers)
{
    init_status_         = 0;
    disable_controllers_ = disable_controllers;
    simulationTime_      = 0;
    trueTime_            = 0;
    frame_nr_            = 0;
    scenarioReader       = new ScenarioReader(&entities_, &catalogs, disable_controllers);
    injected_actions_    = nullptr;
    ghost_               = nullptr;
    SE_Env::Inst().SetGhostMode(GhostMode::NORMAL);
    SE_Env::Inst().SetGhostHeadstart(0.0);
}

int ScenarioEngine::InitScenario(std::string oscFilename, bool disable_controllers)
{
    InitScenarioCommon(disable_controllers);

    std::string oscFilename_no_path = FileNameOf(oscFilename);

    std::vector<std::string> file_name_candidates;

    // Filename as is - look in current directory or absolute path if provided
    file_name_candidates.push_back(oscFilename);

    // Also check registered paths
    for (size_t i = 0; i < SE_Env::Inst().GetPaths().size(); i++)
    {
        file_name_candidates.push_back(CombineDirectoryPathAndFilepath(SE_Env::Inst().GetPaths()[i], oscFilename_no_path));
    }
    size_t i;
    for (i = 0; i < file_name_candidates.size(); i++)
    {
        if (FileExists(file_name_candidates[i].c_str()))
        {
            if (scenarioReader->loadOSCFile(file_name_candidates[i].c_str()) != 0)
            {
                LOG(("Failed to load OpenSCENARIO file " + oscFilename_no_path).c_str());
                return -3;
            }
            else
            {
                break;
            }
        }
    }

    if (i == file_name_candidates.size())
    {
        LOG(("Couldn't locate OpenSCENARIO file " + oscFilename_no_path).c_str());
        return -1;
    }

    if (!scenarioReader->IsLoaded())
    {
        LOG(("Couldn't load OpenSCENARIO file " + oscFilename_no_path).c_str());
        return -2;
    }

    return parseScenario();
}

int ScenarioEngine::InitScenario(const pugi::xml_document& xml_doc, bool disable_controllers)
{
    InitScenarioCommon(disable_controllers);

    if (scenarioReader->loadOSCMem(xml_doc) != 0)
    {
        return -3;
    }

    return parseScenario();
}

ScenarioEngine::~ScenarioEngine()
{
    scenarioReader->UnloadControllers();
    delete scenarioReader;
    scenarioReader = 0;
    LOG("Closing");
}

void ScenarioEngine::UpdateGhostMode()
{
    if (SE_Env::Inst().GetGhostMode() == GhostMode::RESTART)
    {
        simulationTime_ -= SE_Env::Inst().GetGhostHeadstart();
        SE_Env::Inst().SetGhostMode(GhostMode::RESTARTING);
    }
    else if (SE_Env::Inst().GetGhostMode() == GhostMode::RESTARTING)
    {
        if (simulationTime_ > trueTime_ - SMALL_NUMBER)
        {
            SE_Env::Inst().SetGhostMode(GhostMode::NORMAL);
        }
    }
}

int ScenarioEngine::step(double deltaSimTime)
{
    UpdateGhostMode();

    if (frame_nr_ == 0)
    {
        storyBoard.Start(simulationTime_);

        // Set initial values for speed and acceleration derivation
        for (size_t i = 0; i < entities_.object_.size(); i++)
        {
            Object* obj = entities_.object_[i];

            obj->state_old.pos_x  = obj->pos_.GetX();
            obj->state_old.pos_y  = obj->pos_.GetY();
            obj->state_old.pos_z  = obj->pos_.GetZ();
            obj->state_old.vel_x  = obj->pos_.GetVelX();
            obj->state_old.vel_y  = obj->pos_.GetVelY();
            obj->state_old.vel_z  = obj->pos_.GetVelZ();
            obj->state_old.h      = obj->pos_.GetH();
            obj->state_old.h_rate = obj->pos_.GetHRate();
            obj->reset_           = true;
        }
    }
    else
    {
        // reset update bits and indicators of applied control
        for (size_t i = 0; i < entities_.object_.size(); i++)
        {
            Object* obj = entities_.object_[i];

            obj->ClearDirtyBits(Object::DirtyBit::LATERAL | Object::DirtyBit::LONGITUDINAL | Object::DirtyBit::SPEED | Object::DirtyBit::WHEEL_ANGLE |
                                Object::DirtyBit::WHEEL_ROTATION | Object::DirtyBit::ACCELERATION | Object::DirtyBit::CONTROLLER);
            obj->reset_ = false;

            // Fetch dirty bits from gateway, indicating what has been reported externally and needs to be protected
            ObjectState* o = scenarioGateway.getObjectStatePtrById(obj->id_);
            if (o == nullptr)
            {
                LOG("Gateway did not provide state for external car %d", obj->id_);
            }
            else
            {
                if (o->dirty_ & (Object::DirtyBit::LATERAL | Object::DirtyBit::LONGITUDINAL))
                {
                    obj->SetDirtyBits(o->dirty_ & (Object::DirtyBit::LATERAL | Object::DirtyBit::LONGITUDINAL));
                }
                if (o->dirty_ & Object::DirtyBit::SPEED)
                {
                    obj->SetDirtyBits(Object::DirtyBit::SPEED);
                }
                if (o->dirty_ & Object::DirtyBit::ACCELERATION)
                {
                    obj->SetDirtyBits(Object::DirtyBit::ACCELERATION);
                }
                if (o->dirty_ & Object::DirtyBit::WHEEL_ANGLE)
                {
                    obj->SetDirtyBits(Object::DirtyBit::WHEEL_ANGLE);
                }
                if (o->dirty_ & Object::DirtyBit::WHEEL_ROTATION)
                {
                    obj->SetDirtyBits(Object::DirtyBit::WHEEL_ROTATION);
                }
                if (o->dirty_ & Object::DirtyBit::ALIGN_MODE_H_SET)
                {
                    obj->pos_.SetMode(roadmanager::Position::PosModeType::SET,
                                      roadmanager::Position::PosMode::H_MASK & o->state_.pos.GetMode(roadmanager::Position::PosModeType::SET));
                }
                if (o->dirty_ & Object::DirtyBit::ALIGN_MODE_P_SET)
                {
                    obj->pos_.SetMode(roadmanager::Position::PosModeType::SET,
                                      roadmanager::Position::PosMode::P_MASK & o->state_.pos.GetMode(roadmanager::Position::PosModeType::SET));
                }
                if (o->dirty_ & Object::DirtyBit::ALIGN_MODE_R_SET)
                {
                    obj->pos_.SetMode(roadmanager::Position::PosModeType::SET,
                                      roadmanager::Position::PosMode::R_MASK & o->state_.pos.GetMode(roadmanager::Position::PosModeType::SET));
                }
                if (o->dirty_ & Object::DirtyBit::ALIGN_MODE_Z_SET)
                {
                    obj->pos_.SetMode(roadmanager::Position::PosModeType::SET,
                                      roadmanager::Position::PosMode::Z_MASK & o->state_.pos.GetMode(roadmanager::Position::PosModeType::SET));
                }
                if (o->dirty_ & Object::DirtyBit::ALIGN_MODE_H_UPDATE)
                {
                    obj->pos_.SetMode(roadmanager::Position::PosModeType::UPDATE,
                                      roadmanager::Position::PosMode::H_MASK & o->state_.pos.GetMode(roadmanager::Position::PosModeType::UPDATE));
                }
                if (o->dirty_ & Object::DirtyBit::ALIGN_MODE_P_UPDATE)
                {
                    obj->pos_.SetMode(roadmanager::Position::PosModeType::UPDATE,
                                      roadmanager::Position::PosMode::P_MASK & o->state_.pos.GetMode(roadmanager::Position::PosModeType::UPDATE));
                }
                if (o->dirty_ & Object::DirtyBit::ALIGN_MODE_R_UPDATE)
                {
                    obj->pos_.SetMode(roadmanager::Position::PosModeType::UPDATE,
                                      roadmanager::Position::PosMode::R_MASK & o->state_.pos.GetMode(roadmanager::Position::PosModeType::UPDATE));
                }
                if (o->dirty_ & Object::DirtyBit::ALIGN_MODE_Z_UPDATE)
                {
                    obj->pos_.SetMode(roadmanager::Position::PosModeType::UPDATE,
                                      roadmanager::Position::PosMode::Z_MASK & o->state_.pos.GetMode(roadmanager::Position::PosModeType::UPDATE));
                }
            }
        }
    }

    storyBoard.Step(simulationTime_, deltaSimTime);

    if (storyBoard.GetCurrentState() == StoryBoardElement::State::RUNNING)
    {
        // Check for collisions/overlap after first initialization
        if (SE_Env::Inst().GetCollisionDetection() && frame_nr_ == 0)
        {
            DetectCollisions();
        }
    }

    if (storyBoard.GetCurrentState() != StoryBoardElement::State::RUNNING)
    {
        return 1;
    }

    // Step any externally injected actions
    if (injected_actions_ && injected_actions_->size() > 0)
    {
        for (OSCAction* action : *injected_actions_)
        {
            if (action->GetCurrentState() != StoryBoardElement::State::RUNNING)
            {
                action->Start(simulationTime_);
            }
            else
            {
                action->Step(simulationTime_, deltaSimTime);
            }
        }
    }

    // This timestep calculation is due to the Ghost vehicle
    // If both times are equal, it is a normal scenario, or no Ghost teleportation is ongoing -> Step as usual
    // Else if we can take a step, and still not reach the point of teleportation -> Step only simulationTime (That the Ghost runs on)
    // Else, the only thing left is that the next step will take us above the point of teleportation -> Step to that point instead and go on from
    // there

    simulationTime_ += deltaSimTime;
    if (simulationTime_ < 0.0 && simulationTime_ > -SMALL_NUMBER)
    {
        // Avoid -0.000
        simulationTime_ = 0.0;
    }

    if (simulationTime_ > trueTime_)
    {
        trueTime_ = simulationTime_;
    }

    for (size_t i = 0; i < entities_.object_.size(); i++)
    {
        Object* obj = entities_.object_[i];

        // Fetch states from gateway (if available), indicated by dirty bits
        ObjectState* o = scenarioGateway.getObjectStatePtrById(obj->id_);
        if (o != nullptr)
        {
            if (o->dirty_ & (Object::DirtyBit::LATERAL | Object::DirtyBit::LONGITUDINAL))
            {
                obj->pos_ = o->state_.pos;
            }
            if (o->dirty_ & Object::DirtyBit::SPEED)
            {
                obj->speed_ = o->state_.info.speed;
            }
            if (o->dirty_ & Object::DirtyBit::WHEEL_ANGLE)
            {
                obj->wheel_angle_ = o->state_.info.wheel_angle;
            }
            if (o->dirty_ & Object::DirtyBit::WHEEL_ROTATION)
            {
                obj->wheel_rot_ = o->state_.info.wheel_rot;
            }
            o->clearDirtyBits();
        }

        // Do not move objects when speed is zero,
        // and only ghosts allowed to execute during ghost restart
        if (!(obj->IsControllerModeOnDomains(ControlOperationMode::MODE_OVERRIDE, static_cast<unsigned int>(ControlDomains::DOMAIN_LAT_AND_LONG))) &&
            fabs(obj->speed_) > SMALL_NUMBER &&
            // Skip update for non ghost objects during ghost restart
            !(!obj->IsGhost() && SE_Env::Inst().GetGhostMode() == GhostMode::RESTARTING) && !obj->TowVehicle())  // update trailers later
        {
            defaultController(obj, deltaSimTime);
        }

        if (!obj->pos_.GetRoute())
        {
            if (obj->GetJunctionSelectorStrategy() == roadmanager::Junction::JunctionStrategyType::RANDOM && obj->pos_.IsInJunction() &&
                obj->GetJunctionSelectorAngle() >= 0)
            {
                // Set junction selector angle as undefined during junction
                obj->SetJunctionSelectorAngle(std::nan(""));
            }
            else if (obj->GetJunctionSelectorStrategy() == roadmanager::Junction::JunctionStrategyType::RANDOM && !obj->pos_.IsInJunction() &&
                     std::isnan(obj->GetJunctionSelectorAngle()))
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

        // Report updated state to the gateway
        if (scenarioGateway.isObjectReported(obj->id_))
        {
            if (obj->CheckDirtyBits(Object::DirtyBit::LONGITUDINAL | Object::DirtyBit::LATERAL))
            {
                scenarioGateway.updateObjectPos(obj->id_, simulationTime_, &obj->pos_);
            }

            if (obj->CheckDirtyBits(Object::DirtyBit::SPEED))
            {
                scenarioGateway.updateObjectSpeed(obj->id_, simulationTime_, obj->speed_);
            }

            if (obj->CheckDirtyBits(Object::DirtyBit::WHEEL_ANGLE))
            {
                scenarioGateway.updateObjectWheelAngle(obj->id_, simulationTime_, obj->wheel_angle_);
            }

            if (obj->CheckDirtyBits(Object::DirtyBit::WHEEL_ROTATION))
            {
                scenarioGateway.updateObjectWheelRotation(obj->id_, simulationTime_, obj->wheel_rot_);
            }

            if (obj->CheckDirtyBits(Object::DirtyBit::VISIBILITY))
            {
                scenarioGateway.updateObjectVisibilityMask(obj->id_, obj->visibilityMask_);
            }

            if (obj->CheckDirtyBits(Object::DirtyBit::CONTROLLER))
            {
                scenarioGateway.updateObjectControllerType(obj->id_, obj->GetControllerTypeActiveOnDomain(ControlDomains::DOMAIN_LONG));
            }

            // Friction is not considered
        }
        else
        {
            // Object not reported yet, do that
            scenarioGateway.reportObject(obj->id_,
                                         obj->name_,
                                         static_cast<int>(obj->type_),
                                         obj->category_,
                                         obj->role_,
                                         obj->model_id_,
                                         obj->model3d_,
                                         obj->GetControllerTypeActiveOnDomain(ControlDomains::DOMAIN_LONG),
                                         obj->boundingbox_,
                                         static_cast<int>(obj->scaleMode_),
                                         obj->visibilityMask_,
                                         simulationTime_,
                                         obj->speed_,
                                         obj->wheel_angle_,
                                         obj->wheel_rot_,
                                         obj->rear_axle_.positionZ,
                                         obj->front_axle_.positionX,
                                         obj->front_axle_.positionZ,
                                         &obj->pos_);
        }
    }

    for (size_t i = 0; i < scenarioReader->controller_.size(); i++)
    {
        if (scenarioReader->controller_[i]->Active())
        {
            if (SE_Env::Inst().GetGhostMode() != GhostMode::RESTARTING)
            {
                scenarioReader->controller_[i]->Step(deltaSimTime);
            }
        }
    }

    // Update any trailers now that tow vehicles have been updated by Default or custom controllers
    for (size_t i = 0; i < entities_.object_.size(); i++)
    {
        Object*  obj     = entities_.object_[i];
        Vehicle* trailer = static_cast<Vehicle*>(obj->TrailerVehicle());

        if (!obj->TowVehicle() && obj->TrailerVehicle())
        {
            // Found a front tow vehicle, update trailers
            Vehicle* tow_vehicle = static_cast<Vehicle*>(obj);
            while (trailer)
            {
                // Calculate new trailer position and orientation
                ObjectState* o = scenarioGateway.getObjectStatePtrById(tow_vehicle->id_);
                SE_Vector    v0(tow_vehicle->trailer_hitch_->dx_, 0.0);

                // Fetch updated state of tow vehicle from gateway
                roadmanager::Position* tow_pos = &o->state_.pos;
                v0                             = v0.Rotate(tow_pos->GetH()) + SE_Vector(tow_pos->GetX(), tow_pos->GetY());
                SE_Vector v1                   = SE_Vector(trailer->pos_.GetX(), trailer->pos_.GetY()) - v0;
                v1.SetLength(trailer->trailer_coupler_->dx_);
                scenarioGateway.updateObjectWorldPosXYH(trailer->GetId(),
                                                        getSimulationTime(),
                                                        v0.x() + v1.x(),
                                                        v0.y() + v1.y(),
                                                        GetAngleInInterval2PI(atan2(v1.y(), v1.x()) + M_PI));
                trailer->SetSpeed(tow_vehicle->GetSpeed());

                tow_vehicle = trailer;
                trailer     = static_cast<Vehicle*>(trailer->TrailerVehicle());
            }
        }
    }

    // Check some states
    for (size_t i = 0; i < entities_.object_.size(); i++)
    {
        Object* obj = entities_.object_[i];

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

    // Check for collisions
    if (SE_Env::Inst().GetCollisionDetection() && frame_nr_ > 0)
    {
        DetectCollisions();
    }

    frame_nr_++;

    return 0;
}

void ScenarioEngine::printSimulationTime()
{
    LOG("simulationTime = %.2f", simulationTime_);
}

ScenarioGateway* ScenarioEngine::getScenarioGateway()
{
    return &scenarioGateway;
}

int ScenarioEngine::parseScenario()
{
    SetSimulationTime(0);
    SetTrueTime(0);

    scenarioReader->LoadControllers();

    scenarioReader->SetGateway(&scenarioGateway);
    scenarioReader->SetScenarioEngine(this);

    scenarioReader->parseOSCHeader();
    if (scenarioReader->GetVersionMajor() < 1)
    {
        LOG_AND_QUIT("OpenSCENARIO v%d.%d not supported. Please migrate scenario to v1.0 or higher and try again.",
                     scenarioReader->GetVersionMajor(),
                     scenarioReader->GetVersionMinor());
    }
    LOG("Loading %s (v%d.%d)", scenarioReader->getScenarioFilename().c_str(), scenarioReader->GetVersionMajor(), scenarioReader->GetVersionMinor());

    scenarioReader->parseGlobalParameterDeclarations();
    scenarioReader->variables.Print("variables");  // All variables parsed at this point (not the case with parameters)

    // Now that parameter declaration has been parsed, call any registered callbacks before applying the parameters
    if (paramDeclCallback.func != nullptr)
    {
        paramDeclCallback.func(paramDeclCallback.data);
    }

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
            file_name_candidates.push_back(CombineDirectoryPathAndFilepath(SE_Env::Inst().GetPaths()[i], getOdrFilename()));
            file_name_candidates.push_back(CombineDirectoryPathAndFilepath(SE_Env::Inst().GetPaths()[i], FileNameOf(getOdrFilename())));
        }
        size_t i;
        bool   located = false;
        for (i = 0; i < file_name_candidates.size(); i++)
        {
            if (FileExists(file_name_candidates[i].c_str()))
            {
                located = true;
                if (roadmanager::Position::LoadOpenDrive(file_name_candidates[i].c_str()) == true)
                {
                    LOG("Loaded OpenDRIVE: %s", file_name_candidates[i].c_str());
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
            LOG((std::string("Failed to ") + (located ? "load" : "find") + " OpenDRIVE file " + getOdrFilename().c_str()).c_str());
            return -1;
        }
    }

    odrManager = roadmanager::Position::GetOpenDrive();

    scenarioReader->parseCatalogs();
    scenarioReader->parseEntities();

    scenarioReader->parseInit(storyBoard.init_);
    scenarioReader->parseStoryBoard(storyBoard);
    storyBoard.entities_ = &entities_;
#ifdef _USE_OSI
    storyBoard.SetOSIReporter(nullptr);
#endif  // _USE_OSI

    // Now when all entities have been loaded, initialize the controllers
    if (GetDisableControllersFlag() == false)
    {
        for (size_t i = 0; i < scenarioReader->controller_.size(); i++)
        {
            scenarioReader->controller_[i]->Init();
            scenarioReader->controller_[i]->SetScenarioEngine(this);
        }

        // find out maximum headstart time for ghosts
        for (size_t i = 0; i < entities_.object_.size(); i++)
        {
            Object* obj = entities_.object_[i];

            if (obj->IsAnyAssignedControllerOfType(Controller::Type::CONTROLLER_TYPE_FOLLOW_GHOST) ||
                (obj->IsAnyAssignedControllerOfType(Controller::Type::CONTROLLER_TYPE_EXTERNAL) &&
                 (static_cast<ControllerExternal*>(obj->GetAssignedControllerOftype(Controller::Type::CONTROLLER_TYPE_EXTERNAL))->UseGhost())))
            {
                SetupGhost(obj);

                if (obj->ghost_)
                {
                    LOG_ONCE("NOTE: Ghost feature activated. Consider headstart time offset (-%.2f s) when reading log.",
                             obj->ghost_->GetHeadstartTime());

                    if (obj->ghost_->GetHeadstartTime() > SE_Env::Inst().GetGhostHeadstart())
                    {
                        SE_Env::Inst().SetGhostHeadstart(obj->ghost_->GetHeadstartTime());
                        SE_Env::Inst().SetGhostMode(GhostMode::RESTART);
                    }
                }

                ghost_ = obj->ghost_;
                break;  // only consider first ghost
            }
        }
    }

    // Align trailers
    for (size_t i = 0; i < entities_.object_.size(); i++)
    {
        Object* obj = entities_.object_[i];
        if (!obj->TowVehicle() && obj->TrailerVehicle())
        {
            // Found a front tow vehicle, update trailers
            (static_cast<Vehicle*>(obj)->AlignTrailers());
        }
    }

    return 0;
}

int ScenarioEngine::defaultController(Object* obj, double dt)
{
    int    retval  = 0;
    double steplen = obj->speed_ * dt;

    if (!obj->CheckDirtyBits(Object::DirtyBit::LONGITUDINAL))  // No action has updated longitudinal dimension
    {
        if (!obj->IsControllerModeOnDomains(ControlOperationMode::MODE_OVERRIDE, static_cast<unsigned int>(ControlDomains::DOMAIN_LONG)))
        {
            Vehicle* tow_vehicle = static_cast<Vehicle*>(obj->TowVehicle());
            if (tow_vehicle == nullptr)
            {
                retval = static_cast<int>(obj->MoveAlongS(steplen, true));
                if (retval == -1)
                {
                    // Something went wrong, couldn't move vehicle forward. Stop.
                    obj->SetSpeed(0.0);
                }
                obj->SetDirtyBits(Object::DirtyBit::LONGITUDINAL |
                                  Object::DirtyBit::SPEED  // indicate that speed has been applied, prevent automatically set from velocity
                );
            }
        }
    }

    return retval == -1 ? -1 : 0;
}

void ScenarioEngine::prepareGroundTruth(double dt)
{
    for (size_t i = 0; i < entities_.object_.size(); i++)
    {
        // Fetch external states from gateway
        Object*      obj = entities_.object_[i];
        ObjectState* o   = scenarioGateway.getObjectStatePtrById(obj->id_);

        if (o == nullptr)
        {
            LOG("Gateway did not provide state for external car %d", obj->id_);
        }
        else
        {
            if (o->dirty_ & (Object::DirtyBit::LATERAL | Object::DirtyBit::LONGITUDINAL))
            {
                obj->pos_ = o->state_.pos;
                obj->SetDirtyBits(o->dirty_ & (Object::DirtyBit::LATERAL | Object::DirtyBit::LONGITUDINAL));
            }
            if (o->dirty_ & Object::DirtyBit::ACCELERATION)
            {
                obj->pos_.SetAcc(o->state_.pos.GetAccX(), o->state_.pos.GetAccY(), o->state_.pos.GetAccZ());
                obj->SetDirtyBits(Object::DirtyBit::ACCELERATION);
            }
            if (o->dirty_ & Object::DirtyBit::SPEED)
            {
                obj->speed_ = o->state_.info.speed;
                obj->SetDirtyBits(Object::DirtyBit::SPEED);
            }
            if (o->dirty_ & Object::DirtyBit::WHEEL_ANGLE)
            {
                obj->wheel_angle_ = o->state_.info.wheel_angle;
                obj->SetDirtyBits(Object::DirtyBit::WHEEL_ANGLE);
            }
            if (o->dirty_ & Object::DirtyBit::WHEEL_ROTATION)
            {
                obj->wheel_rot_ = o->state_.info.wheel_rot;
                obj->SetDirtyBits(Object::DirtyBit::WHEEL_ROTATION);
            }
        }

        // Calculate resulting updated velocity, acceleration and heading rate (rad/s) NOTE: in global coordinate sys
        double dx = obj->pos_.GetX() - obj->state_old.pos_x;
        double dy = obj->pos_.GetY() - obj->state_old.pos_y;
        double dz = obj->pos_.GetZ() - obj->state_old.pos_z;

        if (frame_nr_ == 1 || (obj->IsGhost() && SE_Env::Inst().GetGhostMode() != GhostMode::RESTART) ||
            (!obj->IsGhost() && SE_Env::Inst().GetGhostMode() != GhostMode::RESTARTING))
        {
            if (dt > SMALL_NUMBER)
            {
                // If velocity has not been reported, calculate it based on movement
                if (!obj->CheckDirtyBits(Object::DirtyBit::VELOCITY))
                {
                    if (obj->CheckDirtyBits(Object::DirtyBit::TELEPORT))
                    {
                        // if teleport occured, calculate approximated velocity vector based on current heading
                        obj->SetVel(obj->speed_ * cos(obj->pos_.GetH()), obj->speed_ * sin(obj->pos_.GetH()), 0.0);
                    }
                    else
                    {
                        // calculate linear velocity
                        obj->SetVel(dx / dt, dy / dt, dz / dt);
                    }
                }

                // If speed has not been reported or set by any controller, calculate it based on velocity
                if (!obj->CheckDirtyBits(Object::DirtyBit::SPEED))
                {
                    obj->SetSpeed(GetLengthOfVector2D(obj->pos_.GetVelX(), obj->pos_.GetVelY()));
                }

                if (!obj->CheckDirtyBits(Object::DirtyBit::ACCELERATION))
                {
                    // If not already reported, calculate linear acceleration
                    obj->SetAcc((obj->pos_.GetVelX() - obj->state_old.vel_x) / dt,
                                (obj->pos_.GetVelY() - obj->state_old.vel_y) / dt,
                                (obj->pos_.GetVelZ() - obj->state_old.vel_z) / dt);
                }

                double heading_diff     = GetAngleDifference(obj->pos_.GetH(), obj->state_old.h);
                double heading_rate_new = heading_diff / dt;
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
                    double steeringAngleTarget = SIGN(obj->GetSpeed()) * M_PI * heading_rate_new / MAX(fabs(obj->GetSpeed()), 1.0);
                    double steeringAngleDiff   = steeringAngleTarget - obj->wheel_angle_;

                    // Turn wheel gradually towards target
                    double steeringAngleStep = SIGN(steeringAngleDiff) * MIN(abs(steeringAngleDiff), 0.5 * dt);

                    obj->wheel_angle_ += steeringAngleStep;
                    obj->SetDirtyBits(Object::DirtyBit::WHEEL_ANGLE);
                }

                if (!obj->CheckDirtyBits(Object::DirtyBit::WHEEL_ROTATION))
                {
                    obj->wheel_rot_ = fmod(obj->wheel_rot_ + obj->speed_ * dt / WHEEL_RADIUS, 2 * M_PI);
                    obj->SetDirtyBits(Object::DirtyBit::WHEEL_ROTATION);
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

            if (obj->CheckDirtyBits(Object::DirtyBit::WHEEL_ANGLE))
            {
                scenarioGateway.updateObjectWheelAngle(obj->id_, simulationTime_, obj->wheel_angle_);
            }

            if (obj->CheckDirtyBits(Object::DirtyBit::WHEEL_ROTATION))
            {
                scenarioGateway.updateObjectWheelRotation(obj->id_, simulationTime_, obj->wheel_rot_);
            }

            // store current values for next loop
            obj->state_old.pos_x  = obj->pos_.GetX();
            obj->state_old.pos_y  = obj->pos_.GetY();
            obj->state_old.pos_z  = obj->pos_.GetZ();
            obj->state_old.vel_x  = obj->pos_.GetVelX();
            obj->state_old.vel_y  = obj->pos_.GetVelY();
            obj->state_old.vel_z  = obj->pos_.GetVelZ();
            obj->state_old.h      = obj->pos_.GetH();
            obj->state_old.h_rate = obj->pos_.GetHRate();

            if (!obj->reset_)
            {
                obj->odometer_ += abs(sqrt(dx * dx + dy * dy));  // odometer always measure all movements as positive, I guess...
            }

            if (!(obj->IsGhost() && SE_Env::Inst().GetGhostMode() == GhostMode::RESTART))  // skip ghost sample during restart
            {
                if (obj->trail_.GetNumberOfVertices() == 0 || simulationTime_ - obj->trail_.GetVertex(-1)->time > GHOST_TRAIL_SAMPLE_TIME)
                {
                    // Only add trail vertex when speed is not stable at 0
                    if (obj->trail_.GetNumberOfVertices() == 0 || fabs(obj->trail_.GetVertex(-1)->speed) > SMALL_NUMBER ||
                        fabs(obj->GetSpeed()) > SMALL_NUMBER)
                    {
                        // If considerable time has passed, copy previous steady-state sample
                        if (obj->trail_.vertex_.size() > 0 && simulationTime_ - obj->trail_.GetVertex(-1)->time > 2 * GHOST_TRAIL_SAMPLE_TIME)
                        {
                            obj->trail_.AddVertex(obj->trail_.vertex_.back());
                            // with modified timestamp
                            obj->trail_.vertex_.back().time = simulationTime_ - GHOST_TRAIL_SAMPLE_TIME;
                        }
                        obj->trail_.AddVertex({std::nan(""),
                                               obj->pos_.GetX(),
                                               obj->pos_.GetY(),
                                               obj->pos_.GetZ(),
                                               obj->pos_.GetH(),
                                               obj->pos_.GetP(),
                                               obj->pos_.GetR(),
                                               obj->pos_.GetTrackId(),
                                               simulationTime_,
                                               obj->GetSpeed(),
                                               obj->pos_.GetAcc(),
                                               0.0,
                                               roadmanager::Position::PosMode::H_REL,
                                               0});
                    }
                }
            }
        }

        // Report updated pos values to the gateway
        scenarioGateway.updateObjectPos(obj->id_, simulationTime_, &obj->pos_);

        // Report friction coefficients to gateway

        double friction[4];
        double friction_global = roadmanager::Position::GetOpenDrive()->GetFriction();

        roadmanager::Position wp;
        wp.CopyRMPos(&obj->pos_);

        if (std::isnan(friction_global))
        {
            for (int j = 0; j < 4; j++)
            {
                // multiple friction values in the road network, need to lookup for each wheel
                Object::Axle* axle = j < 2 ? &obj->front_axle_ : &obj->rear_axle_;
                int           side = j % 2 == 0 ? -1 : 1;

                // Calculate position of the wheel
                double w_pos[2];
                double w_rel_pos[2];

                w_rel_pos[0] = axle->positionX;
                w_rel_pos[1] = side * axle->trackWidth / 2.0;
                RotateVec2D(w_rel_pos[0], w_rel_pos[1], obj->pos_.GetH(), w_pos[0], w_pos[1]);

                w_pos[0] += obj->pos_.GetX();
                w_pos[1] += obj->pos_.GetY();

                wp.SetInertiaPosMode(w_pos[0],
                                     w_pos[1],
                                     0.0,
                                     0.0,
                                     0.0,
                                     0.0,
                                     roadmanager::Position::PosMode::Z_REL | roadmanager::Position::PosMode::H_REL |
                                         roadmanager::Position::PosMode::P_REL | roadmanager::Position::PosMode::R_REL);

                roadmanager::RoadLaneInfo info;
                wp.GetRoadLaneInfo(&info);
                friction[j] = info.friction;

                // Uncomment statement below to print some friction values in terminal
                // printf("time %.2f wheel %d lane %d offset %.2f x %.2f y %.2f friction %.2f\n",
                //       simulationTime_,
                //       (int)j,
                //       (int)wp.GetLaneId(),
                //       wp.GetOffset(),
                //       wp.GetX(),
                //       wp.GetY(),
                //       friction[j]);
            }
        }
        else
        {
            // same friction everywhere
            for (int j = 0; j < 4; j++)
            {
                friction[j] = friction_global;
            }
        }

        scenarioGateway.updateObjectFrictionCoefficients(obj->id_, friction);

        // Now that frame is complete, reset dirty bits to avoid circulation
        if (o)
            o->clearDirtyBits();

        // Clear dirty/update bits for any reported velocity and acceleration values, and flag indicating teleport action
        obj->ClearDirtyBits(Object::DirtyBit::VELOCITY | Object::DirtyBit::ANGULAR_RATE | Object::DirtyBit::ACCELERATION |
                            Object::DirtyBit::ANGULAR_ACC | Object::DirtyBit::TELEPORT);
    }
}

void ScenarioEngine::ReplaceObjectInTrigger(Trigger* trigger, Object* obj1, Object* obj2, double timeOffset, Event* event)
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
                TrigByEntity* trig = static_cast<TrigByEntity*>(cond);

                if (trig->type_ == TrigByEntity::EntityConditionType::COLLISION || trig->type_ == TrigByEntity::EntityConditionType::REACH_POSITION ||
                    trig->type_ == TrigByEntity::EntityConditionType::TRAVELED_DISTANCE || trig->type_ == TrigByEntity::EntityConditionType::SPEED ||
                    trig->type_ == TrigByEntity::EntityConditionType::ACCELERATION || trig->type_ == TrigByEntity::EntityConditionType::END_OF_ROAD ||
                    trig->type_ == TrigByEntity::EntityConditionType::OFF_ROAD || trig->type_ == TrigByEntity::EntityConditionType::STAND_STILL)
                {
                    LOG("Handing over trigger %s to ghost", cond->name_.c_str());

                    for (size_t k = 0; k < trig->triggering_entities_.entity_.size(); k++)
                    {
                        if (trig->triggering_entities_.entity_[k].object_ == obj1)
                        {
                            trig->triggering_entities_.entity_[k].object_ = obj2;
                        }
                        else
                        {
                            CreateGhostTeleport(obj1, obj2, event);
                            LOG("Created new teleport action for ghost and %s trigger (entity %s)",
                                cond->name_.c_str(),
                                trig->triggering_entities_.entity_[k].object_->GetName().c_str());
                        }
                    }
                }
                else if (event != nullptr)
                {
                    CreateGhostTeleport(obj1, obj2, event);
                    LOG("Created new teleport action for ghost and %s trigger", cond->name_.c_str());
                }
            }
            else if (cond->base_type_ == OSCCondition::ConditionType::BY_VALUE)
            {
                TrigByValue* trig = static_cast<TrigByValue*>(cond);
                if (trig->type_ == TrigByValue::Type::SIMULATION_TIME)
                {
                    (static_cast<TrigBySimulationTime*>((trig)))->value_ += timeOffset;
                }
                else if (event != nullptr)
                {
                    CreateGhostTeleport(obj1, obj2, event);
                    LOG("Created new teleport action for ghost and %s trigger", cond->name_.c_str());
                }
            }
        }
    }
}

void ScenarioEngine::CreateGhostTeleport(Object* obj1, Object* obj2, Event* event)
{
    TeleportAction*        myNewAction = new TeleportAction(nullptr);
    roadmanager::Position* pos         = new roadmanager::Position();
    pos->SetMode(roadmanager::Position::PosModeType::INIT,
                 roadmanager::Position::PosMode::Z_REL | roadmanager::Position::PosMode::H_REL | roadmanager::Position::PosMode::P_REL |
                     roadmanager::Position::PosMode::R_REL);
    pos->relative_.dx = 0.0;
    pos->relative_.dy = 0.0;
    pos->relative_.dz = 0.0;
    pos->relative_.dh = 0.0;
    pos->relative_.dp = 0.0;
    pos->relative_.dr = 0.0;
    pos->SetRelativePosition(&obj1->pos_, roadmanager::Position::PositionType::RELATIVE_OBJECT);

    myNewAction->position_       = pos;
    myNewAction->action_type_    = OSCPrivateAction::ActionType::TELEPORT;
    myNewAction->object_         = obj2;
    myNewAction->scenarioEngine_ = this;
    myNewAction->SetName("AddedGhostTeleport");
    myNewAction->SetGhostRestart(true);

    event->action_.insert(event->action_.begin(), myNewAction);
}

void ScenarioEngine::SetupGhost(Object* object)
{
    // FollowGhostController special treatment:
    // Create a new (ghost) vehicle and copy all actions from base object

    Vehicle* ghost = new Vehicle(*(static_cast<Vehicle*>(object)));
    object->SetGhost(ghost);
    ghost->name_ += "_ghost";
    ghost->ghost_     = 0;
    ghost->ghost_Ego_ = object;
    ghost->UnassignControllers();
    ghost->isGhost_ = true;
    ghost->SetHeadstartTime(object->headstart_time_);
    entities_.addObject(ghost, true);
    object->SetHeadstartTime(0);

    int numberOfInitActions = static_cast<int>(storyBoard.init_.private_action_.size());
    for (int i = 0; i < numberOfInitActions; i++)
    {
        OSCPrivateAction* action = storyBoard.init_.private_action_[static_cast<unsigned int>(i)];
        if (action->object_ == object)
        {
            // Copy all actions except ActivateController
            if (action->action_type_ != OSCPrivateAction::ActionType::ACTIVATE_CONTROLLER)
            {
                OSCPrivateAction* newAction = action->Copy();
                action->SetName(action->GetName() + "_ghost-copy");
                newAction->object_ = ghost;
                newAction->SetScenarioEngine(this);
                storyBoard.init_.private_action_.push_back(newAction);
                ghost->initActions_.push_back(newAction);
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
                for (size_t l = 0; l < mg->actor_.size(); l++)
                {
                    if (mg->actor_[l]->object_ == object)
                    {
                        // Replace actor
                        mg->actor_[l]->object_ = ghost;
                    }
                }
                for (size_t l = 0; l < act->maneuverGroup_[k]->maneuver_.size(); l++)
                {
                    Maneuver* maneuver = act->maneuverGroup_[k]->maneuver_[l];
                    for (size_t m = 0; m < maneuver->event_.size(); m++)
                    {
                        Event* event        = maneuver->event_[m];
                        bool   ghostIsActor = false;
                        for (size_t n = 0; n < event->action_.size(); n++)
                        {
                            OSCAction* action = event->action_[n];
                            if (action->GetBaseType() == OSCAction::BaseType::PRIVATE)
                            {
                                OSCPrivateAction* pa = static_cast<OSCPrivateAction*>(action);
                                pa->scenarioEngine_  = this;
                                if (pa->object_ == object)
                                {
                                    // If at least one of the event actions is of relevant subset of action types
                                    // then move the action to the ghost object instance, and also make needed
                                    // changes to the event trigger
                                    if (pa->action_type_ == OSCPrivateAction::ActionType::LONG_SPEED ||
                                        pa->action_type_ == OSCPrivateAction::ActionType::LONG_SPEED_PROFILE ||
                                        pa->action_type_ == OSCPrivateAction::ActionType::LAT_LANE_CHANGE ||
                                        pa->action_type_ == OSCPrivateAction::ActionType::LAT_LANE_OFFSET ||
                                        pa->action_type_ == OSCPrivateAction::ActionType::SYNCHRONIZE_ACTION ||
                                        pa->action_type_ == OSCPrivateAction::ActionType::FOLLOW_TRAJECTORY ||
                                        pa->action_type_ == OSCPrivateAction::ActionType::ASSIGN_ROUTE ||
                                        pa->action_type_ == OSCPrivateAction::ActionType::TELEPORT)
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
                            ReplaceObjectInTrigger(event->start_trigger_, object, ghost, -ghost->GetHeadstartTime(), event);
                        }
                        ghost->addEvent(event);
                    }
                }
            }
        }
    }
}
// Reset events ongoing or finished by ghost
void ScenarioEngine::ResetEvents()
{
    for (size_t i = 0; i < storyBoard.story_.size(); i++)
    {
        Story* story = storyBoard.story_[i];

        for (size_t j = 0; j < story->act_.size(); j++)
        {
            Act* act = story->act_[j];

            for (size_t k = 0; k < act->maneuverGroup_.size(); k++)
            {
                for (size_t l = 0; l < act->maneuverGroup_[k]->maneuver_.size(); l++)
                {
                    Maneuver* maneuver = act->maneuverGroup_[k]->maneuver_[l];

                    for (size_t m = 0; m < maneuver->event_.size(); m++)
                    {
                        Event* event = maneuver->event_[m];

                        // The idea is that ghost having a teleport indicates it's a restart
                        // since all original teleport actions remains on Ego
                        if (event->GetCurrentState() == StoryBoardElement::State::COMPLETE)
                        {
                            bool HasTele = false;
                            for (size_t n = 0; n < event->action_.size(); n++)
                            {
                                OSCAction* action = event->action_[n];
                                if (action->GetBaseType() == OSCAction::BaseType::PRIVATE)
                                {
                                    OSCPrivateAction* pa = static_cast<OSCPrivateAction*>(action);
                                    if (pa->action_type_ == OSCPrivateAction::ActionType::TELEPORT)
                                    {
                                        HasTele = true;
                                        break;
                                    }
                                }
                            }
                            if (HasTele)
                            {
                                for (size_t n = 0; n < event->action_.size(); n++)
                                {
                                    OSCAction* action = event->action_[n];
                                    if (action->GetBaseType() == OSCAction::BaseType::PRIVATE)
                                    {
                                        OSCPrivateAction* pa = static_cast<OSCPrivateAction*>(action);

                                        // If the event doesnt contain a teleport action, and the trigger is not triggable, we reser it, making it
                                        // able to tigger again
                                        if (pa->object_->IsGhost() && event->GetCurrentState() == StoryBoardElement::State::COMPLETE)
                                        {
                                            LOG("Reset event %s: ", event->GetName().c_str());
                                            event->Reset(StoryBoardElement::State::STANDBY);
                                            event->num_executions_ = 0;
                                        }
                                        else
                                        {
                                            event->num_executions_--;
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
}

int ScenarioEngine::DetectCollisions()
{
    collision_pair_.clear();
    for (size_t i = 0; i < entities_.object_.size(); i++)
    {
        Object* obj0 = entities_.object_[i];
        for (size_t j = i + 1; j < entities_.object_.size(); j++)
        {
            Object* obj1 = entities_.object_[j];
            if (obj0->Collision(obj1))
            {
                collision_pair_.push_back({obj0, obj1});
                if (std::find(obj0->collisions_.begin(), obj0->collisions_.end(), obj1) == obj0->collisions_.end())
                {
                    // was not overlapping last timestep, but are now
                    LOG("Collision between %s and %s", obj0->GetName().c_str(), obj1->GetName().c_str());
                    obj0->collisions_.push_back(obj1);
                    obj1->collisions_.push_back(obj0);
                }
            }
            else
            {
                if (std::find(obj0->collisions_.begin(), obj0->collisions_.end(), obj1) != obj0->collisions_.end())
                {
                    // was overlapping last frame, but not anymore
                    LOG("Collision between %s and %s dissolved", obj0->GetName().c_str(), obj1->GetName().c_str());
                    obj0->collisions_.erase(std::remove(obj0->collisions_.begin(), obj0->collisions_.end(), obj1), obj0->collisions_.end());
                    obj1->collisions_.erase(std::remove(obj1->collisions_.begin(), obj1->collisions_.end(), obj0), obj1->collisions_.end());
                }
            }
        }
    }

    // Check for and clear any vanished objects from collision lists
    for (size_t i = 0; i < entities_.object_.size(); i++)
    {
        for (size_t j = 0; j < entities_.object_[i]->collisions_.size(); j++)
        {
            Object* obj = entities_.object_[i];
            if (std::find(entities_.object_.begin(), entities_.object_.end(), obj->collisions_[j]) == entities_.object_.end())
            {
                // object previously collided with pivot object has vanished from the set of entities, remove it from collision list
                LOG("Unregister collision between %s and vanished entity", obj->GetName().c_str());
                obj->collisions_.erase(obj->collisions_.begin() + static_cast<int>(j));
                j--;
            }
        }
    }

    return 0;
}
