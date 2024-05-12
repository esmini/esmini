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

#include "ScenarioReader.hpp"
#include "CommonMini.hpp"
#include "OSCParameterDistribution.hpp"
#include "ControllerSloppyDriver.hpp"
#include "ControllerInteractive.hpp"
#include "ControllerFollowGhost.hpp"
#include "ControllerFollowRoute.hpp"
#ifdef _USE_SUMO
#include "ControllerSumo.hpp"
#endif  // _USE_SUMO
#include "ControllerExternal.hpp"
#include "ControllerRel2Abs.hpp"
#include "ControllerACC.hpp"
#include "ControllerALKS.hpp"
#include "ControllerUDPDriver.hpp"
#include "ControllerECE_ALKS_RefDriver.hpp"
#include "ControllerALKS_R157SM.hpp"
#include "ControllerLooming.hpp"
#include "ControllerOffroadFollower.hpp"

#include <cstdlib>

using namespace scenarioengine;

namespace scenarioengine
{
    static ControllerPool controllerPoolStatic;
    ControllerPool        ScenarioReader::controllerPool_ = controllerPoolStatic;

    Parameters ScenarioReader::parameters;
    Parameters ScenarioReader::variables;
}  // namespace scenarioengine

typedef struct
{
    std::string                    element_name;
    StoryBoardElement::ElementType type;
    TrigByState::CondElementState  state;
    StoryBoardElement             *element;
    TrigByState                   *condition;
} StoryBoardElementTriggerInfo;

static std::vector<StoryBoardElementTriggerInfo> storyboard_element_triggers;

ScenarioReader::ScenarioReader(Entities *entities, Catalogs *catalogs, bool disable_controllers)
    : entities_(entities),
      catalogs_(catalogs),
      disable_controllers_(disable_controllers),
      story_board_(nullptr)
{
    parameters.Clear();
}

ScenarioReader::~ScenarioReader()
{
    for (size_t i = 0; i < controller_.size(); i++)
    {
        delete controller_[i];
    }
    controller_.clear();
}

void ScenarioReader::LoadControllers()
{
    // Register all internal controllers. The user may register custom ones as well before reading the scenario.
    RegisterController(ControllerSloppyDriver::GetTypeNameStatic(), InstantiateControllerSloppyDriver);
    RegisterController(ControllerInteractive::GetTypeNameStatic(), InstantiateControllerInteractive);
    RegisterController(ControllerFollowGhost::GetTypeNameStatic(), InstantiateControllerFollowGhost);
    RegisterController(ControllerFollowRoute::GetTypeNameStatic(), InstantiateControllerFollowRoute);
#ifdef _USE_SUMO
    RegisterController(ControllerSumo::GetTypeNameStatic(), InstantiateControllerSumo);
#endif
    RegisterController(ControllerExternal::GetTypeNameStatic(), InstantiateControllerExternal);
    RegisterController(ControllerRel2Abs::GetTypeNameStatic(), InstantiateControllerRel2Abs);
    RegisterController(ControllerACC::GetTypeNameStatic(), InstantiateControllerACC);
    RegisterController(ControllerALKS::GetTypeNameStatic(), InstantiateControllerALKS);
    RegisterController(ControllerUDPDriver::GetTypeNameStatic(), InstantiateControllerUDPDriver);
    RegisterController(ControllerECE_ALKS_REF_DRIVER::GetTypeNameStatic(), InstantiateControllerECE_ALKS_REF_DRIVER);
    RegisterController(ControllerALKS_R157SM::GetTypeNameStatic(), InstantiateControllerALKS_R157SM);
    RegisterController(ControllerLooming::GetTypeNameStatic(), InstantiateControllerLooming);
    RegisterController(ControllerOffroadFollower::GetTypeNameStatic(), InstantiateControllerOffroadFollower);
}

void ScenarioReader::UnloadControllers()
{
    ScenarioReader::controllerPool_.Clear();
}

int ScenarioReader::RemoveController(Controller *controller)
{
    for (size_t i = 0; i < controller_.size(); i++)
    {
        if (controller_[i] == controller)
        {
            delete controller;
            controller_.erase(controller_.begin() + static_cast<int>(i));
            return 0;
        }
    }

    return -1;
}

int ScenarioReader::loadOSCFile(const char *path)
{
    pugi::xml_parse_result result = doc_.load_file(path);
    if (!result)
    {
        LOG("%s at offset (character position): %d", result.description(), result.offset);
        return -1;
    }

    osc_root_ = doc_.child("OpenSCENARIO");
    if (!osc_root_)
    {
        // Another try
        osc_root_ = doc_.child("OpenScenario");
    }

    if (!osc_root_)
    {
        throw std::runtime_error("Couldn't find OpenSCENARIO or OpenScenario element - check XML!");
    }

    // Apply parameter values from distributions
    OSCParameterDistribution &dist = OSCParameterDistribution::Inst();
    if (dist.GetNumPermutations() > 0)
    {
        if (doc_)
        {
            if (dist.GetNumPermutations() > 0)
            {
                if (doc_.child("OpenSCENARIO") && doc_.child("OpenSCENARIO").child("ParameterDeclarations"))
                {
                    LOG("Parameter permutation %d/%d", dist.GetIndex() + 1, dist.GetNumPermutations());

                    for (unsigned int i = 0; i < dist.GetNumParameters(); i++)
                    {
                        pugi::xml_node node = doc_.child("OpenSCENARIO").child("ParameterDeclarations").child("ParameterDeclaration");
                        for (; node; node = node.next_sibling())
                        {
                            std::string param_name = node.attribute("name").value();
                            if (dist.GetParamName(i) == param_name)
                            {
                                node.attribute("value").set_value(dist.GetParamValue(i).c_str());
                                LOG("   %s: %s", dist.GetParamName(i).c_str(), dist.GetParamValue(i).c_str());
                                break;
                            }
                        }
                        if (!node)
                        {
                            LOG("Distribution parameter %s not found in %s", dist.GetParamName(i).c_str(), path);
                            return -1;
                        }
                    }
                }
            }
        }
    }

    oscFilename_ = path;

    return 0;
}

int ScenarioReader::loadOSCMem(const pugi::xml_document &xml_doc)
{
    LOG("Loading XML document from memory");

    doc_.reset(xml_doc);

    osc_root_ = doc_.child("OpenSCENARIO");
    if (!osc_root_)
    {
        // Another try
        osc_root_ = doc_.child("OpenScenario");
    }

    if (!osc_root_)
    {
        throw std::runtime_error("Couldn't find OpenSCENARIO or OpenScenario element - check XML!");
    }

    oscFilename_ = "inline";

    return 0;
}

int ScenarioReader::RegisterCatalogDirectory(pugi::xml_node catalogDirChild)
{
    if (strcmp(catalogDirChild.name(), "Directory"))
    {
        // Additional subdirectory level. Expect Directory next.
        if (catalogDirChild.child("Directory") == NULL)
        {
            LOG("Catalog %s sub element Directory not found - skipping", catalogDirChild.name());
            return -1;
        }

        catalogDirChild = catalogDirChild.child("Directory");
    }

    std::string dirname = parameters.ReadAttribute(catalogDirChild, "path", true);

    if (dirname == "")
    {
        LOG("Catalog %s missing filename - ignoring", catalogDirChild.name());
        return -1;
    }

    if (FileNameExtOf(dirname) == ".xosc")
    {
        // In case the reference is an actual catalog file, load it now
        // Separate directory and name
        Catalogs::CatalogDirEntry entry = {CatalogType::CATALOG_UNDEFINED, DirNameOf(dirname)};
        catalogs_->catalog_dirs_.push_back(entry);
        if (LoadCatalog(FileNameWithoutExtOf(dirname)) == 0)
        {
            LOG("Catalog %s loaded", dirname.c_str());
        }
    }
    else
    {
        catalogs_->RegisterCatalogDirectory(catalogDirChild.parent().name(), dirname);
    }

    return 0;
}

int ScenarioReader::parseOSCHeader()
{
    pugi::xml_node hdr_node = osc_root_.child("FileHeader");

    versionMajor_ = strtoi(hdr_node.attribute("revMajor").value());
    versionMinor_ = strtoi(hdr_node.attribute("revMinor").value());
    description_  = hdr_node.attribute("description").value();

    return 0;
}

Object *ScenarioReader::ResolveObjectReference(std::string name)
{
    Object *object = entities_->GetObjectByName(name);

    if (object == 0)
    {
        throw std::runtime_error(std::string("Failed to find object: ") + name);
    }

    return object;
}

Catalog *ScenarioReader::LoadCatalog(std::string name)
{
    Catalog *catalog;

    // Check if already loaded
    if ((catalog = catalogs_->FindCatalogByName(name)) != 0)
    {
        // Catalog already loaded
        return catalog;
    }

    // Not found, try to locate it in one the registered catalog directories
    pugi::xml_document       catalog_doc;
    pugi::xml_parse_result   result;
    std::vector<std::string> file_name_candidates;
    for (size_t i = 0; i < catalogs_->catalog_dirs_.size() && !result; i++)
    {
        file_name_candidates.clear();
        // absolute path or relative to current directory
        file_name_candidates.push_back(catalogs_->catalog_dirs_[i].dir_name_ + "/" + name + ".xosc");
        // Then assume relative path to scenario directory - which perhaps should be the expected location
        file_name_candidates.push_back(CombineDirectoryPathAndFilepath(DirNameOf(oscFilename_), catalogs_->catalog_dirs_[i].dir_name_) + "/" + name +
                                       ".xosc");
        // Check registered paths
        for (size_t j = 0; j < SE_Env::Inst().GetPaths().size(); j++)
        {
            file_name_candidates.push_back(
                CombineDirectoryPathAndFilepath(SE_Env::Inst().GetPaths()[j], catalogs_->catalog_dirs_[i].dir_name_ + "/" + name + ".xosc"));
            file_name_candidates.push_back(CombineDirectoryPathAndFilepath(SE_Env::Inst().GetPaths()[j], name + ".xosc"));
        }
        for (size_t j = 0; j < file_name_candidates.size() && !result; j++)
        {
            if (FileExists(file_name_candidates[j].c_str()))
            {
                // Load it
                result = catalog_doc.load_file(file_name_candidates[j].c_str());
            }
        }
    }
    if (!result)
    {
        throw std::runtime_error("Couldn't locate catalog file: " + name + ". " + result.description());
    }

    pugi::xml_node osc_node_ = catalog_doc.child("OpenSCENARIO");
    if (!osc_node_)
    {
        osc_node_ = catalog_doc.child("OpenScenario");
        if (!osc_node_)
        {
            throw std::runtime_error("Couldn't find Catalog OpenSCENARIO or OpenScenario element - check XML!");
        }
    }
    pugi::xml_node catalog_node = osc_node_.child("Catalog");

    catalog        = new Catalog();
    catalog->name_ = name;

    for (pugi::xml_node entry_n = catalog_node.first_child(); entry_n; entry_n = entry_n.next_sibling())
    {
        std::string entry_name = parameters.ReadAttribute(entry_n, "name");

        pugi::xml_document root;
        root.append_copy(entry_n);

        catalog->AddEntry(new Entry(entry_name, std::move(root)));
    }

    // Get type by inspecting first entry
    if (catalog->entry_.size() > 0)
    {
        catalog->type_ = catalog->entry_[0]->type_;
    }
    else
    {
        LOG("Warning: Catalog %s seems to be empty!", catalog->name_.c_str());
    }

    catalogs_->AddCatalog(catalog);

    return catalog;
}

void ScenarioReader::parseRoadNetwork(RoadNetwork &roadNetwork)
{
    pugi::xml_node roadNetworkNode = osc_root_.child("RoadNetwork");

    for (pugi::xml_node roadNetworkChild = roadNetworkNode.first_child(); roadNetworkChild; roadNetworkChild = roadNetworkChild.next_sibling())
    {
        std::string roadNetworkChildName(roadNetworkChild.name());

        if (roadNetworkChildName == "LogicFile")
        {
            parseOSCFile(roadNetwork.logicFile, roadNetworkChild);
        }
        else if (roadNetworkChildName == "SceneGraphFile")
        {
            parseOSCFile(roadNetwork.sceneGraphFile, roadNetworkChild);
        }
    }
}

void ScenarioReader::ParseOSCProperties(OSCProperties &properties, pugi::xml_node &xml_node)
{
    pugi::xml_node properties_node = xml_node.child("Properties");
    if (properties_node != NULL)
    {
        for (pugi::xml_node propertiesChild = properties_node.first_child(); propertiesChild; propertiesChild = propertiesChild.next_sibling())
        {
            if (!strcmp(propertiesChild.name(), "File"))
            {
                properties.file_.filepath_ = parameters.ReadAttribute(propertiesChild, "filepath");
            }
            else if (!strcmp(propertiesChild.name(), "Property"))
            {
                OSCProperties::Property property;
                property.name_  = parameters.ReadAttribute(propertiesChild, "name");
                property.value_ = parameters.ReadAttribute(propertiesChild, "value");
                properties.property_.push_back(property);
            }
            else
            {
                LOG("Unexpected property element: %s", propertiesChild.name());
            }
        }
    }
}

Vehicle *ScenarioReader::createRandomOSCVehicle(std::string name)
{
    Vehicle *vehicle = new Vehicle();

    vehicle->name_     = name;
    vehicle->category_ = Vehicle::Category::CAR;
    vehicle->model_id_ = -1;
    vehicle->model3d_  = "";

    // Set some default bounding box just to avoid division-by-zero-problems
    vehicle->boundingbox_                     = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    vehicle->boundingbox_.dimensions_.length_ = 4.0f;
    vehicle->boundingbox_.dimensions_.width_  = 2.0f;
    vehicle->boundingbox_.dimensions_.height_ = 1.2f;

    return vehicle;
}

roadmanager::CoordinateSystem ScenarioReader::ParseCoordinateSystem(pugi::xml_node node, roadmanager::CoordinateSystem defaultValue)
{
    roadmanager::CoordinateSystem cs = defaultValue;

    std::string str = parameters.ReadAttribute(node, "coordinateSystem");
    if (!str.empty())
    {
        if (GetVersionMajor() == 1 && GetVersionMinor() == 0)
        {
            LOG("coordinateSystem introduced in v1.1. Reading it anyway.");
        }

        if (str == "entity")
        {
            cs = roadmanager::CoordinateSystem::CS_ENTITY;
        }
        else if (str == "lane")
        {
            cs = roadmanager::CoordinateSystem::CS_LANE;
        }
        else if (str == "road")
        {
            cs = roadmanager::CoordinateSystem::CS_ROAD;
        }
        else if (str == "trajectory")
        {
            cs = roadmanager::CoordinateSystem::CS_ROAD;
        }
        else
        {
            LOG_AND_QUIT("Unexpected coordinateSytem: %s", str.c_str());
        }
    }

    return cs;
}

roadmanager::RelativeDistanceType ScenarioReader::ParseRelativeDistanceType(pugi::xml_node node, roadmanager::RelativeDistanceType defaultValue)
{
    roadmanager::RelativeDistanceType rdt = defaultValue;

    std::string str = parameters.ReadAttribute(node, "relativeDistanceType");
    if (!str.empty())
    {
        if (str == "lateral")
        {
            rdt = roadmanager::RelativeDistanceType::REL_DIST_LATERAL;
        }
        else if (str == "longitudinal")
        {
            rdt = roadmanager::RelativeDistanceType::REL_DIST_LONGITUDINAL;
        }
        else if (str == "cartesianDistance")
        {
            if (GetVersionMajor() == 1 && GetVersionMinor() == 1)
            {
                LOG("relativeDistanceType::cartesianDistance depricated in v1.1. Reading it anyway.");
            }

            rdt = roadmanager::RelativeDistanceType::REL_DIST_CARTESIAN;
        }
        else if (str == "euclidianDistance")
        {
            if (GetVersionMajor() == 1 && GetVersionMinor() == 0)
            {
                LOG("relativeDistanceType::euclidianDistance introduced in v1.1. Reading it anyway.");
            }

            rdt = roadmanager::RelativeDistanceType::REL_DIST_EUCLIDIAN;
        }
        else
        {
            LOG_AND_QUIT("Unexcpected relativeDistanceType: %s", str.c_str());
        }
    }

    return rdt;
}

void ScenarioReader::ParseOSCBoundingBox(OSCBoundingBox &boundingbox, pugi::xml_node &xml_node)
{
    pugi::xml_node boundingbox_node = xml_node.child("BoundingBox");
    if (boundingbox_node != NULL)
    {
        for (pugi::xml_node boundingboxChild = boundingbox_node.first_child(); boundingboxChild; boundingboxChild = boundingboxChild.next_sibling())
        {
            std::string boundingboxChildName(boundingboxChild.name());
            if (boundingboxChildName == "Center")
            {
                boundingbox.center_.x_ = std::stof(parameters.ReadAttribute(boundingboxChild, "x"));
                boundingbox.center_.y_ = std::stof(parameters.ReadAttribute(boundingboxChild, "y"));
                boundingbox.center_.z_ = std::stof(parameters.ReadAttribute(boundingboxChild, "z"));
            }
            else if (boundingboxChildName == "Dimensions")
            {
                boundingbox.dimensions_.width_  = std::stof(parameters.ReadAttribute(boundingboxChild, "width"));
                boundingbox.dimensions_.length_ = std::stof(parameters.ReadAttribute(boundingboxChild, "length"));
                boundingbox.dimensions_.height_ = std::stof(parameters.ReadAttribute(boundingboxChild, "height"));
            }
            else
            {
                LOG("Not valid boudingbox attribute name:%s", boundingboxChildName.c_str());
            }
        }
    }
    else
    {
        // Fill empty values to indicate missing bounding box
        boundingbox.center_.x_          = 0;
        boundingbox.center_.y_          = 0;
        boundingbox.center_.z_          = 0;
        boundingbox.dimensions_.length_ = 0;
        boundingbox.dimensions_.width_  = 0;
        boundingbox.dimensions_.height_ = 0;
    }
}

Vehicle *ScenarioReader::parseOSCVehicle(pugi::xml_node vehicleNode)
{
    if (vehicleNode == 0)
    {
        return 0;
    }

    Vehicle *vehicle = new Vehicle();

    // First check for parameter declaration
    pugi::xml_node paramDecl = vehicleNode.child("ParameterDeclarations");

    parameters.CreateRestorePoint();
    parameters.addParameterDeclarations(paramDecl);

    vehicle->typeName_ = parameters.ReadAttribute(vehicleNode, "name");
    vehicle->SetCategory(parameters.ReadAttribute(vehicleNode, "vehicleCategory"));

    if (parameters.ReadAttribute(vehicleNode, "role").empty())
    {
        vehicle->SetRole("none");
    }
    else if (!parameters.ReadAttribute(vehicleNode, "role").empty())
    {
        vehicle->SetRole(parameters.ReadAttribute(vehicleNode, "role"));
    }

    // get File based on Category, and set default 3D model id
    if (vehicle->category_ == Vehicle::Category::BICYCLE)
    {
        vehicle->model_id_ = 9;  // magic number for cyclist, set as default
        vehicle->model3d_  = "cyclist.osgb";
    }
    else if (vehicle->category_ == Vehicle::Category::MOTORBIKE)
    {
        vehicle->model_id_ = 10;  // magic number for motorcyclist, set as default
        vehicle->model3d_  = "mc.osgb";
    }
    else if (vehicle->category_ == Vehicle::Category::TRAILER)
    {
        vehicle->model_id_ = 11;  // magic number for car trailer, set as default
        vehicle->model3d_  = "car_trailer.osgb";
    }
    else
    {
        // magic numbers: If first vehicle make it white, else red
        vehicle->model_id_ = entities_->object_.size() == 0 ? 0 : 2;
        vehicle->model3d_  = entities_->object_.size() == 0 ? "car_white.osgb" : "car_red.osgb";
    }

    ParseOSCProperties(vehicle->properties_, vehicleNode);

    // Overwrite default values if 3D model specified
    if (!vehicleNode.attribute("model3d").empty())
    {
        vehicle->model3d_ = parameters.ReadAttribute(vehicleNode, "model3d");
    }
    else if (vehicle->properties_.file_.filepath_ != "")
    {
        vehicle->model3d_ = vehicle->properties_.file_.filepath_;
    }

    std::string modelIdStr = vehicle->properties_.GetValueStr("model_id");
    if (!modelIdStr.empty())
    {
        vehicle->model_id_ = strtoi(modelIdStr);
        if (SE_Env::Inst().GetOptions().IsOptionArgumentSet("record"))
        {
            CheckModelId(vehicle);
        }
    }

    OSCBoundingBox boundingbox = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    ParseOSCBoundingBox(boundingbox, vehicleNode);
    vehicle->boundingbox_ = boundingbox;

    std::string scaleModeStr = vehicle->properties_.GetValueStr("scaleMode");
    if (!scaleModeStr.empty())
    {
        if (scaleModeStr == "BBToModel")
        {
            vehicle->scaleMode_ = EntityScaleMode::BB_TO_MODEL;
        }
        else if (scaleModeStr == "ModelToBB")
        {
            vehicle->scaleMode_ = EntityScaleMode::MODEL_TO_BB;
        }
        else if (scaleModeStr == "None")
        {
            vehicle->scaleMode_ = EntityScaleMode::NONE;
        }
        else
        {
            delete vehicle;
            LOG_AND_QUIT("Unrecognized entity scale mode: %s", scaleModeStr.c_str());
        }
    }

    // Parse Performance element
    pugi::xml_node performance_node = vehicleNode.child("Performance");
    if (performance_node != NULL)
    {
        if (!(performance_node.attribute("maxSpeed").empty()))
        {
            vehicle->SetMaxSpeed(strtod(parameters.ReadAttribute(performance_node, "maxSpeed")));
        }
        else
        {
            LOG("Info: Missing mandatory Performance maxSpeed for %s, applying default %.2f", vehicle->GetTypeName().c_str(), vehicle->GetMaxSpeed());
        }

        if (!(performance_node.attribute("maxAcceleration").empty()))
        {
            vehicle->SetMaxAcceleration(strtod(parameters.ReadAttribute(performance_node, "maxAcceleration")));
        }
        else
        {
            LOG("Info: Missing mandatory Performance maxAcceleration for %s, applying default %.2f",
                vehicle->GetTypeName().c_str(),
                vehicle->GetMaxAcceleration());
        }

        if (!(performance_node.attribute("maxDeceleration").empty()))
        {
            vehicle->SetMaxDeceleration(strtod(parameters.ReadAttribute(performance_node, "maxDeceleration")));
        }
        else
        {
            LOG("Info: Missing mandatory Performance maxDeceleration for %s, applying default %.2f",
                vehicle->GetTypeName().c_str(),
                vehicle->GetMaxDeceleration());
        }
    }
    else
    {
        LOG("Info: Missing mandatory Performance element for %s, applying defaults maxspeed %.2f maxacc %.2f maxdec %.2f",
            vehicle->GetTypeName().c_str(),
            vehicle->GetMaxSpeed(),
            vehicle->GetMaxAcceleration(),
            vehicle->GetMaxDeceleration());
    }

    pugi::xml_node axles = vehicleNode.child("Axles");
    if (axles != NULL)
    {
        for (pugi::xml_node axle_node = axles.first_child(); axle_node; axle_node = axle_node.next_sibling())
        {
            std::string    axle_name(axle_node.name());
            Vehicle::Axle *axle = nullptr;

            if (axle_name == "FrontAxle")
            {
                axle = &vehicle->front_axle_;
            }
            else if (axle_name == "RearAxle")
            {
                axle = &vehicle->rear_axle_;
            }

            if (axle != nullptr)
            {
                axle->maxSteering   = std::stof(parameters.ReadAttribute(axle_node, "maxSteering"));
                axle->positionX     = std::stof(parameters.ReadAttribute(axle_node, "positionX"));
                axle->positionZ     = std::stof(parameters.ReadAttribute(axle_node, "positionZ"));
                axle->trackWidth    = std::stof(parameters.ReadAttribute(axle_node, "trackWidth"));
                axle->wheelDiameter = std::stof(parameters.ReadAttribute(axle_node, "wheelDiameter"));
            }
        }
    }

    // Trailer related elements
    pugi::xml_node trailer_hitch_node = vehicleNode.child("TrailerHitch");
    if (!trailer_hitch_node.empty())
    {
        vehicle->trailer_hitch_      = std::make_shared<Vehicle::TrailerHitch>();
        vehicle->trailer_hitch_->dx_ = strtod(parameters.ReadAttribute(trailer_hitch_node, "dx"));
    }

    pugi::xml_node trailer_coupler_node = vehicleNode.child("TrailerCoupler");
    if (!trailer_coupler_node.empty())
    {
        vehicle->trailer_coupler_      = std::make_shared<Vehicle::TrailerCoupler>();
        vehicle->trailer_coupler_->dx_ = strtod(parameters.ReadAttribute(trailer_coupler_node, "dx"));
    }

    pugi::xml_node trailer_node = vehicleNode.child("Trailer");
    if (!trailer_node.empty())
    {
        Vehicle *trailer = nullptr;

        for (pugi::xml_node trailer_child_node = trailer_node.first_child(); trailer_child_node && trailer == nullptr;
             trailer_child_node                = trailer_child_node.next_sibling())
        {
            if (!trailer_child_node.empty())
            {
                std::string trailer_child_node_name(trailer_child_node.name());
                Object     *object = nullptr;

                if (trailer_child_node_name == "EntityRef")
                {
                    if (!trailer_child_node.attribute("entityRef").empty())
                    {
                        std::string obj_str = parameters.ReadAttribute(trailer_child_node, "entityRef");
                        if (!obj_str.empty())
                        {
                            object = ResolveObjectReference(obj_str);
                            if (object == nullptr)
                            {
                                LOG_AND_QUIT("Error: Trailer %s not found", parameters.ReadAttribute(trailer_node, "entityRef").c_str());
                            }
                            if (object->type_ != Object::Type::VEHICLE)
                            {
                                LOG_AND_QUIT("Error: Trailer %s is not of Vehicle type", parameters.ReadAttribute(trailer_node, "entityRef").c_str());
                            }
                            trailer = static_cast<Vehicle *>(object);
                        }
                    }
                }
                else if (trailer_child_node_name == "CatalogReference")
                {
                    Entry *entry = ResolveCatalogReference(trailer_child_node);

                    if (entry != nullptr)
                    {
                        if (entry->type_ == CatalogType::CATALOG_VEHICLE)
                        {
                            // Make a new instance from catalog entry
                            trailer = parseOSCVehicle(entry->GetNode());
                        }
                        else
                        {
                            LOG("Unexpected catalog type %s for trailer", entry->GetTypeAsStr().c_str());
                        }
                    }
                }
                else if (trailer_child_node_name == "Vehicle")
                {
                    trailer = parseOSCVehicle(trailer_child_node);
                }
                else
                {
                    LOG("Unexpected Trailer child element %s", trailer_child_node_name.c_str());
                }

                if (trailer != nullptr)
                {
                    if (trailer->trailer_coupler_ == nullptr)
                    {
                        delete trailer;
                        LOG_AND_QUIT("Error: Trailer vehicle %s has no coupler", vehicle->GetName().c_str());
                    }
                    else
                    {
                        vehicle->ConnectTrailer(trailer);
                    }
                }
            }
        }
    }

    parameters.RestoreParameterDeclarations();

    return vehicle;
}

Pedestrian *ScenarioReader::parseOSCPedestrian(pugi::xml_node pedestrianNode)
{
    Pedestrian *pedestrian = new Pedestrian();

    if (pedestrianNode == 0)
    {
        return 0;
    }

    // First check for parameter declaration
    pugi::xml_node paramDecl = pedestrianNode.child("ParameterDeclarations");

    parameters.CreateRestorePoint();
    parameters.addParameterDeclarations(paramDecl);

    pedestrian->typeName_ = parameters.ReadAttribute(pedestrianNode, "name");
    pedestrian->SetCategory(parameters.ReadAttribute(pedestrianNode, "pedestrianCategory"));
    pedestrian->mass_ = strtod(parameters.ReadAttribute(pedestrianNode, "mass"));

    // Parse BoundingBox
    OSCBoundingBox boundingbox = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    ParseOSCBoundingBox(boundingbox, pedestrianNode);
    pedestrian->boundingbox_ = boundingbox;

    if (parameters.ReadAttribute(pedestrianNode, "role").empty())
    {
        pedestrian->setPedRole("none");
    }
    else if (!parameters.ReadAttribute(pedestrianNode, "role").empty())
    {
        pedestrian->setPedRole(parameters.ReadAttribute(pedestrianNode, "role"));
    }

    // Set default model_id, will be overwritten if that property is defined
    if (pedestrian->category_ == Pedestrian::Category::ANIMAL)
    {
        pedestrian->model_id_ = 8;  // magic number for moose, set as default
        pedestrian->model3d_  = "moose_cc0.osgb";
    }
    else
    {
        pedestrian->model_id_ = 7;  // magic number for pedestrian, set as default
        pedestrian->model3d_  = "walkman.osgb";
    }

    ParseOSCProperties(pedestrian->properties_, pedestrianNode);

    // Overwrite default values if 3D model specified
    if (!pedestrianNode.attribute("model3d").empty())
    {
        pedestrian->model3d_ = parameters.ReadAttribute(pedestrianNode, "model3d");
    }
    else if (pedestrian->properties_.file_.filepath_ != "")
    {
        pedestrian->model3d_ = pedestrian->properties_.file_.filepath_;
    }

    std::string modelIdStr = pedestrian->properties_.GetValueStr("model_id");
    if (!modelIdStr.empty())
    {
        pedestrian->model_id_ = strtoi(modelIdStr);
        if (SE_Env::Inst().GetOptions().IsOptionArgumentSet("record"))
        {
            CheckModelId(pedestrian);
        }
    }

    std::string scaleModeStr = pedestrian->properties_.GetValueStr("scaleMode");
    if (!scaleModeStr.empty())
    {
        if (scaleModeStr == "BBToModel")
        {
            pedestrian->scaleMode_ = EntityScaleMode::BB_TO_MODEL;
        }
        else if (scaleModeStr == "ModelToBB")
        {
            pedestrian->scaleMode_ = EntityScaleMode::MODEL_TO_BB;
        }
        else if (scaleModeStr == "None")
        {
            pedestrian->scaleMode_ = EntityScaleMode::NONE;
        }
        else
        {
            LOG_AND_QUIT("Unrecognized entity scale mode: %s", scaleModeStr.c_str());
        }
    }

    parameters.RestoreParameterDeclarations();

    return pedestrian;
}

MiscObject *ScenarioReader::parseOSCMiscObject(pugi::xml_node miscObjectNode)
{
    MiscObject *miscObject = new MiscObject();

    if (miscObjectNode == 0)
    {
        return 0;
    }

    // First check for parameter declaration
    pugi::xml_node paramDecl = miscObjectNode.child("ParameterDeclarations");

    parameters.CreateRestorePoint();
    parameters.addParameterDeclarations(paramDecl);

    miscObject->typeName_ = parameters.ReadAttribute(miscObjectNode, "name");
    miscObject->SetCategory(parameters.ReadAttribute(miscObjectNode, "miscObjectCategory"));
    miscObject->mass_ = strtod(parameters.ReadAttribute(miscObjectNode, "mass"));

    // Parse BoundingBox
    OSCBoundingBox boundingbox = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    ParseOSCBoundingBox(boundingbox, miscObjectNode);
    miscObject->boundingbox_ = boundingbox;

    ParseOSCProperties(miscObject->properties_, miscObjectNode);

    // Overwrite default values if 3D model specified
    if (!miscObjectNode.attribute("model3d").empty())
    {
        miscObject->model3d_ = parameters.ReadAttribute(miscObjectNode, "model3d");
    }
    else if (miscObject->properties_.file_.filepath_ != "")
    {
        miscObject->model3d_ = miscObject->properties_.file_.filepath_;
    }

    std::string modelIdStr = miscObject->properties_.GetValueStr("model_id");
    if (!modelIdStr.empty())
    {
        miscObject->model_id_ = strtoi(modelIdStr);
        if (SE_Env::Inst().GetOptions().IsOptionArgumentSet("record"))
        {
            CheckModelId(miscObject);
        }
    }

    std::string scaleModeStr = miscObject->properties_.GetValueStr("scaleMode");
    if (!scaleModeStr.empty())
    {
        if (scaleModeStr == "BBToModel")
        {
            miscObject->scaleMode_ = EntityScaleMode::BB_TO_MODEL;
        }
        else if (scaleModeStr == "ModelToBB")
        {
            miscObject->scaleMode_ = EntityScaleMode::MODEL_TO_BB;
        }
        else if (scaleModeStr == "None")
        {
            miscObject->scaleMode_ = EntityScaleMode::NONE;
        }
        else
        {
            LOG_AND_QUIT("Unrecognized entity scale mode: %s", scaleModeStr.c_str());
        }
    }

    parameters.RestoreParameterDeclarations();

    return miscObject;
}

Controller *ScenarioReader::parseOSCObjectController(pugi::xml_node controllerNode)
{
    std::string   name       = parameters.ReadAttribute(controllerNode, "name");
    Controller   *controller = 0;
    OSCProperties properties;

    // First check for parameter declaration
    pugi::xml_node paramDecl = controllerNode.child("ParameterDeclarations");

    parameters.CreateRestorePoint();
    parameters.addParameterDeclarations(paramDecl);

    // Then read any properties
    ParseOSCProperties(properties, controllerNode);
    std::string filename = properties.file_.filepath_;

    if (controllerNode == 0)
    {
        LOG("Warning: Empty controller node");
    }

    if (!properties.file_.filepath_.empty())
    {
        // Localize file
        if (!FileExists(filename.c_str()))
        {
            // Then assume relative path to scenario directory - which perhaps should be the expected location
            std::string filename2 = CombineDirectoryPathAndFilepath(DirNameOf(oscFilename_), filename);

            if (!FileExists(filename2.c_str()))
            {
                // Give up
                LOG("Failed to localize controller file %s, also tried %s", filename.c_str(), filename2.c_str());
            }
            else
            {
                // Update file path
                properties.file_.filepath_ = filename2;
            }
        }
    }

    // Find controller type among registered ones
    std::string ctrlType = properties.GetValueStr("esminiController");
    if (ctrlType.empty())
    {
        LOG("Missing esminiController property, using controller name: %s", name.c_str());
        ctrlType = name;
    }

    ControllerPool::ControllerEntry *ctrl_entry = ScenarioReader::controllerPool_.GetControllerByType(ctrlType);
    if (ctrl_entry)
    {
        Controller::InitArgs args;
        args.name       = name;
        args.type       = ctrlType;
        args.entities   = entities_;
        args.gateway    = gateway_;
        args.parameters = &parameters;
        args.properties = &properties;
        controller      = ctrl_entry->instantiateFunction(&args);
    }
    else
    {
        LOG("Unsupported controller type: %s. Falling back to default controller", ctrlType.c_str());
        controller = 0;
    }

    parameters.RestoreParameterDeclarations();

    return controller;
}

roadmanager::Route *ScenarioReader::parseOSCRoute(pugi::xml_node routeNode)
{
    roadmanager::Route *route = new roadmanager::Route;

    route->setName(parameters.ReadAttribute(routeNode, "name"));

    parameters.CreateRestorePoint();

    // Closed attribute not supported by roadmanager yet
    std::string closed_str = parameters.ReadAttribute(routeNode, "closed");
    bool        closed     = false;
    (void)closed;
    if (closed_str == "true" || closed_str == "1")
    {
        closed = true;
    }

    for (pugi::xml_node routeChild = routeNode.first_child(); routeChild; routeChild = routeChild.next_sibling())
    {
        std::string routeChildName(routeChild.name());

        if (routeChildName == "ParameterDeclarations")
        {
            parameters.addParameterDeclarations(routeChild);
        }
        else if (routeChildName == "Waypoint")
        {
            roadmanager::Position::RouteStrategy rs            = roadmanager::Position::RouteStrategy::SHORTEST;
            std::string                          routeStrategy = routeChild.attribute("routeStrategy").value();
            if (routeStrategy == "leastIntersections")
            {
                rs = roadmanager::Position::RouteStrategy::MIN_INTERSECTIONS;
            }
            else if (routeStrategy == "fastest")
            {
                rs = roadmanager::Position::RouteStrategy::FASTEST;
            }
            else
            {  // If shortest or unknown
                rs = roadmanager::Position::RouteStrategy::SHORTEST;
            }

            std::unique_ptr<OSCPosition> pos = std::unique_ptr<OSCPosition>{parseOSCPosition(routeChild.first_child())};
            if (pos)
            {
                roadmanager::Position *p = pos->GetRMPos();
                p->SetRouteStrategy(rs);
                route->AddWaypoint(p);
            }
            else
            {
                LOG("Failed to parse waypoint position");
            }
        }
    }
    route->CheckValid();

    parameters.RestoreParameterDeclarations();

    return route;
}

roadmanager::RMTrajectory *ScenarioReader::parseTrajectoryRef(pugi::xml_node trajNode)
{
    roadmanager::RMTrajectory *traj = 0;

    pugi::xml_node refChild = trajNode.first_child();

    if (!strcmp(refChild.name(), "Trajectory"))
    {
        // Parse inline trajectory
        traj = parseTrajectory(refChild);
    }
    else if (!strcmp(refChild.name(), "CatalogReference"))
    {
        // Find route in catalog
        Entry *entry = ResolveCatalogReference(refChild);

        if (entry == 0)
        {
            throw std::runtime_error("Failed to resolve catalog reference");
        }

        if (entry->type_ == CatalogType::CATALOG_TRAJECTORY)
        {
            // Make a new instance from catalog entry
            traj = parseTrajectory(entry->GetNode());
        }
        else
        {
            LOG("Catalog entry of type %s expected - found %s",
                Entry::GetTypeAsStr_(CatalogType::CATALOG_ROUTE).c_str(),
                entry->GetTypeAsStr().c_str());
            throw std::runtime_error("Failed to resolve catalog reference");
        }
    }
    else
    {
        LOG("Missing TrajectoryRef child element (Trajectory or CatalogReference expected)");
        throw std::runtime_error("Missing TrajectoryRef child element (Trajectory or CatalogReference expected)");
    }

    return traj;
}

void ScenarioReader::parseCatalogs()
{
    pugi::xml_node catalogsNode = osc_root_.child("CatalogLocations");

    for (pugi::xml_node catalogsChild = catalogsNode.first_child(); catalogsChild; catalogsChild = catalogsChild.next_sibling())
    {
        RegisterCatalogDirectory(catalogsChild);
    }
}

void ScenarioReader::parseOSCFile(OSCFile &file, pugi::xml_node fileNode)
{
    file.filepath = parameters.ReadAttribute(fileNode, "filepath");
}

roadmanager::RMTrajectory *ScenarioReader::parseTrajectory(pugi::xml_node node)
{
    roadmanager::RMTrajectory *traj  = new roadmanager::RMTrajectory;
    roadmanager::Shape        *shape = 0;

    parameters.CreateRestorePoint();

    traj->name_   = parameters.ReadAttribute(node, "name");
    traj->closed_ = parameters.ReadAttribute(node, "closed") == "true" ? true : false;

    for (pugi::xml_node childNode = node.first_child(); childNode; childNode = childNode.next_sibling())
    {
        std::string childNodeName(childNode.name());

        if (childNodeName == "ParameterDeclarations")
        {
            parameters.addParameterDeclarations(childNode);
        }
        else if (childNodeName == "Shape")
        {
            pugi::xml_node shapeNode = childNode.first_child();
            if (!shapeNode)
            {
                throw std::runtime_error("Missing Trajectory Shape");
            }

            std::string shapeType = shapeNode.name();
            if (shapeType == "Polyline")
            {
                roadmanager::PolyLineShape *pline = new roadmanager::PolyLineShape();
                for (pugi::xml_node vertexNode = shapeNode.first_child(); vertexNode; vertexNode = vertexNode.next_sibling())
                {
                    pugi::xml_node posNode = vertexNode.child("Position");

                    if (!posNode)
                    {
                        throw std::runtime_error("Missing Trajectory/Polyline/Vertex/Position node");
                    }
                    std::unique_ptr<OSCPosition> pos  = std::unique_ptr<OSCPosition>{parseOSCPosition(posNode)};
                    double                       time = strtod(parameters.ReadAttribute(vertexNode, "time"));
                    pline->AddVertex(*pos->GetRMPos(), time);
                }
                shape = pline;
            }
            else if (shapeType == "Clothoid")
            {
                pugi::xml_node               posNode = shapeNode.child("Position");
                std::unique_ptr<OSCPosition> pos     = std::unique_ptr<OSCPosition>{parseOSCPosition(posNode)};

                double curvature = strtod(parameters.ReadAttribute(shapeNode, "curvature"));

                double curvaturePrime = 0.0;
                if (!shapeNode.attribute("curvaturePrime").empty())
                {
                    // curvaturePrime introduced in OSC v1.1
                    curvaturePrime = strtod(parameters.ReadAttribute(shapeNode, "curvaturePrime"));
                }
                else if (!shapeNode.attribute("curvatureDot").empty())
                {
                    // curvatureDot depricated in OSC v1.1
                    curvaturePrime = strtod(parameters.ReadAttribute(shapeNode, "curvatureDot"));
                }

                double length    = strtod(parameters.ReadAttribute(shapeNode, "length"));
                double startTime = strtod(parameters.ReadAttribute(shapeNode, "startTime"));
                double stopTime  = strtod(parameters.ReadAttribute(shapeNode, "stopTime"));

                LOG("Adding clothoid(x=%.2f y=%.2f h=%.2f curv=%.2f curvDot=%.2f len=%.2f startTime=%.2f stopTime=%.2f)",
                    pos->GetRMPos()->GetX(),
                    pos->GetRMPos()->GetY(),
                    pos->GetRMPos()->GetH(),
                    curvature,
                    curvaturePrime,
                    length,
                    startTime,
                    stopTime);

                roadmanager::ClothoidShape *clothoid =
                    new roadmanager::ClothoidShape(*pos->GetRMPos(), curvature, curvaturePrime, length, startTime, stopTime);

                shape = clothoid;
            }
            else if (shapeType == "ClothoidSpline")
            {
                roadmanager::ClothoidSplineShape *clothoidspline = new roadmanager::ClothoidSplineShape();

                clothoidspline->SetEndTime(shapeNode.attribute("timeEnd").empty() ? 0.0 : strtod(parameters.ReadAttribute(shapeNode, "timeEnd")));

                for (pugi::xml_node segmentNode = shapeNode.child("Segment"); segmentNode; segmentNode = segmentNode.next_sibling("Segment"))
                {
                    pugi::xml_node               posNode = segmentNode.child("PositionStart");
                    std::unique_ptr<OSCPosition> pos;
                    roadmanager::Position       *rm_pos = nullptr;

                    if (posNode)
                    {
                        pos    = std::unique_ptr<OSCPosition>(parseOSCPosition(posNode));
                        rm_pos = pos->GetRMPos();
                    }

                    double curvStart  = std::nan("");  // default is to use end curvature of previous segment
                    double curvEnd    = std::nan("");  // default is to use start curvature of current segment
                    double length     = strtod(parameters.ReadAttribute(segmentNode, "length"));
                    double h_offset   = strtod(parameters.ReadAttribute(segmentNode, "hOffset"));
                    double time_start = strtod(parameters.ReadAttribute(segmentNode, "timeStart"));

                    if (!segmentNode.attribute("curvStart").empty())
                    {
                        curvStart = strtod(parameters.ReadAttribute(segmentNode, "curvStart"));
                    }

                    if (!segmentNode.attribute("curvEnd").empty())
                    {
                        curvEnd = strtod(parameters.ReadAttribute(segmentNode, "curvEnd"));
                    }

                    clothoidspline->AddSegment(rm_pos, curvStart, curvEnd, length, h_offset, time_start);
                }
                shape = clothoidspline;
            }
            else if (shapeType == "Nurbs")
            {
                unsigned int order = static_cast<unsigned int>(strtoi(parameters.ReadAttribute(shapeNode, "order")));

                roadmanager::NurbsShape *nurbs = new roadmanager::NurbsShape(order);
                std::vector<double>      knots;

                // Parse control points and knots
                for (pugi::xml_node nurbsChild = shapeNode.first_child(); nurbsChild; nurbsChild = nurbsChild.next_sibling())
                {
                    std::string nurbsChildName(nurbsChild.name());

                    if (nurbsChildName == "ControlPoint")
                    {
                        pugi::xml_node               posNode = nurbsChild.child("Position");
                        std::unique_ptr<OSCPosition> pos     = std::unique_ptr<OSCPosition>{parseOSCPosition(posNode)};
                        double                       time    = strtod(parameters.ReadAttribute(nurbsChild, "time"));
                        double                       weight  = 1.0;
                        if (!nurbsChild.attribute("weight").empty())
                        {
                            weight = strtod(parameters.ReadAttribute(nurbsChild, "weight"));
                        }
#if 1
                        if (posNode.first_child().child("Orientation"))
                        {
                            pos->GetRMPos()->SetMode(roadmanager::Position::PosModeType::SET, roadmanager::Position::PosMode::H_ABS);
                        }
                        else
                        {
                            pos->GetRMPos()->SetMode(roadmanager::Position::PosModeType::SET, roadmanager::Position::PosMode::H_REL);
                        }
#endif
                        nurbs->AddControlPoint(*pos->GetRMPos(), time, weight);
                    }
                    else if (nurbsChildName == "Knot")
                    {
                        double value = strtod(parameters.ReadAttribute(nurbsChild, "value"));
                        knots.push_back(value);
                    }
                    else
                    {
                        throw std::runtime_error(std::string("Unsupported Nurbs child element: ") + nurbsChildName);
                    }
                }
                if (knots.size() == 0)
                {
                    LOG("No knot vector provided. Creating a simple one.");
                    for (size_t i = 0; i < nurbs->ctrlPoint_.size() + order; i++)
                    {
                        if (i < order)
                        {
                            knots.push_back(0.0);
                        }
                        else if (i > nurbs->ctrlPoint_.size() - 1)
                        {
                            knots.push_back(1.0);
                        }
                        else
                        {
                            knots.push_back(static_cast<double>(i + 1 - order) / static_cast<double>(nurbs->ctrlPoint_.size() + 1 - order));
                        }
                    }
                }
                nurbs->AddKnots(knots);
                shape = nurbs;
            }
            else
            {
                throw std::runtime_error("Unexpected/unsupported Trajectory type: " + shapeType);
            }
            if (!shape)
            {
                throw std::runtime_error("Missing Trajectory/Shape element");
            }
            traj->shape_.reset(shape);
        }
    }

    parameters.RestoreParameterDeclarations();

    return traj;
}

Entry *ScenarioReader::ResolveCatalogReference(pugi::xml_node node)
{
    std::string catalog_name;
    std::string entry_name;

    pugi::xml_node parameterAssignmentsNode = node.child("ParameterAssignments");

    // Read any parameter assignments
    for (pugi::xml_node param_n = parameterAssignmentsNode.child("ParameterAssignment"); param_n;
         param_n                = param_n.next_sibling("ParameterAssignment"))
    {
        OSCParameterDeclarations::ParameterStruct param;
        if (param_n.attribute("parameterRef").value()[0] == PARAMETER_PREFIX)
        {
            param.name = &(param_n.attribute("parameterRef").value()[1]);  // Skip prefix character byte
        }
        else
        {
            param.name = param_n.attribute("parameterRef").value();
        }
        param.value._string = parameters.ReadAttribute(param_n, "value");
        parameters.catalog_param_assignments.push_back(param);
    }

    catalog_name = parameters.ReadAttribute(node, "catalogName");
    entry_name   = parameters.ReadAttribute(node, "entryName");

    Catalog *catalog;

    // Make sure the catalog item is loaded
    if ((catalog = LoadCatalog(catalog_name)) == 0)
    {
        LOG("Failed to load catalog %s", catalog_name.c_str());
        return 0;
    }

    Entry *entry = catalog->FindEntryByName(entry_name);
    if (!entry_name.empty() && entry == 0)
    {
        LOG_AND_QUIT("Failed to look up entry %s in catalog %s", entry_name.c_str(), catalog_name.c_str());

        return 0;
    }

    return entry;
}

bool scenarioengine::ScenarioReader::CheckModelId(Object *object)
{
    std::string filename = SE_Env::Inst().GetModelFilenameById(object->model_id_).c_str();
    if (filename != FileNameOf(object->model3d_))
    {
        LOG("Warning: %s %s model_id %d correponds to %s, not specified 3D model %s",
            Object::Type2String(object->GetType()).c_str(),
            object->GetTypeName().c_str(),
            object->model_id_,
            filename.empty() ? "Unknown" : filename.c_str(),
            FileNameOf(object->model3d_).c_str());
        return false;
    }

    return true;
}

int ScenarioReader::parseEntities()
{
    pugi::xml_node enitiesNode = osc_root_.child("Entities");

    for (pugi::xml_node entitiesChild = enitiesNode.first_child(); entitiesChild; entitiesChild = entitiesChild.next_sibling())
    {
        std::string entitiesChildName(entitiesChild.name());
        if (entitiesChildName == "ScenarioObject")
        {
            Object     *obj  = 0;
            Controller *ctrl = 0;

            // First read the object
            pugi::xml_node objectChild;
            if (!(objectChild = entitiesChild.child("CatalogReference")).empty())
            {
                parameters.CreateRestorePoint();
                Entry *entry = ResolveCatalogReference(objectChild);

                if (entry == 0)
                {
                    // Invalid catalog reference - create random vehicle as fall-back
                    LOG("Could not find catalog vehicle, creating a random car as fall-back");
                    std::string entry_name = parameters.ReadAttribute(objectChild, "entryName");
                    Vehicle    *vehicle    = createRandomOSCVehicle(entry_name);
                    obj                    = vehicle;
                }
                else
                {
                    if (entry->type_ == CatalogType::CATALOG_VEHICLE)
                    {
                        // Make a new instance from catalog entry
                        Vehicle *vehicle = parseOSCVehicle(entry->GetNode());
                        obj              = vehicle;
                    }
                    else if (entry->type_ == CatalogType::CATALOG_PEDESTRIAN)
                    {
                        // Make a new instance from catalog entry
                        Pedestrian *pedestrian = parseOSCPedestrian(entry->GetNode());
                        obj                    = pedestrian;
                    }
                    else if (entry->type_ == CatalogType::CATALOG_MISC_OBJECT)
                    {
                        // Make a new instance from catalog entry
                        MiscObject *miscobj = parseOSCMiscObject(entry->GetNode());
                        obj                 = miscobj;
                    }
                    else
                    {
                        LOG("Unexpected catalog type %s", entry->GetTypeAsStr().c_str());
                    }
                }

                parameters.RestoreParameterDeclarations();
            }
            else if (!(objectChild = entitiesChild.child("Vehicle")).empty())
            {
                Vehicle *vehicle = parseOSCVehicle(objectChild);
                obj              = vehicle;
            }
            else if (!(objectChild = entitiesChild.child("Pedestrian")).empty())
            {
                Pedestrian *pedestrian = parseOSCPedestrian(objectChild);
                obj                    = pedestrian;
            }
            else if (!(objectChild = entitiesChild.child("MiscObject")).empty())
            {
                MiscObject *miscObject = parseOSCMiscObject(objectChild);
                obj                    = miscObject;
            }
            else if (!(objectChild = entitiesChild.child("ExternalObjectReference")).empty())
            {
                LOG("ExternalObjectReference not supported yet");
                return -1;
            }
            else
            {
                LOG("Missing EntityObject");
                return -1;
            }

            if (obj == nullptr)
            {
                LOG("Error: Failed to resolve entity object");
                return -1;
            }

            // Then read any controllers
            for (pugi::xml_node ctrlNode = entitiesChild.child("ObjectController"); ctrlNode; ctrlNode = ctrlNode.next_sibling("ObjectController"))
            {
                if (!disable_controllers_)
                {
                    // get the sub child under ObjectController (should only be one)
                    pugi::xml_node ctrlChild = ctrlNode.first_child();
                    std::string    ctrlChildName(ctrlChild.name());
                    if (ctrlChildName == "CatalogReference")
                    {
                        Entry *entry = ResolveCatalogReference(ctrlChild);

                        if (entry == 0)
                        {
                            LOG("No entry found");
                        }
                        else
                        {
                            if (entry->type_ == CatalogType::CATALOG_CONTROLLER)
                            {
                                ctrl = parseOSCObjectController(entry->GetNode());
                            }
                            else
                            {
                                LOG("Unexpected catalog type %s", entry->GetTypeAsStr().c_str());
                            }
                        }
                    }
                    else
                    {
                        ctrl = parseOSCObjectController(ctrlChild);
                    }

                    if (ctrl)
                    {
                        // ObjectControllers are assigned automatically
                        if (!ctrlNode.attribute("name").empty())
                        {
                            ctrl->SetName(parameters.ReadAttribute(ctrlNode, "name"));
                        }
                        controller_.push_back(ctrl);
                        obj->AssignController(ctrl);
                        ctrl->LinkObject(obj);

#ifdef _USE_SUMO
                        if (ctrl->GetType() == Controller::Type::CONTROLLER_TYPE_SUMO)
                        {
                            // Set template vehicle to be used for all vehicles spawned from SUMO
                            (static_cast<ControllerSumo *>(ctrl))->SetSumoVehicle(obj);
                            obj->id_ = -1;

                            // SUMO controller is special in the sense that it is always active
                            ctrl->Activate(ControlActivationMode::ON,
                                           ControlActivationMode::ON,
                                           ControlActivationMode::ON,
                                           ControlActivationMode::ON);

                            // SUMO controller is not assigned to any scenario vehicle
                        }
#endif  // _USE_SUMO
                    }
                }
            }

            if (!obj->IsAnyAssignedControllerOfType(Controller::Type::CONTROLLER_TYPE_SUMO))
            {
                // Add all vehicles to the entity collection, except SUMO template vehicles
                if (entitiesChild.attribute("name").empty())
                {
                    obj->name_ = obj->typeName_;
                }
                else
                {
                    obj->name_ = parameters.ReadAttribute(entitiesChild, "name");
                }
                entities_->addObject(obj, false);
            }
        }
        else if (entitiesChildName == "EntitySelection")
        {
            LOG("INFO: %s is not implemented yet", entitiesChildName.c_str());
        }
        else
        {
            LOG("Unexpected XML element: %s", entitiesChildName.c_str());
        }
    }
    return 0;
}

void ScenarioReader::parseOSCOrientation(OSCOrientation &orientation, pugi::xml_node orientationNode)
{
    if (!(orientationNode.attribute("h").empty()))
    {
        orientation.h_ = strtod(parameters.ReadAttribute(orientationNode, "h"));
    }
    if (!(orientationNode.attribute("p").empty()))
    {
        orientation.p_ = strtod(parameters.ReadAttribute(orientationNode, "p"));
    }
    if (!(orientationNode.attribute("r").empty()))
    {
        orientation.r_ = strtod(parameters.ReadAttribute(orientationNode, "r"));
    }

    std::string type_str = parameters.ReadAttribute(orientationNode, "type");

    if (type_str == "relative")
    {
        orientation.type_ = roadmanager::Position::OrientationType::ORIENTATION_RELATIVE;
    }
    else if (type_str == "absolute")
    {
        orientation.type_ = roadmanager::Position::OrientationType::ORIENTATION_ABSOLUTE;
    }
    else if (type_str == "")
    {
        LOG("No orientation type specified - using absolute");
        orientation.type_ = roadmanager::Position::OrientationType::ORIENTATION_ABSOLUTE;
    }
    else
    {
        LOG("Invalid orientation type: %d - using absolute", type_str.c_str());
        orientation.type_ = roadmanager::Position::OrientationType::ORIENTATION_ABSOLUTE;
    }
}

OSCPosition *ScenarioReader::parseOSCPosition(pugi::xml_node positionNode, OSCPosition *base_on_pos)
{
    OSCPosition *pos_return = 0;

    pugi::xml_node positionChild = positionNode.first_child();

    std::string positionChildName(positionChild.name());

    if (positionChildName == "WorldPosition")
    {
        double x = std::nan("");
        double y = std::nan("");
        double z = std::nan("");
        double h = std::nan("");
        double p = std::nan("");
        double r = std::nan("");

        if (positionChild.attribute("x"))
        {
            x = strtod(parameters.ReadAttribute(positionChild, "x", true));
        }
        if (positionChild.attribute("y"))
        {
            y = strtod(parameters.ReadAttribute(positionChild, "y", true));
        }
        if (!positionChild.attribute("z").empty())
        {
            z = strtod(parameters.ReadAttribute(positionChild, "z", true));
        }
        if (!positionChild.attribute("h").empty())
        {
            h = strtod(parameters.ReadAttribute(positionChild, "h", true));
        }
        if (!positionChild.attribute("p").empty())
        {
            p = strtod(parameters.ReadAttribute(positionChild, "p", true));
        }
        if (!positionChild.attribute("r").empty())
        {
            r = strtod(parameters.ReadAttribute(positionChild, "r", true));
        }

        if (std::isnan(x) || std::isnan(y))
        {
            LOG_AND_QUIT("Missing x or y attributes!\n");
        }

        OSCPositionWorld *pos = new OSCPositionWorld(x, y, z, h, p, r, base_on_pos);

        pos_return = reinterpret_cast<OSCPosition *>(pos);
    }
    else if (positionChildName == "RelativeWorldPosition")
    {
        double dx, dy, dz;

        dx = strtod(parameters.ReadAttribute(positionChild, "dx"));
        dy = strtod(parameters.ReadAttribute(positionChild, "dy"));
        dz = strtod(parameters.ReadAttribute(positionChild, "dz"));

        Object *object = ResolveObjectReference(parameters.ReadAttribute(positionChild, "entityRef"));

        // Check for optional Orientation element
        pugi::xml_node orientation_node = positionChild.child("Orientation");
        OSCOrientation orientation;
        if (orientation_node)
        {
            parseOSCOrientation(orientation, orientation_node);
        }

        OSCPositionRelativeWorld *pos = new OSCPositionRelativeWorld(object, dx, dy, dz, orientation);

        pos_return = reinterpret_cast<OSCPosition *>(pos);
    }
    else if (positionChildName == "RelativeObjectPosition")
    {
        double dx, dy, dz;

        dx             = strtod(parameters.ReadAttribute(positionChild, "dx"));
        dy             = strtod(parameters.ReadAttribute(positionChild, "dy"));
        dz             = strtod(parameters.ReadAttribute(positionChild, "dz"));
        Object *object = ResolveObjectReference(parameters.ReadAttribute(positionChild, "entityRef"));

        // Check for optional Orientation element
        pugi::xml_node orientation_node = positionChild.child("Orientation");
        OSCOrientation orientation;
        if (orientation_node)
        {
            parseOSCOrientation(orientation, orientation_node);
        }

        OSCPositionRelativeObject *pos = new OSCPositionRelativeObject(object, dx, dy, dz, orientation);

        pos_return = reinterpret_cast<OSCPosition *>(pos);
    }
    else if (positionChildName == "RelativeLanePosition")
    {
        int                                  dLane          = 0;
        double                               ds             = 0.0;
        double                               offset         = 0.0;
        roadmanager::Position::DirectionMode direction_mode = roadmanager::Position::DirectionMode::ALONG_S;

        dLane          = strtoi(parameters.ReadAttribute(positionChild, "dLane"));
        offset         = strtod(parameters.ReadAttribute(positionChild, "offset"));
        Object *object = ResolveObjectReference(parameters.ReadAttribute(positionChild, "entityRef", true));

        // Check for optional Orientation element
        pugi::xml_node orientation_node = positionChild.child("Orientation");
        OSCOrientation orientation;
        if (orientation_node)
        {
            parseOSCOrientation(orientation, orientation_node);
        }
        else
        {
            // If no orientation specified, assume Relative is preferred
            orientation.type_ = roadmanager::Position::OrientationType::ORIENTATION_RELATIVE;
        }

        if (!positionChild.attribute("ds").empty())
        {
            ds             = strtod(parameters.ReadAttribute(positionChild, "ds"));
            direction_mode = roadmanager::Position::DirectionMode::ALONG_S;
        }
        else if (!positionChild.attribute("dsLane").empty())
        {
            ds             = strtod(parameters.ReadAttribute(positionChild, "dsLane"));
            direction_mode = roadmanager::Position::DirectionMode::ALONG_LANE;
        }

        OSCPositionRelativeLane *pos = new OSCPositionRelativeLane(object, dLane, ds, offset, orientation, direction_mode);

        pos_return = reinterpret_cast<OSCPosition *>(pos);
    }
    else if (positionChildName == "RelativeRoadPosition")
    {
        double ds, dt;

        ds             = strtod(parameters.ReadAttribute(positionChild, "ds"));
        dt             = strtod(parameters.ReadAttribute(positionChild, "dt"));
        Object *object = ResolveObjectReference(parameters.ReadAttribute(positionChild, "entityRef"));

        // Check for optional Orientation element
        pugi::xml_node orientation_node = positionChild.child("Orientation");
        OSCOrientation orientation;
        if (orientation_node)
        {
            parseOSCOrientation(orientation, orientation_node);
        }
        else
        {
            // If no orientation specified, assume Relative is preferred
            orientation.type_ = roadmanager::Position::OrientationType::ORIENTATION_RELATIVE;
        }

        OSCPositionRelativeRoad *pos = new OSCPositionRelativeRoad(object, ds, dt, orientation);

        pos_return = reinterpret_cast<OSCPosition *>(pos);
    }
    else if (positionChildName == "RoadPosition")
    {
        int    road_id = strtoi(parameters.ReadAttribute(positionChild, "roadId"));
        double s       = strtod(parameters.ReadAttribute(positionChild, "s"));
        double t       = strtod(parameters.ReadAttribute(positionChild, "t"));

        // Check for optional Orientation element
        pugi::xml_node orientation_node = positionChild.child("Orientation");
        OSCOrientation orientation;
        if (orientation_node)
        {
            parseOSCOrientation(orientation, orientation_node);
        }
        else
        {
            // If no orientation specified, assume Relative is preferred
            orientation.type_ = roadmanager::Position::OrientationType::ORIENTATION_RELATIVE;
        }

        OSCPositionRoad *pos = new OSCPositionRoad(road_id, s, t, orientation);

        pos_return = reinterpret_cast<OSCPosition *>(pos);
    }
    else if (positionChildName == "LanePosition")
    {
        int    road_id = strtoi(parameters.ReadAttribute(positionChild, "roadId"));
        int    lane_id = strtoi(parameters.ReadAttribute(positionChild, "laneId"));
        double s       = strtod(parameters.ReadAttribute(positionChild, "s"));

        double offset = 0;  // Default value of optional parameter
        if (positionChild.attribute("offset"))
        {
            offset = strtod(parameters.ReadAttribute(positionChild, "offset"));
        }

        // Check for optional Orientation element
        pugi::xml_node orientation_node = positionChild.child("Orientation");
        OSCOrientation orientation;
        if (orientation_node)
        {
            parseOSCOrientation(orientation, orientation_node);
        }
        else
        {
            // If no orientation specified, assume Relative is preferred
            orientation.type_ = roadmanager::Position::OrientationType::ORIENTATION_RELATIVE;
        }

        OSCPositionLane *pos = new OSCPositionLane(road_id, lane_id, s, offset, orientation);

        pos_return = reinterpret_cast<OSCPosition *>(pos);
    }
    else if (positionChildName == "RoutePosition")
    {
        std::unique_ptr<roadmanager::Route> route;
        OSCPositionRoute                   *pos         = new OSCPositionRoute;
        OSCOrientation                     *orientation = 0;

        for (pugi::xml_node routeChild = positionChild.first_child(); routeChild; routeChild = routeChild.next_sibling())
        {
            if (routeChild.name() == std::string("RouteRef"))
            {
                for (pugi::xml_node routeRefChild = routeChild.first_child(); routeRefChild; routeRefChild = routeRefChild.next_sibling())
                {
                    std::string routeRefChildName(routeRefChild.name());

                    if (routeRefChildName == "Route")
                    {
                        // Parse inline route
                        route.reset(parseOSCRoute(routeRefChild));
                        if (route.get() == nullptr)
                        {
                            LOG_AND_QUIT("Failed to resolve inline route");
                        }
                    }
                    else if (routeRefChildName == "CatalogReference")
                    {
                        // Find route in catalog
                        parameters.CreateRestorePoint();
                        Entry *entry = ResolveCatalogReference(routeRefChild);

                        if (entry == 0)
                        {
                            throw std::runtime_error("Failed to resolve catalog reference");
                        }

                        if (entry->type_ == CatalogType::CATALOG_ROUTE)
                        {
                            // Make a new instance from catalog entry
                            route.reset(parseOSCRoute(entry->GetNode()));
                            if (route.get() == nullptr)
                            {
                                LOG_AND_QUIT("Failed to resolve catalog route");
                            }
                        }
                        else
                        {
                            throw std::runtime_error(std::string("Found catalog entry ") + entry->name_ + ". But wrong type: " +
                                                     entry->GetTypeAsStr() + ". Expected: " + Entry::GetTypeAsStr_(CatalogType::CATALOG_ROUTE) + ".");
                        }

                        parameters.RestoreParameterDeclarations();
                        if (route == nullptr)
                            LOG_AND_QUIT("Failed to resolve route");
                    }
                }
            }
            else if (routeChild.name() == std::string("Orientation"))
            {
                if (orientation)
                {
                    delete orientation;
                    orientation = nullptr;
                }
                orientation = new OSCOrientation;
                parseOSCOrientation(*orientation, routeChild);
            }
            else if (routeChild.name() == std::string("InRoutePosition"))
            {
                for (pugi::xml_node rPositionChild = routeChild.first_child(); rPositionChild; rPositionChild = rPositionChild.next_sibling())
                {
                    std::string rPositionChildName(rPositionChild.name());

                    if (rPositionChildName == "FromCurrentEntity")
                    {
                        LOG("%s is not implemented", rPositionChildName.c_str());
                    }
                    else if (rPositionChildName == "FromRoadCoordinates")
                    {
                        LOG("%s is not implemented", rPositionChildName.c_str());
                    }
                    else if (rPositionChildName == "FromLaneCoordinates")
                    {
                        double s           = strtod(parameters.ReadAttribute(rPositionChild, "pathS"));
                        int    lane_id     = strtoi(parameters.ReadAttribute(rPositionChild, "laneId"));
                        double lane_offset = 0;

                        pugi::xml_attribute laneOffsetAttribute = rPositionChild.attribute("laneOffset");
                        if (laneOffsetAttribute != NULL)
                        {
                            lane_offset = strtod(parameters.ReadAttribute(rPositionChild, "laneOffset"));
                        }
                        if (orientation)
                        {
                            pos->SetRouteRefLaneCoord(route.get(), s, lane_id, lane_offset, orientation);
                        }
                        else
                        {
                            pos->SetRouteRefLaneCoord(route.get(), s, lane_id, lane_offset);
                            if (lane_id > 0)
                            {
                                pos->SetRouteRelativeHeading(M_PI);
                            }
                        }
                    }
                }
            }
        }

        if (orientation)
        {
            delete orientation;
        }

        pos_return = reinterpret_cast<OSCPosition *>(pos);
    }
    else if (positionChildName == "TrajectoryPosition")
    {
        pugi::xml_node trajectoryRef = positionChild.child("TrajectoryRef");

        if (trajectoryRef.empty())
        {
            LOG("Missing expected TrajectoryRef element");
            throw std::runtime_error("Missing expected TrajectoryRef element");
        }

        roadmanager::RMTrajectory *traj = parseTrajectoryRef(trajectoryRef);

        double s = strtod(parameters.ReadAttribute(positionChild, "s"));
        if (!positionChild.attribute("t").empty())
        {
            LOG("TrajectoryPosition -> t not supported yet, set to zero");
        }
        double t = 0;

        // Check for optional Orientation element
        pugi::xml_node orientation_node = positionChild.child("Orientation");
        OSCOrientation orientation;
        if (orientation_node)
        {
            parseOSCOrientation(orientation, orientation_node);
        }

        OSCPositionTrajectory *pos = new OSCPositionTrajectory(traj, s, t, orientation);

        pos_return = reinterpret_cast<OSCPosition *>(pos);
    }

    if (pos_return == 0)
    {
        throw std::runtime_error(std::string("Failed parse position in node ") + positionNode.name());
    }

    return pos_return;
}

OSCPrivateAction::DynamicsShape ParseDynamicsShape(std::string shape)
{
    if (shape == "linear")
    {
        return OSCPrivateAction::DynamicsShape::LINEAR;
    }
    else if (shape == "sinusoidal")
    {
        return OSCPrivateAction::DynamicsShape::SINUSOIDAL;
    }
    else if (shape == "step")
    {
        return OSCPrivateAction::DynamicsShape::STEP;
    }
    else if (shape == "cubic")
    {
        return OSCPrivateAction::DynamicsShape::CUBIC;
    }
    else
    {
        std::string msg = "Dynamics shape " + shape + " unexpected. Supported: linear, sinusoidal, cubic";
        throw std::runtime_error(msg);
    }

    return OSCPrivateAction::DynamicsShape::SHAPE_UNDEFINED;
}

OSCPrivateAction::DynamicsDimension ParseDynamicsDimension(std::string dimension)
{
    if (dimension.empty())
    {
        LOG("Dynamics dimension missing - fall back to TIME");
        return OSCPrivateAction::DynamicsDimension::TIME;
    }
    else if (dimension == "rate")
    {
        return OSCPrivateAction::DynamicsDimension::RATE;
    }
    else if (dimension == "time")
    {
        return OSCPrivateAction::DynamicsDimension::TIME;
    }
    else if (dimension == "distance")
    {
        return OSCPrivateAction::DynamicsDimension::DISTANCE;
    }
    else
    {
        LOG("Dynamics dimension %s not supported - fall back to TIME", dimension.c_str());
        return OSCPrivateAction::DynamicsDimension::TIME;
    }
}

int ScenarioReader::ParseTransitionDynamics(pugi::xml_node node, OSCPrivateAction::TransitionDynamics &td)
{
    td.shape_ = ParseDynamicsShape(parameters.ReadAttribute(node, "dynamicsShape", true));

    if (td.shape_ == OSCPrivateAction::DynamicsShape::STEP)
    {
        // dimension and value not used in this case - relax attribute requirement
        td.dimension_ = ParseDynamicsDimension(parameters.ReadAttribute(node, "dynamicsDimension", false));
        td.SetParamTargetVal(strtod(parameters.ReadAttribute(node, "value", false)));
    }
    else
    {
        td.dimension_ = ParseDynamicsDimension(parameters.ReadAttribute(node, "dynamicsDimension", true));
        td.SetParamTargetVal(strtod(parameters.ReadAttribute(node, "value", true)));
    }

    return 0;
}

OSCGlobalAction *ScenarioReader::parseOSCGlobalAction(pugi::xml_node actionNode, Event *parent)
{
    OSCGlobalAction *action = 0;

    if (actionNode.first_child() == 0)
    {
        throw std::runtime_error("Missing action child node");
    }

    for (pugi::xml_node actionChild = actionNode.first_child(); actionChild; actionChild = actionChild.next_sibling())
    {
        if (actionChild.name() == std::string("ParameterAction"))
        {
            for (pugi::xml_node paramChild = actionChild.first_child(); paramChild; paramChild = paramChild.next_sibling())
            {
                if (paramChild.name() == std::string("SetAction"))
                {
                    ParameterSetAction *paramSetAction = new ParameterSetAction(action);

                    // give user a message about depricated action.. use variable instead...
                    LOG("Parameter SetAction depricated from OSC 1.2. Please use Variable SetAction instead. Accepting for this time.");

                    paramSetAction->name_       = parameters.ReadAttribute(actionChild, "parameterRef");
                    paramSetAction->value_      = parameters.ReadAttribute(paramChild, "value");
                    paramSetAction->parameters_ = &parameters;

                    action = paramSetAction;
                }
                else
                {
                    LOG("ParameterAction %s not supported yet", paramChild.name());
                }
            }
        }
        else if (actionChild.name() == std::string("VariableAction"))
        {
            for (pugi::xml_node varChild = actionChild.first_child(); varChild; varChild = varChild.next_sibling())
            {
                if (varChild.name() == std::string("SetAction"))
                {
                    VariableSetAction *varSetAction = new VariableSetAction(parent);

                    varSetAction->name_      = variables.ReadAttribute(actionChild, "variableRef");
                    varSetAction->value_     = variables.ReadAttribute(varChild, "value");
                    varSetAction->variables_ = &variables;

                    action = varSetAction;
                }
                else
                {
                    LOG("VariableAction %s not supported yet", varChild.name());
                }
            }
        }
        else if (actionChild.name() == std::string("TrafficAction"))
        {
            pugi::xml_node trafficChild = actionChild.first_child();
            if (!strcmp(trafficChild.name(), "TrafficSwarmAction"))
            {
                SwarmTrafficAction *trafficSwarmAction = new SwarmTrafficAction(parent);

                pugi::xml_node childNode = trafficChild.child("CentralObject");
                if (childNode.empty())
                {
                    childNode = trafficChild.child("CentralSwarmObject");
                    if (!childNode.empty())
                    {
                        LOG("Expected \"CentralObject\", found \"CentralSwarmObject\". Accepted.");
                    }
                }
                if (childNode.empty())
                {
                    LOG("Warning: Missing swarm CentralObject!");
                }

                trafficSwarmAction->SetCentralObject(entities_->GetObjectByName(parameters.ReadAttribute(childNode, "entityRef")));
                // childNode = trafficChild.child("")

                std::string radius, numberOfVehicles, velocity;

                // Inner radius (Circle)
                radius = parameters.ReadAttribute(trafficChild, "innerRadius");
                trafficSwarmAction->SetInnerRadius(std::stod(radius));

                // Semi major axis
                radius = parameters.ReadAttribute(trafficChild, "semiMajorAxis");
                trafficSwarmAction->SetSemiMajorAxes(std::stod(radius));

                // Semi major axis
                radius = parameters.ReadAttribute(trafficChild, "semiMinorAxis");
                trafficSwarmAction->SetSemiMinorAxes(std::stod(radius));

                trafficSwarmAction->SetEntities(entities_);
                trafficSwarmAction->SetGateway(gateway_);
                trafficSwarmAction->SetReader(this);

                // Number of vehicles
                numberOfVehicles = parameters.ReadAttribute(trafficChild, "numberOfVehicles");
                trafficSwarmAction->SetNumberOfVehicles(static_cast<int>(std::stoul(numberOfVehicles)));

                // Velocity
                velocity = parameters.ReadAttribute(trafficChild, "velocity");
                trafficSwarmAction->Setvelocity(std::stod(velocity));

                action = trafficSwarmAction;
            }
        }
        else if (actionChild.name() == std::string("EntityAction"))
        {
            Object *entity;

            entity = ResolveObjectReference(parameters.ReadAttribute(actionChild, "entityRef"));
            if (entity == NULL)
            {
                LOG_AND_QUIT("AddEntityAction: Failed to resolve entityRef %s", parameters.ReadAttribute(actionChild, "entityRef").c_str());
            }

            for (pugi::xml_node eaChild = actionChild.first_child(); eaChild; eaChild = eaChild.next_sibling())
            {
                if (eaChild.name() == std::string("AddEntityAction"))
                {
                    AddEntityAction *addEntityAction = new AddEntityAction(entity, parent);

                    addEntityAction->pos_OSCPosition_.reset(parseOSCPosition(eaChild.child("Position")));
                    addEntityAction->pos_ = addEntityAction->pos_OSCPosition_->GetRMPos();
                    addEntityAction->SetEntities(entities_);

                    action = addEntityAction;
                }
                else if (eaChild.name() == std::string("DeleteEntityAction"))
                {
                    DeleteEntityAction *deleteEntityAction = new DeleteEntityAction(entity, parent);
                    deleteEntityAction->SetEntities(entities_);
                    deleteEntityAction->SetGateway(gateway_);

                    action = deleteEntityAction;
                }
                else
                {
                    LOG("EntityAction %s not supported yet", eaChild.name());
                }
            }
        }
        else
        {
            LOG("Unsupported global action: %s", actionChild.name());
        }
    }

    if (action != 0)
    {
        if (actionNode.parent().attribute("name"))
        {
            action->SetName(parameters.ReadAttribute(actionNode.parent(), "name"));
        }
        else
        {
            action->SetName("no name");
        }
    }

    return action;
}

OSCUserDefinedAction *ScenarioReader::parseOSCUserDefinedAction(pugi::xml_node actionNode, Event *parent)
{
    OSCUserDefinedAction *action = nullptr;

    pugi::xml_node actionChild = actionNode.first_child();
    if (actionChild && actionChild.name() == std::string("CustomCommandAction"))
    {
        action                    = new OSCUserDefinedAction(parent);
        action->user_action_type_ = parameters.ReadAttribute(actionChild, "type");
        action->content_          = actionChild.first_child().value();
    }

    if (action && actionNode.parent().attribute("name"))
    {
        action->SetName(parameters.ReadAttribute(actionNode.parent(), "name"));
    }
    else
    {
        action->SetName("no name");
    }

    return action;
}

ActivateControllerAction *ScenarioReader::parseActivateControllerAction(pugi::xml_node node, Event *parent)
{
    ControlActivationMode lat_mode   = ControlActivationMode::UNDEFINED;
    ControlActivationMode long_mode  = ControlActivationMode::UNDEFINED;
    ControlActivationMode light_mode = ControlActivationMode::UNDEFINED;
    ControlActivationMode anim_mode  = ControlActivationMode::UNDEFINED;

    std::string lat_str   = parameters.ReadAttribute(node, "lateral");
    std::string long_str  = parameters.ReadAttribute(node, "longitudinal");
    std::string light_str = parameters.ReadAttribute(node, "lighting");
    std::string anim_str  = parameters.ReadAttribute(node, "animation");
    std::string name_str  = parameters.ReadAttribute(node, "controllerRef");

    if (lat_str == "false")
    {
        lat_mode = ControlActivationMode::OFF;
    }
    else if (lat_str == "true")
    {
        lat_mode = ControlActivationMode::ON;
    }

    if (long_str == "false")
    {
        long_mode = ControlActivationMode::OFF;
    }
    else if (long_str == "true")
    {
        long_mode = ControlActivationMode::ON;
    }

    if (light_str == "false")
    {
        light_mode = ControlActivationMode::OFF;
    }
    else if (light_str == "true")
    {
        light_mode = ControlActivationMode::ON;
    }

    if (anim_str == "false")
    {
        anim_mode = ControlActivationMode::OFF;
    }
    else if (anim_str == "true")
    {
        LOG("Animation activation is not supported yet");
        anim_mode = ControlActivationMode::ON;
    }

    ActivateControllerAction *activateControllerAction = new ActivateControllerAction(name_str, lat_mode, long_mode, light_mode, anim_mode, parent);

    return activateControllerAction;
}

int ScenarioReader::parseDynamicConstraints(pugi::xml_node dynamics_node, DynamicConstraints &dc, Object *obj)
{
    struct value
    {
        double     *variable;
        double      default_value;
        double      performance_value;
        std::string label;
    };

    value values[] = {
        {&dc.max_speed_, 250.0 / 3.6, obj->performance_.maxSpeed, "maxSpeed"},
        {&dc.max_acceleration_, 5.0, obj->performance_.maxAcceleration, "maxAcceleration"},
        {&dc.max_acceleration_rate_, LARGE_NUMBER, dc.max_acceleration_rate_, "maxAccelerationRate"},
        {&dc.max_deceleration_, 10.0, obj->performance_.maxDeceleration, "maxDeceleration"},
        {&dc.max_deceleration_rate_, LARGE_NUMBER, dc.max_deceleration_rate_, "maxDecelerationRate"},
    };

    for (unsigned int i = 0; i < sizeof(values) / sizeof(value); i++)
    {
        if (dynamics_node.attribute(values[i].label.c_str()).empty())
        {
            *values[i].variable = LARGE_NUMBER;  // Default
        }
        else
        {
            *values[i].variable = strtod(parameters.ReadAttribute(dynamics_node, values[i].label.c_str()));

            if (*values[i].variable < SMALL_NUMBER)
            {
                LOG("parseDynamicConstraints: Unexpected small %s value: %.5f, replacing with %s value %.2f",
                    values[i].label.c_str(),
                    *values[i].variable,
                    values[i].default_value < values[i].performance_value ? "default" : "performance",
                    values[i].default_value < values[i].performance_value ? values[i].default_value : values[i].performance_value);
                *values[i].variable = values[i].default_value < values[i].performance_value ? values[i].default_value : values[i].performance_value;
            }
            else if (*values[i].variable > values[i].performance_value)
            {
                LOG("parseDynamicConstraints: %s value %.2f exceeds object performance value: %.2f, truncating",
                    values[i].label.c_str(),
                    *values[i].variable,
                    values[i].performance_value);
                *values[i].variable = values[i].performance_value;
            }
        }
    }

    return 0;
}

OSCPrivateAction *ScenarioReader::parseOSCPrivateAction(pugi::xml_node actionNode, Object *object, Event *parent)
{
    OSCPrivateAction *action = 0;

    for (pugi::xml_node actionChild = actionNode.first_child(); actionChild; actionChild = actionChild.next_sibling())
    {
        if (actionChild.name() == std::string("LongitudinalAction"))
        {
            for (pugi::xml_node longitudinalChild = actionChild.first_child(); longitudinalChild;
                 longitudinalChild                = longitudinalChild.next_sibling())
            {
                if (longitudinalChild.name() == std::string("SpeedAction"))
                {
                    LongSpeedAction *action_speed = new LongSpeedAction(parent);

                    for (pugi::xml_node speedChild = longitudinalChild.first_child(); speedChild; speedChild = speedChild.next_sibling())
                    {
                        if (speedChild.name() == std::string("SpeedActionDynamics"))
                        {
                            ParseTransitionDynamics(speedChild, action_speed->transition_);
                        }
                        else if (speedChild.name() == std::string("SpeedActionTarget"))
                        {
                            for (pugi::xml_node targetChild = speedChild.first_child(); targetChild; targetChild = targetChild.next_sibling())
                            {
                                if (targetChild.name() == std::string("RelativeTargetSpeed"))
                                {
                                    LongSpeedAction::TargetRelative *target_rel = new LongSpeedAction::TargetRelative;

                                    target_rel->value_ = strtod(parameters.ReadAttribute(targetChild, "value"));

                                    target_rel->continuous_ = (parameters.ReadAttribute(targetChild, "continuous") == "true" ||
                                                               parameters.ReadAttribute(targetChild, "continuous") == "1");

                                    target_rel->object_ = ResolveObjectReference(parameters.ReadAttribute(targetChild, "entityRef"));

                                    std::string value_type = parameters.ReadAttribute(targetChild, "speedTargetValueType");
                                    if (value_type == "delta")
                                    {
                                        target_rel->value_type_ = LongSpeedAction::TargetRelative::ValueType::DELTA;
                                    }
                                    else if (value_type == "factor")
                                    {
                                        target_rel->value_type_ = LongSpeedAction::TargetRelative::ValueType::FACTOR;
                                    }
                                    else if (value_type == "")
                                    {
                                        LOG("Value type missing - falling back to delta");
                                        target_rel->value_type_ = LongSpeedAction::TargetRelative::DELTA;
                                    }
                                    else
                                    {
                                        LOG("Value type %s not valid", value_type.c_str());
                                    }
                                    action_speed->target_.reset(target_rel);
                                }
                                else if (targetChild.name() == std::string("AbsoluteTargetSpeed"))
                                {
                                    LongSpeedAction::TargetAbsolute *target_abs = new LongSpeedAction::TargetAbsolute;

                                    target_abs->value_ = strtod(parameters.ReadAttribute(targetChild, "value"));
                                    action_speed->target_.reset(target_abs);
                                }
                                else
                                {
                                    LOG("Unsupported Target type: %s", targetChild.name());
                                }
                            }
                        }
                    }
                    action = action_speed;
                }
                else if (longitudinalChild.name() == std::string("SpeedProfileAction"))  // v1.2
                {
                    LongSpeedProfileAction *action_speed_profile = new LongSpeedProfileAction(parent);

                    if (longitudinalChild.attribute("followingMode").empty())
                    {
                        LOG("Missing mandatory SpeedProfileAction attribute followingMode, set to position");
                        action_speed_profile->following_mode_ = FollowingMode::POSITION;  // set as default
                    }
                    else
                    {
                        std::string str = parameters.ReadAttribute(longitudinalChild, "followingMode");
                        if (str == "position")
                        {
                            action_speed_profile->following_mode_ = FollowingMode::POSITION;
                        }
                        else if (str == "follow")
                        {
                            action_speed_profile->following_mode_ = FollowingMode::FOLLOW;
                        }
                        else
                        {
                            LOG("Unsupported SpeedProfileAction followingMode value: %, set to position", str.c_str());
                            action_speed_profile->following_mode_ = FollowingMode::POSITION;  // set as default
                        }
                    }

                    for (pugi::xml_node child = longitudinalChild.first_child(); child; child = child.next_sibling())
                    {
                        if (!strcmp(child.name(), "DynamicConstraints"))
                        {
                            parseDynamicConstraints(child, action_speed_profile->dynamics_, object);
                        }
                        else if (!strcmp(child.name(), "EntityRef"))
                        {
                            action_speed_profile->entity_ref_ = ResolveObjectReference(parameters.ReadAttribute(child, "entityRef"));
                        }
                        else if (!strcmp(child.name(), "SpeedProfileEntry"))
                        {
                            LongSpeedProfileAction::Entry entry;

                            if (child.attribute("speed").empty())
                            {
                                LOG("Missing mandatory SpeedProfileAction Entry attribute speed, set to 0");
                                entry.speed_ = 0.0;  // set as default
                            }
                            else
                            {
                                entry.speed_ = strtod(parameters.ReadAttribute(child, "speed"));
                            }

                            if (child.attribute("time").empty())
                            {
                                entry.time_ = -1.0;  // indicate not specified
                            }
                            else
                            {
                                entry.time_ = strtod(parameters.ReadAttribute(child, "time"));
                            }

                            action_speed_profile->entry_.push_back(entry);
                        }
                    }

                    action = action_speed_profile;
                }
                else if (longitudinalChild.name() == std::string("LongitudinalDistanceAction"))
                {
                    LongDistanceAction *action_dist = new LongDistanceAction(parent);

                    pugi::xml_node dynamics_node = longitudinalChild.child("DynamicConstraints");
                    if (dynamics_node != NULL)
                    {
                        parseDynamicConstraints(dynamics_node, action_dist->dynamics_, object);
                    }

                    action_dist->target_object_ = ResolveObjectReference(parameters.ReadAttribute(longitudinalChild, "entityRef"));
                    if (longitudinalChild.attribute("distance"))
                    {
                        action_dist->dist_type_ = LongDistanceAction::DistType::DISTANCE;
                        action_dist->distance_  = strtod(parameters.ReadAttribute(longitudinalChild, "distance"));
                    }
                    else if (longitudinalChild.attribute("timeGap"))
                    {
                        action_dist->dist_type_ = LongDistanceAction::DistType::TIME_GAP;
                        action_dist->distance_  = strtod(parameters.ReadAttribute(longitudinalChild, "timeGap"));
                    }
                    else
                    {
                        LOG("Need distance or timeGap");
                    }

                    std::string continuous   = parameters.ReadAttribute(longitudinalChild, "continuous");
                    action_dist->continuous_ = continuous == "true" ? true : false;

                    std::string freespace = parameters.ReadAttribute(longitudinalChild, "freespace");
                    if (freespace == "true" || freespace == "1")
                        action_dist->freespace_ = true;
                    else
                        action_dist->freespace_ = false;

                    std::string displacement = parameters.ReadAttribute(longitudinalChild, "displacement");
                    if (GetVersionMajor() <= 1 && GetVersionMinor() >= 1)
                    {
                        if (displacement.empty())
                        {
                            LOG("displacement attribute missing, setting default trailingReferencedEntity");
                            action_dist->displacement_ = LongDistanceAction::DisplacementType::TRAILING;
                        }
                        else if (displacement == "any")
                        {
                            action_dist->displacement_ = LongDistanceAction::DisplacementType::ANY;
                        }
                        else if (displacement == "trailingReferencedEntity")
                        {
                            action_dist->displacement_ = LongDistanceAction::DisplacementType::TRAILING;
                        }
                        else if (displacement == "leadingReferencedEntity")
                        {
                            action_dist->displacement_ = LongDistanceAction::DisplacementType::LEADING;
                        }
                        else
                        {
                            LOG_AND_QUIT("Unsupported displacement type: %s", displacement.c_str());
                        }

                        if (action_dist->distance_ < 0.0)
                        {
                            // action_dist->displacement_ != LongDistanceAction::DisplacementType::NONE &&
                            LOG("Negative distance or timeGap not supported in OSC version >= 1.1. Using absolute value. Use displacement to specify leading or trailing behavior.");
                            action_dist->distance_ = abs(action_dist->distance_);
                        }
                    }
                    else if (!displacement.empty())
                    {
                        LOG("LongitudinalDistanceAction displacement not supported in OSC version %d.%d", GetVersionMajor(), GetVersionMinor());
                    }

                    action_dist->cs_ = ParseCoordinateSystem(longitudinalChild, roadmanager::CoordinateSystem::CS_ENTITY);

                    action = action_dist;
                }
            }
        }
        else if (actionChild.name() == std::string("LateralAction"))
        {
            for (pugi::xml_node lateralChild = actionChild.first_child(); lateralChild; lateralChild = lateralChild.next_sibling())
            {
                if (lateralChild.name() == std::string("LaneChangeAction"))
                {
                    LatLaneChangeAction *action_lane = new LatLaneChangeAction(parent);

                    if (!lateralChild.attribute("targetLaneOffset").empty())
                    {
                        action_lane->target_lane_offset_ = strtod(parameters.ReadAttribute(lateralChild, "targetLaneOffset"));
                    }

                    for (pugi::xml_node laneChangeChild = lateralChild.first_child(); laneChangeChild;
                         laneChangeChild                = laneChangeChild.next_sibling())
                    {
                        if (laneChangeChild.name() == std::string("LaneChangeActionDynamics"))
                        {
                            ParseTransitionDynamics(laneChangeChild, action_lane->transition_);
                        }
                        else if (laneChangeChild.name() == std::string("LaneChangeTarget"))
                        {
                            LatLaneChangeAction::Target *target = 0;

                            for (pugi::xml_node targetChild = laneChangeChild.first_child(); targetChild; targetChild = targetChild.next_sibling())
                            {
                                if (targetChild.name() == std::string("RelativeTargetLane"))
                                {
                                    LatLaneChangeAction::TargetRelative *target_rel = new LatLaneChangeAction::TargetRelative;

                                    if ((target_rel->object_ = ResolveObjectReference(parameters.ReadAttribute(targetChild, "entityRef"))) == 0)
                                    {
                                        LOG("Failed to find object %s", parameters.ReadAttribute(targetChild, "entityRef").c_str());
                                        return 0;
                                    }
                                    target_rel->value_ = strtoi(parameters.ReadAttribute(targetChild, "value"));
                                    target             = target_rel;
                                }
                                else if (targetChild.name() == std::string("AbsoluteTargetLane"))
                                {
                                    LatLaneChangeAction::TargetAbsolute *target_abs = new LatLaneChangeAction::TargetAbsolute;

                                    target_abs->value_ = strtoi(parameters.ReadAttribute(targetChild, "value"));
                                    target             = target_abs;
                                }
                                else
                                {
                                    throw std::runtime_error(std::string("Unsupported LaneChangeTarget: ") + targetChild.name());
                                }
                            }
                            if (target)
                            {
                                action_lane->target_.reset(target);
                            }
                        }
                    }
                    action = action_lane;
                }
                else if (lateralChild.name() == std::string("LaneOffsetAction"))
                {
                    LatLaneOffsetAction         *action_lane = new LatLaneOffsetAction(parent);
                    LatLaneOffsetAction::Target *target      = nullptr;
                    for (pugi::xml_node laneOffsetChild = lateralChild.first_child(); laneOffsetChild;
                         laneOffsetChild                = laneOffsetChild.next_sibling())
                    {
                        if (laneOffsetChild.name() == std::string("LaneOffsetActionDynamics"))
                        {
                            if (parameters.ReadAttribute(laneOffsetChild, "maxLateralAcc") != "")
                            {
                                action_lane->max_lateral_acc_ = strtod(parameters.ReadAttribute(laneOffsetChild, "maxLateralAcc"));
                                if (action_lane->max_lateral_acc_ < SMALL_NUMBER)
                                {
                                    action_lane->max_lateral_acc_ = SMALL_NUMBER;
                                }
                            }
                            else
                            {
                                action_lane->max_lateral_acc_ = 0.5;  // Just set some reasonable default value
                                LOG("Missing optional LaneOffsetAction maxLateralAcc attribute. Using default: %.2f", action_lane->max_lateral_acc_);
                            }

                            action_lane->transition_.shape_ = ParseDynamicsShape(parameters.ReadAttribute(laneOffsetChild, "dynamicsShape"));
                        }
                        else if (laneOffsetChild.name() == std::string("LaneOffsetTarget"))
                        {
                            for (pugi::xml_node targetChild = laneOffsetChild.first_child(); targetChild; targetChild = targetChild.next_sibling())
                            {
                                if (targetChild.name() == std::string("RelativeTargetLaneOffset"))
                                {
                                    LatLaneOffsetAction::TargetRelative *target_rel = new LatLaneOffsetAction::TargetRelative;

                                    target_rel->object_ = ResolveObjectReference(parameters.ReadAttribute(targetChild, "entityRef"));
                                    target_rel->value_  = strtod(parameters.ReadAttribute(targetChild, "value"));
                                    target              = target_rel;
                                }
                                else if (targetChild.name() == std::string("AbsoluteTargetLaneOffset"))
                                {
                                    LatLaneOffsetAction::TargetAbsolute *target_abs = new LatLaneOffsetAction::TargetAbsolute;

                                    target_abs->value_ = strtod(parameters.ReadAttribute(targetChild, "value"));
                                    target             = target_abs;
                                }
                            }
                            action_lane->target_.reset(target);
                        }
                    }
                    if (target == nullptr)
                    {
                        throw std::runtime_error("Missing LaneOffset Target");
                    }
                    action = action_lane;
                }
                else
                {
                    LOG("Unsupported element type: %s", lateralChild.name());
                }
            }
        }
        else if (actionChild.name() == std::string("SynchronizeAction"))
        {
            SynchronizeAction *action_synch = new SynchronizeAction(parent);

            std::string master_object_str = parameters.ReadAttribute(actionChild, "masterEntityRef");
            action_synch->master_object_  = ResolveObjectReference(master_object_str);

            pugi::xml_node target_position_master_node = actionChild.child("TargetPositionMaster");
            if (!target_position_master_node)
            {
                LOG("Missing required element \"TargetPositionMaster\"");
                return 0;
            }
            action_synch->target_position_master_OSCPosition_.reset(parseOSCPosition(target_position_master_node));
            action_synch->target_position_master_ = action_synch->target_position_master_OSCPosition_->GetRMPos();
            if (parameters.ReadAttribute(actionChild, "targetToleranceMaster") != "")
            {
                action_synch->tolerance_master_ = strtod(parameters.ReadAttribute(actionChild, "targetToleranceMaster"));
            }

            pugi::xml_node target_position_node = actionChild.child("TargetPosition");
            if (!target_position_node)
            {
                LOG("Missing required element \"TargetPosition\"");
                return 0;
            }
            action_synch->target_position_OSCPosition_.reset(parseOSCPosition(target_position_node));
            action_synch->target_position_ = action_synch->target_position_OSCPosition_->GetRMPos();
            if (parameters.ReadAttribute(actionChild, "targetTolerance") != "")
            {
                action_synch->tolerance_ = strtod(parameters.ReadAttribute(actionChild, "targetTolerance"));
            }

            pugi::xml_node target_speed_node = actionChild.child("FinalSpeed");
            if (target_speed_node)
            {
                pugi::xml_node final_speed_element = target_speed_node.first_child();
                if (final_speed_element.empty())
                {
                    throw std::runtime_error("Unexpected missing FinalSpeed child element");
                }

                if (!strcmp(final_speed_element.name(), "AbsoluteSpeed"))
                {
                    LongSpeedAction::TargetAbsolute *targetSpeedAbs = new LongSpeedAction::TargetAbsolute;
                    targetSpeedAbs->value_                          = strtod(parameters.ReadAttribute(final_speed_element, "value"));
                    action_synch->final_speed_.reset(targetSpeedAbs);
                }
                else if (!strcmp(final_speed_element.name(), "RelativeSpeedToMaster"))
                {
                    LongSpeedAction::TargetRelative *targetSpeedRel = new LongSpeedAction::TargetRelative;

                    targetSpeedRel->value_ = strtod(parameters.ReadAttribute(final_speed_element, "value"));

                    targetSpeedRel->continuous_ = true;  // Continuous adaption needed

                    targetSpeedRel->object_ = action_synch->master_object_;  // Master object is the pivot vehicle

                    std::string value_type = parameters.ReadAttribute(final_speed_element, "speedTargetValueType");
                    if (value_type == "delta")
                    {
                        targetSpeedRel->value_type_ = LongSpeedAction::TargetRelative::ValueType::DELTA;
                    }
                    else if (value_type == "factor")
                    {
                        targetSpeedRel->value_type_ = LongSpeedAction::TargetRelative::ValueType::FACTOR;
                    }
                    else if (value_type == "")
                    {
                        LOG("Value type missing - falling back to delta");
                        targetSpeedRel->value_type_ = LongSpeedAction::TargetRelative::DELTA;
                    }
                    else
                    {
                        LOG("Value type %s not valid", value_type.c_str());
                    }
                    action_synch->final_speed_.reset(targetSpeedRel);
                }
                else
                {
                    LOG("Unexpected FinalSpeed element: %s", final_speed_element.name());
                    throw std::runtime_error("Unexpected FinalSpeed element");
                }

                // Check for optional steady state element
                pugi::xml_node steady_state_node = final_speed_element.first_child();
                if (!steady_state_node.empty())
                {
                    if (GetVersionMajor() <= 1 && GetVersionMinor() < 1)
                    {
                        LOG("SynchronizeAction::SteadyState introduced in v1.1. Reading anyway.");
                    }
                    if (action_synch->final_speed_->type_ == scenarioengine::LongSpeedAction::Target::TargetType::ABSOLUTE_SPEED &&
                        action_synch->final_speed_->GetValue() < SMALL_NUMBER)
                    {
                        LOG("SynchronizeAction steady state with 0 or negative final speed (%.2f) is not supported",
                            action_synch->final_speed_->GetValue());
                        throw std::runtime_error("SynchronizeAction steady state with 0 or negative final speed is not supported");
                    }
                    if (!strcmp(steady_state_node.name(), "TargetDistanceSteadyState"))
                    {
                        action_synch->steadyState_.type_ = SynchronizeAction::SteadyStateType::STEADY_STATE_DIST;
                        action_synch->steadyState_.dist_ = strtod(parameters.ReadAttribute(steady_state_node, "distance"));
                    }
                    else if (!strcmp(steady_state_node.name(), "TargetTimeSteadyState"))
                    {
                        action_synch->steadyState_.type_ = SynchronizeAction::SteadyStateType::STEADY_STATE_TIME;
                        action_synch->steadyState_.time_ = strtod(parameters.ReadAttribute(steady_state_node, "time"));
                    }
                    else if (!strcmp(steady_state_node.name(), "TargetPositionSteadyState"))
                    {
                        action_synch->steadyState_OSCPosition_.reset(parseOSCPosition(steady_state_node));
                        action_synch->steadyState_.type_ = SynchronizeAction::SteadyStateType::STEADY_STATE_POS;
                        action_synch->steadyState_.pos_  = action_synch->steadyState_OSCPosition_->GetRMPos();
                    }
                    else
                    {
                        LOG("Unexpected child element: %s", steady_state_node.name());
                        throw std::runtime_error("Unexpected child element: " + std::string(steady_state_node.name()));
                    }
                }
            }
            action = action_synch;
        }
        else if (actionChild.name() == std::string("TeleportAction"))
        {
            TeleportAction *action_pos = new TeleportAction(parent);
            action_pos->position_OSCPosition_.reset(parseOSCPosition(actionChild.first_child()));
            action_pos->position_ = action_pos->position_OSCPosition_->GetRMPos();
            action                = action_pos;
        }
        else if (actionChild.name() == std::string("TrailerAction"))
        {
            pugi::xml_node trailer_action_node = actionChild.first_child();
            if (trailer_action_node.empty())
            {
                LOG("TrailerAction: missing child element");
                return nullptr;
            }

            if (trailer_action_node.name() == std::string("ConnectTrailerAction"))
            {
                ConnectTrailerAction *action_trailer = new ConnectTrailerAction(parent);
                std::string           trailer_ref    = parameters.ReadAttribute(trailer_action_node, "trailerRef");
                if (trailer_ref.empty())
                {
                    // Try with attribute name from old prototype implementation
                    trailer_ref = parameters.ReadAttribute(trailer_action_node, "trailer");
                    if (!trailer_ref.empty())
                    {
                        LOG("Warning: Accepting trailer ref attribute 'trailer'. Consider use correct 'trailerRef' instead.");
                    }
                }

                if (!trailer_ref.empty())
                {
                    action_trailer->trailer_object_ = ResolveObjectReference(trailer_ref);
                }
                else
                {
                    LOG("TrailerAction: Missing mandatory trailer reference, ignoring action");
                    return nullptr;
                }

                action = action_trailer;
            }
            else if (trailer_action_node.name() == std::string("DisconnectTrailerAction"))
            {
                DisconnectTrailerAction *action_trailer = new DisconnectTrailerAction(parent);
                action                                  = action_trailer;
            }
            else
            {
                LOG("TrailerAction: Unexpected child element name: %s", trailer_action_node.name());
            }
        }
        else if (actionChild.name() == std::string("RoutingAction"))
        {
            for (pugi::xml_node routingChild = actionChild.first_child(); routingChild; routingChild = routingChild.next_sibling())
            {
                if (routingChild.name() == std::string("AssignRouteAction"))
                {
                    for (pugi::xml_node assignRouteChild = routingChild.first_child(); assignRouteChild;
                         assignRouteChild                = assignRouteChild.next_sibling())
                    {
                        if (assignRouteChild.name() == std::string("Route"))
                        {
                            AssignRouteAction *action_follow_route = new AssignRouteAction(parent);
                            action_follow_route->route_.reset(parseOSCRoute(assignRouteChild));
                            action = action_follow_route;
                        }
                        else if (assignRouteChild.name() == std::string("CatalogReference"))
                        {
                            AssignRouteAction *action_assign_route = new AssignRouteAction(parent);

                            // Find route in catalog
                            Entry *entry = ResolveCatalogReference(assignRouteChild);

                            if (entry == 0 || entry->root_ == 0)
                            {
                                throw std::runtime_error("Failed to resolve catalog reference");
                            }

                            if (entry->type_ == CatalogType::CATALOG_ROUTE)
                            {
                                // Make a new instance from catalog entry
                                action_assign_route->route_.reset(parseOSCRoute(entry->GetNode()));
                                action = action_assign_route;
                                break;
                            }
                            else
                            {
                                throw std::runtime_error(std::string("Found catalog entry ") + entry->name_ +
                                                         ". But wrong type: " + entry->GetTypeAsStr() +
                                                         ". Expected: " + Entry::GetTypeAsStr_(CatalogType::CATALOG_ROUTE) + ".");
                            }
                        }
                    }
                }
                else if (routingChild.name() == std::string("FollowTrajectoryAction"))
                {
                    FollowTrajectoryAction *action_follow_trajectory = new FollowTrajectoryAction(parent);

                    if (!routingChild.attribute("initialDistanceOffset").empty())
                    {
                        action_follow_trajectory->initialDistanceOffset_ = strtod(parameters.ReadAttribute(routingChild, "initialDistanceOffset"));
                    }
                    else
                    {
                        action_follow_trajectory->initialDistanceOffset_ = 0.0;
                    }

                    action_follow_trajectory->following_mode_ = FollowingMode::FOLLOW;
                    pugi::xml_node followingModeNode          = routingChild.child("TrajectoryFollowingMode");
                    if (followingModeNode != NULL)
                    {
                        std::string followingMode = parameters.ReadAttribute(followingModeNode, "followingMode");
                        if (followingMode.empty())
                        {
                            LOG("trajectoryFollowingMode followingMode attribute missing, applying \"follow\"");
                        }
                        else if (followingMode == "position")
                        {
                            action_follow_trajectory->following_mode_ = FollowingMode::POSITION;
                        }
                        else if (followingMode != "follow")
                        {
                            LOG("trajectoryFollowingMode %s not supported yet, applying \"follow\"", followingMode.c_str());
                        }
                    }
                    else
                    {
                        LOG("trajectoryFollowingMode missing, applying \"following\"");
                    }

                    for (pugi::xml_node followTrajectoryChild = routingChild.first_child(); followTrajectoryChild;
                         followTrajectoryChild                = followTrajectoryChild.next_sibling())
                    {
                        if (followTrajectoryChild.name() == std::string("TrajectoryRef"))
                        {
                            action_follow_trajectory->traj_.reset(parseTrajectoryRef(followTrajectoryChild));
                        }
                        else if (followTrajectoryChild.name() == std::string("Trajectory"))
                        {
                            action_follow_trajectory->traj_.reset(parseTrajectory(followTrajectoryChild));
                        }
                        else if (followTrajectoryChild.name() == std::string("CatalogReference"))
                        {
                            // Find trajectory in catalog
                            Entry *entry = ResolveCatalogReference(followTrajectoryChild);

                            if (entry == 0 || entry->root_ == 0)
                            {
                                throw std::runtime_error("Failed to resolve catalog reference");
                            }

                            if (entry->type_ == CatalogType::CATALOG_TRAJECTORY)
                            {
                                // Make a new instance from catalog entry
                                action_follow_trajectory->traj_.reset(parseTrajectory(entry->GetNode()));
                            }
                            else
                            {
                                throw std::runtime_error(std::string("Found catalog entry ") + entry->name_ +
                                                         ". But wrong type: " + entry->GetTypeAsStr() +
                                                         ". Expected: " + Entry::GetTypeAsStr_(CatalogType::CATALOG_ROUTE) + ".");
                            }
                        }
                        else if (followTrajectoryChild.name() == std::string("TimeReference"))
                        {
                            pugi::xml_node timingNode = followTrajectoryChild.first_child();
                            if (timingNode && std::string(timingNode.name()) == "Timing")
                            {
                                std::string timeDomain = parameters.ReadAttribute(timingNode, "domainAbsoluteRelative");
                                if (timeDomain == "relative")
                                {
                                    action_follow_trajectory->timing_domain_ = FollowTrajectoryAction::TimingDomain::TIMING_RELATIVE;
                                }
                                else if (timeDomain == "absolute")
                                {
                                    action_follow_trajectory->timing_domain_ = FollowTrajectoryAction::TimingDomain::TIMING_ABSOLUTE;
                                }
                                else
                                {
                                    throw std::runtime_error("Unexpected TimeDomain: " + timeDomain);
                                }

                                action_follow_trajectory->timing_scale_  = strtod(parameters.ReadAttribute(timingNode, "scale"));
                                action_follow_trajectory->timing_offset_ = strtod(parameters.ReadAttribute(timingNode, "offset"));
                            }
                            else if (timingNode && std::string(timingNode.name()) == "None")
                            {
                                action_follow_trajectory->timing_domain_ = FollowTrajectoryAction::TimingDomain::NONE;
                            }
                            else
                            {
                                LOG("Missing TimeReference child element, set to None");
                                action_follow_trajectory->timing_domain_ = FollowTrajectoryAction::TimingDomain::NONE;
                            }
                        }
                    }

                    // Check that trajectory has time duration
                    if (action_follow_trajectory->timing_domain_ != FollowTrajectoryAction::TimingDomain::NONE &&
                        action_follow_trajectory->traj_->GetDuration() < SMALL_NUMBER)
                    {
                        LOG("Warning: FollowTrajectoryAction timeref is != NONE but trajectory duration is 0. Applying timeref=NONE.",
                            action_follow_trajectory->GetName().c_str());
                        action_follow_trajectory->timing_domain_ = FollowTrajectoryAction::TimingDomain::NONE;
                    }

                    // If trajectory contains timestamps and time reference it affects both lateral and longitudinal domains
                    // otherwise only lateral
                    if (action_follow_trajectory->timing_domain_ == FollowTrajectoryAction::TimingDomain::NONE)
                    {
                        action_follow_trajectory->domains_ = static_cast<unsigned int>(ControlDomains::DOMAIN_LAT);
                    }
                    else
                    {
                        action_follow_trajectory->domains_ = static_cast<unsigned int>(ControlDomains::DOMAIN_LAT_AND_LONG);
                    }

                    action = action_follow_trajectory;
                }
                else if (routingChild.name() == std::string("AcquirePositionAction"))
                {
                    AcquirePositionAction *acqPosAction = new AcquirePositionAction(parent);
                    acqPosAction->target_position_OSCPosition_.reset(parseOSCPosition(routingChild.first_child()));
                    acqPosAction->target_position_ = acqPosAction->target_position_OSCPosition_->GetRMPos();
                    action                         = acqPosAction;
                }
                else
                {
                    throw std::runtime_error("Action is not supported: " + std::string(routingChild.name()));
                }
            }
        }
        else if (actionChild.name() == std::string("ActivateControllerAction"))
        {
            if (disable_controllers_)
            {
                continue;
            }
            if (GetVersionMajor() == 1 && GetVersionMinor() == 1)
            {
                LOG("In OSC 1.1 ActivateControllerAction should be placed under ControllerAction. Accepting anyway.");
            }

            ActivateControllerAction *activateControllerAction = parseActivateControllerAction(actionChild, parent);

            action = activateControllerAction;
        }
        else if (actionChild.name() == std::string("ControllerAction"))
        {
            if (disable_controllers_)
            {
                continue;
            }

            for (pugi::xml_node controllerChild = actionChild.first_child(); controllerChild; controllerChild = controllerChild.next_sibling())
            {
                if (controllerChild.name() == std::string("AssignControllerAction"))
                {
                    std::string ctrl_name;
                    if (GetVersionMinor() >= 3)
                    {
                        // expect an "ObjectController" parent umbrella element
                        if (!controllerChild.child("ObjectController").empty())
                        {
                            controllerChild = controllerChild.child("ObjectController");
                            ctrl_name       = parameters.ReadAttribute(controllerChild, "name");
                        }
                    }

                    for (pugi::xml_node controllerDefNode = controllerChild.first_child(); controllerDefNode;
                         controllerDefNode                = controllerDefNode.next_sibling())
                    {
                        Controller *controller = 0;
                        if (controllerDefNode.name() == std::string("Controller"))
                        {
                            controller = parseOSCObjectController(controllerDefNode);
                        }
                        else if (controllerDefNode.name() == std::string("CatalogReference"))
                        {
                            Entry *entry = ResolveCatalogReference(controllerDefNode);

                            if (entry == 0)
                            {
                                LOG("No entry found");
                            }
                            else
                            {
                                if (entry->type_ == CatalogType::CATALOG_CONTROLLER)
                                {
                                    controller = parseOSCObjectController(entry->GetNode());
                                }
                                else
                                {
                                    LOG("Unexpected catalog type %s", entry->GetTypeAsStr().c_str());
                                }
                            }
                        }
                        else
                        {
                            LOG("Unexpected AssignControllerAction subelement: %s", controllerDefNode.name());
                            return 0;
                        }

                        if (controller)
                        {
                            controller->SetName(ctrl_name);
                            controller_.push_back(controller);
                            controller->LinkObject(object);
                        }

                        ControlActivationMode lat_mode   = ControlActivationMode::UNDEFINED;
                        ControlActivationMode long_mode  = ControlActivationMode::UNDEFINED;
                        ControlActivationMode light_mode = ControlActivationMode::UNDEFINED;
                        ControlActivationMode anim_mode  = ControlActivationMode::UNDEFINED;

                        std::string lat_str   = parameters.ReadAttribute(controllerDefNode, "lateral");
                        std::string long_str  = parameters.ReadAttribute(controllerDefNode, "longitudinal");
                        std::string light_str = parameters.ReadAttribute(controllerDefNode, "lighting");
                        std::string anim_str  = parameters.ReadAttribute(controllerDefNode, "animation");

                        if (lat_str == "false")
                        {
                            lat_mode = ControlActivationMode::OFF;
                        }
                        else if (lat_str == "true")
                        {
                            lat_mode = ControlActivationMode::ON;
                        }

                        if (long_str == "false")
                        {
                            long_mode = ControlActivationMode::OFF;
                        }
                        else if (long_str == "true")
                        {
                            long_mode = ControlActivationMode::ON;
                        }

                        if (light_str == "false")
                        {
                            light_mode = ControlActivationMode::OFF;
                        }
                        else if (light_str == "true")
                        {
                            light_mode = ControlActivationMode::ON;
                        }

                        if (anim_str == "false")
                        {
                            anim_mode = ControlActivationMode::OFF;
                        }
                        else if (anim_str == "true")
                        {
                            LOG("Animation activation is not supported yet");
                            anim_mode = ControlActivationMode::ON;
                        }

                        AssignControllerAction *assignControllerAction =
                            new AssignControllerAction(controller, lat_mode, long_mode, light_mode, anim_mode, parent);

                        action = assignControllerAction;
                    }
                }
                else if (controllerChild.name() == std::string("OverrideControllerValueAction"))
                {
                    OverrideControlAction       *override_action = new OverrideControlAction(parent);
                    Object::OverrideActionStatus overrideStatus;
                    bool                         verFromMinor2 = (GetVersionMajor() == 1 && GetVersionMinor() >= 2);

                    for (pugi::xml_node controllerDefNode = controllerChild.first_child(); controllerDefNode;
                         controllerDefNode                = controllerDefNode.next_sibling())
                    {
                        // read active flag
                        overrideStatus.active = parameters.ReadAttribute(controllerDefNode, "active") == "true" ? true : false;

                        if (controllerDefNode.name() == std::string("Throttle"))
                        {
                            double value         = strtod(parameters.ReadAttribute(controllerDefNode, "value"));
                            overrideStatus.type  = static_cast<int>(Object::OverrideType::OVERRIDE_THROTTLE);
                            overrideStatus.value = override_action->RangeCheckAndErrorLog(Object::OverrideType::OVERRIDE_THROTTLE, value);

                            // version 1.2 with throttle attribute
                            if ((verFromMinor2) && (!(controllerDefNode.attribute("maxRate").empty())))
                            {
                                double maxRate         = strtod(parameters.ReadAttribute(controllerDefNode, "maxRate"));
                                overrideStatus.maxRate = maxRate;
                            }
                        }
                        else if (controllerDefNode.name() == std::string("Brake"))
                        {
                            overrideStatus.type             = Object::OverrideType::OVERRIDE_BRAKE;
                            pugi::xml_node brake_input_node = controllerDefNode.first_child();

                            if (brake_input_node.empty())
                            {
                                // No BrakeInput child element
                                double value         = strtod(parameters.ReadAttribute(controllerDefNode, "value"));
                                overrideStatus.value = override_action->RangeCheckAndErrorLog(Object::OverrideType::OVERRIDE_BRAKE, value);
                                if (verFromMinor2)
                                {
                                    LOG("From version 1.2 BrakeInput element should be used instead of value attribute, Accepting anyway");
                                }
                            }
                            else
                            {
                                // BrakeInput child element seems to be present
                                double value = strtod(parameters.ReadAttribute(brake_input_node, "value"));
                                if ((brake_input_node.name() == std::string("BrakeForce")))
                                {
                                    overrideStatus.value_type = static_cast<int>(Object::OverrideBrakeType::Force);
                                    // No upper limit
                                    if (value < 0.0)
                                    {
                                        LOG("Unexpected negative brake force %.2f - ignoring, set to 0", value);
                                        overrideStatus.value = 0.0;
                                    }
                                    else
                                    {
                                        overrideStatus.value = value;
                                    }
                                }
                                else if ((controllerDefNode.first_child().name() == std::string("BrakePercent")))
                                {
                                    overrideStatus.value_type = static_cast<int>(Object::OverrideBrakeType::Percent);
                                    overrideStatus.value      = override_action->RangeCheckAndErrorLog(Object::OverrideType::OVERRIDE_BRAKE, value);
                                }
                                else
                                {
                                    LOG("Unexpected Brake child element: %s", brake_input_node.name());
                                    delete override_action;
                                    return 0;
                                }

                                // Check for optional maxRate attribute
                                if (!brake_input_node.attribute("maxRate").empty())
                                {
                                    overrideStatus.maxRate = strtod(parameters.ReadAttribute(brake_input_node, "maxRate"));
                                }

                                if (!verFromMinor2)
                                {
                                    LOG("Unexpected BrakeInput element in version %d.%d, introduced in OSC 1.2",
                                        GetVersionMajor(),
                                        GetVersionMinor());
                                }
                            }
                        }
                        else if (controllerDefNode.name() == std::string("Clutch"))
                        {
                            double value         = strtod(parameters.ReadAttribute(controllerDefNode, "value"));
                            overrideStatus.type  = Object::OverrideType::OVERRIDE_CLUTCH;
                            overrideStatus.value = override_action->RangeCheckAndErrorLog(Object::OverrideType::OVERRIDE_CLUTCH, value);

                            // version 1.2 with clutch attribute
                            if ((verFromMinor2) && (!(controllerDefNode.attribute("maxRate").empty())))
                            {
                                double maxRate         = strtod(parameters.ReadAttribute(controllerDefNode, "maxRate"));
                                overrideStatus.maxRate = maxRate;
                            }
                        }
                        else if (controllerDefNode.name() == std::string("ParkingBrake"))
                        {
                            overrideStatus.type                     = Object::OverrideType::OVERRIDE_PARKING_BRAKE;
                            pugi::xml_node parking_brake_input_node = controllerDefNode.first_child();

                            if (parking_brake_input_node.empty())
                            {
                                // No BrakeInput child element
                                double value         = strtod(parameters.ReadAttribute(controllerDefNode, "value"));
                                overrideStatus.value = override_action->RangeCheckAndErrorLog(Object::OverrideType::OVERRIDE_PARKING_BRAKE, value);
                                if (verFromMinor2)
                                {
                                    LOG("From version 1.2 BrakeInput element should be used instead of value attribute, Accepting anyway");
                                }
                            }
                            else
                            {
                                // BrakeInput child element seems to be present
                                double value = strtod(parameters.ReadAttribute(parking_brake_input_node, "value"));
                                if ((parking_brake_input_node.name() == std::string("BrakeForce")))
                                {
                                    overrideStatus.value_type = static_cast<int>(Object::OverrideBrakeType::Force);
                                    // No upper limit
                                    if (value < 0.0)
                                    {
                                        LOG("Unexpected negative brake force %.2f - ignoring, set to 0", value);
                                        overrideStatus.value = 0.0;
                                    }
                                    else
                                    {
                                        overrideStatus.value = value;
                                    }
                                }
                                else if ((controllerDefNode.first_child().name() == std::string("BrakePercent")))
                                {
                                    overrideStatus.value_type = static_cast<int>(Object::OverrideBrakeType::Percent);
                                    overrideStatus.value =
                                        override_action->RangeCheckAndErrorLog(Object::OverrideType::OVERRIDE_PARKING_BRAKE, value);
                                }
                                else
                                {
                                    LOG("Unexpected Parking Brake child element: %s", parking_brake_input_node.name());
                                    delete override_action;
                                    return 0;
                                }

                                // Check for optional maxRate attribute
                                if (!parking_brake_input_node.attribute("maxRate").empty())
                                {
                                    overrideStatus.maxRate = strtod(parameters.ReadAttribute(parking_brake_input_node, "maxRate"));
                                }

                                if (!verFromMinor2)
                                {
                                    LOG("Unexpected BrakeInput element in version %d.%d, introduced in OSC 1.2",
                                        GetVersionMajor(),
                                        GetVersionMinor());
                                }
                            }
                        }

                        else if (controllerDefNode.name() == std::string("SteeringWheel"))
                        {
                            double value        = strtod(parameters.ReadAttribute(controllerDefNode, "value"));
                            overrideStatus.type = Object::OverrideType::OVERRIDE_STEERING_WHEEL;
                            overrideStatus.value =
                                override_action->RangeCheckAndErrorLog(Object::OverrideType::OVERRIDE_STEERING_WHEEL, value, -2 * M_PI, 2 * M_PI);
                            // version 1.2 and steering maxRate attribute
                            if ((verFromMinor2) && (!(controllerDefNode.attribute("maxRate").empty())))
                            {
                                double maxRate         = strtod(parameters.ReadAttribute(controllerDefNode, "maxRate"));
                                overrideStatus.maxRate = maxRate;
                            }
                            // version 1.2 and steering maxTorque attribute
                            if ((verFromMinor2) && (!(controllerDefNode.attribute("maxTorque").empty())))
                            {
                                double maxTorque         = strtod(parameters.ReadAttribute(controllerDefNode, "maxTorque"));
                                overrideStatus.maxTorque = maxTorque;
                            }
                        }
                        else if (controllerDefNode.name() == std::string("Gear"))
                        {
                            overrideStatus.type = Object::OverrideType::OVERRIDE_GEAR;
                            // version < 1.2 or no gear type
                            if (controllerDefNode.first_child().empty())
                            {
                                overrideStatus.value_type = static_cast<int>(Object::OverrideGearType::Manual);
                                if (!(controllerDefNode.attribute("number").empty()))  // version < 1.2 with number attribute
                                {
                                    // Skip range check since valid range is [-inf, inf]
                                    overrideStatus.number = static_cast<int>(strtod(parameters.ReadAttribute(controllerDefNode, "number")));
                                }
                                else if (!(controllerDefNode.attribute("value").empty()))  // version 1.1.1 with value attribute
                                {
                                    // Skip range check since valid range is [-inf, inf]
                                    overrideStatus.number = static_cast<int>(strtod(parameters.ReadAttribute(controllerDefNode, "value")));
                                    LOG("Unexpected Gear attribute name, change value to number, Accepting this time");
                                }
                                else
                                {
                                    LOG("Unexpected OverrideControllerValueAction subelement: %s", controllerDefNode.name());
                                    delete override_action;
                                    return 0;
                                }
                            }
                            // version 1.2
                            else
                            {
                                if (controllerDefNode.first_child().name() == std::string("AutomaticGear"))
                                {
                                    int number;
                                    overrideStatus.value_type = static_cast<int>(Object::OverrideGearType::Automatic);
                                    std::string number_str    = parameters.ReadAttribute(controllerDefNode.first_child(), "gear");
                                    if (number_str == std::string("r"))
                                    {
                                        number = -1;
                                    }
                                    else if (number_str == std::string("p"))
                                    {
                                        number = 1;
                                    }
                                    else if (number_str == std::string("n"))
                                    {
                                        number = 0;
                                    }
                                    else if (number_str == std::string("d"))
                                    {
                                        number = 2;
                                    }
                                    else
                                    {
                                        LOG("Unexpected AutomaticGear number: %s", number_str.c_str());
                                        delete override_action;
                                        return 0;
                                    }

                                    // Range check was done above
                                    overrideStatus.number = number;
                                }
                                // version 1.2 with ManualGear attribute
                                else if (controllerDefNode.first_child().name() == std::string("ManualGear"))
                                {
                                    overrideStatus.value_type = static_cast<int>(Object::OverrideGearType::Manual);
                                    // Skip range check since valid range is [-inf, inf]
                                    overrideStatus.number = strtoi(parameters.ReadAttribute(controllerDefNode.first_child(), "number"));
                                }
                                else
                                {
                                    LOG("Unexpected Gear child element: %s", controllerDefNode.first_child().name());
                                    delete override_action;
                                    return 0;
                                }
                                if (!verFromMinor2)
                                {
                                    LOG("Unexpected Gear element in version %d.%d, introduced in OSC 1.2", GetVersionMajor(), GetVersionMinor());
                                }
                            }
                        }
                        else
                        {
                            LOG("Unexpected OverrideControllerValueAction subelement: %s", controllerDefNode.name());
                            delete override_action;
                            return 0;
                        }

                        override_action->AddOverrideStatus(overrideStatus);
                    }

                    action = override_action;
                }
                else if (controllerChild.name() == std::string("ActivateControllerAction"))
                {
                    if (GetVersionMajor() == 1 && GetVersionMinor() == 0)
                    {
                        LOG("In OSC 1.0 ActivateControllerAction should be placed under PrivateAction. Accepting anyway.");
                    }

                    std::string ctrl_name;
                    if (GetVersionMinor() >= 3)
                    {
                        ctrl_name = parameters.ReadAttribute(controllerChild, "objectControllerRef");
                    }

                    ActivateControllerAction *activateControllerAction = parseActivateControllerAction(controllerChild, parent);
                    activateControllerAction->ctrl_name_               = ctrl_name;

                    action = activateControllerAction;
                }
                else
                {
                    LOG("Unexpected ControllerAction subelement: %s", controllerChild.name());
                }
            }
        }
        else if (actionChild.name() == std::string("VisibilityAction"))
        {
            bool graphics = true;
            bool traffic  = true;
            bool sensors  = true;

            // Find attributes
            std::string attributeString = parameters.ReadAttribute(actionChild, "graphics");
            if (attributeString == "false" || attributeString == "False")
            {
                graphics = false;
            }

            attributeString = parameters.ReadAttribute(actionChild, "traffic");
            if (attributeString == "false" || attributeString == "False")
            {
                traffic = false;
            }

            attributeString = parameters.ReadAttribute(actionChild, "sensors");
            if (attributeString == "false" || attributeString == "False")
            {
                sensors = false;
            }

            VisibilityAction *visAction = new VisibilityAction(parent);
            visAction->graphics_        = graphics;
            visAction->traffic_         = traffic;
            visAction->sensors_         = sensors;

            action = visAction;
        }
        else
        {
            throw std::runtime_error("Action is not supported: " + std::string(actionChild.name()));
        }
    }

    if (action != 0)
    {
        if (actionNode.parent().attribute("name"))
        {
            action->SetName(parameters.ReadAttribute(actionNode.parent(), "name"));
        }
        else
        {
            action->SetName("no name");
        }
        action->object_ = object;

        action->SetScenarioEngine(scenarioEngine_);
    }

    return action;
}

void ScenarioReader::parseInit(Init &init)
{
    pugi::xml_node actionsNode = osc_root_.child("Storyboard").child("Init").child("Actions");

    for (pugi::xml_node actionsChild = actionsNode.first_child(); actionsChild; actionsChild = actionsChild.next_sibling())
    {
        std::string actionsChildName(actionsChild.name());

        if (actionsChildName == "GlobalAction")
        {
            LOG("Parsing global action %s", actionsChild.first_child().name());
            OSCGlobalAction *action = parseOSCGlobalAction(actionsChild, nullptr);
            if (action != 0)
            {
                action->SetName("Init " + std::string(actionsChild.first_child().name()));
                init.global_action_.push_back(action);
            }
        }
        else if (actionsChildName == "UserDefined")
        {
            LOG("Init %s is not implemented", actionsChildName.c_str());
        }
        else if (actionsChildName == "Private")
        {
            Object *entityRef;

            entityRef     = ResolveObjectReference(parameters.ReadAttribute(actionsChild, "entityRef"));
            bool teleport = false;
            if (entityRef != NULL)
            {
                for (pugi::xml_node privateChild = actionsChild.first_child(); privateChild; privateChild = privateChild.next_sibling())
                {
                    // Assume children are PrivateActions
                    OSCPrivateAction *action = parseOSCPrivateAction(privateChild, entityRef, nullptr);
                    if (action != 0)
                    {
                        action->SetName("Init " + entityRef->name_ + " " + privateChild.first_child().name());

                        if (action->action_type_ == OSCPrivateAction::ActionType::TELEPORT)
                        {
                            teleport = true;
                        }
                        else if (teleport == false && action->action_type_ == OSCPrivateAction::ActionType::ACTIVATE_CONTROLLER)
                        {
                            LOG("WARNING: Controller activated before positioning (TeleportAction) the entity %s", entityRef->GetName().c_str());
                        }

                        init.private_action_.push_back(action);
                        if (entityRef)
                        {
                            entityRef->initActions_.push_back(action);
                        }
                    }
                }
                entities_->activateObject(entityRef);
            }
        }
    }

    // Now, potentially the list of init actions needs to be sorted based on dependencies
    for (size_t i = 1; i < init.private_action_.size(); i++)
    {
        // If relative actions depends on earlier action, switch position
        for (size_t j = 0; j < i; j++)
        {
            roadmanager::Position *pos = 0;
            if (init.private_action_[j]->action_type_ == OSCPrivateAction::ActionType::TELEPORT)
            {
                TeleportAction *action = static_cast<TeleportAction *>(init.private_action_[j]);
                if (action->position_->GetType() == roadmanager::Position::PositionType::RELATIVE_LANE)
                {
                    pos = action->position_->GetRelativePosition();
                }
                else if (action->position_->GetType() == roadmanager::Position::PositionType::RELATIVE_OBJECT)
                {
                    pos = action->position_->GetRelativePosition();
                }
            }
            if (pos == &init.private_action_[i]->object_->pos_)
            {
                // swap places
                OSCPrivateAction *tmp_action = init.private_action_[i];
                init.private_action_[i]      = init.private_action_[j];
                init.private_action_[j]      = tmp_action;
            }
        }
    }
}

static OSCCondition::ConditionEdge ParseConditionEdge(std::string edge)
{
    if (edge == "rising")
    {
        return OSCCondition::ConditionEdge::RISING;
    }
    else if (edge == "falling")
    {
        return OSCCondition::ConditionEdge::FALLING;
    }
    else if (edge == "risingOrFalling")
    {
        return OSCCondition::ConditionEdge::RISING_OR_FALLING;
    }
    else if (edge == "none")
    {
        return OSCCondition::ConditionEdge::NONE;
    }
    else
    {
        LOG("Unsupported edge: %s", edge.c_str());
    }

    return OSCCondition::ConditionEdge::UNDEFINED;
}

static Rule ParseRule(std::string rule)
{
    if (rule == "greaterThan")
    {
        return Rule::GREATER_THAN;
    }
    else if (rule == "greaterOrEqual")
    {
        return Rule::GREATER_OR_EQUAL;
    }
    else if (rule == "lessThan")
    {
        return Rule::LESS_THAN;
    }
    else if (rule == "lessOrEqual")
    {
        return Rule::LESS_OR_EQUAL;
    }
    else if (rule == "equalTo")
    {
        return Rule::EQUAL_TO;
    }
    else if (rule == "notEqualTo")
    {
        return Rule::NOT_EQUAL_TO;
    }
    else
    {
        LOG("Invalid or missing rule %s", rule.c_str());
    }

    return Rule::UNDEFINED_RULE;
}

static Direction ParseDirection(std::string direction)
{
    if (direction == "longitudinal")
    {
        return Direction::LONGITUDINAL;
    }
    else if (direction == "lateral")
    {
        return Direction::LATERAL;
    }
    else if (direction == "vertical")
    {
        return Direction::VERTICAL;
    }
    else
    {
        LOG("Invalid or missing direction %s", direction.c_str());
        return Direction::UNDEFINED_DIRECTION;
    }

    return Direction::UNDEFINED_DIRECTION;
}

static TrigByState::CondElementState ParseState(std::string state)
{
    if (state == "startTransition")
    {
        return TrigByState::CondElementState::START_TRANSITION;
    }
    else if (state == "endTransition")
    {
        return TrigByState::CondElementState::END_TRANSITION;
    }
    else if (state == "stopTransition")
    {
        return TrigByState::CondElementState::STOP_TRANSITION;
    }
    else if (state == "skipTransition")
    {
        return TrigByState::CondElementState::SKIP_TRANSITION;
    }
    else if (state == "completeState")
    {
        return TrigByState::CondElementState::COMPLETE;
    }
    else if (state == "runningState")
    {
        return TrigByState::CondElementState::RUNNING;
    }
    else if (state == "standbyState")
    {
        return TrigByState::CondElementState::STANDBY;
    }
    else
    {
        LOG("Invalid state %s", state.c_str());
    }

    return TrigByState::CondElementState::UNDEFINED_ELEMENT_STATE;
}

static StoryBoardElement::ElementType ParseElementType(std::string element_type)
{
    if (element_type == "story")
    {
        return StoryBoardElement::ElementType::STORY;
    }
    else if (element_type == "act")
    {
        return StoryBoardElement::ElementType::ACT;
    }
    else if (element_type == "maneuver")
    {
        return StoryBoardElement::ElementType::MANEUVER;
    }
    else if (element_type == "maneuverGroup")
    {
        return StoryBoardElement::ElementType::MANEUVER_GROUP;
    }
    else if (element_type == "event")
    {
        return StoryBoardElement::ElementType::EVENT;
    }
    else if (element_type == "action")
    {
        return StoryBoardElement::ElementType::ACTION;
    }
    else
    {
        LOG("Invalid or unsupported element type %s", element_type.c_str());
    }

    return StoryBoardElement::ElementType::UNDEFINED_ELEMENT_TYPE;
}
// ------------------------------------------
OSCCondition *ScenarioReader::parseOSCCondition(pugi::xml_node conditionNode)
{
    std::string condition_type;

    OSCCondition *condition = 0;

    for (pugi::xml_node conditionChild = conditionNode.first_child(); conditionChild; conditionChild = conditionChild.next_sibling())
    {
        std::string conditionChildName(conditionChild.name());
        if (conditionChildName == "ByEntityCondition")
        {
            pugi::xml_node entity_condition = conditionChild.child("EntityCondition");
            if (entity_condition == NULL)
            {
                throw std::runtime_error("Missing EntityCondition");
            }
            else
            {
                for (pugi::xml_node condition_node = entity_condition.first_child(); condition_node; condition_node = condition_node.next_sibling())
                {
                    condition_type = condition_node.name();
                    if (condition_type == "TimeHeadwayCondition")
                    {
                        TrigByTimeHeadway *trigger = new TrigByTimeHeadway;
                        trigger->object_           = ResolveObjectReference(parameters.ReadAttribute(condition_node, "entityRef"));

                        std::string freespace_str = parameters.ReadAttribute(condition_node, "freespace");
                        if ((freespace_str == "true") || (freespace_str == "1"))
                        {
                            trigger->freespace_ = true;
                        }
                        else
                        {
                            trigger->freespace_ = false;
                        }

                        trigger->cs_          = ParseCoordinateSystem(condition_node, roadmanager::CoordinateSystem::CS_UNDEFINED);
                        trigger->relDistType_ = ParseRelativeDistanceType(condition_node, roadmanager::RelativeDistanceType::REL_DIST_UNDEFINED);

                        if (trigger->cs_ == roadmanager::CoordinateSystem::CS_UNDEFINED &&
                            trigger->relDistType_ == roadmanager::RelativeDistanceType::REL_DIST_UNDEFINED)
                        {
                            // look for v1.0 attribute alongroute
                            std::string along_route_str = parameters.ReadAttribute(condition_node, "alongRoute");
                            if (!along_route_str.empty())
                            {
                                if (GetVersionMajor() == 1 && GetVersionMinor() == 1)
                                {
                                    LOG("alongRoute attribute is depricated from v1.1. Reading it anyway.");
                                }
                                if ((along_route_str == "true") || (along_route_str == "1"))
                                {
                                    trigger->cs_ = roadmanager::CoordinateSystem::CS_ROAD;
                                }
                            }
                        }

                        if (trigger->cs_ == roadmanager::CoordinateSystem::CS_UNDEFINED)
                        {
                            // Set default value
                            trigger->cs_ = roadmanager::CoordinateSystem::CS_ENTITY;
                        }

                        if (trigger->relDistType_ == roadmanager::RelativeDistanceType::REL_DIST_UNDEFINED)
                        {
                            // Set default value
                            trigger->relDistType_ = roadmanager::RelativeDistanceType::REL_DIST_EUCLIDIAN;
                        }

                        trigger->value_ = strtod(parameters.ReadAttribute(condition_node, "value"));
                        trigger->rule_  = ParseRule(parameters.ReadAttribute(condition_node, "rule"));

                        condition = trigger;
                    }
                    else if (condition_type == "TimeToCollisionCondition")
                    {
                        TrigByTimeToCollision *trigger = new TrigByTimeToCollision;

                        pugi::xml_node target      = condition_node.child("TimeToCollisionConditionTarget");
                        pugi::xml_node targetChild = target.first_child();
                        std::string    targetChildName(targetChild.name());
                        if (targetChildName == "Position")
                        {
                            trigger->position_.reset(parseOSCPosition(targetChild));
                        }
                        else if (targetChildName == "EntityRef")
                        {
                            trigger->object_ = ResolveObjectReference(parameters.ReadAttribute(targetChild, "entityRef"));
                        }
                        else
                        {
                            LOG("Unexpected target type: %s", targetChildName.c_str());
                            return 0;
                        }

                        std::string freespace_str = parameters.ReadAttribute(condition_node, "freespace");
                        if ((freespace_str == "true") || (freespace_str == "1"))
                        {
                            trigger->freespace_ = true;
                        }
                        else
                        {
                            trigger->freespace_ = false;
                        }

                        trigger->cs_          = ParseCoordinateSystem(condition_node, roadmanager::CoordinateSystem::CS_UNDEFINED);
                        trigger->relDistType_ = ParseRelativeDistanceType(condition_node, roadmanager::RelativeDistanceType::REL_DIST_UNDEFINED);

                        if (trigger->cs_ == roadmanager::CoordinateSystem::CS_UNDEFINED &&
                            trigger->relDistType_ == roadmanager::RelativeDistanceType::REL_DIST_UNDEFINED)
                        {
                            // look for v1.0 attribute alongroute
                            std::string along_route_str = parameters.ReadAttribute(condition_node, "alongRoute");
                            if (!along_route_str.empty())
                            {
                                if (GetVersionMajor() == 1 && GetVersionMinor() == 1)
                                {
                                    LOG("alongRoute attribute is depricated from v1.1. Reading it anyway.");
                                }
                                if ((along_route_str == "true") || (along_route_str == "1"))
                                {
                                    trigger->cs_ = roadmanager::CoordinateSystem::CS_ROAD;
                                }
                            }
                        }

                        if (trigger->cs_ == roadmanager::CoordinateSystem::CS_UNDEFINED)
                        {
                            // Set default value
                            trigger->cs_ = roadmanager::CoordinateSystem::CS_ENTITY;
                        }

                        if (trigger->relDistType_ == roadmanager::RelativeDistanceType::REL_DIST_UNDEFINED)
                        {
                            // Set default value
                            trigger->relDistType_ = roadmanager::RelativeDistanceType::REL_DIST_EUCLIDIAN;
                        }

                        trigger->value_ = strtod(parameters.ReadAttribute(condition_node, "value"));
                        trigger->rule_  = ParseRule(parameters.ReadAttribute(condition_node, "rule"));

                        condition = trigger;
                    }
                    else if (condition_type == "ReachPositionCondition")
                    {
                        TrigByReachPosition *trigger = new TrigByReachPosition;

                        trigger->tolerance_ = strtod(parameters.ReadAttribute(condition_node, "tolerance", true));

                        // Read position
                        pugi::xml_node pos_node = condition_node.child("Position");
                        trigger->position_.reset(parseOSCPosition(pos_node));
                        if (trigger->position_ && !pos_node.first_child().child("Orientation").empty())
                        {
                            trigger->checkOrientation_ = true;
                        }

                        condition = trigger;
                    }
                    else if (condition_type == "RelativeDistanceCondition")
                    {
                        TrigByRelativeDistance *trigger = new TrigByRelativeDistance;
                        trigger->object_                = ResolveObjectReference(parameters.ReadAttribute(condition_node, "entityRef"));

                        std::string freespace_str = parameters.ReadAttribute(condition_node, "freespace");
                        if ((freespace_str == "true") || (freespace_str == "1"))
                        {
                            trigger->freespace_ = true;
                        }
                        else
                        {
                            trigger->freespace_ = false;
                        }

                        trigger->cs_          = ParseCoordinateSystem(condition_node, roadmanager::CoordinateSystem::CS_ENTITY);
                        trigger->relDistType_ = ParseRelativeDistanceType(condition_node, roadmanager::RelativeDistanceType::REL_DIST_EUCLIDIAN);
                        trigger->value_       = strtod(parameters.ReadAttribute(condition_node, "value"));
                        trigger->rule_        = ParseRule(parameters.ReadAttribute(condition_node, "rule"));

                        condition = trigger;
                    }
                    else if (condition_type == "CollisionCondition")
                    {
                        TrigByCollision *trigger = new TrigByCollision;

                        pugi::xml_node by_type = condition_node.child("ByType");
                        if (by_type)
                        {
                            std::string type_str = parameters.ReadAttribute(by_type, "type");
                            if (type_str == "pedestrian")
                            {
                                trigger->type_ = Object::Type::PEDESTRIAN;
                            }
                            else if (type_str == "vehicle")
                            {
                                trigger->type_ = Object::Type::VEHICLE;
                            }
                            else if (type_str == "miscellaneous")
                            {
                                trigger->type_ = Object::Type::MISC_OBJECT;
                            }
                            else
                            {
                                throw std::runtime_error(std::string("Unexpected ObjectType in CollisionCondition: ") + type_str);
                            }
                        }
                        else  // Assume EntityRef
                        {
                            pugi::xml_node target = condition_node.child("EntityRef");
                            trigger->object_      = ResolveObjectReference(parameters.ReadAttribute(target, "entityRef"));
                            trigger->type_        = Object::Type::TYPE_NONE;
                        }
                        trigger->storyBoard_ = story_board_;

                        condition = trigger;
                    }
                    else if (condition_type == "DistanceCondition")
                    {
                        TrigByDistance *trigger = new TrigByDistance;

                        // Read position
                        pugi::xml_node pos_node = condition_node.child("Position");

                        trigger->position_.reset(parseOSCPosition(pos_node));

                        std::string freespace_str = parameters.ReadAttribute(condition_node, "freespace");
                        if ((freespace_str == "true") || (freespace_str == "1"))
                        {
                            trigger->freespace_ = true;
                        }
                        else
                        {
                            trigger->freespace_ = false;
                        }

                        trigger->cs_          = ParseCoordinateSystem(condition_node, roadmanager::CoordinateSystem::CS_UNDEFINED);
                        trigger->relDistType_ = ParseRelativeDistanceType(condition_node, roadmanager::RelativeDistanceType::REL_DIST_UNDEFINED);

                        if (trigger->cs_ == roadmanager::CoordinateSystem::CS_UNDEFINED &&
                            trigger->relDistType_ == roadmanager::RelativeDistanceType::REL_DIST_UNDEFINED)
                        {
                            // look for v1.0 attribute alongroute
                            std::string along_route_str = parameters.ReadAttribute(condition_node, "alongRoute");
                            if (!along_route_str.empty())
                            {
                                if (GetVersionMajor() == 1 && GetVersionMinor() == 1)
                                {
                                    LOG("alongRoute attribute is depricated from v1.1. Reading it anyway.");
                                }
                                if ((along_route_str == "true") || (along_route_str == "1"))
                                {
                                    trigger->cs_ = roadmanager::CoordinateSystem::CS_ROAD;
                                }
                            }
                        }

                        if (trigger->cs_ == roadmanager::CoordinateSystem::CS_UNDEFINED)
                        {
                            // Set default value
                            trigger->cs_ = roadmanager::CoordinateSystem::CS_ENTITY;
                        }

                        if (trigger->relDistType_ == roadmanager::RelativeDistanceType::REL_DIST_UNDEFINED)
                        {
                            // Set default value
                            trigger->relDistType_ = roadmanager::RelativeDistanceType::REL_DIST_EUCLIDIAN;
                        }

                        trigger->value_ = strtod(parameters.ReadAttribute(condition_node, "value"));
                        trigger->rule_  = ParseRule(parameters.ReadAttribute(condition_node, "rule"));

                        condition = trigger;
                    }
                    else if (condition_type == "TraveledDistanceCondition")
                    {
                        TrigByTraveledDistance *trigger = new TrigByTraveledDistance;

                        trigger->value_ = strtod(parameters.ReadAttribute(condition_node, "value"));

                        condition = trigger;
                    }
                    else if (condition_type == "EndOfRoadCondition")
                    {
                        TrigByEndOfRoad *trigger = new TrigByEndOfRoad;

                        trigger->duration_ = strtod(parameters.ReadAttribute(condition_node, "duration"));

                        condition = trigger;
                    }
                    else if (condition_type == "OffroadCondition")
                    {
                        TrigByOffRoad *trigger = new TrigByOffRoad;

                        trigger->duration_ = strtod(parameters.ReadAttribute(condition_node, "duration"));

                        condition = trigger;
                    }
                    else if (condition_type == "StandStillCondition")
                    {
                        TrigByStandStill *trigger = new TrigByStandStill;

                        trigger->duration_ = strtod(parameters.ReadAttribute(condition_node, "duration"));

                        condition = trigger;
                    }
                    else if (condition_type == "AccelerationCondition")
                    {
                        TrigByAcceleration *trigger = new TrigByAcceleration;

                        trigger->value_ = strtod(parameters.ReadAttribute(condition_node, "value"));
                        trigger->rule_  = ParseRule(parameters.ReadAttribute(condition_node, "rule"));
                        if (!condition_node.attribute("direction").empty())
                        {
                            trigger->direction_ = ParseDirection(parameters.ReadAttribute(condition_node, "direction"));
                        }

                        condition = trigger;
                    }
                    else if (condition_type == "SpeedCondition")
                    {
                        TrigBySpeed *trigger = new TrigBySpeed;

                        trigger->value_ = strtod(parameters.ReadAttribute(condition_node, "value"));
                        trigger->rule_  = ParseRule(parameters.ReadAttribute(condition_node, "rule"));
                        if (!condition_node.attribute("direction").empty())
                        {
                            trigger->direction_ = ParseDirection(parameters.ReadAttribute(condition_node, "direction"));
                        }

                        condition = trigger;
                    }
                    else if (condition_type == "RelativeSpeedCondition")
                    {
                        TrigByRelativeSpeed *trigger = new TrigByRelativeSpeed;

                        trigger->object_ = ResolveObjectReference(parameters.ReadAttribute(condition_node, "entityRef"));
                        trigger->value_  = strtod(parameters.ReadAttribute(condition_node, "value"));
                        trigger->rule_   = ParseRule(parameters.ReadAttribute(condition_node, "rule"));
                        if (!condition_node.attribute("direction").empty())
                        {
                            trigger->direction_ = ParseDirection(parameters.ReadAttribute(condition_node, "direction"));
                        }

                        condition = trigger;
                    }
                    else if (condition_type == "RelativeClearanceCondition")
                    {
                        std::string strTemp;

                        TrigByRelativeClearance *trigger = new TrigByRelativeClearance;

                        if (!parameters.ReadAttribute(condition_node, "distanceForward").empty())
                        {  // populate only if available else use default
                            trigger->distanceForward_ = strtod(parameters.ReadAttribute(condition_node, "distanceForward"));
                            if (trigger->distanceForward_ < 0)
                            {
                                trigger->distanceForward_ = abs(trigger->distanceForward_);
                                LOG("Negative value not allowed as distanceForward in RelativeClearanceCondition. Converted as positive value");
                            }
                        }

                        if (!parameters.ReadAttribute(condition_node, "distanceBackward").empty())
                        {  // populate only if available else use default
                            trigger->distanceBackward_ = strtod(parameters.ReadAttribute(condition_node, "distanceBackward"));
                            if (trigger->distanceBackward_ < 0)
                            {
                                trigger->distanceBackward_ = abs(trigger->distanceBackward_);
                                LOG("Negative value not allowed as distanceBackward in RelativeClearanceCondition. Converted as positive value");
                            }
                        }

                        if (!parameters.ReadAttribute(condition_node, "freeSpace").empty())
                        {
                            strTemp             = parameters.ReadAttribute(condition_node, "freeSpace");
                            trigger->freeSpace_ = strTemp == "true" ? true : false;
                        }
                        else if (!parameters.ReadAttribute(condition_node, "freespace").empty())
                        {
                            strTemp             = parameters.ReadAttribute(condition_node, "freespace");
                            trigger->freeSpace_ = strTemp == "true" ? true : false;
                        }
                        else
                        {  // Provide warring and use default value
                            LOG("FreeSpace is mandatory attribute in RelativeClearanceCondition. Anyway setting it false");
                        }

                        if (!parameters.ReadAttribute(condition_node, "oppositeLanes").empty())
                        {
                            strTemp                 = parameters.ReadAttribute(condition_node, "oppositeLanes");
                            trigger->oppositeLanes_ = strTemp == "true" ? true : false;
                        }
                        else
                        {  // Provide warring and use default value
                            LOG("OppositeLanes is mandatory attribute in RelativeClearanceCondition. Anyway setting it false");
                        }

                        Object *object_;
                        for (pugi::xml_node relClearanceChild = condition_node.first_child(); relClearanceChild;
                             relClearanceChild                = relClearanceChild.next_sibling())
                        {
                            if (std::strcmp(relClearanceChild.name(), "EntityRef") == 0)
                            {  // populate all entity
                                object_ = ResolveObjectReference(parameters.ReadAttribute(relClearanceChild, "entityRef"));
                                trigger->objects_.push_back(object_);
                            }
                            else if (std::strcmp(relClearanceChild.name(), "RelativeLaneRange") == 0)
                            {
                                if (!parameters.ReadAttribute(relClearanceChild, "to").empty())
                                {  // populate only if available else use default
                                    trigger->to_ = strtoi(parameters.ReadAttribute(relClearanceChild, "to"));
                                }

                                if (!parameters.ReadAttribute(relClearanceChild, "from").empty())
                                {  // populate only if available else use default
                                    trigger->from_ = strtoi(parameters.ReadAttribute(relClearanceChild, "from"));
                                }
                            }
                            else
                            {
                                LOG("Unexpected element %s in RelativeClearanceCondition ", relClearanceChild.name());
                            }
                        }

                        trigger->storyBoard_ = story_board_;

                        condition = trigger;
                    }
                    else
                    {
                        LOG_AND_QUIT("Entity condition %s not supported", condition_type.c_str());
                    }
                }
            }

            pugi::xml_node triggering_entities = conditionChild.child("TriggeringEntities");
            if (triggering_entities != NULL)
            {
                TrigByEntity *trigger = static_cast<TrigByEntity *>(condition);

                std::string trig_ent_rule = parameters.ReadAttribute(triggering_entities, "triggeringEntitiesRule");
                if (trig_ent_rule == "any")
                {
                    trigger->triggering_entity_rule_ = TrigByEntity::TriggeringEntitiesRule::ANY;
                }
                else if (trig_ent_rule == "all")
                {
                    trigger->triggering_entity_rule_ = TrigByEntity::TriggeringEntitiesRule::ALL;
                }
                else
                {
                    LOG("Invalid triggering entity type: %s", trig_ent_rule.c_str());
                }

                for (pugi::xml_node triggeringEntitiesChild = triggering_entities.first_child(); triggeringEntitiesChild;
                     triggeringEntitiesChild                = triggeringEntitiesChild.next_sibling())
                {
                    std::string triggeringEntitiesChildName(triggeringEntitiesChild.name());

                    if (triggeringEntitiesChildName == "EntityRef")
                    {
                        TrigByEntity::Entity entity;
                        entity.object_ = ResolveObjectReference(parameters.ReadAttribute(triggeringEntitiesChild, "entityRef"));
                        if (entity.object_ == 0)
                        {
                            throw std::runtime_error("Failed to find referenced entity - see log");
                        }
                        trigger->triggering_entities_.entity_.push_back(entity);
                    }
                }
            }
            else
            {
                throw std::runtime_error("Missing TriggeringEntities");
            }
        }
        else if (conditionChildName == "ByValueCondition")
        {
            for (pugi::xml_node byValueChild = conditionChild.first_child(); byValueChild; byValueChild = byValueChild.next_sibling())
            {
                condition_type = byValueChild.name();
                if (condition_type == "SimulationTimeCondition")
                {
                    TrigBySimulationTime *trigger = new TrigBySimulationTime;
                    trigger->value_               = strtod(parameters.ReadAttribute(byValueChild, "value"));
                    trigger->rule_                = ParseRule(parameters.ReadAttribute(byValueChild, "rule"));
                    condition                     = trigger;
                }
                else if (condition_type == "ParameterCondition")
                {
                    TrigByParameter *trigger = new TrigByParameter;
                    trigger->name_           = parameters.ReadAttribute(byValueChild, "parameterRef");
                    trigger->value_          = parameters.ReadAttribute(byValueChild, "value");
                    trigger->rule_           = ParseRule(parameters.ReadAttribute(byValueChild, "rule"));
                    trigger->parameters_     = &parameters;
                    condition                = trigger;
                }
                else if (condition_type == "VariableCondition")
                {
                    TrigByVariable *trigger = new TrigByVariable;
                    trigger->name_          = variables.ReadAttribute(byValueChild, "variableRef");
                    trigger->value_         = variables.ReadAttribute(byValueChild, "value");
                    trigger->rule_          = ParseRule(variables.ReadAttribute(byValueChild, "rule"));
                    trigger->variables_     = &variables;
                    condition               = trigger;
                }
                else if (condition_type == "StoryboardElementStateCondition")
                {
                    TrigByState                 *trigger = new TrigByState();
                    StoryBoardElementTriggerInfo info;

                    info.element_name = parameters.ReadAttribute(byValueChild, "storyboardElementRef");
                    info.type         = ParseElementType(parameters.ReadAttribute(byValueChild, "storyboardElementType"));
                    info.state        = ParseState(parameters.ReadAttribute(byValueChild, "state"));
                    info.element      = nullptr;
                    info.condition    = trigger;

                    // register this trigger for later resolving storyboard element references
                    storyboard_element_triggers.push_back(info);

                    condition = trigger;
                }
                else
                {
                    LOG("WARNING: ByValueCondition %s not supported yet. Ignoring.", condition_type.c_str());
                }
            }
        }
        else
        {
            throw std::runtime_error("Unsupported condition: " + conditionChildName);
        }
    }

    if (condition == 0)
    {
        return 0;
    }
    condition->name_ = parameters.ReadAttribute(conditionNode, "name");

    if (condition->name_.empty())
    {
        // No name, set a dummy name
        condition->name_ = "no name " + condition_type;
    }

    if (conditionNode.attribute("delay") != NULL)
    {
        condition->delay_ = strtod(parameters.ReadAttribute(conditionNode, "delay"));
    }
    else
    {
        LOG("Attribute \"delay\" missing");
    }

    std::string edge_str = parameters.ReadAttribute(conditionNode, "conditionEdge");
    if (edge_str != "")
    {
        condition->edge_ = ParseConditionEdge(edge_str);
    }
    else
    {
        condition->edge_ = OSCCondition::ConditionEdge::NONE;
    }

    return condition;
}

Trigger *ScenarioReader::parseTrigger(pugi::xml_node triggerNode, bool defaultValue)
{
    Trigger *trigger = new Trigger(defaultValue);

    for (pugi::xml_node cgNode = triggerNode.first_child(); cgNode; cgNode = cgNode.next_sibling())
    {
        ConditionGroup *condition_group = new ConditionGroup;

        for (pugi::xml_node cNode = cgNode.first_child(); cNode; cNode = cNode.next_sibling())
        {
            OSCCondition *condition = parseOSCCondition(cNode);
            if (condition != nullptr)
            {
                condition_group->condition_.push_back(condition);
            }
        }

        trigger->conditionGroup_.push_back(condition_group);
    }

    return trigger;
}

void ScenarioReader::parseOSCManeuver(Maneuver *maneuver, pugi::xml_node maneuverNode, ManeuverGroup *mGroup)
{
    maneuver->SetName(parameters.ReadAttribute(maneuverNode, "name"));

    for (pugi::xml_node maneuverChild = maneuverNode.first_child(); maneuverChild; maneuverChild = maneuverChild.next_sibling())
    {
        std::string maneuverChildName(maneuverChild.name());

        if (maneuverChildName == "ParameterDeclarations")
        {
            parameters.addParameterDeclarations(maneuverChild);
        }
        else if (maneuverChildName == "Event")
        {
            Event *event = new Event(maneuver);

            event->SetName(parameters.ReadAttribute(maneuverChild, "name"));

            std::string prio = parameters.ReadAttribute(maneuverChild, "priority");
            if (prio == "overwrite")
            {
                if (GetVersionMajor() == 1 && GetVersionMinor() > 1)
                {
                    LOG("Info: \"overwrite\" priority deprecated in v1.2. Use \"override\" instead. Accepting it anyway.");
                }
                event->priority_ = Event::Priority::OVERWRITE;
            }
            else if (prio == "override")
            {
                if (GetVersionMajor() == 1 && GetVersionMinor() < 2)
                {
                    LOG("Info: \"override\" priority was introduced in v1.2. Use \"overwrite\" for earlier versions. Accepting it anyway.");
                }
                event->priority_ = Event::Priority::OVERWRITE;
            }
            else if (prio == "skip")
            {
                event->priority_ = Event::Priority::SKIP;
            }
            else if (prio == "parallel")
            {
                event->priority_ = Event::Priority::PARALLEL;
            }
            else
            {
                LOG("Invalid priority: %s", prio.c_str());
            }

            if (parameters.ReadAttribute(maneuverChild, "maximumExecutionCount") != "")
            {
                event->max_num_executions_ = strtoi(parameters.ReadAttribute(maneuverChild, "maximumExecutionCount"));
            }
            else
            {
                event->max_num_executions_ = 1;  // 1 is default
            }

            bool event_added_to_objects = false;

            for (pugi::xml_node eventChild = maneuverChild.first_child(); eventChild; eventChild = eventChild.next_sibling())
            {
                std::string childName(eventChild.name());

                if (childName == "Action")
                {
                    for (pugi::xml_node actionChild = eventChild.first_child(); actionChild; actionChild = actionChild.next_sibling())
                    {
                        std::string actionChildName(actionChild.name());

                        if (actionChildName == "GlobalAction")
                        {
                            OSCGlobalAction *action = parseOSCGlobalAction(actionChild, event);
                            if (action != 0)
                            {
                                event->action_.push_back(static_cast<OSCAction *>(action));
                            }
                        }
                        else if (actionChildName == "UserDefinedAction")
                        {
                            OSCUserDefinedAction *action = parseOSCUserDefinedAction(actionChild, event);
                            if (action != nullptr)
                            {
                                event->action_.push_back(static_cast<OSCAction *>(action));
                            }
                        }
                        else if (actionChildName == "PrivateAction")
                        {
                            if (mGroup->actor_.size() == 0)
                            {
                                LOG_AND_QUIT(
                                    "PrivateAction %s missing actor(s). SelectTriggeringEntities feature not supported yet. Add actor(s) to ManeuverGroup.",
                                    parameters.ReadAttribute(eventChild, "name").c_str());
                            }

                            for (size_t i = 0; i < mGroup->actor_.size(); i++)
                            {
                                OSCPrivateAction *action = parseOSCPrivateAction(actionChild, mGroup->actor_[i]->object_, event);
                                if (action != 0)
                                {
                                    event->action_.push_back(static_cast<OSCAction *>(action));
                                    if (!event_added_to_objects)
                                    {
                                        mGroup->actor_[i]->object_->addEvent(event);
                                    }
                                }
                                else
                                {
                                    LOG("Failed to parse event %s - continue regardless", event->GetName().c_str());
                                }
                            }

                            event_added_to_objects = true;
                        }
                    }
                }
                else if (childName == "StartTrigger")
                {
                    event->start_trigger_ = parseTrigger(eventChild, true);

                    // Check and warn for start trigger with rising edge in combination with simulationTime == 0
                    for (size_t i = 0; i < event->start_trigger_->conditionGroup_.size(); i++)
                    {
                        for (size_t j = 0; j < event->start_trigger_->conditionGroup_[i]->condition_.size(); j++)
                        {
                            OSCCondition *cond = event->start_trigger_->conditionGroup_[i]->condition_[j];
                            if (cond->base_type_ == OSCCondition::ConditionType::BY_VALUE)
                            {
                                TrigByValue *trig = static_cast<TrigByValue *>(cond);
                                if (trig->type_ == TrigByValue::Type::SIMULATION_TIME && trig->edge_ != OSCCondition::NONE &&
                                    fabs((static_cast<TrigBySimulationTime *>((trig)))->value_) < SMALL_NUMBER)
                                {
                                    LOG("Warning: simulationTime = 0 condition used with edge \"%s\" which could be missed. Edge \"none\" is recommended.",
                                        trig->Edge2Str().c_str());
                                }
                            }
                        }
                    }
                }
                else
                {
                    LOG("%s not supported", childName.c_str());
                }
            }

            if (event->start_trigger_ == 0)
            {
                // Add a default (empty) trigger
                event->start_trigger_ = new Trigger(true);
            }
            maneuver->event_.push_back(event);
        }
    }
}

int ScenarioReader::parseStoryBoard(StoryBoard &storyBoard)
{
    story_board_                  = &storyBoard;
    pugi::xml_node storyBoardNode = osc_root_.child("Storyboard");
    pugi::xml_node storyNode      = storyBoardNode.child("Story");

    for (; storyNode; storyNode = storyNode.next_sibling("Story"))
    {
        std::string storyNodeName(storyNode.name());

        if (storyNodeName == "Story")
        {
            std::string name  = parameters.ReadAttribute(storyNode, "name", false);
            Story      *story = new Story(name, &storyBoard);
            storyBoard.story_.push_back(story);

            parameters.CreateRestorePoint();

            if (!strcmp(storyNode.first_child().name(), "ParameterDeclarations"))
            {
                parameters.addParameterDeclarations(storyNode.first_child());
            }

            for (pugi::xml_node storyChild = storyNode.child("Act"); storyChild; storyChild = storyChild.next_sibling("Act"))
            {
                std::string childName(storyChild.name());

                if (childName == "Act")
                {
                    Act *act = new Act(story);
                    story->act_.push_back(act);

                    act->SetName(parameters.ReadAttribute(storyChild, "name"));

                    for (pugi::xml_node actChild = storyChild.first_child(); actChild; actChild = actChild.next_sibling())
                    {
                        std::string actChildName(actChild.name());

                        if (actChildName == "ManeuverGroup")
                        {
                            ManeuverGroup *mGroup = new ManeuverGroup(act);
                            act->maneuverGroup_.push_back(mGroup);

                            if (parameters.ReadAttribute(actChild, "maximumExecutionCount") != "")
                            {
                                mGroup->max_num_executions_ = strtoi(parameters.ReadAttribute(actChild, "maximumExecutionCount"));
                            }
                            else
                            {
                                mGroup->max_num_executions_ = 1;  // 1 is Default
                            }

                            mGroup->SetName(parameters.ReadAttribute(actChild, "name"));

                            pugi::xml_node actors_node = actChild.child("Actors");
                            if (actors_node != NULL)
                            {
                                for (pugi::xml_node actorsChild = actors_node.first_child(); actorsChild; actorsChild = actorsChild.next_sibling())
                                {
                                    ManeuverGroup::Actor *actor = new ManeuverGroup::Actor;

                                    std::string actorsChildName(actorsChild.name());
                                    if (actorsChildName == "EntityRef")
                                    {
                                        if ((actor->object_ = ResolveObjectReference(parameters.ReadAttribute(actorsChild, "entityRef"))) == 0)
                                        {
                                            throw std::runtime_error(std::string("Failed to resolve entityRef ") +
                                                                     parameters.ReadAttribute(actorsChild, "entityRef"));
                                        }
                                    }
                                    else if (actorsChildName == "ByCondition")
                                    {
                                        LOG("Actor by condition - not implemented");
                                    }
                                    mGroup->actor_.push_back(actor);
                                }
                            }

                            for (pugi::xml_node catalog_n = actChild.child("CatalogReference"); catalog_n;
                                 catalog_n                = catalog_n.next_sibling("CatalogReference"))
                            {
                                // Maneuver catalog reference. The catalog entry is simply the maneuver XML node
                                parameters.CreateRestorePoint();
                                Entry *entry = ResolveCatalogReference(catalog_n);

                                if (entry == 0 || entry->root_ == 0)
                                {
                                    throw std::runtime_error("Failed to resolve catalog reference");
                                }

                                if (entry->type_ == CatalogType::CATALOG_MANEUVER)
                                {
                                    Maneuver *maneuver = new Maneuver(mGroup);
                                    mGroup->maneuver_.push_back(maneuver);

                                    // Make a new instance from catalog entry
                                    parseOSCManeuver(maneuver, entry->GetNode(), mGroup);
                                }
                                else
                                {
                                    throw std::runtime_error(std::string("Unexpected catalog type: ") + entry->GetTypeAsStr());
                                }

                                // Remove temporary parameters used for catalog reference
                                parameters.RestoreParameterDeclarations();
                            }

                            for (pugi::xml_node maneuver_n = actChild.child("Maneuver"); maneuver_n; maneuver_n = maneuver_n.next_sibling("Maneuver"))
                            {
                                if (maneuver_n != NULL)
                                {
                                    Maneuver *maneuver = new Maneuver(mGroup);
                                    mGroup->maneuver_.push_back(maneuver);

                                    parseOSCManeuver(maneuver, maneuver_n, mGroup);
                                }
                            }
                        }
                        else if (actChildName == "StartTrigger")
                        {
                            act->start_trigger_ = parseTrigger(actChild, true);
                        }
                        else if (actChildName == "StopTrigger")
                        {
                            act->stop_trigger_ = parseTrigger(actChild, false);
                        }
                    }
                }
            }
            parameters.RestoreParameterDeclarations();
        }
    }

    pugi::xml_node stopTrigger = storyBoardNode.child("StopTrigger");

    if (!stopTrigger.empty())
    {
        storyBoard.stop_trigger_ = parseTrigger(stopTrigger, false);
    }

    // Log parameter declarations
    parameters.Print("parameters");

    // Now when complete storyboard is parsed, resolve storyboard element triggers
    for (auto trigger_info : storyboard_element_triggers)
    {
        // locate storyboard element
        std::vector<StoryBoardElement *> elements = story_board_->FindChildByTypeAndName(trigger_info.type, trigger_info.element_name);

        if (elements.size() > 0)
        {
            if (elements.size() > 1)
            {
                LOG("Warning: Non unique element name: %s, picking first occurrence", trigger_info.element_name.c_str());
            }
            trigger_info.element                          = elements[0];
            trigger_info.condition->element_              = trigger_info.element;
            trigger_info.condition->target_element_state_ = trigger_info.state;
            trigger_info.element->AddTriggerRef(trigger_info.condition);
        }
        else
        {
            LOG_AND_QUIT("Error: StoryboardElement %s not found. Quit.", trigger_info.element_name.c_str());
        }
    }

    storyboard_element_triggers.clear();

    return 0;
}