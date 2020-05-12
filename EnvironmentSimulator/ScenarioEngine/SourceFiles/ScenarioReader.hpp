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
#include "RoadNetwork.hpp"
#include "Catalogs.hpp"
#include "Entities.hpp"
#include "Init.hpp"
#include "Story.hpp"
#include "OSCPosition.hpp"
#include "OSCProperties.hpp"
#include "pugixml.hpp"
#include "OSCGlobalAction.hpp"

#include <iostream>
#include <string>
#include <vector>

namespace scenarioengine
{

	class ScenarioReader
	{
	public:

		ScenarioReader(Entities *entities, Catalogs *catalogs) : objectCnt_(0), entities_(entities), catalogs_(catalogs), paramDeclarationsSize_(0) {}
		int loadOSCFile(const char * path);
		void loadOSCMem(const pugi::xml_document &xml_doch);

		int RegisterCatalogDirectory(pugi::xml_node catalogDirChild);

		// RoadNetwork
		void parseRoadNetwork(RoadNetwork &roadNetwork);
		void parseOSCFile(OSCFile &file, pugi::xml_node fileNode);
		roadmanager::Trajectory* parseTrajectory(pugi::xml_node node);

		// ParameterDeclarations
		void parseGlobalParameterDeclarations();

		// Catalogs
		void parseCatalogs();
		Catalog* LoadCatalog(std::string name);
		roadmanager::Route* parseOSCRoute(pugi::xml_node routeNode);
		void ParseOSCProperties(OSCProperties &properties, pugi::xml_node &xml_node);
		Vehicle* parseOSCVehicle(pugi::xml_node vehicleNode);
		Vehicle* createRandomOSCVehicle(std::string name);

		// Enitites
		int parseEntities();
		Entry* ResolveCatalogReference(pugi::xml_node node);
		Object* FindObjectByName(std::string name);

		// Storyboard - Init
		void parseInit(Init &init);
		OSCPrivateAction *parseOSCPrivateAction(pugi::xml_node actionNode, Object *object);
		OSCGlobalAction *parseOSCGlobalAction(pugi::xml_node actionNode);
		void parseOSCOrientation(OSCOrientation &orientation, pugi::xml_node orientationNode);
		OSCPosition *parseOSCPosition(pugi::xml_node positionNode);

		// Storyboard - Story
		OSCCondition *parseOSCCondition(pugi::xml_node conditionNode);
		Trigger* parseTrigger(pugi::xml_node triggerNode);
		//	void parseOSCConditionGroup(OSCConditionGroup *conditionGroup, pugi::xml_node conditionGroupNode);
		int parseStoryBoard(StoryBoard &storyBoard);
		void parseOSCManeuver(OSCManeuver *maneuver, pugi::xml_node maneuverNode, ManeuverGroup *mGroup);

		// Help functions
		std::string getParameter(OSCParameterDeclarations &parameterDeclarations, std::string name);
		void addParameter(std::string name, std::string value);

		std::string getScenarioFilename() { return oscFilename_; }
	
	private:
		pugi::xml_document doc_;
		OSCParameterDeclarations parameterDeclarations_;
		int objectCnt_;
		std::string oscFilename_;
		Entities *entities_;
		Catalogs *catalogs_;
		int paramDeclarationsSize_;  // original size, exluding added parameters
		std::vector<ParameterStruct> catalog_param_assignments;

		void parseParameterDeclarations(pugi::xml_node xml_node, OSCParameterDeclarations *pd);
		int ParseTransitionDynamics(pugi::xml_node node, OSCPrivateAction::TransitionDynamics& td);
		ConditionGroup* ParseConditionGroup(pugi::xml_node node);
		void addParameterDeclarations(pugi::xml_node xml_node);
		void RestoreParameterDeclarations();  // To what it was before addParameterDeclarations

		// Use always this method when reading attributes, it will resolve any variables
		std::string ReadAttribute(pugi::xml_node, std::string attribute, bool required = false);
	};

}
