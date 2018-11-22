#pragma once
#include "RoadNetwork.hpp"
#include "Catalogs.hpp"
#include "Entities.hpp"
#include "Init.hpp"
#include "Story.hpp"
#include "pugixml.hpp"

#include <iostream>
#include <string>
#include <vector>

class ScenarioReader
{
public:
	ScenarioReader();
	int loadOSCFile(const char * path);

	// RoadNetwork
	void parseRoadNetwork(RoadNetwork &roadNetwork);
	void parseOSCFile(OSCFile &file, pugi::xml_node fileNode);

	// ParameterDeclaration
	void parseParameterDeclaration();

	// Catalogs
	void parseCatalogs(Catalogs &catalogs);
	roadmanager::Route* parseOSCRoute(pugi::xml_node routeNode, Catalogs *catalogs);

	// Enitites
	void parseEntities(Entities &entities);
	void parseOSCCatalogReference(OSCCatalogReference &catalogReference, pugi::xml_node catalogReferenceNode);
	Object* FindObjectByName(std::string name, Entities *entities);

	// Storyboard - Init
	void parseInit(Init &init, Entities *entities, Catalogs *catalogs);
	OSCPrivateAction *parseOSCPrivateAction(pugi::xml_node actionNode, Entities *entities, Object *object, Catalogs *catalogs);
	void parseOSCPosition(roadmanager::Position &position, pugi::xml_node positionNode, Catalogs *catalogs);

	// Storyboard - Story
	OSCCondition *parseOSCCondition(pugi::xml_node conditionNode, Entities *entities);
//	void parseOSCConditionGroup(OSCConditionGroup *conditionGroup, pugi::xml_node conditionGroupNode);
	void parseStory(std::vector<Story*> &storyVector, Entities *entities, Catalogs *catalogs);
	void parseOSCManeuver(OSCManeuver *maneuver, pugi::xml_node maneuverNode, Entities *entities, ActSequence *act_sequence, Catalogs *catalogs);

	// Help functions
	std::string getParameter(std::string name);
	void addParameter(std::string name, std::string value);

private:
	pugi::xml_document doc;
	OSCParameterDeclaration parameterDeclaration;
	int objectCnt;
	std::string oscFilename;

	// Use always this method when reading attributes, it will resolve any variables
	std::string ReadAttribute(pugi::xml_attribute attribute);
};

