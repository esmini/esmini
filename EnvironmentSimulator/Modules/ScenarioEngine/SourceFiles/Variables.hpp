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

#include <string.h>
#include "pugixml.hpp"
#include "OSCVariableDeclarations.hpp"
#include <vector>
#include <stack>

namespace scenarioengine
{
	#define VARIABLE_PREFIX '$'

	class Variables
	{
	public:
		Variables() {}
		std::stack<int> varDeclarationsSize_;  // original size first, then additional layered variable declarations
		std::vector<OSCVariableDeclarations::VariableStruct> catalog_var_assignments;
		OSCVariableDeclarations variableDeclarations_;

		// VariableDeclarations
		void parseGlobalVariableDeclarations(pugi::xml_node osc_root_);
		void parseVariableDeclarations(pugi::xml_node xml_node, OSCVariableDeclarations* pd);
		std::string getVariable(OSCVariableDeclarations& VariableDeclarations, std::string name);
		std::string getVariable(std::string name) { return getVariable(variableDeclarations_, name); }
		OSCVariableDeclarations::VariableStruct* getVariableEntry(std::string name);
		int setVariable(std::string name, std::string value);
		void addVariableDeclarations(pugi::xml_node xml_node);
		void CreateRestorePoint();
		void RestoreVariableDeclarations();  // To what it was before addVariableDeclarations

		int GetNumberOfVariables();
		const char* GetVariableName(int index, OSCVariableDeclarations::VariableType* type);
		int setVariableValue(std::string name, const void* value);
		int setVariableValueByString(std::string name, std::string value);
		int setVariableValue(std::string name, int value);
		int setVariableValue(std::string name, double value);
		int setVariableValue(std::string name, const char* value);
		int setVariableValue(std::string name, bool value);
		int getVariableValue(std::string name, void* value);
		int getVariableValueInt(std::string name, int& value);
		int getVariableValueDouble(std::string name, double& value);
		int getVariableValueString(std::string name, const char* &value);
		int getVariableValueBool(std::string name, bool& value);
		std::string getVariableValueAsString(std::string name);

		std::string ResolveVariablesInString(std::string str);

		// Use always this method when reading attributes, it will resolve any variables
		std::string ReadAttribute(pugi::xml_node, std::string attribute, bool required = false);

		// Will clear all Variable declarations and assignments
		void Clear();

		// Log current set of Variable names and values
		void Print();
	};
}