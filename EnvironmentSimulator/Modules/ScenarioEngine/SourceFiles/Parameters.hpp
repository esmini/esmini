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
#include "OSCParameterDeclarations.hpp"
#include <vector>
#include <stack>

namespace scenarioengine
{
#define PARAMETER_PREFIX '$'

    class Parameters
    {
    public:
        Parameters()
        {
        }
        std::stack<int> paramDeclarationsSize_;  // original size first, then additional layered parameter declarations
        std::vector<OSCParameterDeclarations::ParameterStruct> catalog_param_assignments;
        OSCParameterDeclarations                               parameterDeclarations_;

        // ParameterDeclarations
        void        parseGlobalParameterDeclarations(pugi::xml_node osc_root_);
        void        parseParameterDeclarations(pugi::xml_node xml_node, OSCParameterDeclarations* pd);
        std::string getParameter(OSCParameterDeclarations& parameterDeclarations, std::string name);
        std::string getParameter(std::string name)
        {
            return getParameter(parameterDeclarations_, name);
        }
        OSCParameterDeclarations::ParameterStruct* getParameterEntry(std::string name);
        int                                        setParameter(std::string name, std::string value);
        void                                       addParameterDeclarations(pugi::xml_node xml_node);
        void                                       CreateRestorePoint();
        void                                       RestoreParameterDeclarations();  // To what it was before addParameterDeclarations

        int         GetNumberOfParameters();
        const char* GetParameterName(int index, OSCParameterDeclarations::ParameterType* type);
        int         setParameterValue(std::string name, const void* value);
        int         setParameterValueByString(std::string name, std::string value);
        int         setParameterValue(std::string name, int value);
        int         setParameterValue(std::string name, double value);
        int         setParameterValue(std::string name, const char* value);
        int         setParameterValue(std::string name, bool value);
        int         getParameterValue(std::string name, void* value);
        int         getParameterValueInt(std::string name, int& value);
        int         getParameterValueDouble(std::string name, double& value);
        int         getParameterValueString(std::string name, const char*& value);
        int         getParameterValueBool(std::string name, bool& value);
        std::string getParameterValueAsString(std::string name);

        std::string ResolveParametersInString(std::string str);

        // Use always this method when reading attributes, it will resolve any variables
        std::string ReadAttribute(pugi::xml_node, std::string attribute, bool required = false);

        // bool CheckAttribute(pugi::xml_node, std::string attribute);

        // Will clear all parameter declarations and assignements
        void Clear();

        // Log current set of parameter names and values
        void Print(std::string type);
    };
}  // namespace scenarioengine