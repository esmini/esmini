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
#include <map>

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
        void        parseGlobalParameterDeclarations(pugi::xml_node osc_root);
        void        parseParameterDeclarations(pugi::xml_node declarationsNode, OSCParameterDeclarations* pd);
        std::string getParameter(std::string name);

        // Mutable access deliberately excludes read-only instance snapshots.
        OSCParameterDeclarations::ParameterStruct*       getParameterEntry(std::string name);
        const OSCParameterDeclarations::ParameterStruct* getParameterEntryReadOnly(std::string name) const;
        OSCParameterDeclarations                         CaptureParameterDeclarations(pugi::xml_node declarations) const;
        void RegisterInstanceParameters(const std::string& name, const OSCParameterDeclarations& declarations);
        int  setParameter(std::string name, std::string value);
        void addParameterDeclarations(pugi::xml_node xml_node);
        void CreateRestorePoint();
        void RestoreParameterDeclarations();  // To what it was before addParameterDeclarations

        int         GetNumberOfParameters();
        const char* GetParameterName(int index, OSCParameterDeclarations::ParameterType* type);
        int         setParameterValue(std::string name, const void* value);
        int         setParameterValueByString(std::string name, std::string value);
        int         setParameterValue(std::string name, int value);
        int         setParameterValue(std::string name, double value);
        int         setParameterValue(std::string name, const char* value);
        int         setParameterValue(std::string name, bool value);
        int         getParameterValue(std::string name, void* value) const;
        int         getParameterValueInt(std::string name, int& value) const;
        int         getParameterValueDouble(std::string name, double& value) const;
        int         getParameterValueString(std::string name, const char*& value) const;
        int         getParameterValueBool(std::string name, bool& value) const;
        std::string getParameterValueAsString(std::string name) const;

        std::string ResolveParametersInString(std::string str);

        // Use always this method when reading attributes, it will resolve any variables
        std::string ReadAttribute(pugi::xml_node, std::string attribute_name, bool required = false);

        // bool CheckAttribute(pugi::xml_node, std::string attribute);

        // Will clear all parameter declarations and assignements
        void Clear();

        // Log current set of parameter names and values
        void Print(std::string typestr);

    private:
        std::map<std::string, OSCParameterDeclarations> instance_parameters_;
    };
}  // namespace scenarioengine