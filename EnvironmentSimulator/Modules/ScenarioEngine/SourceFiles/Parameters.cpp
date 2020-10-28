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

#include "Parameters.hpp"

#define PARAMETER_PREFIX "$"

using namespace scenarioengine;

void Parameters::addParameterDeclarations(pugi::xml_node xml_node)
{
	parseParameterDeclarations(xml_node, &parameterDeclarations_);
}

void Parameters::parseGlobalParameterDeclarations(pugi::xml_document* doc_)
{
	parseParameterDeclarations(doc_->child("OpenSCENARIO").child("ParameterDeclarations"), &parameterDeclarations_);
	paramDeclarationsSize_ = (int)parameterDeclarations_.Parameter.size();
}

void Parameters::RestoreParameterDeclarations()
{
	parameterDeclarations_.Parameter.erase(
		parameterDeclarations_.Parameter.begin(),
		parameterDeclarations_.Parameter.begin() + parameterDeclarations_.Parameter.size() - paramDeclarationsSize_);
	catalog_param_assignments.clear();
}

void Parameters::addParameter(std::string name, std::string value)
{
	ParameterStruct param;

	LOG("adding %s = %s", name.c_str(), value.c_str());

	param.name = name;
	param.type = "string";
	param.value = value;

	parameterDeclarations_.Parameter.insert(parameterDeclarations_.Parameter.begin(), param);
}

std::string Parameters::getParameter(OSCParameterDeclarations& parameterDeclaration, std::string name)
{
	LOG("Resolve parameter %s", name.c_str());

	// If string already present in parameterDeclaration
	for (size_t i = 0; i < parameterDeclaration.Parameter.size(); i++)
	{
		if (PARAMETER_PREFIX + parameterDeclaration.Parameter[i].name == name || // parameter names should not include prefix
			parameterDeclaration.Parameter[i].name == name)  // But support also parameter name including prefix
		{
			LOG("%s replaced with %s", name.c_str(), parameterDeclaration.Parameter[i].value.c_str());
			return parameterDeclaration.Parameter[i].value;
		}
	}
	LOG("Failed to resolve parameter %s", name.c_str());
	throw std::runtime_error("Failed to resolve parameter");
	return 0;
}

std::string Parameters::ReadAttribute(pugi::xml_node node, std::string attribute_name, bool required)
{
	if (!strcmp(attribute_name.c_str(), ""))
	{
		if (required)
		{
			LOG("Warning: Empty attribute");
		}
		return "";
	}

	pugi::xml_attribute attr;

	if ((attr = node.attribute(attribute_name.c_str())))
	{
		if (attr.value()[0] == '$')
		{
			// Resolve variable
			return getParameter(parameterDeclarations_, attr.value());
		}
		else
		{
			return attr.value();
		}
	}
	else
	{
		if (required)
		{
			LOG("Warning: missing required attribute: %s", attribute_name.c_str());
		}
	}

	return "";
}

void Parameters::parseParameterDeclarations(pugi::xml_node parameterDeclarationsNode, OSCParameterDeclarations* pd)
{
	LOG("Parsing ParameterDeclarations");

	for (pugi::xml_node pdChild = parameterDeclarationsNode.first_child(); pdChild; pdChild = pdChild.next_sibling())
	{
		ParameterStruct param;

		param.name = pdChild.attribute("name").value();

		// Check for catalog parameter assignements, overriding default value
		param.value = pdChild.attribute("value").value();
		for (size_t i = 0; i < catalog_param_assignments.size(); i++)
		{
			if (param.name == catalog_param_assignments[i].name)
			{
				param.value = catalog_param_assignments[i].value;
				break;
			}
		}
		param.type = pdChild.attribute("parameterType").value();
		pd->Parameter.insert(pd->Parameter.begin(), param);
	}
}


