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

#include "Variables.hpp"
#include "simple_expr.h"

using namespace scenarioengine;

void Variables::addVariableDeclarations(pugi::xml_node xml_node)
{
	varDeclarationsSize_.push((int)variableDeclarations_.Variable.size());
	parseVariableDeclarations(xml_node, &variableDeclarations_);
}

void Variables::parseGlobalVariableDeclarations(pugi::xml_node osc_root_)
{
	if (variableDeclarations_.Variable.size() != 0)
	{
		LOG("Unexpected non empty variableDeclarations_ when about to parse global declarations");
	}
	parseVariableDeclarations(osc_root_.child("VariableDeclarations"), &variableDeclarations_);
}

void Variables::CreateRestorePoint()
{
	varDeclarationsSize_.push((int)variableDeclarations_.Variable.size());
}

void Variables::RestoreVariableDeclarations()
{
	if (!varDeclarationsSize_.empty())
	{
		variableDeclarations_.Variable.erase(
			variableDeclarations_.Variable.begin(),
			variableDeclarations_.Variable.begin() + variableDeclarations_.Variable.size() - varDeclarationsSize_.top());
		varDeclarationsSize_.pop();
		catalog_var_assignments.clear();
	}
	else
	{
		LOG("Unexpected empty variabledeclaration counter, can't clear local declarations");
	}
}

int Variables::setVariable(std::string name, std::string value)
{
	// If string already present in variableDeclaration
	for (size_t i = 0; i < variableDeclarations_.Variable.size(); i++)
	{
		if (VARIABLE_PREFIX + variableDeclarations_.Variable[i].name == name || // variable names should not include prefix
			variableDeclarations_.Variable[i].name == name)  // But support also variable name including prefix
		{
			variableDeclarations_.Variable[i].value._string = value;
			return 0;
		}
	}

	return -1;
}

std::string Variables::getVariable(OSCVariableDeclarations& variableDeclaration, std::string name)
{
	// If string already present in variableDeclaration
	for (size_t i = 0; i < variableDeclaration.Variable.size(); i++)
	{
		if (VARIABLE_PREFIX + variableDeclaration.Variable[i].name == name || // variable names should not include prefix
			variableDeclaration.Variable[i].name == name)  // But support also variable name including prefix
		{
			return variableDeclaration.Variable[i].value._string;
		}
	}
	LOG("Failed to resolve variable %s", name.c_str());
	throw std::runtime_error("Failed to resolve variable");
}

OSCVariableDeclarations::VariableStruct* Variables::getVariableEntry(std::string name)
{
	// If string already present in variableDeclaration
	for (size_t i = 0; i < variableDeclarations_.Variable.size(); i++)
	{
		if (VARIABLE_PREFIX + variableDeclarations_.Variable[i].name == name || // variable names should not include prefix
			variableDeclarations_.Variable[i].name == name)  // But support also variable name including prefix
		{
			return &variableDeclarations_.Variable[i];
		}
	}

	return 0;
}

int Variables::GetNumberOfVariables()
{
	return (int)variableDeclarations_.Variable.size();
}

const char* Variables::GetVariableName(int index, OSCVariableDeclarations::VariableType* type)
{
	if (index < 0 || index >= variableDeclarations_.Variable.size())
	{
		LOG_AND_QUIT("index %d out of range [0:%d]", index, variableDeclarations_.Variable.size() - 1);
		return 0;
	}

	*type = variableDeclarations_.Variable[index].type;

	return variableDeclarations_.Variable[index].name.c_str();
}

int Variables::setVariableValue(std::string name, const void* value)
{
	OSCVariableDeclarations::VariableStruct* ps = getVariableEntry(name);

	if(!ps)
	{
		return -1;
	}

	if (ps->type == OSCVariableDeclarations::VariableType::VAR_TYPE_INTEGER)
	{
		ps->value._int = *((int*)value);
		ps->value._string = std::to_string(ps->value._int);
	}
	else if (ps->type == OSCVariableDeclarations::VariableType::VAR_TYPE_DOUBLE)
	{
		ps->value._double = *((double*)value);
		ps->value._string = std::to_string(ps->value._double);
	}
	else if (ps->type == OSCVariableDeclarations::VariableType::VAR_TYPE_STRING)
	{
		ps->value._string = *((std::string*)value);
	}
	else if (ps->type == OSCVariableDeclarations::VariableType::VAR_TYPE_BOOL)
	{
		ps->value._bool = *((bool*)value);
		ps->value._string = ps->value._bool == true ? "true" : "false";
	}
	else
	{
		LOG("Unexpected type: %d", ps->type);
		return -1;
	}

	return 0;
}

int Variables::getVariableValue(std::string name, void* value)
{
	OSCVariableDeclarations::VariableStruct* ps = getVariableEntry(name);

	if (!ps)
	{
		return -1;
	}

	if (ps->type == OSCVariableDeclarations::VariableType::VAR_TYPE_INTEGER)
	{
		*((int*)value) = ps->value._int;
	}
	else if (ps->type == OSCVariableDeclarations::VariableType::VAR_TYPE_DOUBLE)
	{
		*((double*)value) = ps->value._double;
	}
	else if (ps->type == OSCVariableDeclarations::VariableType::VAR_TYPE_BOOL)
	{
		*((bool*)value) = ps->value._bool;
	}
	else if (ps->type == OSCVariableDeclarations::VariableType::VAR_TYPE_STRING)
	{
		*((std::string*)value) = ps->value._string;
	}
	else
	{
		LOG("Unexpected type: %d", ps->type);
		return -1;
	}

	return 0;
}

int Variables::getVariableValueInt(std::string name, int &value)
{
	OSCVariableDeclarations::VariableStruct* ps = getVariableEntry(name);

	if (!ps || ps->type != OSCVariableDeclarations::VariableType::VAR_TYPE_INTEGER)
	{
		return -1;
	}

	value = ps->value._int;

	return 0;
}

int Variables::getVariableValueDouble(std::string name, double& value)
{
	OSCVariableDeclarations::VariableStruct* ps = getVariableEntry(name);

	if (!ps || ps->type != OSCVariableDeclarations::VariableType::VAR_TYPE_DOUBLE)
	{
		return -1;
	}

	value = ps->value._double;

	return 0;
}

int Variables::getVariableValueString(std::string name, const char*& value)
{
	OSCVariableDeclarations::VariableStruct* ps = getVariableEntry(name);

	if (!ps || ps->type != OSCVariableDeclarations::VariableType::VAR_TYPE_STRING)
	{
		return -1;
	}

	value = ps->value._string.c_str();

	return 0;
}

int Variables::getVariableValueBool(std::string name, bool& value)
{
	OSCVariableDeclarations::VariableStruct* ps = getVariableEntry(name);

	if (!ps || ps->type != OSCVariableDeclarations::VariableType::VAR_TYPE_BOOL)
	{
		return -1;
	}

	value = ps->value._bool;

	return 0;
}

std::string Variables::getVariableValueAsString(std::string name)
{
	OSCVariableDeclarations::VariableStruct* ps = getVariableEntry(name);

	if (!ps)
	{
		return "";
	}

	if (ps->type == OSCVariableDeclarations::VariableType::VAR_TYPE_STRING)
	{
		return ps->value._string;
	}
	else if (ps->type == OSCVariableDeclarations::VariableType::VAR_TYPE_INTEGER)
	{
		return std::to_string(ps->value._int);
	}
	else if (ps->type == OSCVariableDeclarations::VariableType::VAR_TYPE_DOUBLE)
	{
		return std::to_string(ps->value._double);
	}
	else if (ps->type == OSCVariableDeclarations::VariableType::VAR_TYPE_BOOL)
	{
		return ps->value._bool ? "true" : "false";
	}
	else
	{
		return "";
	}
}

int Variables::setVariableValueByString(std::string name, std::string value)
{
	OSCVariableDeclarations::VariableStruct* ps = getVariableEntry(name);

	if (!ps)
	{
		return -1;
	}

	// Always set string value
	ps->value._string = value;

	if (ps->type == OSCVariableDeclarations::VariableType::VAR_TYPE_INTEGER)
	{
		ps->value._int = strtoi(value);
	}
	else if (ps->type == OSCVariableDeclarations::VariableType::VAR_TYPE_DOUBLE)
	{
		ps->value._double = strtod(value);
	}
	else if (ps->type == OSCVariableDeclarations::VariableType::VAR_TYPE_BOOL)
	{
		ps->value._bool = (value == "true" ? true : false);
	}
	else if (ps->type != OSCVariableDeclarations::VariableType::VAR_TYPE_STRING)
	{
		LOG("Unexpected type: %d", ps->type);
		return -1;
	}

	return 0;
}

int Variables::setVariableValue(std::string name, int value)
{
	OSCVariableDeclarations::VariableStruct* ps = getVariableEntry(name);

	if (!ps || ps->type != OSCVariableDeclarations::VariableType::VAR_TYPE_INTEGER)
	{
		return -1;
	}

	ps->value._int = value;
	ps->value._string = std::to_string(ps->value._int);

	return 0;
}

int Variables::setVariableValue(std::string name, double value)
{
	OSCVariableDeclarations::VariableStruct* ps = getVariableEntry(name);

	if (!ps || ps->type != OSCVariableDeclarations::VariableType::VAR_TYPE_DOUBLE)
	{
		return -1;
	}

	ps->value._double = value;
	ps->value._string = std::to_string(ps->value._double);

	return 0;
}

int Variables::setVariableValue(std::string name, const char* value)
{
	OSCVariableDeclarations::VariableStruct* ps = getVariableEntry(name);

	if (!ps || ps->type != OSCVariableDeclarations::VariableType::VAR_TYPE_STRING)
	{
		return -1;
	}

	ps->value._string = value;

	return 0;
}

int Variables::setVariableValue(std::string name, bool value)
{
	OSCVariableDeclarations::VariableStruct* ps = getVariableEntry(name);

	if (!ps || ps->type != OSCVariableDeclarations::VariableType::VAR_TYPE_BOOL)
	{
		return -1;
	}

	ps->value._bool = value;
	ps->value._string = ps->value._bool == true ? "true" : "false";

	return 0;
}

std::string Variables::ResolveVariablesInString(std::string str)
{
	size_t found;
	while ((found = str.find("$")) != std::string::npos)
	{
		size_t found_space = str.find_first_of(" ({)}-+*/%^!|&<>=,", found);
		if (found_space != std::string::npos)
		{
			str.replace(found, found_space-found, getVariable(variableDeclarations_, str.substr(found, found_space - found)));
		}
		else
		{
			str.replace(found, std::string::npos, getVariable(variableDeclarations_, str.substr(found)));
		}
	}

	return str;
}

static void ReplaceStringInPlace(std::string& subject, const std::string& search, const std::string& replace) {
	size_t pos = 0;
	while ((pos = subject.find(search, pos)) != std::string::npos) {
		subject.replace(pos, search.length(), replace);
		pos += replace.length();
	}
}

std::string Variables::ReadAttribute(pugi::xml_node node, std::string attribute_name, bool required)
{
	if (!strcmp(attribute_name.c_str(), ""))
	{
		if (required)
		{
			LOG_AND_QUIT("Warning: Request to read empty attribute name in XML node %s", node.name());
		}
		return "";
	}

	pugi::xml_attribute attr;

	if ((attr = node.attribute(attribute_name.c_str())))
	{
		if (attr.value()[0] == '$' && strlen(attr.value()) > 1)
		{
			if (attr.value()[1] == '{' && strlen(attr.value()) > 2)
			{
				// Resolve expression
				std::string expr = attr.value();
				std::size_t found = expr.find('}', 2);
				if (found != std::string::npos)
				{
					expr = expr.substr(2, found - 2);  // trim to bare expression, exclude '{' and '}'
					expr = ResolveVariablesInString(expr);  // replace variables by their values

					// Convert from OpenSCENARIO 1.1 operator names to expr op names
					ReplaceStringInPlace(expr, "not ", "!");
					ReplaceStringInPlace(expr, "not(", "!(");
					ReplaceStringInPlace(expr, "and ", "&& ");
					ReplaceStringInPlace(expr, "or ", "|| ");
					ReplaceStringInPlace(expr, "true ", "1 ");
					ReplaceStringInPlace(expr, "false ", "0 ");

					double value = eval_expr(expr.c_str());
					if (isnan(value))
					{
						LOG_AND_QUIT("Failed to evaluate the expression : % s\n", attr.value());
					}

					LOG("Expr %s = %s = %.10lf", attr.value(), expr.c_str(), value);
					return std::to_string(value);
				}
				else
				{
					LOG_AND_QUIT("Expression syntax error: %s, missing end '}'", attr.value());
				}
			}
			else
			{
				// Resolve variable
				return getVariable(variableDeclarations_, attr.value());
			}
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
			LOG_AND_QUIT("Error: missing required attribute: %s -> %s", node.name(), attribute_name.c_str());
		}
	}

	return "";
}

void Variables::parseVariableDeclarations(pugi::xml_node variableDeclarationsNode, OSCVariableDeclarations* pd)
{
	for (pugi::xml_node pdChild = variableDeclarationsNode.first_child(); pdChild; pdChild = pdChild.next_sibling())
	{
		OSCVariableDeclarations::VariableStruct var = { "", OSCVariableDeclarations::VariableType::VAR_TYPE_STRING, {0, 0, "", false} };

		var.name = pdChild.attribute("name").value();

		// Check for catalog variable assignments, overriding default value
		// Start from end of variable list, in case of duplicates we want the most recent
		var.value._string = ReadAttribute(pdChild, "value");
		for (int i = (int)catalog_var_assignments.size() - 1; i >= 0; i--)
		{
			if (var.name == catalog_var_assignments[i].name)
			{
				var.value._string = catalog_var_assignments[i].value._string;
				break;
			}
		}

		std::string type_str;
		if (pdChild.attribute("variableType"))
		{
			type_str = pdChild.attribute("variableType").value();
		}
		else
		{
			LOG_TRACE_AND_QUIT("Missing variable type (or wrongly spelled attribute) for %s", var.name.c_str());
		}

		if (type_str == "integer" || type_str == "int")
		{
			if (type_str == "int")
			{
				LOG("INFO: int type should renamed into integer - accepting int this time.");
			}
			var.type = OSCVariableDeclarations::VariableType::VAR_TYPE_INTEGER;
			var.value._int = strtoi(var.value._string);
		}
		else if (type_str == "double")
		{
			var.type = OSCVariableDeclarations::VariableType::VAR_TYPE_DOUBLE;
			var.value._double = strtod(var.value._string);
		}
		else if (type_str == "boolean" || type_str == "bool")
		{
			if (type_str == "bool")
			{
				LOG("INFO: bool type should renamed into boolean - accepting bool this time.");
			}

			var.type = OSCVariableDeclarations::VariableType::VAR_TYPE_BOOL;
			var.value._bool = var.value._string == "true" ? true : false;
		}
		else if (type_str == "string")
		{
			var.type = OSCVariableDeclarations::VariableType::VAR_TYPE_STRING;
		}
		else if (type_str == "unsignedInt" || type_str == "unsignedShort" || type_str == "dateTime")
		{
			LOG_TRACE("Type %s is not supported yet", type_str.c_str());
		}
		else
		{
			LOG_TRACE_AND_QUIT("Unexpected Type: %s", type_str.c_str());
		}
		pd->Variable.insert(pd->Variable.begin(), var);
	}
}


void Variables::Clear()
{
	variableDeclarations_.Variable.clear();
	while (!varDeclarationsSize_.empty())
	{
		varDeclarationsSize_.pop();
	}
	catalog_var_assignments.clear();
}

void Variables::Print()
{
	LOG("%d variables%s", variableDeclarations_.Variable.size(), variableDeclarations_.Variable.size() > 0 ? ":" : "");

	for (size_t i = 0; i < variableDeclarations_.Variable.size(); i++)
	{
		LOG("   %s = %s", variableDeclarations_.Variable[i].name.c_str(), variableDeclarations_.Variable[i].value._string.c_str());
	}
}
