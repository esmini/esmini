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
#include "ScenarioReader.hpp"
#include "simple_expr.h"
#include "logger.hpp"

using namespace scenarioengine;

void Parameters::addParameterDeclarations(pugi::xml_node xml_node)
{
    paramDeclarationsSize_.push(static_cast<int>(parameterDeclarations_.Parameter.size()));
    parseParameterDeclarations(xml_node, &parameterDeclarations_);
}

void Parameters::parseGlobalParameterDeclarations(pugi::xml_node osc_root)
{
    if (parameterDeclarations_.Parameter.size() != 0)
    {
        LOG_INFO("ParameterDeclarations contains parameters with modified values, all parameters and variables will be re-evaluated");
    }

    parseParameterDeclarations(osc_root, &parameterDeclarations_);
}

void Parameters::CreateRestorePoint()
{
    paramDeclarationsSize_.push(static_cast<int>(parameterDeclarations_.Parameter.size()));
}

void Parameters::RestoreParameterDeclarations()
{
    if (!paramDeclarationsSize_.empty())
    {
        parameterDeclarations_.Parameter.erase(
            parameterDeclarations_.Parameter.begin(),
            parameterDeclarations_.Parameter.begin() + static_cast<int>(parameterDeclarations_.Parameter.size()) - paramDeclarationsSize_.top());
        paramDeclarationsSize_.pop();
        catalog_param_assignments.clear();
    }
    else
    {
        LOG_ERROR("Unexpected empty parameterdeclaration counter, can't clear local declarations");
    }
}

int Parameters::setParameter(std::string name, std::string value)
{
    // If string already present in parameterDeclaration
    for (size_t i = 0; i < parameterDeclarations_.Parameter.size(); i++)
    {
        if (PARAMETER_PREFIX + parameterDeclarations_.Parameter[i].name == name ||  // parameter names should not include prefix
            parameterDeclarations_.Parameter[i].name == name)                       // But support also parameter name including prefix
        {
            parameterDeclarations_.Parameter[i].value._string = value;
            return 0;
        }
    }

    return -1;
}

std::string Parameters::getParameter(std::string name)
{
    // If string already present in parameterDeclaration
    const std::vector<OSCParameterDeclarations::ParameterStruct>& parameters = ScenarioReader::parameters.parameterDeclarations_.Parameter;
    for (size_t i = 0; i < parameters.size(); i++)
    {
        if (PARAMETER_PREFIX + parameters[i].name == name ||  // parameter names should not include prefix
            parameters[i].name == name)                       // But support also parameter name including prefix
        {
            return parameters[i].value._string;
        }
    }
    LOG_ERROR("Failed to resolve parameter {}", name);
    throw std::runtime_error("Failed to resolve parameter");
}

OSCParameterDeclarations::ParameterStruct* Parameters::getParameterEntry(std::string name)
{
    // If string already present in parameterDeclaration
    for (size_t i = 0; i < parameterDeclarations_.Parameter.size(); i++)
    {
        if (PARAMETER_PREFIX + parameterDeclarations_.Parameter[i].name == name ||  // parameter names should not include prefix
            parameterDeclarations_.Parameter[i].name == name)                       // But support also parameter name including prefix
        {
            return &parameterDeclarations_.Parameter[i];
        }
    }

    return 0;
}

int Parameters::GetNumberOfParameters()
{
    return static_cast<int>(parameterDeclarations_.Parameter.size());
}

const char* Parameters::GetParameterName(int index, OSCParameterDeclarations::ParameterType* type)
{
    if (index < 0 || static_cast<unsigned int>(index) >= parameterDeclarations_.Parameter.size())
    {
        LOG_ERROR_AND_QUIT("index {} out of range [0:{}]", index, parameterDeclarations_.Parameter.size() - 1);
        return 0;
    }

    *type = parameterDeclarations_.Parameter[static_cast<unsigned int>(index)].type;

    return parameterDeclarations_.Parameter[static_cast<unsigned int>(index)].name.c_str();
}

int Parameters::setParameterValue(std::string name, const void* value)
{
    OSCParameterDeclarations::ParameterStruct* ps = getParameterEntry(name);

    if (!ps)
    {
        return -1;
    }

    if (ps->type == OSCParameterDeclarations::ParameterType::PARAM_TYPE_INTEGER)
    {
        ps->value._int    = *(reinterpret_cast<const int*>(value));
        ps->value._string = std::to_string(ps->value._int);
    }
    else if (ps->type == OSCParameterDeclarations::ParameterType::PARAM_TYPE_DOUBLE)
    {
        ps->value._double = *(reinterpret_cast<const double*>(value));
        ps->value._string = std::to_string(ps->value._double);
    }
    else if (ps->type == OSCParameterDeclarations::ParameterType::PARAM_TYPE_STRING)
    {
        ps->value._string = *(reinterpret_cast<const std::string*>(value));
    }
    else if (ps->type == OSCParameterDeclarations::ParameterType::PARAM_TYPE_BOOL)
    {
        ps->value._bool   = *(reinterpret_cast<const bool*>(value));
        ps->value._string = ps->value._bool == true ? "true" : "false";
    }
    else
    {
        LOG_ERROR("Unexpected type: {}", ps->type);
        return -1;
    }

    ps->dirty = true;

    return 0;
}

int Parameters::getParameterValue(std::string name, void* value)
{
    OSCParameterDeclarations::ParameterStruct* ps = getParameterEntry(name);

    if (!ps)
    {
        return -1;
    }

    if (ps->type == OSCParameterDeclarations::ParameterType::PARAM_TYPE_INTEGER)
    {
        *(static_cast<int*>(value)) = ps->value._int;
    }
    else if (ps->type == OSCParameterDeclarations::ParameterType::PARAM_TYPE_DOUBLE)
    {
        *(static_cast<double*>(value)) = ps->value._double;
    }
    else if (ps->type == OSCParameterDeclarations::ParameterType::PARAM_TYPE_BOOL)
    {
        *(static_cast<bool*>(value)) = ps->value._bool;
    }
    else if (ps->type == OSCParameterDeclarations::ParameterType::PARAM_TYPE_STRING)
    {
        *(static_cast<std::string*>(value)) = ps->value._string;
    }
    else
    {
        LOG_ERROR("Unexpected type: {}", ps->type);
        return -1;
    }

    return 0;
}

int Parameters::getParameterValueInt(std::string name, int& value)
{
    OSCParameterDeclarations::ParameterStruct* ps = getParameterEntry(name);

    if (!ps || ps->type != OSCParameterDeclarations::ParameterType::PARAM_TYPE_INTEGER)
    {
        return -1;
    }

    value = ps->value._int;

    return 0;
}

int Parameters::getParameterValueDouble(std::string name, double& value)
{
    OSCParameterDeclarations::ParameterStruct* ps = getParameterEntry(name);

    if (!ps || ps->type != OSCParameterDeclarations::ParameterType::PARAM_TYPE_DOUBLE)
    {
        return -1;
    }

    value = ps->value._double;

    return 0;
}

int Parameters::getParameterValueString(std::string name, const char*& value)
{
    OSCParameterDeclarations::ParameterStruct* ps = getParameterEntry(name);

    if (!ps || ps->type != OSCParameterDeclarations::ParameterType::PARAM_TYPE_STRING)
    {
        return -1;
    }

    value = ps->value._string.c_str();

    return 0;
}

int Parameters::getParameterValueBool(std::string name, bool& value)
{
    OSCParameterDeclarations::ParameterStruct* ps = getParameterEntry(name);

    if (!ps || ps->type != OSCParameterDeclarations::ParameterType::PARAM_TYPE_BOOL)
    {
        return -1;
    }

    value = ps->value._bool;

    return 0;
}

std::string Parameters::getParameterValueAsString(std::string name)
{
    OSCParameterDeclarations::ParameterStruct* ps = getParameterEntry(name);

    if (!ps)
    {
        return "";
    }

    if (ps->type == OSCParameterDeclarations::ParameterType::PARAM_TYPE_STRING)
    {
        return ps->value._string;
    }
    else if (ps->type == OSCParameterDeclarations::ParameterType::PARAM_TYPE_INTEGER)
    {
        return std::to_string(ps->value._int);
    }
    else if (ps->type == OSCParameterDeclarations::ParameterType::PARAM_TYPE_DOUBLE)
    {
        return std::to_string(ps->value._double);
    }
    else if (ps->type == OSCParameterDeclarations::ParameterType::PARAM_TYPE_BOOL)
    {
        return ps->value._bool ? "true" : "false";
    }
    else
    {
        return "";
    }
}

int Parameters::setParameterValueByString(std::string name, std::string value)
{
    OSCParameterDeclarations::ParameterStruct* ps = getParameterEntry(name);

    if (!ps)
    {
        return -1;
    }

    // Always set string value
    ps->value._string = value;

    if (ps->type == OSCParameterDeclarations::ParameterType::PARAM_TYPE_INTEGER)
    {
        ps->value._int = strtoi(value);
    }
    else if (ps->type == OSCParameterDeclarations::ParameterType::PARAM_TYPE_DOUBLE)
    {
        ps->value._double = strtod(value);
    }
    else if (ps->type == OSCParameterDeclarations::ParameterType::PARAM_TYPE_BOOL)
    {
        ps->value._bool = (value == "true" ? true : false);
    }
    else if (ps->type != OSCParameterDeclarations::ParameterType::PARAM_TYPE_STRING)
    {
        LOG_ERROR("Unexpected type: {}", ps->type);
        return -1;
    }

    ps->dirty = true;

    return 0;
}

int Parameters::setParameterValue(std::string name, int value)
{
    OSCParameterDeclarations::ParameterStruct* ps = getParameterEntry(name);

    if (!ps || ps->type != OSCParameterDeclarations::ParameterType::PARAM_TYPE_INTEGER)
    {
        return -1;
    }

    ps->value._int    = value;
    ps->value._string = std::to_string(ps->value._int);
    ps->dirty         = true;

    return 0;
}

int Parameters::setParameterValue(std::string name, double value)
{
    OSCParameterDeclarations::ParameterStruct* ps = getParameterEntry(name);

    if (!ps || ps->type != OSCParameterDeclarations::ParameterType::PARAM_TYPE_DOUBLE)
    {
        return -1;
    }

    ps->value._double = value;
    ps->value._string = std::to_string(ps->value._double);
    ps->dirty         = true;

    return 0;
}

int Parameters::setParameterValue(std::string name, const char* value)
{
    OSCParameterDeclarations::ParameterStruct* ps = getParameterEntry(name);

    if (!ps || ps->type != OSCParameterDeclarations::ParameterType::PARAM_TYPE_STRING)
    {
        return -1;
    }

    ps->value._string = value;
    ps->dirty         = true;

    return 0;
}

int Parameters::setParameterValue(std::string name, bool value)
{
    OSCParameterDeclarations::ParameterStruct* ps = getParameterEntry(name);

    if (!ps || ps->type != OSCParameterDeclarations::ParameterType::PARAM_TYPE_BOOL)
    {
        return -1;
    }

    ps->value._bool   = value;
    ps->value._string = ps->value._bool == true ? "true" : "false";
    ps->dirty         = true;

    return 0;
}

std::string Parameters::ResolveParametersInString(std::string str)
{
    size_t found;
    while ((found = str.find("$")) != std::string::npos)
    {
        size_t found_space = str.find_first_of(" ({)}-+*/%^!|&<>=,", found);
        if (found_space != std::string::npos)
        {
            str.replace(found, found_space - found, getParameter(str.substr(found, found_space - found)));
        }
        else
        {
            str.replace(found, std::string::npos, getParameter(str.substr(found)));
        }
    }

    return str;
}

static void ReplaceStringInPlace(std::string& subject, const std::string& search, const std::string& replace)
{
    size_t pos = 0;
    while ((pos = subject.find(search, pos)) != std::string::npos)
    {
        subject.replace(pos, search.length(), replace);
        pos += replace.length();
    }
}

// std::string ReadAttributeFrom

std::string Parameters::ReadAttribute(pugi::xml_node node, std::string attribute_name, bool required)
{
    std::string return_value;

    if (!strcmp(attribute_name.c_str(), ""))
    {
        if (required)
        {
            LOG_ERROR_AND_QUIT("Warning: Request to read empty attribute name in XML node {}", node.name());
        }
        return return_value;
    }

    pugi::xml_attribute attr;

    if ((attr = node.attribute(attribute_name.c_str())))
    {
        if (attr.value()[0] == '$' && strlen(attr.value()) > 1)
        {
            if (attr.value()[1] == '{' && strlen(attr.value()) > 2)
            {
                // Resolve expression
                std::string expr  = attr.value();
                std::size_t found = expr.find('}', 2);
                if (found != std::string::npos)
                {
                    expr = expr.substr(2, found - 2);        // trim to bare expression, exclude '{' and '}'
                    expr = ResolveParametersInString(expr);  // replace parameters by their values

                    // Convert from OpenSCENARIO 1.1 operator names to expr op names
                    ReplaceStringInPlace(expr, "not ", "!");
                    ReplaceStringInPlace(expr, "not(", "!(");
                    ReplaceStringInPlace(expr, "and ", "&& ");
                    ReplaceStringInPlace(expr, "or ", "|| ");
                    ReplaceStringInPlace(expr, "true ", "1 ");
                    ReplaceStringInPlace(expr, "false ", "0 ");

                    ExprReturnStruct rs = eval_expr(expr.c_str());
                    if (rs.type == EXPR_RETURN_UNDEFINED && isnan(rs._double))
                    {
                        LOG_ERROR_AND_QUIT("Failed to evaluate the expression : {}\n", attr.value());
                    }

                    if (rs.type == EXPR_RETURN_DOUBLE)
                    {
                        LOG_INFO("Expr {} = {} = {:.10f}", attr.value(), expr, rs._double);
                        return_value = std::to_string(rs._double);
                    }
                    else if (rs.type == EXPR_RETURN_STRING)
                    {
                        LOG_INFO("Expr {} = {} = {}", attr.value(), expr, rs._string.string);
                        return_value = rs._string.string;
                    }
                    clear_expr_result(&rs);
                }
                else
                {
                    LOG_ERROR_AND_QUIT("Expression syntax error: {}, missing end '}'", attr.value());
                }
            }
            else
            {
                // Resolve variable
                return_value = getParameter(attr.value());
            }
        }
        else
        {
            return_value = attr.value();
        }
    }
    else
    {
        if (required)
        {
            LOG_ERROR_AND_QUIT("Error: missing required attribute: {} -> {}", node.name(), attribute_name.c_str());
        }
    }

    return return_value;
}

void Parameters::parseParameterDeclarations(pugi::xml_node declarationsNode, OSCParameterDeclarations* pd)
{
    bool is_variable = false;
    if (!strcmp(declarationsNode.name(), "VariableDeclarations"))
    {
        is_variable = true;
    }

    for (pugi::xml_node pdChild = declarationsNode.first_child(); pdChild; pdChild = pdChild.next_sibling())
    {
        OSCParameterDeclarations::ParameterStruct param = {"", OSCParameterDeclarations::ParameterType::PARAM_TYPE_STRING, {0, 0, "", false}};

        param.name     = pdChild.attribute("name").value();
        param.variable = is_variable;

        // Check for catalog parameter assignements, overriding default value
        // Start from end of parameter list, in case of duplicates we want the most recent
        param.value._string = ReadAttribute(pdChild, "value");
        for (int i = static_cast<int>(catalog_param_assignments.size()) - 1; i >= 0; i--)
        {
            if (param.name == catalog_param_assignments[static_cast<unsigned int>(i)].name)
            {
                param.value._string = catalog_param_assignments[static_cast<unsigned int>(i)].value._string;
                break;
            }
        }

        std::string type_str;
        if (pdChild.attribute("parameterType"))
        {
            type_str = pdChild.attribute("parameterType").value();
        }
        else if (pdChild.attribute("variableType"))
        {
            type_str = pdChild.attribute("variableType").value();
        }
        else
        {
            LOG_ERROR_AND_QUIT("Missing parameter or variable type (or wrongly spelled attribute) for {}", param.name);
        }

        if (type_str == "integer" || type_str == "int")
        {
            param.type       = OSCParameterDeclarations::ParameterType::PARAM_TYPE_INTEGER;
            param.value._int = strtoi(param.value._string);
        }
        else if (type_str == "double")
        {
            param.type          = OSCParameterDeclarations::ParameterType::PARAM_TYPE_DOUBLE;
            param.value._double = strtod(param.value._string);
        }
        else if (type_str == "boolean" || type_str == "bool")
        {
            if (type_str == "bool")
            {
                LOG_WARN("INFO: bool type should renamed into boolean - accepting bool this time.");
            }

            param.type        = OSCParameterDeclarations::ParameterType::PARAM_TYPE_BOOL;
            param.value._bool = param.value._string == "true" ? true : false;
        }
        else if (type_str == "string")
        {
            param.type = OSCParameterDeclarations::ParameterType::PARAM_TYPE_STRING;
        }
        else if (type_str == "unsignedInt" || type_str == "unsignedShort" || type_str == "dateTime")
        {
            LOG_ERROR("Type {} is not supported yet", type_str);
        }
        else
        {
            LOG_ERROR_AND_QUIT("Unexpected Type: {}", type_str);
        }

        auto pos = std::find_if(pd->Parameter.begin(), pd->Parameter.end(), [&param](const auto& p) { return p.name == param.name; });
        if (pos != pd->Parameter.end())
        {
            if (pos->dirty)
            {
                LOG_WARN("Parameter value {} updated from {} to {}", pos->name, pos->value._string, param.value._string);
                continue;
            }
            else
            {
                LOG_WARN("Parameter {} redefined. Previous value {} will be shadowed by {}", pos->name, pos->value._string, param.value._string);
            }
        }

        pd->Parameter.insert(pd->Parameter.begin(), param);
    }
}

void Parameters::Clear()
{
    parameterDeclarations_.Parameter.clear();
    while (!paramDeclarationsSize_.empty())
    {
        paramDeclarationsSize_.pop();
    }
    catalog_param_assignments.clear();
}

void Parameters::Print(std::string typestr)
{
    LOG_INFO("{} {}{}", parameterDeclarations_.Parameter.size(), typestr, parameterDeclarations_.Parameter.size() > 0 ? ":" : "");

    for (size_t i = 0; i < parameterDeclarations_.Parameter.size(); i++)
    {
        LOG_INFO("   {} = {}", parameterDeclarations_.Parameter[i].name, parameterDeclarations_.Parameter[i].value._string);
    }
}

// bool Parameters::CheckAttribute(pugi::xml_node node, std::string attribute_name)
// {
// 	bool check = ReadAttribute(node, attribute_name);
// 	return check
// }
