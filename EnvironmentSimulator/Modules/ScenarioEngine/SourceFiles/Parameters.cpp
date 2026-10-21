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
#include <algorithm>

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
    auto* parameter = getParameterEntry(name);
    if (parameter != nullptr)
    {
        parameter->value._string = value;
        return 0;
    }

    return -1;
}

std::string Parameters::getParameter(std::string name)
{
    const auto* parameter = ScenarioReader::parameters.getParameterEntryReadOnly(name);
    if (parameter != nullptr)
    {
        return parameter->value._string;
    }
    LOG_ERROR_AND_QUIT("Failed to resolve parameter {}", name);

    return "";  // will not be reached, but added to avoid compiler warning
}

OSCParameterDeclarations::ParameterStruct* Parameters::getParameterEntry(std::string name)
{
    if (name.find("::") != std::string::npos)
    {
        LOG_ERROR("Instance parameter {} is read-only", name);
        return nullptr;
    }
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

const OSCParameterDeclarations::ParameterStruct* Parameters::getParameterEntryReadOnly(std::string name) const
{
    const std::string reference = name;
    if (!name.empty() && name.front() == PARAMETER_PREFIX)
    {
        name.erase(0, 1);
    }

    const auto                      separator    = name.find("::");
    const OSCParameterDeclarations* declarations = &parameterDeclarations_;
    if (separator != std::string::npos)
    {
        const auto is_identifier = [](const std::string& identifier)
        {
            return identifier.find_first_of("ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz_") == 0 &&
                   identifier.find_first_not_of("ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz_0123456789") == std::string::npos;
        };
        if (!is_identifier(name.substr(0, separator)) || !is_identifier(name.substr(separator + 2)))
        {
            LOG_ERROR("Invalid instance parameter reference {}: expected $instanceName::parameterName with identifier names", reference);
            return nullptr;
        }
        const auto instance = instance_parameters_.find(name.substr(0, separator));
        if (instance == instance_parameters_.end())
        {
            LOG_ERROR("Failed to resolve instance parameter {}: instance {} is not available (forward/self references are not supported)",
                      name,
                      name.substr(0, separator));
            return nullptr;
        }
        declarations = &instance->second;
        name.erase(0, separator + 2);
    }

    const std::string& lookup_name = separator == std::string::npos ? reference : name;
    const auto         parameter =
        std::find_if(declarations->Parameter.begin(),
                     declarations->Parameter.end(),
                     [&lookup_name](const auto& entry) { return entry.name == lookup_name || PARAMETER_PREFIX + entry.name == lookup_name; });
    if (parameter != declarations->Parameter.end())
    {
        return &*parameter;
    }

    if (separator != std::string::npos)
    {
        LOG_ERROR("Failed to resolve instance parameter {}: {} is not declared on the instance", reference, name);
    }

    return nullptr;
}

OSCParameterDeclarations Parameters::CaptureParameterDeclarations(pugi::xml_node declarations) const
{
    OSCParameterDeclarations snapshot;
    for (auto declaration : declarations.children("ParameterDeclaration"))
    {
        const auto* parameter = getParameterEntryReadOnly(declaration.attribute("name").value());
        if (parameter == nullptr)
        {
            LOG_ERROR_AND_QUIT("Failed to capture instance parameter {}", declaration.attribute("name").value());
        }
        else
        {
            snapshot.Parameter.push_back(*parameter);
        }
    }
    return snapshot;
}

void Parameters::RegisterInstanceParameters(const std::string& name, const OSCParameterDeclarations& declarations)
{
    if (name.empty() || name.find("::") != std::string::npos)
    {
        LOG_ERROR_AND_QUIT("Invalid instance parameter namespace {}: expected a non-empty name without '::'", name);
    }
    if (!instance_parameters_.emplace(name, declarations).second)
    {
        LOG_ERROR_AND_QUIT("Duplicate instance parameter namespace {}", name);
    }
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

int Parameters::getParameterValue(std::string name, void* value) const
{
    const auto* ps = getParameterEntryReadOnly(name);

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

int Parameters::getParameterValueInt(std::string name, int& value) const
{
    const auto* ps = getParameterEntryReadOnly(name);

    if (!ps || ps->type != OSCParameterDeclarations::ParameterType::PARAM_TYPE_INTEGER)
    {
        return -1;
    }

    value = ps->value._int;

    return 0;
}

int Parameters::getParameterValueDouble(std::string name, double& value) const
{
    const auto* ps = getParameterEntryReadOnly(name);

    if (!ps || ps->type != OSCParameterDeclarations::ParameterType::PARAM_TYPE_DOUBLE)
    {
        return -1;
    }

    value = ps->value._double;

    return 0;
}

int Parameters::getParameterValueString(std::string name, const char*& value) const
{
    const auto* ps = getParameterEntryReadOnly(name);

    if (!ps || ps->type != OSCParameterDeclarations::ParameterType::PARAM_TYPE_STRING)
    {
        return -1;
    }

    value = ps->value._string.c_str();

    return 0;
}

int Parameters::getParameterValueBool(std::string name, bool& value) const
{
    const auto* ps = getParameterEntryReadOnly(name);

    if (!ps || ps->type != OSCParameterDeclarations::ParameterType::PARAM_TYPE_BOOL)
    {
        return -1;
    }

    value = ps->value._bool;

    return 0;
}

std::string Parameters::getParameterValueAsString(std::string name) const
{
    const auto* ps = getParameterEntryReadOnly(name);

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
                        if (SE_Env::Inst().GetOptions().GetOptionSetByEnum(esmini_options::CONFIG_ENUM::SAVE_XOSC_RESOLVED))
                        {
                            attr.set_value(rs._double);
                        }
                        return_value = std::to_string(rs._double);
                    }
                    else if (rs.type == EXPR_RETURN_STRING)
                    {
                        LOG_INFO("Expr {} = {} = {}", attr.value(), expr, rs._string.string);
                        if (SE_Env::Inst().GetOptions().GetOptionSetByEnum(esmini_options::CONFIG_ENUM::SAVE_XOSC_RESOLVED))
                        {
                            attr.set_value(rs._string.string);
                        }
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
                if (SE_Env::Inst().GetOptions().GetOptionSetByEnum(esmini_options::CONFIG_ENUM::SAVE_XOSC_RESOLVED))
                {
                    attr.set_value(return_value.c_str());
                }
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
        if (param.name.find("::") != std::string::npos)
        {
            LOG_ERROR_AND_QUIT("Invalid parameter name {}: '::' is reserved for instance parameter references", param.name);
        }

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
                LOG_WARN("Parameter value {} updated from {} to {}", pos->name, param.value._string, pos->value._string);
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
    instance_parameters_.clear();
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
