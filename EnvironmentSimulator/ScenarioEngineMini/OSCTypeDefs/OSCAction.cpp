#include "OSCAction.hpp"

using namespace scenarioengine;

std::string OSCAction::basetype2str(BaseType type)
{
	if (type == BaseType::GLOBAL)
	{
		return "Global";
	}
	else if(type == BaseType::PRIVATE)
	{
		return "Private";
	}
	else if (type == BaseType::USER_DEFINED)
	{
		return "User defined";
	}
	else
	{
		LOG("Undefined Base Type: %d", type);
	}

	return std::string();
}
