#include "Entities.hpp"
#include "CommonMini.hpp"


Entities::Entities()
{
}


void Entities::printEntities()
{
	LOG("---------------------------------------");
	LOG("Print of Entities ");

	for (int i = 0; i < Object.size(); i++)
	{
		LOG("Entities - Object ");
		LOG("\tname = %s", Object[i].name.c_str());
		LOG("Entities - Object - Vehicle");
		Object[i].Vehicle.printOSCVehicle();

		LOG("Entities - Object - Controller");
		Object[i].Controller.CatalogReference.printOSCCatalogReference();

		LOG("Entities - Object - Controller");
		Object[i].Controller.Driver.printOSCDriver();

		LOG("Entities - Object - Controller - PedestrianController");
		Object[i].Controller.PedestrianController.printOSCPedestrianController();
		LOG("---------------------------------------");
	}
}


