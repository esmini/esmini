#include "Entities.hpp"


Entities::Entities()
{
	std::cout << "Entities: New Entities created" << std::endl;
}


void Entities::printEntities()
{
	std::cout << "---------------------------------------" << std::endl;
	std::cout << "Print of Entities " << std::endl;

	for (int i = 0; i < Object.size(); i++)
	{
		std::cout << "Entities - Object " << std::endl;
		std::cout << "\t" << "name = " << Object[i].name << std::endl;
		std::cout << std::endl;

		std::cout << "Entities - Object - Vehicle" << std::endl;
		Object[i].Vehicle.printOSCVehicle();

		std::cout << "Entities - Object - Controller" << std::endl;
		Object[i].Controller.CatalogReference.printOSCCatalogReference();

		std::cout << "Entities - Object - Controller" << std::endl;
		Object[i].Controller.Driver.printOSCDriver();

		std::cout << "Entities - Object - Controller - PedestrianController" << std::endl;
		Object[i].Controller.PedestrianController.printOSCPedestrianController();
		std::cout << "---------------------------------------" << std::endl;
		std::cout << std::endl;
	}
}


