## esmini OSMP FMU

This example compiles an [OSMP FMU](https://github.com/OpenSimulationInterface/osi-sensor-model-packaging) with which esmini can be used in an FMU-based co-simulation. <br>
Esmini is initialized via the API in the *doInit* function of the FMU.
The path to the xosc file has to be set with the FMI parameter *xosc_path*. <br>
In the *doCalc* function (a subsequent function of *doStep*), the OSI ground truth data is fetched via the API and copied to an OSI SensorView message.
The SensorView is the output of the FMU.

### Usage
- First build esmini according to the [build instructions](https://esmini.github.io/#_build_esmini_quick_guide).
- Build the FMU.

  ```bash
  mkdir build
  cd build
  cmake ..
  cmake --build .
  ```

- Run the FMU in a co-simulation, e.g. by using the open source framework [OpenMCx](https://github.com/eclipse/openmcx).
Be sure to set the OpenScenario file as FMI parameter *xosc_path*. 
Furthermore, you can set, if the scenario viewer is displayed with the boolean FMI parameter *use_viewer*.

### Example co-simulation
An example system structure definition file (ssd) is included in this example.
It connects this esmini OSI Source FMU to the [OSMPDummySensor](https://github.com/OpenSimulationInterface/osi-sensor-model-packaging/tree/master/examples/OSMPDummySensor), as shown in the image below.
![example_system_structure.png](example_system_structure.png)
Be sure to set the OpenScenario file as FMI parameter *xosc_path* in the ssd file. 