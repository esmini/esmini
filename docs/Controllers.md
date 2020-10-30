# Controllers in esmini

## Controller concept
OpenSCENARIO provides the controller concept as a way to outsource control of scenario entities. From the [User Guide](https://releases.asam.net/OpenSCENARIO/1.0.0/ASAM_OpenSCENARIO_BS-1-2_User-Guide_V1-0-0.html#_general_concepts):


>Controllers can be assigned to ScenarioObjects of type Vehicle or Pedestrian. Once assigned, Controllers are activated for a given domain (i.e. longitudinal or lateral) using the ActivateControllerAction (Private action).

>While the ActivateControllerAction is executing, the Controller assigned to that ScenarioObject will manage that domain. Controllers may be internal (part of the simulator) or external (defined in another file). Intended use cases for Controllers include:
>- Specifying that a vehicle should be controlled by the system under test
>- Defining "smart actor" behavior, where a Controller will take intelligent decisions in response to the road network and/or other vehicles. Hence, Controllers can be used, for example, to make agents in a scenario behave in a human-like way
>- Assigning a vehicle to direct human control
>
>The Controller element contains Properties, which can be used to specify Controller behavior either directly or by a File reference.

## Background and motivation
esmini version < 2.0 totally lacked support for controllers. Instead some similar functionality were implemented as part of different example-applications. For example, EgoSimulator provided interactive control of one vehicle. There was also an "external" mode which allowed for an external vehicle simulator to report current position for the typical Ego (VUT/SUT...) vehicle.

First, this approach made the example code of simple applications complex. Secondly, it limited the use cases of esmini since functionality was tightly embedded in the applications.

Controllers provides a much more flexible way of adding functionality to esmini, in a way harmonizing with the standard (OpenSCENARIO 1.0). So it's a win-win. One side effect of this "outsourcing" of functionality is that the demo applications can be reduced to a minimum both in number and size.

## Brief on implementation
Controllers are introduced in esmini v2.0. Briefly it works as follows:

There is a collection of embedded controllers coming with esmini. Each controller inherit from the base class Controller ([Controller.h](https://github.com/esmini/esmini/blob/c98a3d5db483ee2142c2c01f4b100eec36cc4e90/EnvironmentSimulator/Modules/Controllers/Controller.hpp#L30)/cpp). In order for esmini to be aware of the existense of a controller it has to be registered. This is done through the ScenarioReader method [RegisterController](https://github.com/esmini/esmini/blob/c98a3d5db483ee2142c2c01f4b100eec36cc4e90/EnvironmentSimulator/Modules/ScenarioEngine/SourceFiles/ScenarioReader.hpp#L118). It will put the controller into a collection to have available when the scenario is being parsed. So all controllers need to be registered prior to loading the scenario.

A controller is registered with the following information:
1. Its static name which is used as identifier. 
2. A pointer to a function instantiating the controller.  

This arhictecture allows for an external module to create a controller and registering it without modifying any of esmini modules. So it's a semi-plugin concept you can say. 

> Note: Even though ScenarioReader have a [helper function](https://github.com/esmini/esmini/blob/c98a3d5db483ee2142c2c01f4b100eec36cc4e90/EnvironmentSimulator/Modules/ScenarioEngine/SourceFiles/ScenarioReader.cpp#L40) for registering all the embedded controllers the RegisterController can be called from any module directly, at any time prior to scenario initialization.

esmini catalog framework supports controllers as well, so controllers can be defined in catalogs as presets, just like vehicles and routes for example.

## How it works for the user

Controllers are completely handled in the OpenSCENARIO file. With one exception: esmini provides the `--disable-controllers` option which totally ignore any controllers, just performing the scenario with DefaultControllers, which can be handy when previewing and debugging scenarios.

In terms of OpenSCENARIO a controller is assigned to an entity (object) in any of two ways:
1. As part of the Entities section and ScenarioObject definition, using the `ObjectController` element.
2. The `AssignControllerAction` which can be triggered as any action at any time during the simulation.

Example 1 using ObjectController:

```
<Entities>
   <ScenarioObject name="Ego">
     <CatalogReference catalogName="VehicleCatalog" entryName="$HostVehicle"/>
      <ObjectController>
          <Controller name="MyController" >
             <Properties>
                 <Property name="esminiController" value="SloppyDriverController" />
                 <Property name="sloppiness" value="0.9" />
             </Properties>
          </Controller>
      </ObjectController>        
   </ScenarioObject>
</Entities>
```

Example 2 using ObjectController and catalog reference:
```
<Entities>
   <ScenarioObject name="Ego">
     <CatalogReference catalogName="VehicleCatalog" entryName="$HostVehicle"/>
        <ObjectController>
            <CatalogReference catalogName="ControllerCatalog" entryName="interactiveDriver" />
        </ObjectController>
   </ScenarioObject>
</Entities>
```

Once assigned the controller must finally be activated using the `ActivateControllerAction` which can be triggered at any time. It provides an attribute to specify which domain(s) to control: Lateral, Longitudinal or both.

Example 1 part of Init:
```
<Init>
   <Actions>
      <Private entityRef="Ego">
         <PrivateAction>
            <TeleportAction>
               <Position>
                  <LanePosition roadId="0" laneId="-3" offset="0" s="$EgoStartS"/>
               </Position>
            </TeleportAction>
         </PrivateAction>
         <PrivateAction>
              <ActivateControllerAction longitudinal="true" lateral="true" />
         </PrivateAction>
      </Private>
   </Actions>
</Init>
```

Example 2 in Storyboard (both assign and activate):
```
<Event name="InteractiveEvent" maximumExecutionCount="1" priority="overwrite">
   <Action name="AssignControllerAction">
     <PrivateAction>
          <ControllerAction>
              <AssignControllerAction>
                  <CatalogReference catalogName="ControllerCatalog" entryName="interactiveDriver" />
              </AssignControllerAction> 
          </ControllerAction>
      </PrivateAction>
   </Action>
   <Action name="ActivateControllerAction2">
      <PrivateAction>
          <ActivateControllerAction longitudinal="true" lateral="true" />
      </PrivateAction>
   </Action>
   <StartTrigger>
      <ConditionGroup>
         <Condition name="" delay="0" conditionEdge="none">
            <ByValueCondition>
               <SimulationTimeCondition value="20" rule="greaterThan"/>
            </ByValueCondition>
         </Condition>
      </ConditionGroup>
   </StartTrigger>
</Event>                  

```

To disable any controller simply apply `ActivateControllerAction` on no domain, like:
```
<ActivateControllerAction longitudinal="false" lateral="false" />
``` 
That should work in all OpenSCENARIO supporting tools. In esmini you can also assign "DefaultController" which will disconnect any assigned controller.


## esmini embedded controllers
Below is the list of available controllers in esmini. Note that only DefaultController is related to the OpenSCENARIO standard. The other ones are esmini-specific and will not work in other tools. So far there is no standard plugin-arhcitecture for controllers, but future may bring...

#### DefaultController
Performs actions exactly as specified in the OpenSCENARIO file. Assigned to entities by default. Can be assigned at any point to "unassign" any currently assigned controller.

#### InteractiveController
A simple vehicle model controlled by the user via keyboard (arrow keys). [cut-in_interactive.xosc](https://github.com/esmini/esmini/blob/master/resources/xosc/cut-in_interactive.xosc)

Domain support: `longitudinal` and/or `lateral` (independent)

Properties: None

#### FollowGhost
A simple driver model. A ghost-twin performing the events a few seconds ahead. The entity will then do its best to  mimic the motion and speed of the ghost by follow the trajectory. The primary purpose of this controller is to provide an example, but it can be useful to smoothen out the sometimes synthetic feel of pure default controller. See [follow_ghost.xosc](https://github.com/esmini/esmini/blob/master/resources/xosc/follow_ghost.xosc) for an example.

Domain support: `longitudinal` and `lateral` (in combination only).

Properties: 
- `headstartTime` (seconds): Ghost will be launched at `time = 0 - headstartTime`

#### ExternalController
The entity will not be moved by the scenario. Instead its state (position, rotation ...) is expected to be reported from external simulator via API, e.g. [SE_ReportObjectPos](https://github.com/esmini/esmini/blob/c98a3d5db483ee2142c2c01f4b100eec36cc4e90/EnvironmentSimulator/Libraries/esminiLib/esminiLib.hpp#L136). Ghost trajectory can optionally be created for an external driver model to use as reference.

Domain support: `longitudinal` and/or `lateral` (independent).

Properties: 
- `mode` (override/additive): Disable default controller (override - default) or apply default controller before handing over to this controller (additive)
- `useGhost` (true/false): Enable or disable (default) the ghost feature
- `headstartTime` (seconds): Optional ghost will be launched at `time = 0 - headstartTime`

#### SumoController 
A way of integrating SUMO controlled vehicles in a scenario. OpenSCENARIO vehicles are reported to SUMO, and SUMO vehicles are reported back to esmini. A reference to a SUMO config file is provided as input to the controller. See [cut-in_sumo.xosc](https://github.com/esmini/esmini/blob/master/resources/xosc/cut-in_sumo.xosc) for an example.

Domain support: `longitudinal` and `lateral` (in combination only).

Properties:
- `file`: Path to the SUMO configuration file


> Note: Sumo controller is always and automatically assigned and activated from start of the simulation. 

#### SloppyDriverController
 Another example of a driver model, adding some random speed and lateral deviation to the default road following behavior. This controller is useless, it's pure purpose is to provide an example of separating lontitudinal and lateral control. See [controller_test.xosc](https://github.com/esmini/esmini/blob/master/resources/xosc/controller_test.xosc) for an example.

Domain support: `longitudinal` and/or `lateral` (independent).

Properties:
- `sloppiness` (double [0:1]): Level of sloppiness, 0=perfect 1=bad
- - `mode` (override/additive): Disable default controller (override - default) or apply default controller before handing over to this controller (additive)

