console.log("openscenario begin")
// get js lib
const esminijs = window.esmini
// get an instance
const esminilib = esminijs()

// open sceanario content
const xoscContent = `
<?xml version="1.0" encoding="UTF-8"?>
<OpenSCENARIO>
  <FileHeader author="SimOne" date="2022-09-07T16:45:01" description="双侧车道线巡航-直道" revMajor="1" revMinor="0" />
  <ParameterDeclarations>
    <ParameterDeclaration name="$Ego_vi" parameterType="double" value="19.444444444444443" />
    <ParameterDeclaration name="$Ego_vt" parameterType="double" value="22.22222222222222" />
  </ParameterDeclarations>
  <CatalogLocations />
  <RoadNetwork>
    <LogicFile filepath="QS0101-D0909-set60kph-straight.xodr" />
    <SceneGraphFile filepath="" />
  </RoadNetwork>
  <Entities>
    <ScenarioObject name="Ego">
      <Vehicle name="simonedriver" vehicleCategory="car">
        <ParameterDeclarations />
        <Performance maxAcceleration="200" maxDeceleration="10" maxSpeed="69.444" />
        <BoundingBox>
          <Center x="1.5" y="0" z="0.9" />
          <Dimensions height="1.48" length="4.93" width="1.86" />
        </BoundingBox>
        <Axles>
          <FrontAxle maxSteering="0.5" positionX="3.1" positionZ="0.3" trackWidth="1.8" wheelDiameter="0.6" />
          <RearAxle maxSteering="0" positionX="0" positionZ="0.3" trackWidth="1.8" wheelDiameter="0.6" />
        </Axles>
        <Properties>
          <Property name="category" value="Ego" />
          <Property name="model" value="simonedriver" />
          <Property name="name" value="Ego" />
        </Properties>
      </Vehicle>
      <ObjectController>
        <Controller name="SimOneDriver">
          <Properties>
            <Property name="name" value="SimOneDriver" />
          </Properties>
        </Controller>
      </ObjectController>
    </ScenarioObject>
  </Entities>
  <Storyboard>
    <Init>
      <Actions>
        <GlobalAction>
          <EnvironmentAction>
            <Environment name="Environment">
              <TimeOfDay animation="false" dateTime="2022-09-07T10:00:00" />
              <Weather cloudState="free">
                <Sun azimuth="0" elevation="1.57" intensity="60000" />
                <Fog visualRange="1200" />
                <Precipitation intensity="0" precipitationType="dry" />
              </Weather>
              <RoadCondition frictionScaleFactor="0.8">
                <Properties>
                  <Property name="humidityLevel" value="0.1" />
                  <Property name="dirtyLevel" value="0.1" />
                </Properties>
              </RoadCondition>
            </Environment>
          </EnvironmentAction>
        </GlobalAction>
        <Private entityRef="Ego">
          <PrivateAction>
            <LongitudinalAction>
              <SpeedAction>
                <SpeedActionDynamics dynamicsDimension="time" dynamicsShape="step" value="0" />
                <SpeedActionTarget>
                  <AbsoluteTargetSpeed value="$Ego_vi" />
                </SpeedActionTarget>
              </SpeedAction>
            </LongitudinalAction>
          </PrivateAction>
          <PrivateAction>
            <TeleportAction>
              <Position>
                <LanePosition laneId="-2" offset="0" roadId="1" s="100">
                  <Orientation h="0" p="0" r="0" type="absolute" />
                </LanePosition>
              </Position>
            </TeleportAction>
          </PrivateAction>
        </Private>
      </Actions>
    </Init>
    <Story name="MyStory">
      <ParameterDeclarations />
      <Act name="MyAct1">
        <ManeuverGroup maximumExecutionCount="1" name="MySequence1">
          <Actors selectTriggeringEntities="false">
            <EntityRef entityRef="Ego" />
          </Actors>
          <Maneuver name="Maneuver send_command_action">
            <Event name="Event1" priority="overwrite">
              <Action name="Action1">
                <UserDefinedAction>
                  <CustomCommandAction type="Command">CruiseSpeed=60</CustomCommandAction>
                </UserDefinedAction>
              </Action>
              <StartTrigger>
                <ConditionGroup>
                  <Condition conditionEdge="none" delay="0" name="TimeCondition1">
                    <ByValueCondition>
                      <SimulationTimeCondition rule="greaterThan" value="30" />
                    </ByValueCondition>
                  </Condition>
                </ConditionGroup>
              </StartTrigger>
            </Event>
          </Maneuver>
        </ManeuverGroup>
        <StartTrigger>
          <ConditionGroup>
            <Condition conditionEdge="none" delay="0" name="TimeCondition3">
              <ByValueCondition>
                <SimulationTimeCondition rule="greaterThan" value="0" />
              </ByValueCondition>
            </Condition>
          </ConditionGroup>
        </StartTrigger>
      </Act>
    </Story>
    <StopTrigger>
      <ConditionGroup>
        <Condition conditionEdge="none" delay="0" name="TimeConditionStop">
          <ByValueCondition>
            <SimulationTimeCondition rule="greaterThan" value="60" />
          </ByValueCondition>
        </Condition>
      </ConditionGroup>
    </StopTrigger>
  </Storyboard>
</OpenSCENARIO>
`
// open drive content
const xodrContent = `
<?xml version="1.0" encoding="utf-8" ?>
<OpenDRIVE>
	<header revMajor="1" revMinor="4" name="" version="2.00" date="2021-05-14T11:48:57" north="20.999260" west="-323.636046" east="411.636046" south="-11.999260" vendor="51World">
		<geoReference/>
		<userData code="originLat" value="0.000000"/>
		<userData code="originLong" value="0.000000"/>
		<userData code="originAlt" value="0.000000"/>
		<userData code="originHdg" value="0.000000"/>
		<userData code="RnkMajorVersion" value="1"/>
		<userData code="RnkMinorVersion" value="0"/>
		<userData code="localEnuExt" value="6378137.000000,0.000000,0.000000,;0.000000,1.000000,0.000000,;0.000000,0.000000,1.000000,;1.000000,0.000000,0.000000,"/>
	</header>
	<road junction="-1" length="10000.000000" name="Road(1)" id="1">
		<type s="0.000000" type="town">
			<speed max="undefined" unit="km/h"/>
		</type>
		<planView>
			<geometry hdg="6.272301" length="10000.000000" s="0.000000" x="0.000000" y="0.000000">
				<line/>
			</geometry>
		</planView>
		<elevationProfile>
			<elevation a="0.000000" b="0.000000" c="0.000000" d="0.000000" s="0.000000"/>
		</elevationProfile>
		<lateralProfile>
			<crossfall a="0.000000" b="0.000000" c="0.000000" d="0.000000" s="0.000000" side="left"/>
			<crossfall a="0.000000" b="0.000000" c="0.000000" d="0.000000" s="0.000000" side="right"/>
			<superelevation a="0.000000" b="0.000000" c="0.000000" d="0.000000" s="0.000000"/>
		</lateralProfile>
		<lanes>
			<laneSection s="0.000000" singleSide="false">
				<center>
					<lane id="0" level="false" type="none">
						<roadMark sOffset="0.000000" color="yellow" height="0.000000" laneChange="none" material="" type="solid" weight="standard" width="0.150000"/>
					</lane>
				</center>
				<left>
					<lane id="1" level="false" type="driving">
						<height sOffset="0.000000" inner="0.000000" outer="0.000000"/>
						<height sOffset="10000.000000" inner="0.000000" outer="0.000000"/>
						<roadMark sOffset="0.000000" color="white" height="0.000000" laneChange="both" material="" type="broken" weight="standard" width="0.150000"/>
						<width sOffset="0.000000" a="3.500000" b="0.000000" c="0.000000" d="0.000000"/>
					</lane>
					<lane id="2" level="false" type="driving">
						<height sOffset="0.000000" inner="0.000000" outer="0.000000"/>
						<height sOffset="10000.000000" inner="0.000000" outer="0.000000"/>
						<roadMark sOffset="0.000000" color="white" height="0.000000" laneChange="both" material="" type="broken" weight="standard" width="0.150000"/>
						<width sOffset="0.000000" a="3.500000" b="0.000000" c="0.000000" d="0.000000"/>
					</lane>
					<lane id="3" level="false" type="driving">
						<height sOffset="0.000000" inner="0.000000" outer="0.000000"/>
						<height sOffset="10000.000000" inner="0.000000" outer="0.000000"/>
						<roadMark sOffset="0.000000" color="white" height="0.000000" laneChange="both" material="" type="solid" weight="standard" width="0.150000"/>
						<width sOffset="0.000000" a="3.500000" b="0.000000" c="0.000000" d="0.000000"/>
					</lane>
				</left>
				<right>
					<lane id="-1" level="false" type="driving">
						<height sOffset="0.000000" inner="0.000000" outer="0.000000"/>
						<height sOffset="10000.000000" inner="0.000000" outer="0.000000"/>
						<roadMark sOffset="0.000000" color="white" height="0.000000" laneChange="both" material="" type="broken" weight="standard" width="0.150000"/>
						<width sOffset="0.000000" a="3.500000" b="0.000000" c="0.000000" d="0.000000"/>
					</lane>
					<lane id="-2" level="false" type="driving">
						<height sOffset="0.000000" inner="0.000000" outer="0.000000"/>
						<height sOffset="10000.000000" inner="0.000000" outer="0.000000"/>
						<roadMark sOffset="0.000000" color="white" height="0.000000" laneChange="both" material="" type="broken" weight="standard" width="0.150000"/>
						<width sOffset="0.000000" a="3.500000" b="0.000000" c="0.000000" d="0.000000"/>
					</lane>
					<lane id="-3" level="false" type="driving">
						<height sOffset="0.000000" inner="0.000000" outer="0.000000"/>
						<height sOffset="10000.000000" inner="0.000000" outer="0.000000"/>
						<roadMark sOffset="0.000000" color="white" height="0.000000" laneChange="both" material="" type="solid" weight="standard" width="0.150000"/>
						<width sOffset="0.000000" a="3.500000" b="0.000000" c="0.000000" d="0.000000"/>
					</lane>
				</right>
			</laneSection>
			<laneOffset a="0.000000" b="0.000000" c="0.000000" d="0.000000" s="0.000000"/>
		</lanes>
		<objects/>
		<signals/>
	</road>
</OpenDRIVE>
`

const xoscName = 'QS0101-D0909-set60kph-straight.xosc'
const xodrName = 'QS0101-D0909-set60kph-straight.xodr'
// bind files
esminilib['FS_createDataFile']('.', xoscName, xoscContent, true, true)
esminilib['FS_createDataFile']('.', xodrName, xodrContent, true, true)

// 
let openScenario = new esminilib.OpenScenario('./' + xoscName, {
    max_loop: 10e3, // max number of frames 
    min_time_step: 1.0 / 120.0, // min frame gap
    max_time_step: 1.0 / 25, //  max frame gap
    dt: 0.0 // frame gap
  })

let objectStates = openScenario.get_object_state_by_second(60, 1)

for (let i = 0; i < objectStates.size(); i++) {
    const objectState = objectStates.get(i)
    console.info('object %s pos[x:%d,y:%d,z:%d]', objectState.name, objectState.x, objectState.y, objectState.z)
  }