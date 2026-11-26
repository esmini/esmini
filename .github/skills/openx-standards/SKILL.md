---
name: openx-standards
description: Use when investigating or implementing ASAM OpenSCENARIO XML (OSC, .xosc) or OpenDRIVE (ODR, .xodr) behavior in esmini - parsing, attributes/enums, semantics, coordinate conventions, version differences, spec compliance, or deciding which spec chapter/class page to read. Gives the cheapest lookup order (local XSDs and coverage files first, targeted spec pages last) and maps spec concepts to esmini code.
---

# OpenX standards lookup (OpenSCENARIO XML + OpenDRIVE)

Goal: find the authoritative answer with as little reading as possible. The spec HTML pages are large (typically 40-180 KB each), so **never fetch whole pages or the full index "just to look around"**.

## 1. Cheapest-first lookup order

1. **Local XSD schemas** (`resources/schema/`) - exact element/attribute names, types, required vs optional, enum values, per version. Use `grep` with context, not full reads.
   - OSC: `OpenSCENARIOv1.0.xsd`, `v1.1.1`, `v1.2`, `v1.3`, `v1.4`
   - ODR: `OpenDRIVE_1.4H.xsd`, `OpenDRIVE_1.5.xsd`, `OpenDRIVE_1.6/`..`OpenDRIVE_1.9/` (split per topic: `*_Core`, `*_Road`, `*_Lane`, `*_Junction`, `*_Object`, `*_Signal`, `*_Railroad`)
   - Example: `grep -n -A12 'name="LaneChangeAction"' resources/schema/OpenSCENARIOv1.3.xsd`
   - XSDs do **not** contain semantics or rules - go to step 4 for those.
2. **Coverage files** - what esmini implements per element. Check before assuming a feature is supported.
   - [osc_coverage.txt](../../../osc_coverage.txt): OSC element tree with `Yes` / blank / notes.
   - [odr_coverage.txt](../../../odr_coverage.txt): ODR element tree with `YES` / `NO`; written for ODR 1.5, with features from later versions marked by their version number.
3. **esmini's own docs and release notes** - `docs/user_guide/known-deviations.adoc` (intentional deviations from the specs), `release_notes.md` (grep for the feature/version), other `docs/user_guide/*.adoc`, `docs/osc-extensions.xml` (esmini-specific extensions).
4. **Online spec, one targeted page** (see URL patterns below). Fetch with a small `max_length` and paginate with `start_index`; stop as soon as the answer is found.
5. **Existing scenarios/roads as worked examples** - `resources/xosc`, `resources/xodr`, `EnvironmentSimulator/Unittest/{xosc,xodr}`, and submodules `test/ASAM-OSC-XML-scenarios`, `test/OSC-ALKS-scenarios`. Grep for the element name.

## 2. Versions

- esmini README states support for selected features of OSC **1.0-1.4**, ODR **1.4-1.9** and OSI **3.5, 3.8** (coverage limited, see the coverage files). Files declare their version in `<FileHeader revMajor= revMinor=>` (OSC) / `<header revMajor= revMinor=>` (ODR); esmini branches on it, e.g. `versionMinor_` in `ScenarioReader` and `OpenDrive`.
- ODR support includes recent additions (e.g. 1.7 direct junctions, 1.9 temporary lane layers - see `release_notes.md`).
- Behavior can differ between spec versions (e.g. OSC >= 1.1 uses absolute `distance`/`timeGap` plus `displacement`). Always confirm which version applies and read the matching version's page.
- Latest published: **OSC XML 1.4.0**, **OpenDRIVE 1.9.0**. For older versions replace the version segment in the URLs (e.g. `v1.3.1`); verify the exact tag from the landing pages below.

## 3. Online spec URL patterns (verified; HTML, Antora-generated)

Landing pages (redirect to latest):
- OSC: https://publications.pages.asam.net/standards/ASAM_OpenSCENARIO
- ODR: https://publications.pages.asam.net/standards/ASAM_OpenDRIVE

Bases (pinned - deep `latest/` links for OSC class pages return 404, so pin the version):
- `OSC = https://publications.pages.asam.net/standards/ASAM_OpenSCENARIO/ASAM_OpenSCENARIO_XML/v1.4.0/`
- `ODR = https://publications.pages.asam.net/standards/ASAM_OpenDRIVE/ASAM_OpenDRIVE_Specification/v1.9.0/specification/`

### OpenSCENARIO XML

| Need | URL (append to `OSC`) |
| --- | --- |
| **Any element / attribute / enum** (best value per byte) | `generated/content/<Name>.html`, e.g. [LaneChangeAction](https://publications.pages.asam.net/standards/ASAM_OpenSCENARIO/ASAM_OpenSCENARIO_XML/v1.4.0/generated/content/LaneChangeAction.html). Names are the XSD type names without the `Type` suffix (`Position`, `TeleportAction`, `TimeHeadwayCondition`, `FollowingMode`, `ReferenceContext`, `LaneLayerType`...). Interfaces like `CatalogElement`, `Entity`, `StoryboardElement` live here too. |
| Which actions/conditions exist and where they may be used (private/global/user-defined, init vs event) | `12_annexes/12_01_action_table.html` |
| Entity-type restrictions (what applies to vehicle/pedestrian/misc object) | `12_annexes/12_02_entity_type_constraints.html` |
| Validation rules the checker enforces | `12_annexes/12_03_checker_rules.html` |
| Coordinate systems, orientation, reference contexts | `06_general_concepts/06_03_coordinate_systems.html` |
| Distances (longitudinal/lateral, freespace, `CoordinateSystem`, `RelativeDistanceType`) | `06_general_concepts/06_04_distances.html` |
| Speed / dynamics shapes and steady-state | `06_general_concepts/06_05_speed.html` |
| Controllers, override actions | `06_general_concepts/06_06_controllers.html` |
| Routes / trajectories (shapes, `FollowingMode`) | `06_general_concepts/06_08_routes.html`, `06_09_trajectories.html` |
| Traffic (swarm/source/sink), traffic signals | `06_general_concepts/06_10_traffic_simulation.html`, `06_11_traffic_signals.html` |
| Variables, properties, monitors | `06_general_concepts/06_12_variables.html`, `06_13_properties.html`, `06_14_monitors.html` |
| Storyboard/entities, ManeuverGroup/Event/Maneuver, actions, conditions/triggers (structure) | `07_components_scenario/07_02_storyboard_entities.html` ... `07_06_conditions_triggers.html` |
| **Runtime semantics**: element states, transitions, execution order, init section, priority, `maximumExecutionCount`, `StoryboardElementState` | `08_runtime_scenario/08_01_states.html`, `08_02_transitions.html`, `08_03_execution.html`, `08_04_storyboard_element_execution.html`, `08_05_init_section_execution.html` |
| Parameters, expressions, parameter distributions | `09_reuse_mechanisms/09_01_parameters.html`, `09_02_parameter_expressions.html`, `09_03_parameter_distribution.html` |
| Catalogs and catalog reference resolution | `09_reuse_mechanisms/09_04_catalogs.html`, `09_06_resolve_catalog.html` |
| Backward compatibility / changes between versions | `05_backward_compatibility/01_backward_compatibility.html` |
| Worked examples (cut-in, trailer, signalized intersection...) | `11_examples/11_01_cutin.html` ... `11_12_traffic_signals_actions_conditions.html` |

### OpenDRIVE

| Need (append to `ODR`) | Page |
| --- | --- |
| Root element, header, element overview, attribute conventions | `06_general_architecture/06_03_root_element.html`, `06_04_header.html`, `06_05_overview_elements.html`, `06_02_attributes.html` |
| Coordinate systems: inertial, **s/t/h reference line**, local, georeferencing | `08_coordinate_systems/08_02_inertial_coordinate_system.html`, `08_03_reference_line_coordinate_system.html`, `08_04_local_coordinate_system.html`, `08_05_geo_referencing.html` |
| Geometry primitives (line, spiral, arc, paramPoly3, deprecated poly3) | `09_geometries/09_03_straight_line.html` ... `09_07_poly3.html` |
| Road linkage, road type/speed, elevation/superelevation/shape | `10_roads/10_03_road_linkage.html`, `10_04_road_type.html`, `10_05_elevation.html` |
| Lanes: sections, offset, linkage, width/geometry, properties (speed, access...), markings, special rules | `11_lanes/11_04_lane_sections.html`, `11_05_lane_offset.html`, `11_06_lane_link.html`, `11_07_lane_geometry.html`, `11_08_lane_properties.html`, `11_09_lane_road_markings.html`, `11_10_specific_lane_rules.html` |
| Lane layers (1.9 temporary lanes) and lane groups | `11_lanes/11_02_lane_layers.html`, `11_03_lane_groups.html` |
| Junctions: common, incoming/connecting roads, **direct junctions**, virtual, crossings, boundary, elevation grid | `12_junctions/12_02_common_junctions.html` ... `12_14_signal_synchronization_groups.html` |
| Objects (outline, repeat, lane validity, tunnels, bridges) | `13_objects/13_01_introduction.html` ... `13_14_object_examples.html` |
| Signals (validity, dependency, controllers, semantics) | `14_signals/14_01_introduction.html` ... `14_08_signal_semantics.html` |
| Smoothness/continuity rules (lane width, offset...) | `16_annexes/smoothness_of_lanes/top_ter_smoothness_of_lanes.html` |
| Checker rules (what a valid file must satisfy) | `16_annexes/map_rules.html` (~180 KB - grep, don't read) |
| Enumerations / data types | `16_annexes/enumerations/map_uml_enumerations.html` (~150 KB), `16_annexes/map_uml_data_types.html` - **prefer the local XSD** for enum values |
| What changed per version | `16_annexes/revision_history/ASAM_OpenDRIVE_revision_history_V1-9-0.html` (and `V1-8-0`), `05_backward_compatibility/05_backward_compatibility.html` |
| Companion guides | Junction guideline `.../ASAM_OpenDRIVE_Junction_guideline/1.0.0/junction-guidelines/index.html`, signal catalog `.../ASAM_OpenDRIVE_Signal_reference/1.0.0/signal-catalog/index.html` (under `https://publications.pages.asam.net/standards/`) |

ODR has no per-element "class" pages; element/attribute tables are inside the topic chapter above. The chapter number matches the topic (9 geometries, 10 roads, 11 lanes, 12 junctions, 13 objects, 14 signals, 15 railroads).

### Fetching tips (credit-saving)

- Use `web_fetch` with `max_length` ~4000-6000 and `start_index` to page; the converted markdown drops the navigation boilerplate.
- To search a big page without reading it, download it once and grep it, e.g.
  `curl -s <url> -o $env:TEMP\page.html` then grep for the attribute/keyword. Never save downloads into the repo.
- Downloadable deliverables (XSD, examples, UML model) are linked from the "Deliverables" menu of each spec index; the repo already ships the XSDs.

## 4. Spec concept -> esmini code

OpenSCENARIO (`EnvironmentSimulator/Modules/ScenarioEngine/`):

| Spec concept | Where in esmini |
| --- | --- |
| XML parsing of everything | `SourceFiles/ScenarioReader.cpp` - `parseOSCHeader`, `parseRoadNetwork`, `parseCatalogs`, `parseEntities`, `parseOSCVehicle/Pedestrian/MiscObject/ObjectController`, `parseOSCPosition`, `parseOSCOrientation`, `parseOSCRoute`, `parseTrajectory`, `parseInit`, `parseOSCPrivateAction`, `parseOSCGlobalAction`, `parseOSCUserDefinedAction`, `parseOSCCondition`, `parseTrigger`, `parseOSCManeuver`, `parseStoryBoard`, `parseOSCEnvironment`, `parseOSCFile` |
| Storyboard/Story/Act/ManeuverGroup/Event element state machine | `SourceFiles/Storyboard.*`, `SourceFiles/StoryboardElement.*` |
| Private (entity) actions | `OSCTypeDefs/OSCPrivateAction.*` |
| Global actions (environment, entity, traffic swarm/source/sink, infrastructure) | `OSCTypeDefs/OSCGlobalAction.*`, `OSCEnvironment.*`, `OSCSwarmTrafficGeometry.*` |
| Conditions / triggers | `OSCTypeDefs/OSCCondition.*`, `SourceFiles/Storyboard.*` |
| Positions (World, Road, Lane, Relative*, Route, Trajectory, Geo) | `OSCTypeDefs/OSCPosition.*` -> `roadmanager::Position` |
| Entities, entity objects, trailers | `SourceFiles/Entities.*` |
| Catalogs and catalog parameter resolution | `SourceFiles/Catalogs.*`, `SourceFiles/Parameters.*`, `OSCTypeDefs/OSCParameterDeclarations.hpp` |
| Parameters, expressions (`${...}`) | `SourceFiles/Parameters.*`, `SourceFiles/simple_expr.*` |
| Parameter value distributions | `OSCTypeDefs/OSCParameterDistribution.*` |
| Controllers (`ObjectController`, `ControllerType`) | `Modules/Controllers/` |
| Engine loop / execution order | `SourceFiles/ScenarioEngine.cpp` |

OpenDRIVE (`EnvironmentSimulator/Modules/RoadManager/RoadManager.{hpp,cpp}`):

| Spec concept | Where in esmini |
| --- | --- |
| File parsing (`<OpenDRIVE>`, header, roads, junctions, signals, objects) | `OpenDrive::ParseOpenDriveXML`, `LoadOpenDriveFile`, `LoadOpenDriveFromXMLString` |
| `road`, planView geometries, elevation, lanes | `class Road`; planView geometry: `Geometry` base with `Line`, `Arc`, `Spiral`, `Poly3`, `ParamPoly3` |
| `laneSection`, `lane`, widths, links | `class LaneSection`, `class Lane` |
| `junction`, connections, direct junctions | `class Junction` |
| s/t/h <-> x/y/z, lane/road/world positions, relative positions | `class Position` |
| Routes, trajectories, shapes (polyline, clothoid, NURBS) | `class Route`, `class RMTrajectory`, `class Shape` / `ClothoidShape` / `NurbsShape` |
| Route finding across lanes | `LaneIndependentRouter.*` |

## 5. Practical guidance

- When spec and esmini differ, check `docs/user_guide/known-deviations.adoc`, the coverage files and `release_notes.md` first - the deviation may be intentional.
- New OSC/ODR features: update the relevant parser in `ScenarioReader.cpp`/`RoadManager.cpp`, add a test scenario or road under `EnvironmentSimulator/Unittest/{xosc,xodr}` or `resources/`, cover it in gtest and/or `test/smoke_test.py`, validate test files with `scripts/run_schema_comply.py` (picks the XSD from the file's `revMinor`), update `osc_coverage.txt` (OSC) or `odr_coverage.txt` (ODR) and `release_notes.md`.
