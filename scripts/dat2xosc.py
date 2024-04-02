'''
    Replace complete ManeuverGroup with a single FollowTrajectory action for specified entities ("all" is default)
    
    The new action will be located in the Storyboard Init section.

    Run without arguments to see Usage info, e.g.
        python dat_to_xosc.py
        
    Limitations:
       - Parameters are not resolved. So, for example, $owner as actor will not be identified and ManeuverGroup not deleted
       - Conditions referring to elements in removed ManeuverGroup will fail
'''

import os
import lxml.etree as ET
import multiprocessing as mp
from argparse import ArgumentParser
from dat import DATFile

def parse_args() -> any:
    parser = ArgumentParser(
        description="Convert dat file to xosc trajectory file",
    )
    parser.add_argument(
        "--dat-path",
        type=str,
        required=True,
        help="Specify the path to the folder containing the dat-files",
    )
    parser.add_argument(
        "--xosc-path",
        type=str,
        default="",
        required=False,
        help="Specify the path to the folder containing the xosc-files which shall be copied and have trajectories inserted",
    )
    parser.add_argument(
        "--output-path",
        type=str,
        required=True,
        help="The destination of the new xosc files"
    )
    parser.add_argument(
        "--replace-entity",
        nargs="+",
        default=["all"],
        required=False,
        help="The name of the entity to replace, all replaces all of them (TODO)"
    )
    parser.add_argument(
        "--trajectory-type",
        type=str,
        default="WorldPosition",
        required=False,
        help="The type of trajectory to create, WorldPosition (default) or LanePosition supported"
    )
    parser.add_argument(
        "--match",
        type=str,
        default="dat",
        required=False,
        help="Whether to find one xosc per [dat] (default) or to find all dats matching substring of one [xosc]"
    )
    parser.add_argument(
        "--modulo",
        type=int,
        default=10,
        required=False,
        help="How many timesteps to skip when creating the trajectory, i.e. 1000Hz trajectory to 100Hz, use modulo (%) 10"
    )
    parser.add_argument(
        "--keep-controllers",
        default=True,
        required=False,
        action="store_true",
        help="If the controllers should be kept for each entity"
    )
    parser.add_argument(
        "--pool",
        type=int,
        default=1,
        required=False,
        action="store",
        help="Convert in parallel or one by one, default one by one"
    )
    return parser.parse_args()

def get_column_idx(row: list, *keys: str) -> int:
    indices = []
    for key in keys:
        for i, col in enumerate(row):
            if key.lower() == col.lower().strip():
                indices.append(i)

    return indices

def create_polyline_from_dat(datfile: DATFile, entity: str, modulo: int) -> dict:
    keys_to_extract = ["time", "name", "roadId", "laneId", "offset", "s", "h", "x", "y", "z"]
    polyline = {"times": [],
                "positions": {
                    "s": [],
                    "offset": [],
                    "laneId": [],
                    "roadId": [],
                    "h": [],
                    "x": [],
                    "y": [],
                    "z": []
                    }
                }
    try:
        time_i, name_i, road_i, lane_i, offset_i, s_i, h_i, x_i, y_i, z_i = get_column_idx(datfile.get_labels_line_array(), *keys_to_extract)
    except:
        print("Failed to extract keys")
    counter = 0
    for data in datfile.data:
        data_array = datfile.get_data_line_array(data)
        if data_array[time_i] >= 0.0 and data_array[name_i] == entity:
            if counter % modulo == 0:
                polyline["times"].append(data_array[time_i])
                polyline["positions"]["s"].append(data_array[s_i])
                polyline["positions"]["offset"].append(data_array[offset_i])
                polyline["positions"]["laneId"].append(data_array[lane_i])
                polyline["positions"]["roadId"].append(data_array[road_i])
                polyline["positions"]["h"].append(data_array[h_i])
                polyline["positions"]["x"].append(data_array[x_i])
                polyline["positions"]["y"].append(data_array[y_i])
                polyline["positions"]["z"].append(data_array[z_i])
            counter += 1

    return polyline

def create_destination_folder(path: str) -> None:
    if not os.path.exists(path):
        try:
            os.mkdir(path)
        except:
            print(f"Failed to create {path}")
            exit()
    return

def find_all_entities(xosc_tree: ET) -> None:
    xosc_root = xosc_tree.getroot()

    entities = [key.attrib['name'] for key in xosc_root.findall("Entities/") if key.tag == 'ScenarioObject']
    # maneuver_groups = [key for key in xosc_root.findall("Storyboard/Story/Act/") if key.tag == 'ManeuverGroup']
    return entities

def delete_entity_maneuvergroup(xosc_tree: ET, entity: str) -> None:
    xosc_root = xosc_tree.getroot()
    maneuver_groups = [key for key in xosc_root.findall("Storyboard/Story/Act/") if key.tag == 'ManeuverGroup']
    for maneuver_group in maneuver_groups:
        if maneuver_group.find("Actors/EntityRef").values()[0] == entity: # can be list somehow?
            maneuver_group.getparent().remove(maneuver_group)

    return

def delete_entity_init_actions(xosc_tree, entity, keep_controller_action) -> None:
    xosc_root = xosc_tree.getroot()
    init_action = [init for init in xosc_root.findall("Storyboard/Init/Actions/") if init.values()[0] == entity][0] # can be several?

    for action in init_action.getchildren():
        if keep_controller_action and action.getchildren()[0].tag == "ControllerAction":
            continue
        action.getparent().remove(action)

    return

def add_entity_trajectory(xosc_tree: ET, entity: str, trajectory: dict, trajectory_type: str) -> None:
    xosc_root = xosc_tree.getroot()
    init_action = [init for init in xosc_root.findall("Storyboard/Init/Actions/") if init.values()[0] == entity][0] # can be several?

    traj_tree_names = ["RoutingAction", "FollowTrajectoryAction", "TrajectoryRef", "Trajectory"]

    new_private_action = ET.Element("PrivateAction")
    entry = new_private_action
    for name in traj_tree_names:
        entry = ET.SubElement(entry, name)
        if name == "FollowTrajectoryAction":
            time_reference = ET.SubElement(entry, "TimeReference")
            timing = ET.SubElement(time_reference, "Timing")
            timing.set("domainAbsoluteRelative", "absolute")
            timing.set("scale", "1.0")
            timing.set("offset", "0.0")
            following_mode = ET.SubElement(entry, "TrajectoryFollowingMode")
            following_mode.set("followingMode", "position")

    entry.set("name", f"{entity}_traj")
    entry.set("closed", "False")

    ET.SubElement(entry, "ParameterDeclarations") # Needed?

    traj_names = ["Shape", "Polyline"]
    for name in traj_names:
        entry = ET.SubElement(entry, name)

    for i in range(len(trajectory["times"])):
        vertex = ET.SubElement(entry, "Vertex")
        vertex.set("time", str(trajectory["times"][i]))
        position = ET.SubElement(vertex, "Position")
        if trajectory_type == "LanePosition":
            lane_position = ET.SubElement(position, trajectory_type)
            lane_position.set("s", str(trajectory["positions"]["s"][i]))
            lane_position.set("offset", str(trajectory["positions"]["offset"][i]))
            lane_position.set("laneId", str(trajectory["positions"]["laneId"][i]))
            lane_position.set("roadId", str(trajectory["positions"]["roadId"][i]))
            lane_position.set("h", str(trajectory["positions"]["h"][i]))
        elif trajectory_type == "WorldPosition":
            world_position = ET.SubElement(position, trajectory_type)
            world_position.set("x", str(trajectory["positions"]["x"][i]))
            world_position.set("y", str(trajectory["positions"]["y"][i]))
            world_position.set("z", str(trajectory["positions"]["z"][i]))
            world_position.set("h", str(trajectory["positions"]["h"][i]))

    init_action.insert(0, new_private_action)

def write_xosc(xosc_root, output_path) -> None:
    try:
        xosc_root.write(output_path, pretty_print=True)
    except:
        print(f"Failed to write new xosc to {output_path}")
        exit()

    return

def parse_xosc(xosc_path: str) -> ET:
    parser = ET.XMLParser(remove_blank_text=False)
    xosc_tree = ET.parse(xosc_path, parser)

    return xosc_tree

def generate_xosc(dat_path: str, xosc_path: str, output_path: str, replace_entity: list, keep_controllers: bool, trajectory_type: str, modulo: int) -> None:

    xosc_tree = parse_xosc(xosc_path)

    for dat in dat_path:
        dat_data = DATFile(dat)
        if os.path.isdir(output_path):
            new_xosc = os.path.join(output_path, f"{dat.split(os.path.sep)[-1].split('.dat')[0]}.xosc")
        else:
            new_xosc = output_path

        if 'all' in replace_entity:
            # resolve all entity names in the scenario
            replace_entity = find_all_entities(xosc_tree)

        polylines = {entity: {} for entity in replace_entity}
        for entity in replace_entity:
            polylines[entity] = create_polyline_from_dat(dat_data, entity, modulo)

        for entity in polylines:
            delete_entity_init_actions(xosc_tree, entity, keep_controllers)
            delete_entity_maneuvergroup(xosc_tree, entity)
            add_entity_trajectory(xosc_tree, entity, polylines[entity], trajectory_type)
            write_xosc(xosc_tree, new_xosc)

    return

def match_xosc(xosc_path: str, dat_path: str) -> dict:
    xosc_files = [os.path.join(xosc_path, osc) for osc in os.listdir(xosc_path) if osc.endswith(".xosc")]
    ret = { osc : [] for osc in xosc_files}
    for root, _, files in os.walk(dat_path):
        for file in files:
            for osc in xosc_files:
                osc_name = osc.split(os.path.sep)[-1].split(".xosc")[0]
                if osc_name in file and file.endswith(".dat"):
                    ret[osc].append(os.path.join(root, file))

    return ret

def match_dat(xosc_path: str, dat_path: str) -> dict:
    dat_files = []
    ret = {}

    if os.path.isfile(dat_path):
        dat_files.append(dat_path)
    else:
        for root, _, files in os.walk(dat_path):
            for file in files:
                if file.endswith(".dat"):
                    dat_files.append(os.path.join(root, file))

    if len(dat_files) == 0:
        print(f"No datfiles found in {dat_path}")
        return

    for dat in dat_files:
        dat_name = dat.split(os.path.sep)[-1].split(".dat")[0]

        if os.path.isfile(xosc_path):
            if not xosc_path.endswith(".xosc"):
                print(f"{xosc_path} not an .xosc file")
                return
            ret[xosc_path] = [dat]
        else:
            for root, _, files in os.walk(xosc_path):
                for file in files:
                    if file.endswith(".xosc") and dat_name in file:
                        ret[os.path.join(root, file)] = [dat]

    return ret

def main():
    args = parse_args()

    if args.replace_entity[0] != "all" and not args.xosc_path:
        print("Must give an xosc-file to replace an entity")
        return

    output_folder = os.path.dirname(os.path.abspath(args.output_path))
    if not os.path.exists(output_folder):
        print(f"Creating {output_folder} to put new xosc in")
        create_destination_folder(output_folder)

    if args.match == "xosc":
        run_dict = match_xosc(args.xosc_path, args.dat_path)
    elif args.match == "dat":
        run_dict = match_dat(args.xosc_path, args.dat_path)
    else:
        print(f"Match argument must be [dat] or [xosc], not {args.match}")
        return

    if args.pool == 1:
        for key in run_dict:
            generate_xosc(run_dict[key],
                            key,
                          args.output_path,
                          args.replace_entity,
                          args.keep_controllers,
                          args.trajectory_type,
                          args.modulo)
    else:
        results = []
        pool = mp.Pool(args.pool)
        for key in run_dict:
            result = pool.apply_async(generate_xosc, (run_dict[key],
                                                      key,
                                                      args.output_path,
                                                      args.replace_entity,
                                                      args.keep_controllers,
                                                      args.trajectory_type,
                                                      args.modulo))
            results.append(result)
        for result in results:
            result.get()
    print("Done")

    return

if __name__ == "__main__":
    main()