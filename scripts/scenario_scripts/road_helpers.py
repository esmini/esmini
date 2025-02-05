import xml.etree.ElementTree as ET

def get_lanesection_ids(lane_element, all_types: bool = False):
    """
    Extracts the IDs of lanes of specific types (e.g., "driving", "offRamp", "onRamp") 
    from the given lane section element.

    This function searches for lane elements within the "left" and "right" subsections 
    of a "laneSection" in the provided XML structure. It collects the IDs of lanes 
    that match specific types.

    Parameters:
        lane_element (Element): An XML element representing a lane section, expected 
                                to contain "left" and "right" subsections with lane 
                                definitions.

    Returns:
        list[int]: A list of integers representing the IDs of lanes that are of type 
                "driving", "offRamp", or "onRamp". If no lanes of these types are 
                found, an empty list is returned.
    """
    lane_ids = []
    for side in ["left", "right"]:
        element = lane_element.find(f"laneSection/{side}")
        if element is not None:
            for lane in element.findall("lane"):
                if all_types:
                    lane_ids.append(int(lane.get("id")))
                elif lane.get("type") in ["driving", "offRamp", "onRamp"]:
                    lane_ids.append(int(lane.get("id")))

    return lane_ids

def get_first_last_road_geom(planview_element: ET.Element) -> tuple:
    """
    Extracts the coordinates of the first and last geometry elements from a plan view.

    This function retrieves the "geometry" elements within the given plan view 
    element and extracts the (x, y) coordinates of the first and last geometry 
    entries.

    Parameters:
        planview_element (Element): An XML element representing the "planView" section 
                                    of a road, containing "geometry" child elements.

    Returns:
        tuple[tuple[float, float], tuple[float, float]]: A tuple containing two coordinate 
        pairs representing the (x, y) positions of the first and last geometry elements. 
        If no geometry elements are found, both tuples are empty.
    """
    first_geom = ()
    last_geom = ()
    geoms = planview_element.findall("geometry")
    if geoms is not None:
        first_geom = (float(geoms[0].get("x")), float(geoms[0].get("y")))
        last_geom = (float(geoms[-1].get("x")), float(geoms[-1].get("y")))
    
    return first_geom, last_geom

def get_roads_within_bounds(road_dict: dict, location: tuple, boundaries: tuple, condition: str = "start_and_stop") -> list:
    """
    Identifies roads within a specified boundary region based on their start and stop coordinates.

    This function checks which roads from a given dictionary fall within a specified 
    rectangular boundary around a central location. The inclusion criteria can be 
    adjusted using different conditions to check either the start, stop, or both 
    coordinates of the road.

    Parameters:
        road_dict (dict): A dictionary containing road data, where each road entry 
                          includes "first_geom" and "last_geom" as (x, y) coordinate 
                          tuples.
        location (tuple[float, float]): The central point (x, y) around which the boundary is defined.
        boundaries (tuple[float, float]): The (x, y) offsets from the location, defining 
                                          the rectangular boundary region.
        condition (str, optional): Defines the criteria for road inclusion:
            - "start_and_stop" (default): Both start and stop coordinates must be within bounds.
            - "start_or_stop": Either start or stop coordinate must be within bounds.
            - "start": Only the start coordinate must be within bounds.
            - "stop": Only the stop coordinate must be within bounds.

    Returns:
        list[str]: A list of road keys that satisfy the boundary condition.
    """
    loc_max = (location[0]+boundaries[0], location[1]+boundaries[1])
    loc_min = (location[0]-boundaries[0], location[1]-boundaries[1])

    roads_within_bounds = []
    for road in road_dict:
        if road in ["total_road_length", "drivable_lanes_length"]: # Ignore these 2 summary keys
            continue
        road_start = road_dict[road]["first_geom"]
        road_stop = road_dict[road]["last_geom"]

        road_start_in_bounds = road_start[0] < loc_max[0] and road_start[0] > loc_min[0] and road_start[1] < loc_max[1] and road_start[1] > loc_min[1]
        road_stop_in_bounds = road_stop[0] < loc_max[0] and road_stop[0] > loc_min[0] and road_stop[1] < loc_max[1] and road_stop[1] > loc_min[1]

        if condition == "start_and_stop":
            boundary_condition = road_start_in_bounds and road_stop_in_bounds
        elif condition == "start_or_stop":
            boundary_condition = road_start_in_bounds or road_stop_in_bounds
        elif condition == "start":
            boundary_condition = road_start_in_bounds
        elif condition == "stop":
            boundary_condition = road_stop_in_bounds
        else:
            print(f"Not valid condition {condition}, using start_and_stop")
            boundary_condition = road_start_in_bounds and road_stop_in_bounds

        if boundary_condition:
            roads_within_bounds.append(road)
    
    return roads_within_bounds


def slice_road(tree: ET.ElementTree, roads_to_keep: list, output: str) -> None:
    """
    Removes roads not listed in `roads_to_keep` from an OpenDRIVE XML file and 
    updates junctions accordingly.

    This function iterates through the roads in the XML structure and removes 
    any that are not specified in `roads_to_keep`. It also updates junction elements 
    by removing connections that reference deleted roads.

    Parameters:
        tree (ET.ElementTree): An XML tree representing the OpenDRIVE road network.
        roads_to_keep (list): A list of integers representing road IDs that should be retained.
        output (str): The file path where the modified XML will be saved.

    Returns:
        None: The function modifies the XML tree in place and writes the changes to `output`.
    """
    root = tree.getroot()
    road_elem = root.findall('road')
    if road_elem is None:
        print("No road in file")
        return
    
    removed_roads = []
    for element in road_elem:
        rid = int(element.get("id"))
        if rid not in roads_to_keep:
            root.remove(element)
            removed_roads.append(rid)
    
    junction_elem = root.findall("junction")
    for junc in junction_elem:
        connections = junc.findall("connection")
        removed_connections = 0
        num_connections = len(connections)
        if connections:
            for connection in connections:
                if int(connection.get("incomingRoad")) in removed_roads or int(connection.get("connectingRoad") in removed_roads):
                    junc.remove(connection)
                    removed_connections += 1

            if removed_connections == num_connections:
                root.remove(junc)

    tree.write(output)


def parse_road(roadfile: str, junctions: bool = False, all_lane_types: bool = False) -> tuple:
    """
    Parses an OpenDRIVE road network file to extract road information and compute 
    road statistics.

    This function reads an OpenDRIVE-compatible XML file, identifies road elements, 
    and calculates the total road length and the cumulative length of drivable lanes. 
    It also collects detailed information about each road's length and associated lane IDs.

    Parameters:
        roadfile (str): Path to the OpenDRIVE XML file containing road data.

    Returns:
        dict: A dictionary containing:
              - "total_road_length" (float): The cumulative length of all roads.
              - "drivable_lanes_length" (float): The cumulative length of all drivable lanes.
              - Additional keys for each road ID with values as dictionaries containing:
                - "length" (float): The length of the road.
                - "lane_ids" (list[int]): The IDs of drivable lanes on the road.

              Example:
              {
                  "total_road_length": 500.0,
                  "drivable_lanes_length": 1500.0,
                  1: {
                      "first_geom": (0.0, 0.0),
                      "last_geom": (0.0, 0.0), # If road is one road, straight 500m only 1 geometry.
                      "length": 500.0,
                      "lane_ids": [1, 2, 3]
                  }
              }
    """
    road_dict = { "total_road_length" : 0.0 ,
        "drivable_lanes_length" : 0.0 }

    tree = ET.parse(roadfile)
    root = tree.getroot()
    road_elem = root.findall('road')
    if road_elem is None:
        print("No road in file")
        return
   
    for element in road_elem:
        road_id = int(element.get("id"))
        if int(element.get("junction")) != -1 and not junctions:
                continue # Skip junctions to avoid overlapping traffic
        first_geom, last_geom = get_first_last_road_geom(element.find("planView"))
        lanes = get_lanesection_ids(element.find("lanes"), all_types=all_lane_types)
        
        road_dict[road_id] = {
            "first_geom": first_geom,
            "last_geom": last_geom,
            "length":  float(element.get("length")),
            "lane_ids": lanes
        }
        road_dict["total_road_length"] += road_dict[road_id]["length"]
        road_dict["drivable_lanes_length"] += road_dict[road_id]["length"] * len(lanes)

    return tree, road_dict
