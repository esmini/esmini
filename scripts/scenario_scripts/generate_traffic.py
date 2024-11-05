import random
import xml.etree.ElementTree as ET

def get_lanesection_ids(lane_element):
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
                if lane.get("type") in ["driving", "offRamp", "onRamp"]:
                    lane_ids.append(int(lane.get("id")))

    return lane_ids
    
def parse_road(roadfile: str) -> dict:
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
        if int(element.get("junction")) == 1:
                continue # Skip junctions to avoid overlapping traffic
        lanes = get_lanesection_ids(element.find("lanes"))
        road_dict[road_id] = {
            "length":  float(element.get("length")),
            "lane_ids": lanes
        }
        road_dict["total_road_length"] += road_dict[road_id]["length"]
        road_dict["drivable_lanes_length"] += road_dict[road_id]["length"] * len(lanes)

    return road_dict

def get_vehicle_types(catalog_path) -> list:
    """
    Extracts information about specific vehicle types from an XML vehicle catalog.

    This function parses a vehicle catalog XML file, identifies vehicles of the 
    "car" category that are not trailers, and retrieves their names and lengths.
    For a specific vehicle ("car_blue"), the length is hardcoded to "4.5" if not provided.

    Parameters:
        catalog_path (str): Path to the XML file containing the vehicle catalog.

    Returns:
        list: A list of dictionaries, each representing a vehicle with:
              - "name" (str): The name of the vehicle.
              - "length" (float): The length of the vehicle.

              Example:
              [
                  {"name": "car_white", "length": 4.2},
                  {"name": "car_blue", "length": 4.5}
              ]
    """
    # Parse the XML file using ElementTree
    tree = ET.parse(catalog_path)
    root = tree.getroot()

    # Find all Vehicle elements
    vehicles = root.findall(".//Vehicle")

    vehicle_data = []
    for vehicle in vehicles:
        name = vehicle.get("name")
        veh_type = vehicle.get("vehicleCategory") 
        dimensions = vehicle.find(".//BoundingBox/Dimensions")
        if veh_type == "car" and "trailer" not in name and dimensions is not None:
            length = dimensions.get("length")
            if name == "car_blue": # Car blue refers to variable in catalog, hard-code it here
                length = "4.5"
            if length is not None and length.replace('.', '', 1).isdigit():
                vehicle_data.append({"name": name, "length": float(length)})
            else:
                print(f"Skipping vehicle {name} due to invalid length: {length}")

    return vehicle_data

def get_vehicle_positions(roadfile, ego_pos: tuple, density: float, catalog_path: str) -> list:
    """
    Generates random vehicle positions along lanes of a road network while ensuring no overlap with the ego vehicle.

    This function calculates the positions of other vehicles based on road geometry and lane information. 
    It factors in a specified vehicle density and ensures that vehicles do not overlap with each other 
    or with the ego vehicle.

    Parameters:
        roadfile (str): Path to the OpenDRIVE (.xodr) road file.
        ego_pos (tuple): The position of the ego vehicle, specified as (s, t, lane_id, road_id):
                        - s (float): Longitudinal position along the road.
                        - t (float): Lateral offset (usually 0 for center of the lane).
                        - lane_id (int): Lane identifier.
                        - road_id (int): Road identifier.
        density (float): Desired vehicle density, representing the number of cars per 100 meters.
        catalog_path (str): Path to the vehicle catalog XML file, used to fetch vehicle types.

    Returns:
        list: A dictionary mapping vehicle indices to their properties, where each entry contains:
              - "position" (tuple): Vehicle position as (s, t, lane_id, road_id).
              - "catalog_name" (str): The name of the vehicle model from the catalog.

              Example:
              {
                  0: {"position": (120.5, 0, 2, 1), "catalog_name": "car_white"},
                  1: {"position": (130.7, 0, 2, 1), "catalog_name": "car_blue"},
                  ...
              }
    """
    positions = {}
    ego_s, _, ego_lid, ego_rid = ego_pos
    road_dict = parse_road(roadfile)
    vehicles = get_vehicle_types(catalog_path)
    
    car_factor = density # cars/100m
    car_density = int(100/car_factor)
    i = 0
    for road_id in list(road_dict)[2:]:
        section_length = road_dict[road_id]["length"]
        for lane_id in road_dict[road_id]["lane_ids"]:
            min_sample = 0
            for s in range(0, int(section_length), car_density):
                target_type = vehicles[int(random.uniform(0, len(vehicles)-1))]
                target_name = target_type["name"]
                target_length = target_type["length"]
                min_sample = s + target_length # add a car length to avoid on top of eachother

                if car_density > section_length:
                    continue # No cars on roads shorter than density

                s_noise = random.uniform(min_sample, min_sample + car_density - target_length)

                if (ego_s - target_length < s_noise < ego_s + target_length) and lane_id == ego_lid and road_id == ego_rid:
                    continue # Don't place a target on top of ego

                if s_noise > section_length - target_length or s_noise < target_length: # Max noise
                    continue
                
                positions[i] = {}
                positions[i]["position"] = (s_noise, 0, lane_id, road_id)
                positions[i]["catalog_name"] = target_name
                i += 1
    
    return positions
