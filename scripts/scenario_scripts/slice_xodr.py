"""
  "Slices" a roadfile and generates a new roadfile containing only the "sliced" area while preserving the original coordinates of the roads.

  NOTE: This script will not cut/move any roads, i.e. make a 500m road -> 250m etc., it will only remove all roads outside the defined boundaries.

  Run script from esmini root, example:
    - python scripts/scenario_scripts/slice_xodr.py

  Generated files ends up in resources/xodr, multi_intersections_sliced.xodr
"""
from road_helpers import get_roads_within_bounds, parse_road, slice_road

# Roadfile to slice
ROADFILE = "resources/xodr/multi_intersections.xodr"

if __name__ == "__main__":
    # Define the center of the location to slice (x, y)
    location = (290, 0)

    # Define the rectangle size to slice (x, y)
    bounds = (130, 30)

    # Parse the road, gets the xml tree and the road attributes back
    tree, road_dict = parse_road(ROADFILE, junctions=True, all_lane_types=True)

    # Find all the roads within the defined bounds using condition of what parts of the road to include
    # (start of road and/or stop of road, or just start/stop)
    roads_to_keep = get_roads_within_bounds(road_dict=road_dict, location=location, boundaries=bounds, condition="start_and_stop")

    # Slice the road and write the output to new file
    slice_road(tree=tree, roads_to_keep=roads_to_keep, output=ROADFILE.replace(".xodr", "_sliced.xodr"))