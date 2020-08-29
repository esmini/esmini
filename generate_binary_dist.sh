#!/bin/sh

rm -rf esmini-demo 

mkdir esmini-demo

rsync -R \
resources/xosc/basic_hybrid.xosc \
resources/xosc/cut-in.xosc \
resources/xosc/lane_change.xosc \
resources/xosc/cut-in_simple.xosc \
resources/xosc/highway_merge.xosc \
resources/xosc/highway_merge_advanced.xosc \
resources/xosc/distance_test.xosc \
resources/xosc/left-hand-traffic_internal.xosc \
resources/xosc/ltap-od.xosc \
resources/xosc/synchronize.xosc \
resources/xosc/parking_lot.xosc \
resources/xosc/cut-in_internal.xosc \
resources/xosc/cut-in_sumo.xosc \
resources/xosc/lane_change_simple.xosc \
resources/xosc/Catalogs/Maneuvers/*.* \
resources/xosc/Catalogs/Routes/*.* \
resources/xosc/Catalogs/Vehicles/*.* \
resources/xodr/soderleden.xodr \
resources/xodr/e6mini.xodr \
resources/xodr/fabriksgatan.xodr \
resources/xodr/jolengatan.xodr \
resources/xodr/straight_500m.xodr \
resources/xodr/curve_r100.xodr \
resources/models/bus_blue.osgb \
resources/models/car_blue.osgb \
resources/models/car_red.osgb \
resources/models/car_white.osgb \
resources/models/car_yellow.osgb \
resources/models/truck_yellow.osgb \
resources/models/van_red.osgb \
resources/models/shadow_face.osgb \
resources/models/soderleden.osgb \
resources/models/e6mini.osgb \
resources/models/fabriksgatan.osgb \
resources/models/jolengatan.osgb \
resources/models/straight_500m.osgb \
resources/models/parking_lot.osgb \
resources/models/curve_r100.osgb \
resources/sumo_inputs/e6mini* \
run/EgoSimulator/run_basic_hybrid.* \
run/EgoSimulator/run_ltap-od_external.* \
run/EgoSimulator/run_ltap-od_internal.* \
run/EgoSimulator/run_highway-merge_advanced_internal.* \
run/EgoSimulator/run_cut-in_internal.* \
run/EgoSimulator/run_cut-in_external.* \
run/EgoSimulator/run_cut-in_sumo.* \
run/EgoSimulator/run_lane_change.* \
run/EgoSimulator/run_lane_change_simple.* \
run/EgoSimulator/run_left-hand-traffic_internal.* \
run/EgoSimulator/run_dist_test.* \
run/EgoSimulator/run_synchronize_internal.* \
run/EgoSimulator/run_synchronize_external.* \
run/EgoSimulator/run_parking_lot.* \
docs/readme.txt \
docs/commands.txt \
3rd_party_terms_and_licenses/* \
LICENSE \
release_notes.txt \
version.txt \
bin/EgoSimulator \
esmini-demo

cp -r EnvironmentSimulator/Hello-World_coding-example esmini-demo
cp bin/libScenarioEngineDLL.* esmini-demo/Hello-World_coding-example
cp EnvironmentSimulator/ScenarioEngineDLL/scenarioenginedll.hpp esmini-demo/Hello-World_coding-example

rm -f esmini-demo*.zip

zip -r esmini-demo.zip esmini-demo
