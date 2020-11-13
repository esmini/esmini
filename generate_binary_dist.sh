#!/bin/sh

rm -rf esmini-demo 

mkdir esmini-demo

rsync -R \
resources/xosc/follow_ghost.xosc \
resources/xosc/cut-in.xosc \
resources/xosc/cut-in_interactive.xosc \
resources/xosc/cut-in_sumo.xosc \
resources/xosc/cut-in_simple.xosc \
resources/xosc/cut-in_visibility.xosc \
resources/xosc/lane_change.xosc \
resources/xosc/highway_merge.xosc \
resources/xosc/highway_merge_advanced.xosc \
resources/xosc/distance_test.xosc \
resources/xosc/left-hand-traffic_internal.xosc \
resources/xosc/ltap-od.xosc \
resources/xosc/synchronize.xosc \
resources/xosc/parking_lot.xosc \
resources/xosc/trajectory-test.xosc \
resources/xosc/lane_change_simple.xosc \
resources/xosc/pedestrian.xosc \
resources/xosc/pedestrian_collision.xosc \
resources/xosc/controller_test.xosc \
resources/xosc/Catalogs/Maneuvers/*.* \
resources/xosc/Catalogs/Routes/*.* \
resources/xosc/Catalogs/Vehicles/*.* \
resources/xosc/Catalogs/Controllers/*.* \
resources/xosc/Catalogs/Pedestrians/*.* \
resources/xodr/soderleden.xodr \
resources/xodr/e6mini.xodr \
resources/xodr/fabriksgatan.xodr \
resources/xodr/jolengatan.xodr \
resources/xodr/straight_500m.xodr \
resources/xodr/curve_r100.xodr \
resources/xodr/multi_intersections.xodr \
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
resources/models/walkman.osgb \
resources/models/multi_intersections.osgb \
resources/sumo_inputs/e6mini* \
run/esmini/run_follow_ghost.* \
run/esmini/run_ltap-od_interactive.* \
run/esmini/run_ltap-od.* \
run/esmini/run_highway-merge_advanced_internal.* \
run/esmini/run_cut-in.* \
run/esmini/run_cut-in_interactive.* \
run/esmini/run_cut-in_sumo.* \
run/esmini/run_lane_change.* \
run/esmini/run_lane_change_simple.* \
run/esmini/run_left-hand-traffic_internal.* \
run/esmini/run_dist_test.* \
run/esmini/run_synchronize.* \
run/esmini/run_synchronize_interactive.* \
run/esmini/run_parking_lot.* \
run/esmini/run_trajectory-test.* \
run/esmini/run_pedestrian*.* \
run/esmini/run_controller_test*.* \
run/odrviewer/run_e6mini*.* \
run/odrviewer/run_multi_intersections*.* \
docs/readme.txt \
docs/commands.txt \
3rd_party_terms_and_licenses/* \
LICENSE \
release_notes.md \
version.txt \
bin/esmini \
bin/odrviewer \
esmini-demo

cp bin/esmini esmini-demo/bin/EgoSimulator
cp -r Hello-World_coding-example esmini-demo
cp bin/libesminiLib.* esmini-demo/Hello-World_coding-example
cp EnvironmentSimulator/Libraries/esminiLib/esminiLib.hpp esmini-demo/Hello-World_coding-example
mkdir esmini-demo/include
cp EnvironmentSimulator/Libraries/esminiLib/esminiLib.hpp esmini-demo/include

rm -f esmini-demo*.zip

zip -r esmini-demo.zip esmini-demo
