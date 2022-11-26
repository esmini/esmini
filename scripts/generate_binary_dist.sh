#!/bin/sh

rm -rf esmini-demo 

mkdir esmini-demo

rsync -R \
resources/xosc/follow_ghost.xosc \
resources/xosc/cut-in.xosc \
resources/xosc/cut-in_parameter_set.xosc \
resources/xosc/cut-in_interactive.xosc \
resources/xosc/cut-in_sumo.xosc \
resources/xosc/cut-in_simple.xosc \
resources/xosc/cut-in_visibility.xosc \
resources/xosc/lane_change.xosc \
resources/xosc/highway_merge.xosc \
resources/xosc/highway_merge_advanced.xosc \
resources/xosc/distance_test.xosc \
resources/xosc/left-hand-traffic_by_heading.xosc \
resources/xosc/left-hand-traffic_using_road_rule.xosc \
resources/xosc/ltap-od.xosc \
resources/xosc/synchronize.xosc \
resources/xosc/parking_lot.xosc \
resources/xosc/trajectory-test.xosc \
resources/xosc/lane_change_simple.xosc \
resources/xosc/pedestrian.xosc \
resources/xosc/pedestrian_collision.xosc \
resources/xosc/controller_test.xosc \
resources/xosc/slow-lead-vehicle.xosc \
resources/xosc/synch_with_steady_state.xosc \
resources/xosc/lane_change_crest.xosc \
resources/xosc/routing-test.xosc \
resources/xosc/drop-bike.xosc \
resources/xosc/sumo-test.xosc \
resources/xosc/acc-test.xosc \
resources/xosc/alks-test.xosc \
resources/xosc/swarm.xosc \
resources/xosc/lane-change_clothoid_based_trajectory.xosc \
resources/xosc/trailers.xosc \
resources/xosc/speed-profile.xosc \
resources/xosc/alks_r157_cut_in_quick_brake.xosc \
resources/xosc/Catalogs/Maneuvers/*.* \
resources/xosc/Catalogs/Routes/*.* \
resources/xosc/Catalogs/Vehicles/*.* \
resources/xosc/Catalogs/Controllers/*.* \
resources/xosc/Catalogs/Pedestrians/*.* \
resources/xodr/soderleden.xodr \
resources/xodr/e6mini.xodr \
resources/xodr/e6mini-lht.xodr \
resources/xodr/fabriksgatan.xodr \
resources/xodr/jolengatan.xodr \
resources/xodr/straight_500m.xodr \
resources/xodr/straight_500m_signs.xodr \
resources/xodr/curve_r100.xodr \
resources/xodr/multi_intersections.xodr \
resources/xodr/curves_elevation.xodr \
resources/xodr/crest-curve.xodr \
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
resources/models/cyclist.osgb \
resources/models/mc.osgb \
resources/models/moose_cc0.osgb \
resources/models/multi_intersections.osgb \
resources/models/curves_elevation.osgb \
resources/models/de_274-30.osgb \
resources/models/de_274-50.osgb \
resources/models/se_c_31-3.osgb \
resources/models/se_c_31-5.osgb \
resources/models/de_123.osgb \
resources/models/se_a_20-1.osgb \
resources/models/pole.osgb \
resources/models/asphalt.jpg \
resources/models/grass.jpg \
resources/models/railing.osgb \
resources/models/railing-pole.osgb \
resources/models/guide-post.osgb \
resources/models/cone-100.osgb \
resources/models/box_cc_by.osgb \
resources/models/car_trailer.osgb \
resources/models/semi_tractor.osgb \
resources/models/semi_trailer.osgb \
resources/models/truck_trailer.osgb \
resources/models/hedge.osgb \
resources/models/fence_plank.osgb \
resources/models/fence_pole.osgb \
resources/sumo_inputs/e6mini* \
resources/sumo_inputs/multi_intersections* \
resources/traffic_signals/* \
run/esmini/run_follow_ghost.* \
run/esmini/run_ltap-od_interactive.* \
run/esmini/run_ltap-od.* \
run/esmini/run_highway-merge_advanced_internal.* \
run/esmini/run_cut-in.* \
run/esmini/run_cut-in_interactive.* \
run/esmini/run_cut-in_sumo.* \
run/esmini/run_lane_change.* \
run/esmini/run_lane_change_simple.* \
run/esmini/run_left-hand-traffic.* \
run/esmini/run_dist_test.* \
run/esmini/run_synchronize.* \
run/esmini/run_synchronize_interactive.* \
run/esmini/run_parking_lot.* \
run/esmini/run_trajectory-test.* \
run/esmini/run_pedestrian*.* \
run/esmini/run_controller_test*.* \
run/esmini/run_synch_with_steady_state*.* \
run/esmini/run_lane_change_crest.* \
run/esmini/run_lane_change_crest_driver-view.* \
run/esmini/run_routing-test.* \
run/esmini/run_drop-bike.* \
run/esmini/run_sumo-test.* \
run/esmini/run_acc-test.* \
run/esmini/run_swarm.* \
run/esmini/run_trailers.* \
run/odrviewer/run_e6mini*.* \
run/odrviewer/run_multi_intersections*.* \
run/replayer/run_and_plot_cut-in_speed*.* \
run/replayer/run_and_plot_ltap-od_pos*.* \
run/replayer/run_and_replay_lane-change_clothoid*.* \
scripts/dat.py \
scripts/plot_dat.py \
scripts/dat2csv.py \
scripts/osi2csv.py \
scripts/run_distribution.py \
scripts/udp_driver/*.* \
scripts/udp_driver/osi3/*.* \
docs/readme.txt \
docs/commands.txt \
3rd_party_terms_and_licenses/* \
LICENSE \
release_notes.md \
version.txt \
bin/esmini \
bin/odrviewer \
bin/replayer \
bin/dat2csv \
bin/odrplot \
EnvironmentSimulator/Applications/odrplot/xodr.py \
esmini-demo

cp -r Hello-World_coding-example esmini-demo
cp bin/libesminiLib.* esmini-demo/Hello-World_coding-example
cp EnvironmentSimulator/Libraries/esminiLib/esminiLib.hpp esmini-demo/Hello-World_coding-example
mkdir esmini-demo/include
cp EnvironmentSimulator/Libraries/esminiLib/esminiLib.hpp esmini-demo/include

rm -f esmini-demo*.zip

zip -r esmini-demo.zip esmini-demo
