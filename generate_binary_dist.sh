#!/bin/sh

rm -f esmini-demo_ubuntu.zip

zip esmini-demo_ubuntu.zip \
resources/xosc/basic_hybrid.xosc \
resources/xosc/cut-in.xosc \
resources/xosc/lane_change.xosc \
resources/xosc/cut-in_simple.xosc \
resources/xosc/highway_merge.xosc \
resources/xosc/highway_merge_advanced.xosc \
resources/xosc/ltap-od_two_targets.xosc \
resources/xosc/distance_test.xosc \
resources/xosc/left-hand-traffic_internal.xosc \
resources/xosc/ltap-od-synch.xosc \
resources/xosc/synchronize.xosc \
resources/xosc/parking_lot.xosc \
resources/xosc/Catalogs/Maneuvers/*.* \
resources/xosc/Catalogs/Routes/*.* \
resources/xosc/Catalogs/Vehicles/*.* \
resources/xosc/extensions/*.* \
resources/xodr/soderleden.xodr \
resources/xodr/e6mini.xodr \
resources/xodr/fabriksgatan.xodr \
resources/xodr/jolengatan.xodr \
resources/xodr/straight_500m.xodr \
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
run/EgoSimulator/run_basic_hybrid.bat \
run/EgoSimulator/run_ltap-od-synch_external.bat \
run/EgoSimulator/run_ltap-od_two_targets_internal.bat \
run/EgoSimulator/run_highway-merge_advanced_internal.bat \
run/EgoSimulator/run_cut-in_internal.bat \
run/EgoSimulator/run_cut-in_external.bat \
run/EgoSimulator/run_lane_change.bat \
run/EgoSimulator/run_left-hand-traffic_internal.bat \
run/EgoSimulator/run_dist_test.bat \
run/EgoSimulator/run_synchronize_internal.bat \
run/EgoSimulator/run_synchronize_external.bat \
run/EgoSimulator/run_parking_lot.bat \
run/readme.txt \
3rd_party_terms_and_licenses/* \
LICENSE \
release_notes.txt \
bin/EgoSimulator
