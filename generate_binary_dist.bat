set target_dir=esmini-demo

IF NOT EXIST %target_dir% ( mkdir %target_dir% )
IF NOT EXIST %target_dir%\resources ( mkdir %target_dir%\resources )
IF NOT EXIST %target_dir%\resources\xosc ( mkdir %target_dir%\resources\xosc )
IF NOT EXIST %target_dir%\resources\xosc\Catalogs ( mkdir %target_dir%\resources\xosc\Catalogs )
IF NOT EXIST %target_dir%\resources\xosc\Catalogs\Maneuvers ( mkdir %target_dir%\resources\xosc\Catalogs\Maneuvers )
IF NOT EXIST %target_dir%\resources\xosc\Catalogs\Routes ( mkdir %target_dir%\resources\xosc\Catalogs\Routes )
IF NOT EXIST %target_dir%\resources\xosc\Catalogs\Vehicles ( mkdir %target_dir%\resources\xosc\Catalogs\Vehicles )
IF NOT EXIST %target_dir%\resources\xosc\Catalogs\Controllers ( mkdir %target_dir%\resources\xosc\Catalogs\Controllers )
IF NOT EXIST %target_dir%\resources\xosc\Catalogs\Pedestrians ( mkdir %target_dir%\resources\xosc\Catalogs\Pedestrians )
IF NOT EXIST %target_dir%\resources\xodr ( mkdir %target_dir%\resources\xodr )
IF NOT EXIST %target_dir%\resources\models ( mkdir %target_dir%\resources\models )
IF NOT EXIST %target_dir%\resources\sumo_inputs ( mkdir %target_dir%\resources\sumo_inputs )
IF NOT EXIST %target_dir%\run ( mkdir %target_dir%\run )
IF NOT EXIST %target_dir%\run\esmini ( mkdir %target_dir%\run\esmini )
IF NOT EXIST %target_dir%\run\odrviewer ( mkdir %target_dir%\run\odrviewer )
IF NOT EXIST %target_dir%\run\replayer ( mkdir %target_dir%\run\replayer )
IF NOT EXIST %target_dir%\bin ( mkdir %target_dir%\bin )
IF NOT EXIST %target_dir%\lib ( mkdir %target_dir%\lib )
IF NOT EXIST %target_dir%\scripts ( mkdir %target_dir%\scripts )
IF NOT EXIST %target_dir%\include ( mkdir %target_dir%\include )
IF NOT EXIST %target_dir%\3rd_party_terms_and_licenses ( mkdir %target_dir%\3rd_party_terms_and_licenses )
IF NOT EXIST %target_dir%\Hello-World_coding-example ( mkdir %target_dir%\Hello-World_coding-example )

copy resources\xosc\follow_ghost.xosc %target_dir%\resources\xosc /y
copy resources\xosc\cut-in.xosc %target_dir%\resources\xosc /y
copy resources\xosc\cut-in_interactive.xosc %target_dir%\resources\xosc /y
copy resources\xosc\cut-in_sumo.xosc %target_dir%\resources\xosc /y
copy resources\xosc\cut-in_simple.xosc %target_dir%\resources\xosc /y
copy resources\xosc\cut-in_visibility.xosc %target_dir%\resources\xosc /y
copy resources\xosc\lane_change.xosc %target_dir%\resources\xosc /y
copy resources\xosc\highway_merge.xosc %target_dir%\resources\xosc /y
copy resources\xosc\highway_merge_advanced.xosc %target_dir%\resources\xosc /y
copy resources\xosc\distance_test.xosc %target_dir%\resources\xosc /y
copy resources\xosc\left-hand-traffic_by_heading.xosc %target_dir%\resources\xosc /y
copy resources\xosc\left-hand-traffic_using_road_rule.xosc %target_dir%\resources\xosc /y
copy resources\xosc\ltap-od.xosc %target_dir%\resources\xosc /y
copy resources\xosc\synchronize.xosc %target_dir%\resources\xosc /y
copy resources\xosc\parking_lot.xosc %target_dir%\resources\xosc /y
copy resources\xosc\trajectory-test.xosc %target_dir%\resources\xosc /y
copy resources\xosc\lane_change_simple.xosc %target_dir%\resources\xosc /y
copy resources\xosc\pedestrian.xosc %target_dir%\resources\xosc /y
copy resources\xosc\pedestrian_collision.xosc %target_dir%\resources\xosc /y
copy resources\xosc\controller_test.xosc %target_dir%\resources\xosc /y
copy resources\xosc\slow-lead-vehicle.xosc %target_dir%\resources\xosc /y
copy resources\xosc\synch_with_steady_state.xosc %target_dir%\resources\xosc /y
copy resources\xosc\lane_change_crest.xosc %target_dir%\resources\xosc /y
copy resources\xosc\routing-test.xosc %target_dir%\resources\xosc /y
copy resources\xosc\drop-bike.xosc %target_dir%\resources\xosc /y
copy resources\xosc\sumo-test.xosc %target_dir%\resources\xosc /y
copy resources\xosc\acc-test.xosc %target_dir%\resources\xosc /y

copy resources\xosc\Catalogs\Maneuvers\*.* %target_dir%\resources\xosc\Catalogs\Maneuvers /y
copy resources\xosc\Catalogs\Routes\*.* %target_dir%\resources\xosc\Catalogs\Routes /y
copy resources\xosc\Catalogs\Vehicles\*.* %target_dir%\resources\xosc\Catalogs\Vehicles /y
copy resources\xosc\Catalogs\Controllers\*.* %target_dir%\resources\xosc\Catalogs\Controllers /y
copy resources\xosc\Catalogs\Pedestrians\*.* %target_dir%\resources\xosc\Catalogs\Pedestrians /y

copy resources\xodr\soderleden.xodr %target_dir%\resources\xodr /y
copy resources\xodr\e6mini.xodr %target_dir%\resources\xodr /y
copy resources\xodr\e6mini-lht.xodr %target_dir%\resources\xodr /y
copy resources\xodr\fabriksgatan.xodr %target_dir%\resources\xodr /y
copy resources\xodr\jolengatan.xodr %target_dir%\resources\xodr /y
copy resources\xodr\straight_500m.xodr %target_dir%\resources\xodr /y
copy resources\xodr\straight_500m_signs.xodr %target_dir%\resources\xodr /y
copy resources\xodr\curve_r100.xodr %target_dir%\resources\xodr /y
copy resources\xodr\multi_intersections.xodr %target_dir%\resources\xodr /y
copy resources\xodr\curves_elevation.xodr %target_dir%\resources\xodr /y
copy resources\xodr\crest-curve.xodr %target_dir%\resources\xodr /y

copy resources\models\bus_blue.osgb %target_dir%\resources\models /y
copy resources\models\car_blue.osgb %target_dir%\resources\models /y
copy resources\models\car_red.osgb %target_dir%\resources\models /y
copy resources\models\car_white.osgb %target_dir%\resources\models /y
copy resources\models\car_yellow.osgb %target_dir%\resources\models /y
copy resources\models\truck_yellow.osgb %target_dir%\resources\models /y
copy resources\models\van_red.osgb %target_dir%\resources\models /y
copy resources\models\shadow_face.osgb %target_dir%\resources\models /y
copy resources\models\soderleden.osgb %target_dir%\resources\models /y
copy resources\models\e6mini.osgb %target_dir%\resources\models /y
copy resources\models\fabriksgatan.osgb %target_dir%\resources\models /y
copy resources\models\jolengatan.osgb %target_dir%\resources\models /y
copy resources\models\straight_500m.osgb %target_dir%\resources\models /y
copy resources\models\parking_lot.osgb %target_dir%\resources\models /y
copy resources\models\curve_r100.osgb %target_dir%\resources\models /y
copy resources\models\walkman.osgb %target_dir%\resources\models /y
copy resources\models\cyclist.osgb %target_dir%\resources\models /y
copy resources\models\multi_intersections.osgb %target_dir%\resources\models /y
copy resources\models\curves_elevation.osgb %target_dir%\resources\models /y
copy resources\models\pole.osgb %target_dir%\resources\models /y
copy resources\models\Swe*.osgb %target_dir%\resources\models /y
copy resources\models\asphalt.jpg %target_dir%\resources\models /y
copy resources\models\grass.jpg %target_dir%\resources\models /y
copy resources\models\railing.osgb %target_dir%\resources\models /y
copy resources\models\railing-pole.osgb %target_dir%\resources\models /y
copy resources\models\guide-post.osgb %target_dir%\resources\models /y

copy resources\sumo_inputs\e6mini* %target_dir%\resources\sumo_inputs /y
copy resources\sumo_inputs\multi_intersections* %target_dir%\resources\sumo_inputs /y

copy run\esmini\run_follow_ghost.bat %target_dir%\run\esmini /y
copy run\esmini\run_ltap-od_interactive.bat %target_dir%\run\esmini /y
copy run\esmini\run_ltap-od.bat %target_dir%\run\esmini /y
copy run\esmini\run_highway-merge_advanced_internal.bat %target_dir%\run\esmini /y
copy run\esmini\run_cut-in.bat %target_dir%\run\esmini /y
copy run\esmini\run_cut-in_interactive.bat %target_dir%\run\esmini /y
copy run\esmini\run_cut-in_sumo.bat %target_dir%\run\esmini /y
copy run\esmini\run_lane_change.bat %target_dir%\run\esmini /y
copy run\esmini\run_lane_change_simple.bat %target_dir%\run\esmini /y
copy run\esmini\run_left-hand-traffic.bat %target_dir%\run\esmini /y
copy run\esmini\run_dist_test.bat %target_dir%\run\esmini /y
copy run\esmini\run_synchronize.bat %target_dir%\run\esmini /y
copy run\esmini\run_synchronize_interactive.bat %target_dir%\run\esmini /y
copy run\esmini\run_parking_lot.bat %target_dir%\run\esmini /y
copy run\esmini\run_trajectory-test.bat %target_dir%\run\esmini /y
copy run\esmini\run_pedestrian.bat %target_dir%\run\esmini /y
copy run\esmini\run_pedestrian_collision.bat %target_dir%\run\esmini /y
copy run\esmini\run_controller_test.bat %target_dir%\run\esmini /y
copy run\esmini\run_synch_with_steady_state.bat %target_dir%\run\esmini /y
copy run\esmini\run_lane_change_crest.bat %target_dir%\run\esmini /y
copy run\esmini\run_lane_change_crest_driver-view.bat %target_dir%\run\esmini /y
copy run\esmini\run_routing-test.bat %target_dir%\run\esmini /y
copy run\esmini\run_drop-bike.bat %target_dir%\run\esmini /y
copy run\esmini\run_sumo-test.bat %target_dir%\run\esmini /y
copy run\esmini\run_acc-test.bat %target_dir%\run\esmini /y

copy run\odrviewer\run_e6mini.bat %target_dir%\run\odrviewer /y
copy run\odrviewer\run_multi_intersections.bat %target_dir%\run\odrviewer /y

copy run\replayer\run_and_plot_cut-in_speed.bat %target_dir%\run\replayer /y
copy run\replayer\run_and_plot_ltap-od_pos.bat %target_dir%\run\replayer /y

copy scripts\plot_csv.py %target_dir%\scripts /y

copy docs\readme.txt %target_dir% /y
copy docs\commands.txt %target_dir% /y

copy 3rd_party_terms_and_licenses\* %target_dir%\3rd_party_terms_and_licenses /y
copy LICENSE %target_dir% /y
copy release_notes.md %target_dir% /y
copy version.txt %target_dir% /y

copy bin\esmini.exe %target_dir%\bin /y
copy bin\replayer.exe %target_dir%\bin /y
copy bin\dat2csv.exe %target_dir%\bin /y
copy bin\esmini.exe %target_dir%\bin\EgoSimulator.exe /y
copy bin\odrviewer.exe %target_dir%\bin /y
copy bin\esminiLib.* %target_dir%\lib /y
copy EnvironmentSimulator\Libraries\esminiLib\esminiLib.hpp %target_dir%\include /y

copy Hello-World_coding-example esmini-demo\Hello-World_coding-example
copy bin\esminiLib.* esmini-demo\Hello-World_coding-example
copy EnvironmentSimulator\Libraries\esminiLib\esminiLib.hpp esmini-demo\Hello-World_coding-example
