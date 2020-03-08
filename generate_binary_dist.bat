set target_dir=esmini-demo

IF NOT EXIST %target_dir% ( mkdir %target_dir% )
IF NOT EXIST %target_dir%\resources ( mkdir %target_dir%\resources )
IF NOT EXIST %target_dir%\resources\xosc ( mkdir %target_dir%\resources\xosc )
IF NOT EXIST %target_dir%\resources\xosc\Catalogs ( mkdir %target_dir%\resources\xosc\Catalogs )
IF NOT EXIST %target_dir%\resources\xosc\Catalogs\Maneuvers ( mkdir %target_dir%\resources\xosc\Catalogs\Maneuvers )
IF NOT EXIST %target_dir%\resources\xosc\Catalogs\Routes ( mkdir %target_dir%\resources\xosc\Catalogs\Routes )
IF NOT EXIST %target_dir%\resources\xosc\Catalogs\Vehicles ( mkdir %target_dir%\resources\xosc\Catalogs\Vehicles )
IF NOT EXIST %target_dir%\resources\xosc\extensions ( mkdir %target_dir%\resources\xosc\extensions )
IF NOT EXIST %target_dir%\resources\xodr ( mkdir %target_dir%\resources\xodr )
IF NOT EXIST %target_dir%\resources\models ( mkdir %target_dir%\resources\models )
IF NOT EXIST %target_dir%\run ( mkdir %target_dir%\run )
IF NOT EXIST %target_dir%\run\EgoSimulator ( mkdir %target_dir%\run\EgoSimulator )
IF NOT EXIST %target_dir%\bin ( mkdir %target_dir%\bin )
IF NOT EXIST %target_dir%\3rd_party_terms_and_licenses ( mkdir %target_dir%\3rd_party_terms_and_licenses )

copy resources\xosc\basic_hybrid.xosc %target_dir%\resources\xosc /y
copy resources\xosc\cut-in.xosc %target_dir%\resources\xosc /y
copy resources\xosc\lane_change.xosc %target_dir%\resources\xosc /y
copy resources\xosc\cut-in_simple.xosc %target_dir%\resources\xosc /y
copy resources\xosc\highway_merge.xosc %target_dir%\resources\xosc /y
copy resources\xosc\highway_merge_advanced.xosc %target_dir%\resources\xosc /y
copy resources\xosc\ltap-od_two_targets.xosc %target_dir%\resources\xosc /y
copy resources\xosc\distance_test.xosc %target_dir%\resources\xosc /y
copy resources\xosc\left-hand-traffic_internal.xosc %target_dir%\resources\xosc /y
copy resources\xosc\ltap-od-synch.xosc %target_dir%\resources\xosc /y
copy resources\xosc\synchronize.xosc %target_dir%\resources\xosc /y
copy resources\xosc\parking_lot.xosc %target_dir%\resources\xosc /y
copy resources\xosc\Catalogs\Maneuvers\*.* %target_dir%\resources\xosc\Catalogs\Maneuvers /y
copy resources\xosc\Catalogs\Routes\*.* %target_dir%\resources\xosc\Catalogs\Routes /y
copy resources\xosc\Catalogs\Vehicles\*.* %target_dir%\resources\xosc\Catalogs\Vehicles /y
copy resources\xosc\extensions\*.* %target_dir%\resources\xosc\extensions /y

copy resources\xodr\soderleden.xodr %target_dir%\resources\xodr /y
copy resources\xodr\e6mini.xodr %target_dir%\resources\xodr /y
copy resources\xodr\fabriksgatan.xodr %target_dir%\resources\xodr /y
copy resources\xodr\jolengatan.xodr %target_dir%\resources\xodr /y
copy resources\xodr\straight_500m.xodr %target_dir%\resources\xodr /y

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

copy run\EgoSimulator\run_basic_hybrid.bat %target_dir%\run\EgoSimulator /y
copy run\EgoSimulator\run_ltap-od-synch_external.bat %target_dir%\run\EgoSimulator /y
copy run\EgoSimulator\run_ltap-od_two_targets_internal.bat %target_dir%\run\EgoSimulator /y
copy run\EgoSimulator\run_highway-merge_advanced_internal.bat %target_dir%\run\EgoSimulator /y
copy run\EgoSimulator\run_cut-in_internal.bat %target_dir%\run\EgoSimulator /y
copy run\EgoSimulator\run_cut-in_external.bat %target_dir%\run\EgoSimulator /y
copy run\EgoSimulator\run_lane_change.bat %target_dir%\run\EgoSimulator /y
copy run\EgoSimulator\run_left-hand-traffic_internal.bat %target_dir%\run\EgoSimulator /y
copy run\EgoSimulator\run_dist_test.bat %target_dir%\run\EgoSimulator /y
copy run\EgoSimulator\run_synchronize_internal.bat %target_dir%\run\EgoSimulator /y
copy run\EgoSimulator\run_synchronize_external.bat %target_dir%\run\EgoSimulator /y
copy run\EgoSimulator\run_parking_lot.bat %target_dir%\run\EgoSimulator /y

copy run\readme.txt %target_dir% /y

copy 3rd_party_terms_and_licenses\* %target_dir%\3rd_party_terms_and_licenses /y
copy LICENSE %target_dir% /y
copy release_notes.txt %target_dir% /y

copy bin\EgoSimulator.exe %target_dir%\bin /y
