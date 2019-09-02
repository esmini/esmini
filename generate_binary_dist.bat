set target_dir=esmini-demo

IF NOT EXIST %target_dir% ( mkdir %target_dir% )
IF NOT EXIST %target_dir%\resources ( mkdir %target_dir%\resources )
IF NOT EXIST %target_dir%\resources\xosc ( mkdir %target_dir%\resources\xosc )
IF NOT EXIST %target_dir%\resources\xosc\Catalogs ( mkdir %target_dir%\resources\xosc\Catalogs )
IF NOT EXIST %target_dir%\resources\xodr ( mkdir %target_dir%\resources\xodr )
IF NOT EXIST %target_dir%\resources\models ( mkdir %target_dir%\resources\models )
IF NOT EXIST %target_dir%\run ( mkdir %target_dir%\run )
IF NOT EXIST %target_dir%\run\EgoSimulator ( mkdir %target_dir%\run\EgoSimulator )
IF NOT EXIST %target_dir%\bin\win64\Release ( mkdir %target_dir%\bin\win64\Release )
IF NOT EXIST %target_dir%\3rd_party_terms_and_licenses ( mkdir %target_dir%\3rd_party_terms_and_licenses )

copy resources\xosc\cut-in.xosc %target_dir%\resources\xosc /y
copy resources\xosc\cut-in_cr.xosc %target_dir%\resources\xosc /y
copy resources\xosc\cut-in_simple.xosc %target_dir%\resources\xosc /y
copy resources\xosc\highway_merge.xosc %target_dir%\resources\xosc /y
copy resources\xosc\highway_merge_advanced.xosc %target_dir%\resources\xosc /y
copy resources\xosc\ltap-od_two_targets.xosc %target_dir%\resources\xosc /y
copy resources\xosc\ltap-od.xosc %target_dir%\resources\xosc /y
copy resources\xosc\distance_test.xosc %target_dir%\resources\xosc /y
copy resources\xosc\Catalogs\*.* %target_dir%\resources\xosc\Catalogs /y

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

copy run\EgoSimulator\run_ltap-od_two_targets-external.bat %target_dir%\run\EgoSimulator /y
copy run\EgoSimulator\run_ltap-od_two_targets-internal.bat %target_dir%\run\EgoSimulator /y
copy run\EgoSimulator\run_highway_merge_advanced-external.bat %target_dir%\run\EgoSimulator /y
copy run\EgoSimulator\run_highway_merge_advanced-internal.bat %target_dir%\run\EgoSimulator /y
copy run\EgoSimulator\run_cut-in_internal.bat %target_dir%\run\EgoSimulator /y
copy run\EgoSimulator\run_cut-in_external.bat %target_dir%\run\EgoSimulator /y
copy run\EgoSimulator\run_cut-in_simple.bat %target_dir%\run\EgoSimulator /y
copy run\EgoSimulator\run_cut-in_cr_internal.bat %target_dir%\run\EgoSimulator /y
copy run\EgoSimulator\run_cut-in_cr_external.bat %target_dir%\run\EgoSimulator /y
copy run\EgoSimulator\run_dist_test.bat %target_dir%\run\EgoSimulator /y
copy run\EgoSimulator\run_left-hand-traffic_internal.bat %target_dir%\run\EgoSimulator /y

copy run\readme.txt %target_dir% /y

copy 3rd_party_terms_and_licenses\* %target_dir%\3rd_party_terms_and_licenses /y
copy LICENSE %target_dir% /y

copy buildVS15_64\EnvironmentSimulator\EgoSimulator\Release\EgoSimulator.exe %target_dir%\bin\win64\Release /y
