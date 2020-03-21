"../../bin/EnvironmentSimulator" --headless --osc ../../resources/xosc/ltap-od.xosc --record simulation.dat --control internal
"../../bin/Replayer" --file simulation.dat --time_scale 1.0 --res_path ../../Resources --window 50 50 800 400  
"../../bin/Replayer" --file simulation.dat --time_scale 5.0 --res_path ../../Resources --window 50 50 800 400  
"../../bin/Replayer" --file simulation.dat --time_scale 0.25 --res_path ../../Resources --window 50 50 800 400  
