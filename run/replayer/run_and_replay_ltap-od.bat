"../../bin/esmini" --headless --disable_controllers --osc ../../resources/xosc/ltap-od.xosc --record simulation.dat --fixed_timestep 0.01
"../../bin/replayer" --file simulation.dat --time_scale 1.0 --res_path ../../Resources --window 60 60 800 400  
"../../bin/replayer" --file simulation.dat --time_scale 5.0 --res_path ../../Resources --window 60 60 800 400  
"../../bin/replayer" --file simulation.dat --time_scale 0.25 --res_path ../../Resources --window 60 60 800 400  
