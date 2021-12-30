"../../bin/esmini" --headless --osc ../../resources/xosc/lane-change_clothoid_based_trajectory.xosc --record simulation.dat --fixed_timestep 0.01
"../../bin/replayer" --file simulation.dat --time_scale 1.0 --res_path ../../Resources --window 60 60 800 400 --quit_at_end
