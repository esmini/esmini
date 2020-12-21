"../../bin/esmini" --headless --disable_controllers --osc ../../resources/xosc/cut-in.xosc --disable_controllers --record simulation.dat --fixed_timestep 0.01
"../../bin/dat2csv" "simulation.dat"
python "../../scripts/plot_csv.py" --x_axis time --param speed simulation.csv