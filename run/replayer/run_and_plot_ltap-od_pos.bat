"../../bin/esmini" --headless --disable_controllers --osc ../../resources/xosc/ltap-od.xosc --record simulation.dat --fixed_timestep 0.01
"../../bin/dat2csv" "simulation.dat"
python "../../scripts/plot_csv.py" --x_axis x --param y --equal_axis_aspect simulation.csv