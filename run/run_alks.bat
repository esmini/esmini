@rem Run all ALKS scenarios in superspeed for inspection
@rem Windows only

cd ../test/OSC-ALKS-scenarios/Scenarios
del *.dat *.osi log.txt
set esm="%~dp0/../bin/esmini"
set rep="%~dp0/../bin/replayer"

forfiles /m "*.xosc" /c ^"cmd /c ^
echo @path ^
&%esm% "--headless" "--fixed_timestep" "0.1" "--record" @path".dat" "--osc" @path^
&%rep% "--window" "60" "60" "800" "400" "--res_path" "./" "--file" @path".dat" "--time_scale" "5.0" "--quit_at_end"

del *.dat *.osi log.txt

cd ../../../run
