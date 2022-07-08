#!/bin/bash

# Automate the procedure to apply textures files to 3D sign models
# Example usage: ./bake_signs.sh -s 1.5 ./se/lores/se_c_31-??.png

scale=1.0
model_template="sign_model.dae"

while getopts hs: opt
do
    case $opt in
    s) scale=$OPTARG;;
    h) printf "Usage: %s: [-s scale_factor] image_files\n" $0
          exit 1;;
    esac
done
shift $(($OPTIND - 1))

for file in "$@"
do
  echo processing $file
  cp $file sign_image.png
  name=`basename $file`
  osgconv $model_template ${name%.*}.osgb -s $scale,$scale,$scale --use-world-frame
done
