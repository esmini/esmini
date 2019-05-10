# Linux Build and Run Instructions

## Install dependencies

On Ubuntu 16.04 (all these packages are probably not required)

```
sudo apt install build-essential git pkg-config libgl1-mesa-dev libpthread-stubs0-dev libjpeg-dev libxml2-dev libpng12-dev libtiff5-dev libgdal-dev libpoppler-dev libdcmtk-dev libjasper-dev libgstreamer1.0-dev libgtk2.0-dev libcairo2-dev libpoppler-glib-dev libxrandr-dev libxinerama-dev curl
```

Ubuntu 18.04

```
sudo apt install build-essential git pkg-config libgl1-mesa-dev libpthread-stubs0-dev libjpeg-dev libxml2-dev libpng-dev libtiff5-dev libgdal-dev libpoppler-dev libdcmtk-dev libgstreamer1.0-dev libgtk2.0-dev libcairo2-dev libpoppler-glib-dev libxrandr-dev libxinerama-dev curl
```

## Building OpenSceneGraph

```
cd ~/
git clone https://github.com/openscenegraph/OpenSceneGraph
cd OpenSceneGraph
git checkout OpenSceneGraph-3.6.3
mkdir build
cd build
cmake ../
make -j4
sudo make install
sudo ldconfig
```

## Building Environment Simulator

```
cd ~/
git clone https://github.com/esmini/esmini
cd esmini
```

Copy OpenSceneGraph files to expected path

```
mkdir -p externals/OpenSceneGraph/v10/build
OSG_PATH=~/OpenSceneGraph
cp -a $OSG_PATH/build/lib externals/OpenSceneGraph/v10/
cp -a $OSG_PATH/build/include externals/OpenSceneGraph/v10/build
cp -a $OSG_PATH/include externals/OpenSceneGraph/v10/
```

We are ready to build

```
mkdir build
cd build
cmake ../ -DUSE_OSG=true
make -j4
```

Try running an example

```
cd EnvironmentSimulator/OpenDriveViewer
./OpenDriveViewer --odr ../../../resources/xodr/straight_500m.xodr --model ../../../resources/models/straight_500m.osgb --window 50 50 800 400
```

or

```
cd EnvironmentSimulator/EnvironmentSimulator
./EnvironmentSimulator --osc ../../../resources/xosc/highway_merge.xosc --window 50 50 800 400
```

or

```
cd EnvironmentSimulator/EgoSimulator
./EgoSimulator --osc ../../../resources/xosc/highway_merge.xosc --ext_control on --window 50 50 800 400
```

