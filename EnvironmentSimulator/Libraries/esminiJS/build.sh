if [ ! -d "build/" ]; then
    mkdir "build/"
fi

cd build/
emcmake cmake ..  
emmake make
cp esmini.js ../example/