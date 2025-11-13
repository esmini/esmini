if [ ! -d "build/" ]; then
    mkdir "build/"
fi

cd build/
emcmake cmake ..  
emmake make -j $(nproc)
cp esmini.js ../example/