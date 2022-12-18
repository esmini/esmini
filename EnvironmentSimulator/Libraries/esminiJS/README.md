## esminijs 
- 只是一个例子，不是完整的功能（Just an example, not full functionality)
- 只是一个例子，不是完整的功能（Just an example, not full functionality)
- 只是一个例子，不是完整的功能（Just an example, not full functionality)

## points of attention
- 1、每个文件在读取之前都需要关联，resources 中包含的资源不可以被使用(Each file needs to be associated before reading, and the resources contained in resources cannot be used)
- 2、功能未充分测试（Functionality not fully tested）

## Install
Tested on Windows 10 (from CMD), macOS (Catalina) and Linux (Ubuntu 22.04, glic 2.35, gcc/g++ 11.3)

### 1. Install emscripten

* Get the emsdk repo

    ```
    git clone https://github.com/emscripten-core/emsdk.git
    cd emsdk
    ```

* Fetch the latest version of the emsdk (not needed the first time you clone)

    ```
    git pull
    ```

* Download and install the latest SDK tools

    Linux & Mac
    ```
    ./emsdk install latest
    ```
    Windows cmd
    ```
    emsdk install latest
    ```
* Make the "latest" SDK "active" for the current user. (writes .emscripten file)

    Linux & Mac
    ```
    ./emsdk activate latest
    ```
    Windows cmd
    ```
    emsdk activate latest
    ```

* Activate PATH and other environment variables in the current terminal

    Linux & Mac
    ```
    source ./emsdk_env.sh
    ```
    Windows cmd
    ```
    emsdk_env.bat
    ```

### 2. Install other system tools and libraries

Linux
```
sudo apt install cmake  bzip2
```
Windows cmd

* If not already done, install cmake (https://cmake.org/download/)
* Note: bzip2 not needed (it seems)
* Download and extract ninja binary (https://github.com/ninja-build/ninja/releases) to some folder

```
set path=%path%;<folder containing ninja executable>
```

Mac: ? (how do you install cmake and bzip2 on Mac?)

### 3. Build js

```
cd esmini/EnvironmentSimulator/Libraries/esminiJS
```

Linux & Mac
```
./build.sh
```

Windows cmd:
```
mkdir build
cd build
emcmake cmake ..
emmake ninja
copy esmini.js ..\example /y
cd ..
```

### 4. check esmini.js
* Check existence and date

    Linux & Mac
    ```
    ll example/esmini.js
    ```

    Windows cmd
    ```
    dir example\esmini.js
    ```

* Load examples/index.html in browser
    - The following text should appear: `Will not use threejs, please see the console output` (no graphics to expect)
    - Check console for info printouts, e.g. Ego position
