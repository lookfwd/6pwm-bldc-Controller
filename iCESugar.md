# How to iCESugar

```
git clone --recurse-submodules https://github.com/YosysHQ/yosys.git

sudo apt-get update
sudo apt-get install build-essential clang lld bison flex \
	libreadline-dev gawk tcl-dev libffi-dev git \
	graphviz xdot pkg-config python3 libboost-system-dev \
	libboost-python-dev libboost-filesystem-dev zlib1g-dev

cd yosys
make config-gcc
make -j $(( $(nproc) + 1 ))
sudo make install
cd ..

sudo apt install cmake libboost-all-dev libftdi-dev

#git clone --recursive https://github.com/YosysHQ/prjtrellis
#cd prjtrellis/libtrellis
#cmake -DCMAKE_INSTALL_PREFIX=/usr/local .
#make -j $(( $(nproc) + 1 ))
#sudo make install
#cd ../..

# https://prjicestorm.readthedocs.io/en/latest/overview.html
git clone https://github.com/YosysHQ/icestorm.git icestorm
cd icestorm
make -j $(( $(nproc) + 1 ))
sudo make install
cd ..

sudo apt install libeigen3-dev


git clone --recurse-submodules https://github.com/YosysHQ/nextpnr.git
cd nextpnr
mkdir -p build && cd build
cmake .. -DARCH=ice40 -DCMAKE_INSTALL_PREFIX=/usr/local
make -j $(( $(nproc) + 1 ))
sudo make install
cd ../..
```

Run the examples:

```
git clone https://github.com/damdoy/ice40_ultraplus_examples.git
cd /ice40_ultraplus_examples/leds
ls -la leds.bin 
```

```
-rw-rw-r-- 1 ubuntu ubuntu 104090 May  9 19:25 leds.bin
```

Drag-n-drop leds.bin to the iCELink directory and you will see it working

[![](https://img.youtube.com/vi/Mp8vq3lygMc/0.jpg)](https://www.youtube.com/watch?v=Mp8vq3lygMc)


To build and test this repo...

```
git clone git@github.com:lookfwd/icestick-6pwm-bldc-Controller.git
cd icestick-6pwm-bldc-Controller
make clean && rm log.txt && make 2>&1 | tee log.txt
``` 
