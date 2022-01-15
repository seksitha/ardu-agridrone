## add and generate binding
`cd libraries/AP_Scripting/generator`
`make run` 

## build by using vagrant from SITL

1. `cd ~/ardupilot/Tools/vagrant`
2. `vagrant up && vagrant ssh` 
3. `cd ..`
4. `./waf configure --board fmuv3` // ./waf configure --board CUAVv5Nano
5. `./waf copter`

## after build go to: 
* `build/fmuv3/bin/arducopter.hex`
* copy the file for apm_planner for uploading firmware


# build by docker
run `git submodule update --init --recursive` or the build will fail
### need to copy docker sh exec from dev brach 
1. `complete` and `environment_install` folder to Tools
### docker run --rm -it -v `pwd`:/ardupilot sitha-ardu:latest bash
1. `waf configure --board fmuv3`
2. `waf copter` 

# build by vagrant

`cd Tools/vagrant`
`vagrant up`

##### in case ubuntu install fail apt-get install
copy `environment_install from master/dev branch`
`ssh vagrant`
`cd ../../vagrant`
`cd vagrant`
`sudo apt-get --assume-yes install build-essential ccache g++ gawk git make wget cmake libtool libxml2-dev libxslt1-dev python-dev python-pip python-setuptools python-numpy python-pyparsing xterm python-matplotlib python-serial python-scipy python-opencv libcsfml-dev libcsfml-audio2.4 libcsfml-dev libcsfml-graphics2.4 libcsfml-network2.4 libcsfml-system2.4 libcsfml-window2.4 libsfml-audio2.4 libsfml-dev libsfml-graphics2.4 libsfml-network2.4 libsfml-system2.4 libsfml-window2.4 python-yaml python-argparse python-wxgtk3.0 libtool-bin g++-arm-linux-gnueabihf pkg-config-arm-linux-gnueabihf lcov gcovr`

`./install`

### Use CUAV PW-link and forward the packet to virtualbox mission planner

`mavproxy.py --master=udp:192.168.4.2:14550 --out=udp:192.168.4.3:14551`
`mavproxy.py --master=udp:localhost:14550 --out=udp:localhost:14555 --out=udp:localhost:14554`