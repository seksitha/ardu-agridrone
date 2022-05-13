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
1. folder of `complete` and `environment_install` 
### docker run --rm -it -v `pwd`:/ardupilot sitha-ardu:latest bash
1. `waf configure --board fmuv3`
2. `waf copter` 

# build by vagrant
- first need to add some config to `Vagrantfile` so vagrant can ssh to the box

`config.ssh.private_key_path = ["~/.ssh/seksitha", "~/.vagrant.d/insecure_private_key"]` \
`config.vm.provision "file", source: "~/.ssh/seksitha.pub", destination: "~/.ssh/authorized_keys"`

- Then we can build

`cd Tools/vagrant` \
`vagrant up`

##### in case ubuntu install fail apt-get install
copy `environment_install from master/dev branch` \
`ssh vagrant` \
`cd ../../vagrant` \
`cd vagrant` \
`if install fail change package version to 2.4 instead of 2.5 in install-prereqs-ubuntu.sh` \
`sudo ./Tools/vagrant/initvagrant.sh`

- for windows need to install `Xming` or you fail because it try to open extra terminal and cause error of `xTerm`. I don't know `cywine` but I also install it

- There are serveral erro in running `.sh` file because of the `\r` need to run command `sed -i 's/\r$//' filename`  


### Use CUAV PW-link and forward the packet to virtualbox mission planner

`mavproxy.py --master=udp:192.168.4.2:14550 --out=udp:192.168.4.3:14551` \
`mavproxy.py --master=udp:localhost:14550 --out=udp:localhost:14555 --out=udp:localhost:14554`

mavproxy.py --master=tcp:10.0.2.15:5760 --out 10.0.2.15:14551