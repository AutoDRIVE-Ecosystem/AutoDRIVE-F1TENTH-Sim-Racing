# AutoDRIVE-F1TENTH-Sim-Racing
F1TENTH Digital Twin Autonomous Sim-Racing League using AutoDRIVE Ecosystem

## This repository is currently a work-in-progress and under heavy development. Stable versions will be soon available under Releases.

WSL2 Windows 10/11: run XServer with -nowgl (and -ac):
`"C:\Program Files\VcXsrv\vcxsrv.exe" :0 -multiwindow -clipboard -nowgl -ac`
see https://askubuntu.com/a/1394781/498339

Build Simulator Container:

`docker build --tag autodriveecosystem/autodrive_f1tenth_sim:v2024.1 -f autodrive_simulator.Dockerfile .`

Launch Simulator Container:

`xhost local:root`

`docker run --rm -it --network=host --ipc=host -v /tmp/.X11-unix:/tmp.X11-umix:rw --env DISPLAY --privileged --gpus all autodriveecosystem/autodrive_f1tenth_sim:v2024.1`

Test GUI via Container:
Install x11 apps: `apt-get install x11-apps`
Run x11 eyes: `xeyes`

Build Devkit Container:

`docker build --tag autodriveecosystem/autodrive_f1tenth_api:v2024.1 -f autodrive_devkit.Dockerfile .`

Launch Devkit Container:

`xhost local:root`

`docker run --rm -it --network=host --ipc=host -v /tmp/.X11-unix:/tmp.X11-umix:rw --env DISPLAY --privileged --gpus all autodriveecosystem/autodrive_f1tenth_api:v2024.1`

Exit any Container
`exit`
