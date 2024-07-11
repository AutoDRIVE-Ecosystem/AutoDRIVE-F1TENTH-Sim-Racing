# AutoDRIVE-F1TENTH-Sim-Racing
F1TENTH Digital Twin Autonomous Sim-Racing League using AutoDRIVE Ecosystem

> [!NOTE]
> **This repository is currently a work-in-progress and under heavy development. Stable versions will be soon available under [Releases](https://github.com/AutoDRIVE-Ecosystem/AutoDRIVE-F1TENTH-Sim-Racing/releases).**

## Build Simulator Container:

```bash
docker build --tag autodriveecosystem/autodrive_f1tenth_sim:v2024.1 -f autodrive_simulator.Dockerfile .
```

## Run Simulator Container:

```bash
xhost local:root
docker run --rm -it --network=host --ipc=host -v /tmp/.X11-unix:/tmp.X11-umix:rw --env DISPLAY --privileged --gpus all autodriveecosystem/autodrive_f1tenth_sim:v2024.1
```

## Build Devkit Container:

```bash
docker build --tag autodriveecosystem/autodrive_f1tenth_api:v2024.1 -f autodrive_devkit.Dockerfile .
```

## Run Devkit Container:

```bash
xhost local:root
docker run --rm -it --network=host --ipc=host -v /tmp/.X11-unix:/tmp.X11-umix:rw --env DISPLAY --privileged --gpus all autodriveecosystem/autodrive_f1tenth_api:v2024.1
```

## Generally Helpful Docker Tips
1. To access the container while it is running, execute the following command in a new terminal window to start a new bash session inside the container:
```bash
docker exec -it <container_name> bash
```

2. To exit the bash session(s), simply execute:
```bash
exit
```

3. To kill the container, execute the following command:
```bash
docker kill <container_name>
```

4. To remove the container, simply execute:
```bash
docker rm <container_name>
```

5. Running or caching multiple docker images, containers, volumes, and networks can quickly consume a lot of disk space. Hence, it is always a good idea to frequently check docker disk utilization:
```bash
docker system df
```

6. To avoid utilizing a lot of disk space, it is a good idea to frequently purge docker resources such as images, containers, volumes, and networks that are unused or dangling (i.e. not tagged or associated with a container). There are several ways with many options to achieve this, please refer to appropriate documentation. The easiest way (but a potentially dangerous one) is to use a single command to clean up all the docker resources (dangling or otherwise):
```bash
docker system prune -a
```
