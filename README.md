# AutoDRIVE-F1TENTH-Sim-Racing
F1TENTH Digital Twin Autonomous Sim-Racing League using AutoDRIVE Ecosystem

> [!NOTE]
> - The setup has been only tested on the [Ubuntu](https://ubuntu.com) operating system.
> - It is assumed that [Docker](https://docs.docker.com/engine/install) is installed.
> - It is assumed that if the Docker container is to take advantage of an NVIDIA GPU, the host machine has been properly configured by installing the necessary NVIDIA GPU drivers and the [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/index.html).

## Build AutoDRIVE Simulator Container:

```bash
docker build --tag autodriveecosystem/autodrive_f1tenth_sim:v2024.1 -f autodrive_simulator.Dockerfile .
```

## Run AutoDRIVE Simulator Container:

```bash
xhost local:root
docker run --name autodrive_f1tenth_sim --rm -it --network=host --ipc=host -v /tmp/.X11-unix:/tmp.X11-umix:rw --env DISPLAY --privileged --gpus all autodriveecosystem/autodrive_f1tenth_sim:v2024.1
```

## Push AutoDRIVE Simulator Container:

1. Run the image you created in the previous step inside a container:
```bash
xhost local:root
docker run --name autodrive_f1tenth_sim --rm -it --network=host --ipc=host -v /tmp/.X11-unix:/tmp.X11-umix:rw --env DISPLAY --privileged --gpus all autodriveecosystem/autodrive_f1tenth_sim:v2024.1
```

2. In a new terminal window, list all containers and make a note of the desired `CONTAINER ID`:
```bash
docker ps -a
```

3. Commit changes to Docker Hub:
```bash
docker commit -m "AutoDRIVE-F1TENTH-SimRacing" -a "AutoDRIVE Ecosystem" <CONTAINER ID> autodriveecosystem/autodrive_f1tenth_sim:v2024.1
```

4. Login to Docker Hub:
```bash
docker login
```

5. Push the container to Docker Hub, once done, you should be able to see your repository on Docker Hub:
```bash
docker push autodriveecosystem/autodrive_f1tenth_sim:v2024.1
```

## Build AutoDRIVE Devkit Container:

```bash
docker build --tag autodriveecosystem/autodrive_f1tenth_api:v2024.1 -f autodrive_devkit.Dockerfile .
```

## Run AutoDRIVE Devkit Container:

```bash
xhost local:root
docker run --name autodrive_f1tenth_api --rm -it --network=host --ipc=host -v /tmp/.X11-unix:/tmp.X11-umix:rw --env DISPLAY --privileged --gpus all autodriveecosystem/autodrive_f1tenth_api:v2024.1
```

## Push AutoDRIVE Devkit Container:

1. Run the image you created in the previous step inside a container:
```bash
xhost local:root
docker run --name autodrive_f1tenth_api --rm -it --network=host --ipc=host -v /tmp/.X11-unix:/tmp.X11-umix:rw --env DISPLAY --privileged --gpus all autodriveecosystem/autodrive_f1tenth_api:v2024.1
```

2. In a new terminal window, list all containers and make a note of the desired `CONTAINER ID`:
```bash
docker ps -a
```

3. Commit changes to Docker Hub:
```bash
docker commit -m "AutoDRIVE-F1TENTH-SimRacing" -a "AutoDRIVE Ecosystem" <CONTAINER ID> autodriveecosystem/autodrive_f1tenth_api:v2024.1
```

4. Login to Docker Hub:
```bash
docker login
```

5. Push the container to Docker Hub, once done, you should be able to see your repository on Docker Hub:
```bash
docker push autodriveecosystem/autodrive_f1tenth_api:v2024.1
```

## Generally Helpful Docker Tips:
1. To access the container while it is running, execute the following command in a new terminal window to start a new bash session inside the container:
```bash
docker exec -it <CONTAINER NAME> bash
```

2. To exit the bash session(s), simply execute:
```bash
exit
```

3. To kill the container, execute the following command:
```bash
docker kill <CONTAINER NAME>
```

4. To remove the container, simply execute:
```bash
docker rm <CONTAINER NAME>
```

5. Running or caching multiple docker images, containers, volumes, and networks can quickly consume a lot of disk space. Hence, it is always a good idea to frequently check docker disk utilization:
```bash
docker system df
```

6. To avoid utilizing a lot of disk space, it is a good idea to frequently purge docker resources such as images, containers, volumes, and networks that are unused or dangling (i.e. not tagged or associated with a container). There are several ways with many options to achieve this, please refer to appropriate documentation. The easiest way (but a potentially dangerous one) is to use a single command to clean up all the docker resources (dangling or otherwise):
```bash
docker system prune -a
```
