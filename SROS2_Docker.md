# Testing in a docker container (Fast-RTPS only)

## Building the docker image

To setup the environment, first clone this repository:

```
mkdir ~/sros2_demo && cd ~/sros2_demo
git clone https://github.com/ros2/sros2.git
```

And then build the docker container:

```
cd ~/sros2_demo/sros2/sros2_docker/docker
./build_sros2.sh
```

This may take a few minutes.

Finally enter the container:

```
./run_sros2_container.sh
```

Hooray we are now ready to start the demo.

## Run the demo

We will start by creating a set of keys for our DDS participant:

```
cd /root/sros2_demo
ros2 security create_keystore demo_keys
ros2 security create_key demo_keys talker
ros2 security create_key demo_keys listener
```

And now start testing!

In the terminal inside docker, run:

```
ros2 run demo_nodes_py talker
```

This will start a python executable that publishes messages periodically.

Now let's open another terminal into the container by opening a new terminal, then running:

```
docker exec -ti <CONTAINER_NAME> /bin/bash
ros2 run demo_nodes_cpp listener
```

Hooray our nodes are authenticated and talking encrypted-talk!

