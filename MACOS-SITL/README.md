# Software-in-the-Loop Simulator for MacOS

This guide contains instructions on how to set up SITL on ARM-based Macs through Docker.

## Getting the Docker image

You first need to get the Docker image by following the steps below:

1. Make sure you have Docker Desktop installed on your machine

2. Make sure you have the latest version of this repo

3. Open up Docker Desktop

4. Open up a terminal session, navigate to `drone-2026/MACOS-SITL` and type the following:

    ```zsh
    docker build -t drone-sim-macos:latest .
    ```

    This is going to take a while.

## Instantiating a container

After you have the docker image, the next step is to create a container from the image:

```zsh
docker run --rm -it -p 5900:5900 drone-sim-macos:latest
```

* ```--rm``` removes the container instance after quitting
* ```-it``` allows you to interact with the container (through terminal sessions)
* ```-p 5900:5900``` creates a mapping between the port 5900 on your machine and the port 5900 inside the container. More on this in the next section.

## Setting up RealVNC Viewer

Download [RealVNC Viewer](https://www.realvnc.com/en/connect/download/viewer/?lai_vid=JGlLWyj1OULgq&lai_sr=0-4&lai_sl=l&lai_p=1&lai_na=611310). For what we are doing, it is free to use. This is needed for the Gazebo UI to properly display. Open up the RealVNC Viewer and type ```localhost:5900``` in the search bar and connect to it. This tells the VNC client on your machine to listen to port 5900. You will get a warning saying that the connect to that port isn't encrypted; this is fine because the port isn't exposed to any external network. The VNC server inside the container will send Gazebo UI stream to the port 5900 inside the container (this has been configured in the image), then Docker will route the traffic to the port 5900 on your machine (because of ```-p 5900:5900```), and then the RealVNC Viewer (the client) will be able to render the Gazebo UI.

## Running an example SITL session

There are a few more manual steps to do. Here's what the workflow looks like:

1. Open up Docker Desktop

2. Instantiate a container:

    ```zsh
    docker run --rm -it -p 5900:5900 143f0fd9033f
    ```

3. Open up the RealVNC Viewer and connect to ```localhost:5900```

4. Inside the container, run

    ```bash
    make px4_sitl gz_x500
    ```

    Tip: if the terminal is messy, press the enter/return key a few times then type ```clear``` to clean up the mess, and then run the command.

    After the build is successful and you are inside the PX4 shell, open up a second terminal window for step 6 (this is a good practice anyways). You can do so with first ```ctrl + B``` then ```"``` (```shift + '```). More tmux tricks at the end of this guide.

5. Before moving on, it's good to check that you can see the Gazebo UI in your RealVNC Viewer.

6. Now we need to manually connect PX4 to QGroundControl. The QGroundControl app is NOT inside the container; instead, make sure you have it installed on your machine.
    * Open up QGroundControl.
    * Inside the container, run the following in the second terminal window (not the PX4 shell!):

        ```bash
        getent ahostsv4 host.docker.internal
        ```

        This essentially outputs the IPv4 address of the container, and all the data that gets sent to this IP will automatically be rerouted to your machine by Docker. Copy this IP.

    * Now in the PX4 shell, run the following to connect PX4 to QGC through MAVLink:

        ```shell
        mavlink start -x -u 14500 -r 4000000 -t [the IP you just copied]
        ```

    * Now you should see from QGC that it is connected to the drone.

7. To run an example script that uses ROS to command the drone to arm and takeoff to 5 meters, you can run (not in the PX4 shell!)

    ```bash
    ros2 run px4_ros_com offboard_control
    ```

    You can observe the drone moving in Gazebo, and the battery percentage dropping in QGC.

## Terminal multiplexor (tmux) tricks

* ctrl + B and " (shift+') adds another terminal to the above.
* ctrl + B and % (shift+5) adds another terminal to the side.
* ctrl + B and ARROWKEYS (left right up down) allows you to navigate between terminals
* ctrl + B and [ allows you to scroll up in the current terminal you're in to look at previous stuff. hit escape to go back to normal.
* ctrl + B and R to refresh the screen
