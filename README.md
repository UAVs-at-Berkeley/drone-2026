# drone-2026

##SITL (software in the loop simulator)

How to download/install run the sitl docker image:
1. Make sure you have docker installed on your computer
2. Download this repository (i.e. clone it or download it directly)
3. Navigate to the /sitl directory in a terminal
4. To build the docker image, run: 

    docker build -t sitl .

    4a: this is a CHONKER. Expect it to take around 30 min - 1 hr. to build, and takes about
        2 GB for the Ubuntu OS
        13 GB for SITL + dependencies
        ? GB for QGroundControl
        ? GB for ROS2
        for a total of ? GB
        The benefit is that you will never have to worry about this step ever again, and the simulator should always work.* knock on wood

5. To start the docker container, run: 

    docker run -it -e DISPLAY=host.docker.internal:0 sitl

    The -it flag means "in an interactive terminal"
    The -e flag means "with environment variable"; we want to set the DISPLAY to connect to your device so you can see the Gazebo simulator window and interact with it

    5a: For the display to work, you need to have an "X" server running on your computer. On Windows, you can do this by downloading XLaunch and starting up a server. On Mac, this is typically accomplished with XQuartz. THIS IS IMPORTANT, otherwise, the GUI's will never show up.

6. Once in the container, you should see a bash terminal. you can try typing "ls" or "help", these should work as expected. You are now inside a virtual environment running Ubuntu.

7. To start the simulation, run (in bash):

    make px4_sitl gz_x500

    If successful, you should see the **PX4** logo come up, along with a window for the Gazebo simulator and the "drone" sitting on the ground.


### SITL debugging stuff

Q: When I run the simulation in step 7, I don't see a Gazebo window

A: It's likely a problem with your display forwarding. Check 5a. A good way to test is, once in the docker container bash, run 

apt-get install x11-apps 

and then run the command xeyes. If your display forwarding is working, you should get a window in the top left with a pair of googly eyes. If the googly eyes are not there, it's your display forwarding that still isn't working. You can run export DISPLAY=... when you're already in the bash terminal to try to change this.
