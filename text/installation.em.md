Installation
============

All ROS software is organized in modular units called packages. A ROS package may be added to your system by checking out and building the repository which contains it; the ROS system contains tools which help you do this, including resolving dependencies which that package may have on other packages in the ROS ecosystem.

For Ubuntu, many ROS packages are also available through the native Ubuntu package management system, APT. All of the core ROS packages have been available this way from early on, and today many third party ROS packages are also conveniently available through APT. 

To get started, first make sure you're on an Ubuntu @(ubuntu_num) machine.

The following snippet adds the ROS package repo to Ubuntu, adds the public key which verifies that the packages have not been tampered with since release, and triggers APT to update its list of available packages for us.

~~~bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu @(ubuntu) main" \
       > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update
~~~

Now that APT knows about ROS, we can install it:

~~~bash
sudo apt-get install ros-@(rosdistro)-ros-base
~~~

You'll need to download about 200MB of archives from packages.ros.org to do the minimal "base" install of ROS @(rosdistro.capitalize()). It's a much bigger download to perform the full desktop install---you can do that later once you actually need it.

That's it, you have the core ROS software on your computer, now let's build!
