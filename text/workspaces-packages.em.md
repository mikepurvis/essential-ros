Workspaces & Packages
=====================

A major concept in ROS is workspaces. A workspace, at its core, is a handful of environment variables which tell key ROS tools how and where to find packages. First, we're going to explore the workspace provided with the APT-installed ROS packages, and then see how to create a source workspace, where package development may occur.

The APT Workspace
-----------------

If you've installed ROS @(rosdistro.capitalize()) via APT, then you have packages installed in the `/opt/ros/@(rosdistro)` directory. There's a workspace provided for these packages, which you can use like so:

~~~bash
$ source /opt/ros/@(rosdistro)/setup.bash
~~~

Eventually, you'll probably want this in your `.bashrc` file. For now, though, just run it in the terminal.

Now your system knows about the installed ROS packages. You can prove this to yourself with `rospack find`, which outputs the path to a given package. Try using it to locate the `roscpp` package:

~~~bash
$ rospack find roscpp
/opt/ros/@(rosdistro)/share/roscpp
~~~

In addition to knowing where all your packages are, the workspace adds tools like rospack to the system path. These are all prefixed with either `ros` or `catkin`. Try typing the prefixes and using tab completion to get a sense of the ROS utilities available in this environment.

Creating an Overlay Workspace
-----------------------------

The overlay workspace will still permit you to use all of the packages in the underlying workspace, but any instances of packages in the overlay will supercede their counterparts in workspaces below. This is a powerful system which lets you mix APT and source packages. To create your overlay, execute the following:

~~~bash
$ mkdir -p ~/ros_ws/src && cd ~/ros_ws
$ catkin_init_workspace src
$ catkin_make
~~~

That's it, your workspace is created. To use the workspace, source its `setup.bash` file:

~~~bash
$ source ~/ros_ws/devel/setup.bash
~~~

As it stands, the newly-created workspace doesn't contain anything, so there's no visible change. But keep following as we add a package to this new workspace.

Creating a Package
------------------

Now that there's a workspace to put it in, we can create a package. Navigate into the workspace's `src` directory, and create a package there:

~~~bash
$ cd ~/ros_ws/src
$ catkin_create_pkg shiny_thing
~~~

Now, demonstrate that we can find the new package:

~~~bash
$ rospack find shiny_thing
/home/user/ros_ws/src/shiny_thing
~~~

Use the `roscd` command to jump inside the package folder and look around:

~~~bash
$ roscd shiny_thing
$ ls
CMakeLists.txt  package.xml
~~~

This is what a bare-minimum package looks like---a `CMakeLists.txt` file, which tells catkin how to build the package, and a `package.xml`, with metadata like the name, authorship details, and dependencies. Neither file needs to be changed for now, but both are heavily commented with instructions for typical configurations, so when the time comes, there are examples to follow.

There are some optional tips below, or you can jump directly to the next chapter to get started on building a C++ ROS node. 

Workspace Suggested Practices
-----------------------------

Workspaces are lightweight and disposable. Use a separate one for each task you're working on, and avoid getting too attached to them. Reference this section in the future if you need help getting another one set up quickly.

If you're predominantly developing in ROS @(rosdistro.capitalize), it does make sense to source its APT workspace in your `.bashrc` file. In addition, you can create aliases to workspaces which you are using a lot. And example configuration with a `kf` shortcut for jumping into a Kingfisher workspace might look like the following:

~~~bash
source /opt/ros/@(rosdistro)/setup.bash
alias kf="source ${HOME}/kf_ws/devel/setup.bash"
~~~

A more complex, but generalized version of this same approach will let you jump into a `kf_ws` workspace with `ws kf_ws`. It also includes tab completion, so that you can simply type `ws<tab>`, and see a list of workspaces in the root of your home directory:

~~~bash
source /opt/ros/@(rosdistro)/setup.bash
function ws() {
  source ~/$1/devel/setup.bash
  roscd && cd ..
}
function _ws() {
  cur="${COMP_WORDS[COMP_CWORD]}"
  workspaces=$(find "${HOME}" -maxdepth 3 -path "*devel/setup.bash" \
      | cut -d/ -f 4 | tr '\n' ' ')
  COMPREPLY=( $(compgen -W "${workspaces}" -- ${cur}) )
}
complete -F _ws ws
~~~


