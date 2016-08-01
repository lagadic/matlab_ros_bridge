# matlab_ros_bridge

This page contains the source code of a set of Matlab C++ S-functions that can be used to:

* synchronize simulink with the system clock, thus obtaining a soft-real-time execution;
* interface simulink blocks with other ROS nodes using ROS messages.

This project is based on a work started by Martin Riedel and Riccardo Spica at the Max Plank Institute for Biological Cybernetics in Tuebingen (Germany).
This fork is currently supported by [Riccardo Spica](mailto:riccardo.spica@irisa.fr) and [Giovanni Claudio](mailto:giovanni.claudio@irisa.fr) at the Inria Istitute in Rennes (France).

The software is released under the BSD license. See the LICENSE file in this repository for more information.

##Configuring Matlab

Before proceeding with the next steps you need to configure Matlab mex compiler.
 
For Matlab 2014b and older, run matlab and type the following command in the command window:
```matlab
mex -setup
```
When you get the following message:

    Enter the number of the compiler (0-1):
    
Enter 1 to select the first option. This should create a mex option file in ~/.matlab/R2014a/mexopts.sh.
Open this file and make sure that the correct version of the gnu compiler is used.

For Matlab 2015a and newer, run matlab and type the following commands in the command window
```matlab
mex -setup
mex -setup C++
```
This should create two files in ~/.matlab/<matlab_version>/ named mex_C_glnxa64.xml and mex_C++_glnxa64.xml.
Open mex_C++_glnxa64.xml and substitute all instances of "g++" and "$GCC" with "g++-4.7" (or 4.4 for older versions of Matlab).
Open mex_C_glnxa64.xml and substitute all instances of "gcc" and "$GCC" with "gcc-4.7" (or 4.4 for older versions of Matlab).

##Compiling the bridge

The process of compiling the matlab_ros_bridge is not straightforward. The main problem is that MATLAB doesn't use the system distribution of boost but instead comes with its ows shipped version, which can be found in, e.g. 'matlabroot/bin/glnxa64/'.
Since the mex files that we generate will run inside matlab it is important that they are linked against the same version of boost that is used in MATLAB. Moreover, since the mex files will also be linked to ROS libraries, we also need to recompile ROS and link it to the same version of boost.
Finally we also need to compile everyting (boost, ros and our mex files) using a c/c++ compiler officially supported by the MATLAB distribution that we are using.

###Compiling boost

This section will guide you through the process of compiling the required version of boost with the compiler supported by your Matlab version. Before doing this you should try using one of the precompiled boost distributions available in the download section of this repository. Check the version correspondance table below in this page to find the correct download link.
If you cannot find a precompiled boost download link for you setup then keep reading, otherwise skip to the following section.

1. Download the correct version of boost from [here](http://www.boost.org/users/history/) in a folder of your choice (later referred to as <boost_dir>). To know which version you need you can type, in a matlab command window, the following command: 

    ```matlab
    ls([matlabroot '/bin/glnxa64/libboost_date_time*'])
    ```
2. Open a new terminal and navigate to <boost_dir> and give the following command: 

    ```bash
    $ ./bootstrap.sh --prefix=path/to/boost/installation/prefix
    ```
3. Edit the file <boost_dir>/project-config.jam with your favourite tool and substitute: 

    >using gcc;
    
    with (e.g.) 
    
    >using gcc : 4.4 : g++-4.4 : <cxxflags>-std=c++0x ;
    
    The exact version of gcc/g++ that you need to use depends on the matlab release (check it on [Matlab](http://www.mathworks.it/support/sysreq/previous_releases.html) website).

4. Build boost by doing:

    ```bash
    $ ./bjam link=shared -j8 install
    ```
Note: while building Boost you might encounter [this pseudo-bug](https://svn.boost.org/trac/boost/ticket/6940) due to an incompatibility between older versions of Boost and the new C11 standard. To solve this you can either substitute all occurrences of `TIME_UTC` in all Boost headers with `TIME_UTC_` (as done in more recent versions of Boost). For istance, in version 1.49.0, you need to modify <boost_dir>/boost/thread/xtime.hpp and <boost_dir>/libs/thread/src/pthread/timeconv.inl.

In some cases you might be able to solve this issue by modifying <boost_dir>/project-config.jam so as to change

>using gcc : 4.4 : g++-4.4 ;

to

>using gcc : 4.4 : g++-4.4 : <cxxflags>-U_GNU_SOURCE ;

If you are building boost on a x64 system you might also encounter [this bug](https://svn.boost.org/trac/boost/ticket/6851). In this case just apply the proposed fix.

###Compiling ROS

5. Follow the instructions for your ROS distribution on `http://wiki.ros.org/<distro>/Installation/Source` (e.g. for [Indigo](http://wiki.ros.org/indigo/Installation/Source)), to install ROS-Comm in the "wet" version until you need to compile. DON'T COMPILE NOW (it means that you don't have to do a catkin_make or catkin_make_isolated). To reduce the number of ros packages to compile, you can also initialize your workspace with

    ```bash
    $ rosinstall_generator roscpp geometry_msgs std_msgs sensor_msgs shape_msgs --rosdistro indigo --deps --wet-only --tar > mrb.rosinstall
    ```

6. In a terminal navigate to the src directory of the catkin workspace created in the previous step and do:

    ```bash
    $ wstool set matlab_ros_bridge --git https://github.com/lagadic/matlab_ros_bridge.git
    $ wstool update matlab_ros_bridge
    ```
7. Before compiling you might also need to modify the file src/roscpp/src/libros/param.cpp as described [here](https://github.com/ros/ros_comm/commit/0a589a52f5296bb3002a2f97912989715f064630).

8. Compile ros and the bridge with:

    ```bash
    $ src/catkin/bin/catkin_make --cmake-args -DBOOST_ROOT=path/to/boost/installation/prefix -DBoost_NO_SYSTEM_PATHS=ON -DCMAKE_C_COMPILER=/usr/bin/gcc-4.7 -DCMAKE_CXX_COMPILER=/usr/bin/g++-4.7 -DMATLAB_DIR=/usr/local/MATLAB/R2015b
    ```
    note that you might need to change this command according to your <boost_dir>, Matlab path and compiler version.
    Add `install` at the end or run `catkin_make install` if desired.
    For old versions of ROS (before Indigo) you may need to use the command `catkin_make_isolated` instead of `catkin_make`.
    Note: the incompatibility issue discussed in the previous section might cause `rosbag` (and possibly other packages) to fail building. If this is the case, either substitute all occurrences of `TIME_UTC` in all rosbag source files with `TIME_UTC_` or add `-DCMAKE_CXX_FLAGS=-U_GNU_SOURCE` to your `catkin_make` command above.

9. Make sure you have "sourced" your workspace by running:

    ```bash
    $ source /path/to/your/catkin_ws/devel_isolated/setup.bash
    ````
    
    Navigate to the build directory of the package `matlab_ros_bridge`. It should be  `catkin_ws/build/` or `catkin_ws/build_isolated/matlab_ros_bridge` if you used `catkin_make_isolated`. Now generate the simulink block library by running:

    ```bash
    $ cmake .
    $ make generate_library
    ```

###Running MATLAB

10. In your [MATLAB Startup File](http://www.mathworks.it/it/help/matlab/matlab_env/startup-options.html) add the following lines:

    ```matlab
    addpath(fullfile('path','to','your','catkin_ws','devel_isolated','matlab_ros_bridge','share','matlab_ros_bridge'));
    run(fullfile('path','to','your','catkin_ws','devel_isolated','matlab_ros_bridge','share','matlab_ros_bridge','setup.m'));
    ```
11. To run matlab open a terminal and type the following:

    ```matlab
    $ source /path/to/your/catkin_ws/devel_isolated/setup.bash
    $ matlab
    ```

###Testing that everything works

12. In your matlab command window navigate to the folder `/path/to/your/catkin_ws/src/matlab_ros_bridge/matlab_ros_bridge/models` and type

    ```matlab
    Tsim = 2e-5;
    ```
    
    Now open the model `test.slx` and try to run it in all different running mode.

##Note

It might be possible (but it has never been tested) to avoid compiling boost and try to link ros and the mex files against the boost libraries contained in the MATLAB installation directory.


##Matlab boost and cpp version correspondances and download link

Matlab version  | gcc supported version | shipped boost vesion | compiled boost download | Matlab compiled version
------------- | -------------- | ------------- | ------------- | ------------- |
2012a  | GNU gcc/g++ 4.4.x | [1.44.0](http://sourceforge.net/projects/boost/files/boost/1.44.0/boost_1_44_0.tar.gz/download) | [boost_1_44_0_gcc_4_4.tar.bz2](https://github.com/lagadic/matlab_ros_bridge/releases/download/v0.1/boost_1_44_0_gcc_4_4.tar.bz2) | [boost_R2012a_x64.tar.gz](https://github.com/lagadic/matlab_ros_bridge/releases/download/v0.1/boost_R2012a_x64.tar.gz)
2012b  | GNU gcc/g++ 4.4.x | [1.44.0](http://sourceforge.net/projects/boost/files/boost/1.44.0/boost_1_44_0.tar.gz/download) | [boost_1_44_0_gcc_4_4.tar.bz2](https://github.com/lagadic/matlab_ros_bridge/releases/download/v0.1/boost_1_44_0_gcc_4_4.tar.bz2) | [boost_R2012b_x64.tar.gz](https://github.com/lagadic/matlab_ros_bridge/releases/download/v0.1/boost_R2012b_x64.tar.gz)
2013a  | GNU gcc/g++ 4.4.x | [1.49.0](http://sourceforge.net/projects/boost/files/boost/1.49.0/boost_1_49_0.tar.gz/download) | [boost_1_49_0_gcc_4_4.tar.bz2](https://github.com/lagadic/matlab_ros_bridge/releases/download/v0.1/boost_1_49_0_gcc_4_4.tar.bz2) | [boost_R2013a_x64.tar.gz](https://github.com/lagadic/matlab_ros_bridge/releases/download/v0.1/boost_R2013a_x64.tar.gz)
2013b  | GNU gcc/g++ 4.7.x | [1.49.0](http://sourceforge.net/projects/boost/files/boost/1.49.0/boost_1_49_0.tar.gz/download) | [boost_1_49_0_gcc_4_7.tar.bz2](https://github.com/lagadic/matlab_ros_bridge/releases/download/v0.1/boost_1_49_0_gcc_4_7.tar.bz2) | [boost_R2013b_x64.tar.gz](https://github.com/lagadic/matlab_ros_bridge/releases/download/v0.1/boost_R2013b_x64.tar.gz)
2014a  | GNU gcc/g++ 4.7.x | [1.49.0](http://sourceforge.net/projects/boost/files/boost/1.49.0/boost_1_49_0.tar.gz/download) | [boost_1_49_0_gcc_4_7.tar.bz2](https://github.com/lagadic/matlab_ros_bridge/releases/download/v0.1/boost_1_49_0_gcc_4_7.tar.bz2) | [boost_R2014a_x64.tar.gz](https://github.com/lagadic/matlab_ros_bridge/releases/download/v0.1/boost_R2014a_x64.tar.gz)
2015a  | GNU gcc/g++ 4.7.x | [1.49.0](http://sourceforge.net/projects/boost/files/boost/1.49.0/boost_1_49_0.tar.gz/download) | [boost_1_49_0_gcc_4_7.tar.bz2](https://github.com/lagadic/matlab_ros_bridge/releases/download/v0.1/boost_1_49_0_gcc_4_7.tar.bz2) | [boost_R2014a_x64.tar.gz](https://github.com/lagadic/matlab_ros_bridge/releases/download/v0.1/boost_R2015a_x64.tar.gz)
2015b  | GNU gcc/g++ 4.7.x | [1.49.0](http://sourceforge.net/projects/boost/files/boost/1.49.0/boost_1_49_0.tar.gz/download) | [boost_1_49_0_gcc_4_7.tar.bz2](https://github.com/lagadic/matlab_ros_bridge/releases/download/v0.1/boost_1_49_0_gcc_4_7.tar.bz2) | [boost_R2014a_x64.tar.gz](https://github.com/lagadic/matlab_ros_bridge/releases/download/v0.1/boost_R2015b_x64.tar.gz)

##Ros/Matlab combinatios that have already been tested:

Ros\Matlab  | __2012a__ | __2012b__ | __2013a__ | __2013b__ | __2014a__ | __2014b__ | __2015a__ | __2015b__ |
----------- | --------- | --------- | --------- | --------- | --------- | --------- | --------- | --------- |
__Groovy__ | never tested | never tested | never tested | never tested | never tested | never tested | never tested | never tested |
__Hydro__ | never tested | never tested | never tested | working | working | never tested | never tested | never tested |
__Indigo__ | never tested | never tested | never tested | never tested | working | working | working | working |
__Jade__ | never tested | never tested | never tested | never tested | never tested | never tested | never tested | never tested |
