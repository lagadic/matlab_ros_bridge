/********************************************************************
 * 
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2012, Max-Planck-Gesellschaft
 * Copyright (c) 2012-2015, Inria
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ********************************************************************/


#ifndef __ROSMATLABBRIDGEDEFINESHPP__
#define __ROSMATLABBRIDGEDEFINESHPP__

#ifndef MATLAB_MEX_FILE
#define SFUNPRINTF printf
#else
//#define SFUNPRINTF mexPrintf
#include <cstdio>
#define COMBINE1(X,Y) X##Y  // helper macro
#define COMBINE(X,Y) COMBINE1(X,Y)
#define SFUNPRINTF(...) \
    char COMBINE(str,__LINE__)[1000]; std::sprintf(COMBINE(str,__LINE__), __VA_ARGS__); std::cout << COMBINE(str,__LINE__);
#endif

// Custom ROS INIT
static void initROS(SimStruct *S)
{
    // Don't to anything before here.
    if (!ros::isInitialized()) {

        int argc = 0;
        char* argv[0];
        SFUNPRINTF("Initializing ROS!\n");

        ros::init(argc, argv, "matlab", ros::init_options::NoSigintHandler
                        | ros::init_options::AnonymousName
                        | ros::init_options::NoRosout);

        SFUNPRINTF("Done Initializing ROS!\n");

    } else {
        //SFUNPRINTF("ROS already initialized!\n");
    }
}

#endif //__ROSMATLABBRIDGEDEFINESHPP__
