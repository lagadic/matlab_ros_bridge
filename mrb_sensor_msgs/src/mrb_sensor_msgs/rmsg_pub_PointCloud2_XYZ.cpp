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


/*
 * You must specify the S_FUNCTION_NAME as the name of your S-function
 * (i.e. replace sfuntmpl_basic with the name of your S-function).
 */

#define S_FUNCTION_NAME  rmsg_pub_PointCloud2_XYZ
#define S_FUNCTION_LEVEL 2

#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)


/*
 * Need to include simstruc.h for the definition of the SimStruct and
 * its associated macro definitions.
 */
#include "simstruc.h"

#pragma push_macro("RT")
#undef RT

#include <ros/ros.h>

// Generic Publisher
#include <matlab_ros_bridge/GenericPublisher.hpp>

// Message
#include <sensor_msgs/PointCloud2.h>

#pragma pop_macro("RT")

#include <matlab_ros_bridge/RosMatlabBrigdeDefines.hpp>

#define MDL_CHECK_PARAMETERS
#if defined(MDL_CHECK_PARAMETERS) && defined(MATLAB_MEX_FILE)
/* Function: mdlCheckParameters =============================================
 * Abstract:
 *    Validate our parameters to verify:
 *     o The numerator must be of a lower order than the denominator.
 *     o The sample time must be a real positive nonzero value.
 */
static void
mdlCheckParameters(SimStruct *S)
{
	//  SFUNPRINTF("Calling mdlCheckParameters");
    
  // Tsim
	if (mxIsEmpty( ssGetSFcnParam(S,0)) ||
			mxIsSparse( ssGetSFcnParam(S,0)) ||
			mxIsComplex( ssGetSFcnParam(S,0)) ||
			mxIsLogical( ssGetSFcnParam(S,0)) ||
			!mxIsNumeric( ssGetSFcnParam(S,0)) ||
			!mxIsDouble( ssGetSFcnParam(S,0)) ||
			mxGetNumberOfElements(ssGetSFcnParam(S,0)) != 1)
  {
		ssSetErrorStatus(S,"Simulation time must be a single double Value");
		return;
	}

	//Topic
	if (!mxIsChar( ssGetSFcnParam(S,1)) )
  {
		ssSetErrorStatus(S,"Topic must be char array (string)");
		return;
	}

	// Frame_id
	if (!mxIsChar( ssGetSFcnParam(S,2)) )
  {
		ssSetErrorStatus(S,"Frame_id must be char array (string)");
		return;
	}

  // Width
  if (mxIsEmpty( ssGetSFcnParam(S,3)) ||
          mxIsSparse( ssGetSFcnParam(S,3)) ||
          mxIsComplex( ssGetSFcnParam(S,3)) ||
          mxIsLogical( ssGetSFcnParam(S,3)) ||
          mxIsInt32( ssGetSFcnParam(S,3)) )
  {
      ssSetErrorStatus(S,"Width must be a Uint32.");
      return;
  }

  // Height
  if (mxIsEmpty( ssGetSFcnParam(S,4)) ||
          mxIsSparse( ssGetSFcnParam(S,4)) ||
          mxIsComplex( ssGetSFcnParam(S,4)) ||
          mxIsLogical( ssGetSFcnParam(S,4)) ||
          mxIsInt32( ssGetSFcnParam(S,4)) )
  {
      ssSetErrorStatus(S,"Height must be a Uint32.");
      return;
  }
}
#endif /* MDL_CHECK_PARAMETERS */



/* Error handling
 * --------------
 *
 * You should use the following technique to report errors encountered within
 * an S-function:
 *
 *       ssSetErrorStatus(S,"Error encountered due to ...");
 *       return;
 *
 * Note that the 2nd argument to ssSetErrorStatus must be persistent memory.
 * It cannot be a local variable. For example the following will cause
 * unpredictable errors:
 *
 *      mdlOutputs()
 *      {
 *         char msg[256];         {ILLEGAL: to fix use "static char msg[256];"}
 *         sprintf(msg,"Error due to %s", string);
 *         ssSetErrorStatus(S,msg);
 *         return;
 *      }
 *
 * See matlabroot/simulink/src/sfuntmpl_doc.c for more details.
 */

/*====================*
 * S-function methods *
 *====================*/


//double Tsim;

/* Function: mdlInitializeSizes ===============================================
 * Abstract:
 *    The sizes information is used by Simulink to determine the S-function
 *    block's characteristics (number of inputs, outputs, states, etc.).
 */
static void mdlInitializeSizes(SimStruct *S)
{
  /* See sfuntmpl_doc.c for more details on the macros below */

  ssSetNumSFcnParams(S, 5);  /* Number of expected parameters */
#if defined(MATLAB_MEX_FILE)
  if (ssGetNumSFcnParams(S) == ssGetSFcnParamsCount(S))
  {
    mdlCheckParameters(S);
    if (ssGetErrorStatus(S) != NULL)
    {
      return;
    }
  }
  else
  {
    return; /* Parameter mismatch will be reported by Simulink. */
  }
#endif

  const uint32_t width  = mxGetScalar(ssGetSFcnParam(S,3));
  const uint32_t height = mxGetScalar(ssGetSFcnParam(S,4));

  ssSetNumContStates(S, 0);
  ssSetNumDiscStates(S, 0);

  if (!ssSetNumInputPorts(S, 1))
    return;

  ssSetInputPortMatrixDimensions(S, 0, 1, 4*height*width); // points


  for (int_T i = 0; i < ssGetNumInputPorts(S); ++i)
  {
    /*direct input signal access*/
    ssSetInputPortRequiredContiguous(S, i, true); 
    /*
     * Set direct feedthrough flag (1=yes, 0=no).
     * A port has direct feedthrough if the input is used in either
     * the mdlOutputs or mdlGetTimeOfNextVarHit functions.
     * See matlabroot/simulink/src/sfuntmpl_directfeed.txt.
     */
    ssSetInputPortDirectFeedThrough(S, i, 1);
  }

  if (!ssSetNumOutputPorts(S, 0))
    return;

  ssSetNumSampleTimes(S, 1);
  ssSetNumRWork(S, 0);
  ssSetNumIWork(S, 2); // width, height
  ssSetNumPWork(S, 2); // 1 publisher, frame_id
  ssSetNumModes(S, 0);
  ssSetNumNonsampledZCs(S, 0);

  /* Specify the sim state compliance to be same as a built-in block */
  ssSetSimStateCompliance(S, USE_DEFAULT_SIM_STATE);

  ssSetOptions(S, 0);
}



/* Function: mdlInitializeSampleTimes =========================================
 * Abstract:
 *    This function is used to specify the sample time(s) for your
 *    S-function. You must register the same number of sample times as
 *    specified in ssSetNumSampleTimes.
 */

static void
mdlInitializeSampleTimes(SimStruct *S)
{
    real_T Tsim = mxGetScalar(ssGetSFcnParam(S, 0));
    ssSetSampleTime(S, 0, Tsim);                      //DISCRETE_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);

}



#define MDL_INITIALIZE_CONDITIONS   /* Change to #undef to remove function */
#if defined(MDL_INITIALIZE_CONDITIONS)
/* Function: mdlInitializeConditions ========================================
 * Abstract:
 *    In this function, you should initialize the continuous and discrete
 *    states for your S-function block.  The initial states are placed
 *    in the state vector, ssGetContStates(S) or ssGetRealDiscStates(S).
 *    You can also perform any other initialization activities that your
 *    S-function may require. Note, this routine will be called at the
 *    start of simulation and if it is present in an enabled subsystem
 *    configured to reset states, it will be call when the enabled subsystem
 *    restarts execution to reset the states.
 */
static void
mdlInitializeConditions(SimStruct *S)
{
}
#endif /* MDL_INITIALIZE_CONDITIONS */

#define MDL_START  /* Change to #undef to remove function */
#if defined(MDL_START) 
/* Function: mdlStart =======================================================
 * Abstract:
 *    This function is called once at start of model execution. If you
 *    have states that should be initialized once, this is the place
 *    to do it.
 */

static void
mdlStart(SimStruct *S)
{   
  SFUNPRINTF("Creating Instance of %s.\n", TOSTRING(S_FUNCTION_NAME));
  // init ROS if not yet done.
  initROS(S);

  void** vecPWork = ssGetPWork(S);

  //Two params will be stored
  //Width and Height
  ssGetIWork(S)[0] = (int)mxGetScalar(ssGetSFcnParam(S, 3));
  ssGetIWork(S)[1] = (int)mxGetScalar(ssGetSFcnParam(S, 4));

  ros::NodeHandle nodeHandle(ros::this_node::getName());

  // get Topic Strings
  size_t buflen = mxGetN((ssGetSFcnParam(S, 1)))*sizeof(mxChar)+1;
  char* topic = (char*)mxMalloc(buflen);
  mxGetString((ssGetSFcnParam(S, 1)), topic, buflen);

  //get frame_id
  buflen = mxGetN((ssGetSFcnParam(S, 2)))*sizeof(mxChar)+1;
  char* frame_id = (char*)mxMalloc(buflen);
  mxGetString((ssGetSFcnParam(S, 2)), frame_id, buflen);
  vecPWork[0] = new std::string(frame_id);

  //SFUNPRINTF("The string being passed as a Paramater is - %s\n ", topic);
  GenericPublisher<sensor_msgs::PointCloud2>* pub = new GenericPublisher<sensor_msgs::PointCloud2>(nodeHandle, topic, 10);
  vecPWork[1] = pub;

  // free char array
  mxFree(topic);
}
#endif /*  MDL_START */



/* Function: mdlOutputs =======================================================
 * Abstract:
 *    In this function, you compute the outputs of your S-function
 *    block.
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{   
  // get Objects
  void** vecPWork = ssGetPWork(S);

  const int width = ssGetIWork(S)[0];
  const int height = ssGetIWork(S)[1];

  const std::string * frame_id = (const std::string *) vecPWork[0];
  GenericPublisher<sensor_msgs::PointCloud2>* pub
    = (GenericPublisher<sensor_msgs::PointCloud2>*)vecPWork[1];

  // get Pointers
  // accessing inputs
  const real_T * points = (const real_T *) ssGetInputPortSignal(S,0);
  
  sensor_msgs::PointCloud2 msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = *frame_id;
  msg.height = height;
  msg.width = width;
  
  msg.fields.resize(3);
  msg.fields[0].name.assign("x");
  msg.fields[0].offset = 0;
  msg.fields[0].datatype = 7;
  msg.fields[0].count = 1;

  msg.fields[1].name.assign("y");
  msg.fields[1].offset = 4;
  msg.fields[1].datatype = 7;
  msg.fields[1].count = 1;

  msg.fields[2].name.assign("z");
  msg.fields[2].offset = 8;
  msg.fields[2].datatype = 7;
  msg.fields[2].count = 1;

  msg.is_bigendian = false;
  msg.point_step = 16;
  msg.row_step = msg.point_step * msg.width;
  if(height == 1)
    msg.is_dense = true;
  else
    msg.is_dense = false;
   
  msg.data.resize(height*width*msg.point_step);

  float temp[3];
  for(int i=0; i<height*width; ++i)
  {
    temp[0] = points[4*i+0];
    temp[1] = points[4*i+1];
    temp[2] = points[4*i+2];
    memcpy(&msg.data[i * msg.point_step + msg.fields[0].offset], &temp, sizeof(float)*3);
  }

  pub->publish(msg);
}



#define MDL_UPDATE  /* Change to #undef to remove function */
#if defined(MDL_UPDATE)
  /* Function: mdlUpdate ======================================================
   * Abstract:
   *    This function is called once for every major integration time step.
   *    Discrete states are typically updated here, but this function is useful
   *    for performing any tasks that should only take place once per
   *    integration step.
   */
  static void mdlUpdate(SimStruct *S, int_T tid)
  {
  }
#endif /* MDL_UPDATE */



#define MDL_DERIVATIVES  /* Change to #undef to remove function */
#if defined(MDL_DERIVATIVES)
  /* Function: mdlDerivatives =================================================
   * Abstract:
   *    In this function, you compute the S-function block's derivatives.
   *    The derivatives are placed in the derivative vector, ssGetdX(S).
   */
  static void mdlDerivatives(SimStruct *S)
  {
  }
#endif /* MDL_DERIVATIVES */



/* Function: mdlTerminate =====================================================
 * Abstract:
 *    In this function, you should perform any actions that are necessary
 *    at the termination of a simulation.  For example, if memory was
 *    allocated in mdlStart, this is the place to free it.
 */
static void mdlTerminate(SimStruct *S)
{
    // get Objects
    void** vecPWork = ssGetPWork(S);

    GenericPublisher<sensor_msgs::PointCloud2>* pub = (GenericPublisher<sensor_msgs::PointCloud2>*)vecPWork[1];
    std::string * frame_id = (std::string*) vecPWork[0];

    // cleanup
    delete pub;
    delete frame_id;


    SFUNPRINTF("Terminating Instance of %s.\n", TOSTRING(S_FUNCTION_NAME));
}



/*=============================*
 * Required S-function trailer *
 *=============================*/

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
