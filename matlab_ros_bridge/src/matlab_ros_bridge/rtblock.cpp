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

#define S_FUNCTION_NAME  rtblock
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

//#include <string>
#include <ros/ros.h>

#include "matlab_ros_bridge/RosMatlabBrigdeDefines.hpp"

#pragma pop_macro("RT")

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

    ssSetNumSFcnParams(S, 4);  /* Number of expected parameters */
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        /* Return if number of expected != number of actual parameters */
        return;
    }

    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);

    if (!ssSetNumInputPorts(S, 0)) return;
    //ssSetInputPortWidth(S, 0, 1);
    //ssSetInputPortRequiredContiguous(S, 0, true); /*direct input signal access*/
    /*
     * Set direct feedthrough flag (1=yes, 0=no).
     * A port has direct feedthrough if the input is used in either
     * the mdlOutputs or mdlGetTimeOfNextVarHit functions.
     * See matlabroot/simulink/src/sfuntmpl_directfeed.txt.
     */
    //ssSetInputPortDirectFeedThrough(S, 0, 1);

    if (!ssSetNumOutputPorts(S, 2)) return;
    ssSetOutputPortWidth(S, 0, 1); // port 0 is 1 value (rtTimeDelay)
    ssSetOutputPortWidth(S, 1, 1); // port 1 is 1 value (currentRosTime)


    ssSetNumSampleTimes(S, 1);
    ssSetNumRWork(S, 2); // Tsim, simulationTime, showEveryNumSeconds
    ssSetNumIWork(S, 1); // CycleCounter
    ssSetNumPWork(S, 2); // startTime,lastExecTime
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
static void mdlInitializeSampleTimes(SimStruct *S)
{
    real_T Tsim = mxGetScalar(ssGetSFcnParam(S, 0));
    ssSetSampleTime(S, 0, Tsim);                      //CONTINUOUS_SAMPLE_TIME);
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
  static void mdlInitializeConditions(SimStruct *S)
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

static void mdlStart(SimStruct *S)
{  
    // Notify
    SFUNPRINTF("Starting Instance of %s.\n", TOSTRING(S_FUNCTION_NAME));
    // init ROS if not yet done.
    initROS(S);
//    ros::NodeHandle nodeHandle(ros::this_node::getName());
    ros::Time::init();

    void** vecPWork = ssGetPWork(S);
    ros::Time* firstExecTime = new ros::Time(ros::Time::now());
    vecPWork[0] = firstExecTime;

    ros::Time* lastExecTime = new ros::Time(*firstExecTime);
    vecPWork[1] = lastExecTime;

    real_T* vecRWork = ssGetRWork(S);
    vecRWork[0] = mxGetScalar(ssGetSFcnParam(S, 0)); // Tsim
    vecRWork[1] = 0.0; // simulationTime 

    int_T* vecIWork = ssGetIWork(S);
    vecIWork[0] = 1; // initialize step counter

  }
#endif /*  MDL_START */



/* Function: mdlOutputs =======================================================
 * Abstract:
 *    In this function, you compute the outputs of your S-function
 *    block.
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{   
    real_T* vecRWork = ssGetRWork(S);
    void** vecPWork = ssGetPWork(S);
    int_T* vecIWork = ssGetIWork(S);
    ros::Time* firstExecTime = (ros::Time*)vecPWork[0];
    ros::Time* lastExecTime = (ros::Time*)vecPWork[1];
    int_T showEveryNumCycles = (int_T)mxGetScalar(ssGetSFcnParam(S, 2));
    real_T slowDownFactor = (real_T)mxGetScalar(ssGetSFcnParam(S,3));
    

    //simulationTime += Tsim;
    vecRWork[1] += slowDownFactor * vecRWork[0];
    
    ros::Time currentTime = ros::Time::now();
    if (*firstExecTime > currentTime){
    	ssWarning(S, "Resetting all timers due to negative elapsed time! \
    			This should not happen in normal running conditions unless \
    			the simulation has just started or /use_sim_time has been changed.");
    	*firstExecTime = currentTime;
    	vecRWork[1] = 0.0;
    }
    ros::Time timeElapsed((currentTime - *firstExecTime).toSec());
    *lastExecTime = currentTime;

    ros::Duration sleepTime = ros::Time(vecRWork[1]) - timeElapsed;

    if (sleepTime.toSec() > 0.0) {
        sleepTime.sleep();
    }
    else if (sleepTime.toSec() < -1.0 * mxGetScalar(ssGetSFcnParam(S, 1))) { // show error only if the delay time is above a certain threshold (2nd parameter)
	    if (showEveryNumCycles > 0) { // never show error when (3rd parameter) <= 0
            	if (vecIWork[0] >= showEveryNumCycles) { // show only every (3rd parameter) cycles 
		
            SFUNPRINTF("Beware! rtTimer is negative! Time: %f\n", sleepTime.toSec());
			vecIWork[0] = 1;
	    	}
            	else {
	    	    ++vecIWork[0];
            	}
	    }
    }

    // output of sleepTime
    real_T *output = (real_T*)ssGetOutputPortSignal(S,0);
    output[0] = sleepTime.toSec();
    // output of current ros time
    output[1] = ros::Time::now().toSec();
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
    real_T* vecRWork = ssGetRWork(S);
    void** vecPWork = ssGetPWork(S);

    ros::Time* firstExecTime = (ros::Time*)vecPWork[0];
    ros::Time* lastExecTime = (ros::Time*)vecPWork[1];

    SFUNPRINTF("Simulation Time: %f\n", vecRWork[1]);
    SFUNPRINTF("Real Time: %f\n", (*lastExecTime-*firstExecTime).toSec());
    
    delete lastExecTime;
    
    SFUNPRINTF("Terminating Instance of %s.\n", TOSTRING(S_FUNCTION_NAME));
}


/*======================================================*
 * See sfuntmpl_doc.c for the optional S-function methods *
 *======================================================*/

/*=============================*
 * Required S-function trailer *
 *=============================*/

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
