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


#ifndef GENERICSUBSCRIBER_HPP_
#define GENERICSUBSCRIBER_HPP_

#include <boost/thread/mutex.hpp>

#include <semaphore.h>
//#include <sys/sem.h>
//#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>


#include <ros/node_handle.h>


#ifndef GENERATESEMAPHORENAME
#define GENERATESEMAPHORENAME

#endif

class GenericSubscriber_base {
public:
    static
    void resetSemaphore(sem_t* sem){
    	int valp;
    	sem_getvalue(sem, &valp);
    	while ( valp > 0){
    		sem_wait(sem);
    		sem_getvalue(sem, &valp);
    	}
    }

    static
    std::string generateSemaphoreName(std::string const & topicName) {
    	std::string semName(topicName);
    	if (topicName[0] == '/'){
    		semName.erase(0,1);
    	}
    	std::replace(semName.begin(), semName.end(), '/', '_');
    	semName = "slk." + semName;
    	return semName;
    }
};

template <class T_>
class GenericSubscriber : public GenericSubscriber_base {
protected:
    boost::shared_ptr<T_ const> lastMsg;
    mutable boost::mutex lastMsgLock;
    const std::string semName;
    sem_t * newMsg;

    // Subscriber
    ros::Subscriber sub;

    void msgCallback(boost::shared_ptr<T_ const> msg) {
        boost::mutex::scoped_lock lock(lastMsgLock);
        lastMsg = msg;
//        sem_post(newMsg);
    }

public:

    GenericSubscriber(ros::NodeHandle handle, const std::string& topicName, int queue_size) :
	semName(generateSemaphoreName(topicName)),
	newMsg(sem_open(semName.c_str(), O_CREAT, 0644, 0)),
	sub(handle.subscribe(topicName, queue_size, &GenericSubscriber<T_>::msgCallback, this)){
//    	std::cout << "Opening semaphore in " << __func__ << " in file /dev/shm/sem." << semName << std::endl;

    	if (newMsg == SEM_FAILED){
    		//TODO: ssSetErrorStatus(S, strerror(errno));
//    		std::cout << " returned SEM_FAILED with error: " << strerror(errno);
    	}
    }

    virtual ~GenericSubscriber() {
        boost::mutex::scoped_lock lock(lastMsgLock);
//        lastMsg = boost::shared_ptr<T_ const> (new T_);
    	sem_close(newMsg);
    	sem_unlink(semName.c_str());
    }

    boost::shared_ptr<T_ const> getLastMsg() const {
        boost::mutex::scoped_lock lock(lastMsgLock);
        return lastMsg;
    }

};



#endif /* GENERICSUBSCRIBER_HPP_ */
