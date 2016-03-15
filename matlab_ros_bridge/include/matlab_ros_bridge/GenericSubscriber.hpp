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
#include <boost/asio.hpp>
#include <boost/thread/thread.hpp>

#include <ros/node_handle.h>

template <class T_>
class GenericSubscriber {
protected:
	boost::shared_ptr<T_ const> lastMsg;
	mutable boost::mutex lastMsgLock;
	bool newMsgReceived;
	boost::condition_variable newMsgCondition;

	const std::string tName;

	// Subscriber
	ros::Subscriber sub;

	void msgCallback(boost::shared_ptr<T_ const> msg) {
		boost::mutex::scoped_lock lock(lastMsgLock);
		lastMsg = msg;
		newMsgReceived = true;
		newMsgCondition.notify_one();
	}

public:

	std::string getTopicName(){return tName;};

	void resetSem(){
		boost::mutex::scoped_lock lock(lastMsgLock);
		newMsgReceived = false;
	}

	int waitMsg(const double timeout=-1){
		boost::mutex::scoped_lock lock(lastMsgLock);
		if (timeout<0){
			while (!newMsgReceived) newMsgCondition.wait(lock);
		} else {
			boost::system_time const waitTime = boost::get_system_time() + boost::posix_time::seconds(timeout);
			while (!newMsgReceived){
				if(!newMsgCondition.timed_wait(lock, waitTime)){
					return -1;
				}
			}
		}
		return 0;
	}

	GenericSubscriber(ros::NodeHandle handle, const std::string& topicName, int queue_size) :
		newMsgReceived(false),
		sub(handle.subscribe(topicName, queue_size, &GenericSubscriber<T_>::msgCallback, this)),
		tName(topicName){

	}

	virtual ~GenericSubscriber() {
		boost::mutex::scoped_lock lock(lastMsgLock);
	}

	boost::shared_ptr<T_ const> getLastMsg() {
		boost::mutex::scoped_lock lock(lastMsgLock);
		newMsgReceived = false;
		return lastMsg;
	}

};



#endif /* GENERICSUBSCRIBER_HPP_ */
