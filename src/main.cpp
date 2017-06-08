/*
 * BSD 3-Clause License
 * Copyright (c) 2017, NCSR "Demokritos"
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the conditions at the
 * bottom of this file are met.
 */


#include "ros/ros.h"
#include "ros/topic_manager.h"
#include "rostune/NodeLogline.h"
#include "rostune/TopicLogline.h"

#include <algorithm>
#include <sstream>
#include "nodestats.h"

#define MAX_NODES  1024
#define MAX_TOPICS 1024


std::string  nodeNames[MAX_NODES];
uint64_t  prevcputimes[MAX_NODES];
uint64_t prevwalltimes[MAX_NODES];
int num_nodes = 0;

std::string  topicNames[MAX_TOPICS];
int num_topics = 0;



void mknodemsg( rostune::NodeLogline& msg, const std::string name, const int i )
{
  uint64_t cputime, all_mem, resident_mem;

  msg.node_name = name;
  int pid = nodestats::getPid( name );
  nodestats::cpuload( pid, cputime, all_mem, resident_mem );
  msg.header.stamp = ros::Time::now();
  msg.cputime = cputime;
  msg.all_memory = all_mem;
  msg.resident_memory = resident_mem;
  msg.diffcputime = cputime - prevcputimes[i];
  uint64_t wtime = msg.header.stamp.sec + msg.header.stamp.nsec/1000;
  msg.diffwalltime = wtime - prevwalltimes[i];
  prevcputimes[i] = cputime;
  prevwalltimes[i] = wtime;
}


void mktopicmsg( rostune::TopicLogline& msg, const std::string name, const int i )
{
  msg.topic_name = name;
  ROS_INFO( "XXX: %s", name.c_str() );
}

int main( int argc, char **argv )
{
  ros::init( argc, argv, "rostune" );
  ros::NodeHandle n;
  ros::Publisher nodestats_pub = n.advertise<rostune::NodeLogline>( "node_stats", 1000 );
  ros::Publisher topicstats_pub = n.advertise<rostune::TopicLogline>( "topic_stats", 1000 );
  ros::Rate loop_rate( 10 );
  static const ros::TopicManagerPtr& tp = ros::TopicManager::instance();

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;

  while( ros::ok() )
  {
    /* Get CPU and memory usage stats about all nodes */
      
    ros::V_string nodelist;
    bool succ = ros::master::getNodes( nodelist );
    if( !succ ) { loop_rate.sleep(); break; }
    
    for( int i=0; i<num_nodes; ++i ) {
      // Look for the node in the list of current nodes
      std::vector<std::string>::iterator it = std::find( nodelist.begin(), nodelist.end(), nodeNames[i] );
      if( it != nodelist.end() ) {
	// node i is still here, fetch cpudata and build a message
	rostune::NodeLogline msg;
	mknodemsg( msg, *it, i );
	nodestats_pub.publish( msg );
	
	// remove from vector, no need to look here again as nodenames appear once
	*it = nodelist.back();
	nodelist.pop_back();
      }
      else {
	// node i is gone
	if( i < num_nodes ) {
	  // overwrite with the last node, unless this is the last node
	  nodeNames[i] = nodeNames[num_nodes];
	  prevcputimes[i] = prevcputimes[num_nodes];
	  prevwalltimes[i] = prevwalltimes[num_nodes];
	  --i; // make the loop go over the new i
	}
	// either way, the array is now shorter
	--num_nodes;
      }
    }

    // now add what's left of the nodelist to my nodes
    for( ros::V_string::const_iterator q = nodelist.begin(); q != nodelist.end(); ++q ) {
      rostune::NodeLogline msg;
      if( num_nodes < MAX_NODES ) {
	++num_nodes;
	nodeNames[num_nodes] = *q;
	prevcputimes[num_nodes] = 0;
	prevwalltimes[num_nodes] = 0;
	mknodemsg( msg, *q, num_nodes );
	nodestats_pub.publish( msg );
      }
    }


    /* Get bandwidth and frequency stats about all topics */

    ros::master::V_TopicInfo topiclist;
    succ = ros::master::getTopics( topiclist );
    if( !succ ) { loop_rate.sleep(); break; }

    /*
    for(  
      const ros::master::TopicInfo& info = *it;
      std::cout << "Topic : " << it - topiclist.begin() << ": " << info.name << " -> " << info.datatype << std::endl;
      }*/

    //TODO: if debug
    std::stringstream str;
    for( int k = 0; k < num_topics; ++k ) { str << topicNames[k] << " "; }
    ROS_DEBUG( "Active topics: %s", str.str().c_str() );

    for( int i = 0; i < num_topics; ++i ) {
      // Look for the topic in the list of topics *with at least one subscriber*.
      // Topics without subscribers are treated as non-existing, to avoid forcing
      // the circulation of messages that wouldn't otherwise be circulated
      // as there are no subscribers.

      // NOTE: The for loop below actually means
      // ros::master::V_TopicInfo::iterator it = std::find( topiclist.begin(), topiclist.end(), topicNames[i] );
      // but cannot use std::find() because TopicInfo does not implement operator==
      ros::master::V_TopicInfo::iterator it;
      for( it = topiclist.begin();
	   (it != topiclist.end()) && (topicNames[i] != it->name); ++it);

      if( it != topiclist.end() ) {
	ROS_DEBUG( "Found %s in list of topics", it->name.c_str() );
	ROS_DEBUG( "Branch 1: %s has %d subscribers", it->name.c_str(), tp->getNumSubscribers(it->name) );
      }
      if( it != topiclist.end() && tp->getNumSubscribers(it->name) > 0 ) {
	// topic i is still here and has at least one subscriber
	rostune::TopicLogline msg;
	mktopicmsg( msg, it->name, i );
	topicstats_pub.publish( msg );
	
	// remove from vector, no need to look here again as topicnames appear once
	*it = topiclist.back();
	topiclist.pop_back();
      }
      else {
	// topic i is gone or has no subscribers
	if( i < num_topics ) {
	  // overwrite with the last topic, unless this is the last topic
	  topicNames[i] = topicNames[num_nodes];
	  // TODO: copy other stats
	  //prevcputimes[i] = prevcputimes[num_nodes];
	  //prevwalltimes[i] = prevwalltimes[num_nodes];
	  --i; // make the loop go over the new i
	}
	// either way, the array is now shorter
	--num_topics;
      }
    }
    
    // now add what's left of the topiclist to my topics
    for( ros::master::V_TopicInfo::iterator q = topiclist.begin(); q != topiclist.end(); ++q ) {
      ROS_DEBUG( "Branch 2: %s has %d subscribers", q->name.c_str(), tp->getNumSubscribers(q->name) );
      //const ros::master::TopicInfo& info = *q;
      rostune::TopicLogline msg;
      if( num_topics < MAX_TOPICS ) {
	++num_topics;
	topicNames[num_topics] = q->name;
	// FIX prevcputimes[num_topics] = 0;
	// FIX prevwalltimes[num_topics] = 0;
	mktopicmsg( msg, q->name, num_nodes );
	topicstats_pub.publish( msg );
      }
    }
    
    ros::spinOnce();
    loop_rate.sleep();
    ++count;

  }
  return 0;
}






/*
BSD 3-Clause License
Copyright (c) 2017, NCSR "Demokritos"
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:

 * Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in
   the documentation and/or other materials provided with the
   distribution.

 * Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived
   from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
