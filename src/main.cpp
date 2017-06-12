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
#include "topicstats.h"

#include "rostune/SingleTopicStats.h"
#include "rostune/MultipleTopicStats.h"

#include <thread>
#include <forward_list>

#define MAX_NODES  1024
#define MAX_TOPICS 1024


std::string  nodeNames[MAX_NODES];
uint64_t  prevcputimes[MAX_NODES];
uint64_t prevwalltimes[MAX_NODES];
int num_nodes = 0;
bool first_time = true;

std::forward_list<topicstats::TopicStats> monitored_topics;
std::vector<std::string> excluded_nodes;
std::vector<std::string> excluded_topics;


void spinToWin(){
  ros::spin();
}



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
  uint64_t wtime = msg.header.stamp.sec + msg.header.stamp.nsec / 1000;
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
  ros::Publisher nodestats_pub = n.advertise<rostune::NodeLogline>( "rostune/node_stats", 1000 );
  ros::Publisher topicstats_pub = n.advertise<rostune::MultipleTopicStats>("rostune/topic_stats", 1);
  ros::Rate loop_rate( 10 );
  static const ros::TopicManagerPtr& tp = ros::TopicManager::instance();

  n.getParam("rostune/excluded_nodes", excluded_nodes);
  n.getParam("rostune/excluded_topics", excluded_topics);

  // Excluded nodes by default
  if(excluded_nodes.size() == 0){
    excluded_topics.push_back("/rosout");
    excluded_topics.push_back("/rostune");
  }

  // Excluded topics by default
  if(excluded_topics.size() == 0){
    excluded_topics.push_back("/rosout");
    excluded_topics.push_back("/rosout_agg");
    excluded_topics.push_back("/rostune/node_stats");
    excluded_topics.push_back("/rostune/topic_stats");
  }


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

    if (succ) {

      for( auto it = monitored_topics.begin(); it != monitored_topics.end(); it++ ) {	
	// Look for the topic in the list of topics *with at least two subscribers*.
	// Topics with one subscriber (myself) are treated as non-existing, to avoid
	// forcing the circulation of messages that wouldn't otherwise be circulated
	// as there are no subscribers.


	// NOTE: The for loop below looks for *it inside topiclist
	ros::master::V_TopicInfo::iterator it2;
	for( it2 = topiclist.begin();
	     (it2 != topiclist.end()) && !(*it == it2->name); ++it2 );

	if( it2 != topiclist.end() ) {
	  ROS_DEBUG( "Found %s in list of topics", it->topic_name.c_str() );
	  ROS_DEBUG( "Branch 1: %s has %d subscribers", it->topic_name.c_str(), tp->getNumSubscribers(it->topic_name) );
	}
	
	if( it2 != topiclist.end() && tp->getNumSubscribers(it->topic_name) > 1 ) {
	  // topic i is still here and has at least two subscribers (one is myself)
	  // remove from vector, no need to look here again as topicnames appear once
	  *it2 = topiclist.back();
	  topiclist.pop_back();
	}
	// If we need to make a distinction between topic not exists and no subs:
	//else if( it2 == topiclist.end() ) {
	//  // DO something about topic disappeared
	//}
	else {
	  // the topic does not exist any more or the
	  // topic exists, but has no subscribers except myself
	  it->subscriber.shutdown();
	}
      }

      std::cout << "XXXX ";
      for( ros::master::V_TopicInfo::iterator q = topiclist.begin(); q != topiclist.end(); ++q ) {
	std::cout << q->name << "(" << tp->getNumSubscribers(q->name) << ") ";
      }
	// now add what's left of the topiclist to my topics
      for( ros::master::V_TopicInfo::iterator q = topiclist.begin(); q != topiclist.end(); ++q ) {
	ROS_DEBUG( "Branch 2: %s has %d subscribers", q->name.c_str(), tp->getNumSubscribers(q->name) );
	
	// check that this topic should not be excluded
	bool exclude_this = false;
	for(unsigned i=0;i<excluded_topics.size();i++){
	  if(q->name.compare(excluded_topics[i]) == 0){
	    exclude_this = true;
	    break;
	  }
	}
	
	if( !exclude_this ) {
	  monitored_topics.emplace_front(q->name);
	  monitored_topics.front().subscriber = n.subscribe(q->name, 1, &topicstats::TopicStats::callback, &monitored_topics.front());
	  ROS_INFO( "Subscribed to %s", q->name.c_str() );
	}
      }
    }
	
    rostune::MultipleTopicStats mts;
    std::vector<rostune::SingleTopicStats> stats_data;

    for (auto it = monitored_topics.begin(); it != monitored_topics.end(); it++) {
      rostune::SingleTopicStats sts;
      sts.name = it->topic_name;
      sts.num_of_messages = it->num_of_messages;
      sts.bytes_per_second = it->bytes_per_second;
      sts.avg_bytes_per_msg = it->avg_bytes_per_msg;
      sts.avg_msgs_per_sec = it->avg_msgs_per_sec;
      sts.total_bytes = it->total_bytes;
      stats_data.push_back(sts);
    }

    mts.header.stamp = ros::Time::now();
    mts.topics = stats_data;
    topicstats_pub.publish(mts);

    if(first_time){
      std::thread spinner_thread(spinToWin);
      spinner_thread.detach();
      first_time = false;
    }

    loop_rate.sleep();
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
