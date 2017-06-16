/*
 * This file is part of rostune
 * https://github.com/roboskel/rostune
 *
 * BSD 3-Clause License
 * Copyright (c) 2017, NCSR "Demokritos"
 * All rights reserved.
 *
 * Authors:
 * Georgios Stavrinos, https://github.com/gstavrinos
 * Stasinos Konstantopoulos, https://github.com/stasinos
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the conditions at the
 * bottom of this file are met.
 */

#ifndef __USE_POSIX
#define __USE_POSIX
#endif

#include <forward_list>
#include <algorithm>
#include <cstdlib>
#include <sstream>
#include <thread>
#include <limits>

#include "ros/topic_manager.h"
#include "ros/ros.h"

#include "rostune/MultipleTopicStats.h"
#include "rostune/MultipleNodeStats.h"
#include "rostune/SingleTopicStats.h"
#include "rostune/SingleNodeStats.h"
#include "nodestats.h"
#include "topicstats.h"


bool first_time = true;
std::string my_hostname;

std::forward_list<nodestats::NodeStats> monitored_nodes;
std::forward_list<topicstats::TopicStats> monitored_topics;
std::vector<std::string> excluded_nodes;
std::vector<std::string> excluded_topics;

void spinToWin(){
  ros::spin();
}


unsigned getNumberOfSubscribers(std::string topic_name){
  unsigned nos = 0;
  // The snippet below returns the topics are are being subscribed, along with their subscribers.
  XmlRpc::XmlRpcValue args, result, payload;
  args[0] = "rostune";
  if( ros::master::execute( "getSystemState", args, result, payload, true ) ) {
    XmlRpc::XmlRpcValue & subscribers = payload[1];
    for(int i=0; i<subscribers.size(); i++) {
      XmlRpc::XmlRpcValue & topic = subscribers[i];
      if(topic_name.compare(topic[0]) == 0){
        nos = topic[1].size();
        break;
      }
    }
  }
  return nos;
}

std::vector<std::string> getTopicPublishers(std::string topic_name){
  std::vector<std::string> ret;
  // The snippet below returns all the published topics along with their publishers.
  XmlRpc::XmlRpcValue args, result, payload;
  args[0] = "rostune";
  if( ros::master::execute( "getSystemState", args, result, payload, true ) ) {
    XmlRpc::XmlRpcValue & publishers = payload[0];
    for(unsigned i=0; i<publishers.size(); i++) {
      XmlRpc::XmlRpcValue & topic = publishers[i];
      if(topic_name.compare(topic[0]) == 0){
        for(unsigned j=0;j<topic[1].size();j++){
          ret.push_back(topic[1][j]);
        }
      }
    }
  }
  return ret;
}

std::string getNodeHostname(std::string node_name){
  XmlRpc::XmlRpcValue args, result, payload;
  args[0] = "rostune";
  args[1] = node_name;
  if( ros::master::execute( "lookupNode", args, result, payload, true ) ) {
    return result[2];
  }
  return "";
}

bool myMachinePublishes(std::string topic_name){
  std::vector<std::string> publishers = getTopicPublishers(topic_name);
  for(unsigned i=0;i<publishers.size();i++){
    std::string host = getNodeHostname(publishers[i]);
    host = host.substr(host.find("://")+3);
    host = host.substr(0, host.find(":"));
    if (host.compare(my_hostname) == 0){
      return true;
    }
  }
  return false;
}

std::string prettyPrinter( ros::V_string list ) {
  std::stringstream retv;
  for( auto it = list.begin(); it != list.end(); ++it ) { retv << *it << " "; }
  return retv.str();
}

std::string prettyPrinter( std::forward_list<nodestats::NodeStats> list ) {
  std::stringstream retv;
  for( auto it = list.begin(); it != list.end(); ++it ) { retv << it->name << " "; }
  return retv.str();
}

std::string prettyPrinter( std::forward_list<topicstats::TopicStats> list ) {
  std::stringstream retv;
  for( auto it = list.begin(); it != list.end(); ++it ) { retv << it->topic_name << " "; }
  return retv.str();
}


void mknodemsg( rostune::SingleNodeStats& msg, const std::string name, nodestats::NodeStats& nodeStats )
{
  uint64_t cputime, all_mem, resident_mem;

  msg.name = name;
  int pid = nodestats::getPid( name );
  nodestats::cpuload( pid, cputime, all_mem, resident_mem );
  msg.cputime = cputime;
  msg.all_memory = all_mem / 1024;
  msg.resident_memory = resident_mem / 1024;
  msg.diffcputime = cputime - nodeStats.prevcputimes;
  ros::Time t = ros::Time::now();
  uint64_t wtime = t.sec * 1000 + t.nsec / 1000000;
  msg.diffwalltime = wtime - nodeStats.prevwalltimes;
  nodeStats.prevcputimes = cputime;
  nodeStats.prevwalltimes = wtime;
}


int main( int argc, char **argv )
{
  // TODO for version 2:
  // Check if rostune is already running on this machine!
  std::string my_name = "rostune";
  ros::init( argc, argv, my_name, ros::init_options::InitOption::AnonymousName );
  my_name = "/" + my_name;
  ros::NodeHandle n("~");
  ros::Publisher nodestats_pub = n.advertise<rostune::MultipleNodeStats>( "/rostune/node_stats", 1 );
  ros::Publisher topicstats_pub = n.advertise<rostune::MultipleTopicStats>("/rostune/topic_stats", 1);
  ros::Rate loop_rate( 10 );
  static const ros::TopicManagerPtr& tp = ros::TopicManager::instance();

  n.getParam("excluded_nodes", excluded_nodes);
  n.getParam("excluded_topics", excluded_topics);

  char hostname[HOST_NAME_MAX];
  gethostname(hostname, HOST_NAME_MAX);
  if (hostname != NULL){
    my_hostname = hostname;
  }

  // Excluded nodes by default
  if(excluded_nodes.size() == 0){
    excluded_topics.push_back("/rosout");
  }

  // Excluded topics by default
  if(excluded_topics.size() == 0){
    excluded_topics.push_back("/rosout");
    excluded_topics.push_back("/rosout_agg");
  }
  excluded_topics.push_back("/rostune/topic_stats");
  excluded_topics.push_back("/rostune/node_stats");

  while( ros::ok() )
  {
    // Get CPU and memory usage stats about all nodes
    ros::V_string nodelist;
    bool succ = ros::master::getNodes( nodelist );

    if( succ ) {

      rostune::MultipleNodeStats mns;
      mns.hostname = my_hostname;

      for( auto nodeIt = monitored_nodes.begin(); nodeIt != monitored_nodes.end(); ) {

        // Look for the node in the list of current nodes
        std::vector<std::string>::iterator it = std::find( nodelist.begin(), nodelist.end(), nodeIt->name );

        if( it != nodelist.end() ) {

          // node nodeIt still here, fetch cpudata and build a message
          rostune::SingleNodeStats sns;
          mknodemsg( sns, *it, *nodeIt );
          mns.nodes.push_back(sns);

          // remove from vector, no need to look here again as nodenames appear once
          *it = nodelist.back();
          nodelist.pop_back();

          // move on to the next monitored_nodes item
          ++nodeIt;
        }
        else {
          // node nodeIt has disappeared
          if( *nodeIt == monitored_nodes.front() ) {
            // iterIt is the front element. Just pop it, but first advance the iterator.
            ROS_DEBUG( "Purging from the front of the list node %s", nodeIt->name.c_str() );
            ++nodeIt;
            monitored_nodes.pop_front();
            ROS_DEBUG( "List after purge: %s",
                 prettyPrinter(monitored_nodes).c_str() );
          }
          else {
            // overwrite with the front node, and remove front
            ROS_DEBUG( "Purging from the middle of the list node %s", nodeIt->name.c_str() );
            nodeIt->copy( monitored_nodes.front() );
            monitored_nodes.pop_front();
            // work on this node, without for-looping
            // (going through for-loop would skip this element)
            ROS_DEBUG( "List after purge: %s",
                 prettyPrinter(monitored_nodes).c_str() );
          }
        }
      } // end for all monitored_nodes

      ROS_DEBUG( "List of current nodes after comparison with my list: %s",
        prettyPrinter(monitored_nodes).c_str() );

      // now add what's left of the nodelist to my monitored_nodes
      for( ros::V_string::const_iterator q = nodelist.begin(); q != nodelist.end(); ++q ) {
        // check if this node should be excluded
        bool exclude_this = false;
        for(unsigned i=0;i<excluded_nodes.size();i++){
          // Apart from the excluded_nodes, 
          // also exclude all the nodes that their name starts with /rostune
          // TODO add this on the final README, because it might cause conflicts with future packages!
          if(q->compare(excluded_nodes[i]) == 0 || q->compare(0, my_name.size(), my_name) == 0){
            exclude_this = true;
            break;
          }
        }
        if(!exclude_this){
          monitored_nodes.emplace_front( *q );
          rostune::SingleNodeStats sns;
          mknodemsg( sns, *q, monitored_nodes.front() );
          mns.nodes.push_back(sns);
        }
      }

      mns.header.stamp = ros::Time::now();
      mns.nodes = mns.nodes;

      for (auto it = mns.nodes.begin(); it != mns.nodes.end(); it++){
        mns.total_cputime += it->cputime;
        mns.total_diffcputime += it->diffcputime;
        mns.total_diffwalltime += it->diffwalltime;
        mns.total_all_memory += it->all_memory;
        mns.total_resident_memory += it->resident_memory;
      }

      nodestats_pub.publish(mns);
    }

    // Get bandwidth and frequency stats about all topics
    ros::master::V_TopicInfo topiclist;
    succ = ros::master::getTopics( topiclist );

    if (succ) {

      for( auto it = monitored_topics.begin(); it != monitored_topics.end(); it++ ) {

        // NOTE: The for loop below looks for *it inside topiclist
        bool found = false;
        ros::master::V_TopicInfo::iterator f_index = topiclist.begin();
        ros::master::V_TopicInfo::iterator it2;
        for( it2 = topiclist.begin(); it2 != topiclist.end(); ++it2 ){
          if(*it == it2->name){
            found = true;
            f_index = it2;
            break;
          }
        }

        if( found ) {
          ROS_DEBUG( "Found %s in list of topics", it->topic_name.c_str() );
          ROS_DEBUG( "Branch 1: %s has %d subscribers", it->topic_name.c_str(), getNumberOfSubscribers(it->topic_name) );
        }

        if( found && getNumberOfSubscribers(it->topic_name) > 1 ) {
          // topic i is still here and has at least one subscriber (myself is excluded)
          // remove from vector, no need to look here again as topicnames appear once
          topiclist.erase(f_index);
          //*it2 = topiclist.back();
          //topiclist.pop_back();
        }
        // If we need to make a distinction between topic not exists and no subs:
        //else if( it2 == topiclist.end() ) {
        //  // DO something about topic disappeared
        //}
        else if(it->subscribed){
          // the topic does not exist any more or the
          it->subscriber.shutdown();
          it->subscribed = false;
          ROS_INFO( "Unsubscribed from %s", it->topic_name.c_str() );
        }
      }
      // now add what's left of the topiclist to my topics
      for( ros::master::V_TopicInfo::iterator q = topiclist.begin(); q != topiclist.end(); ++q ) {
        ROS_DEBUG( "Branch 2: %s has %d subscribers", q->name.c_str(), getNumberOfSubscribers(q->name) );

        // check if this topic should be excluded
        bool exclude_this = false;
        for(unsigned i=0;i<excluded_topics.size();i++){
          if(q->name.compare(excluded_topics[i]) == 0){
            exclude_this = true;
            break;
          }
        }

        if( !exclude_this ) {
          // TODO think of a strategy when a topic is being published from my machine and another one!
          if(myMachinePublishes(q->name) && getNumberOfSubscribers(q->name) >= 1){
            auto it = monitored_topics.begin();
            for(; it != monitored_topics.end(); it++ ) {
              if(it->topic_name == q->name){
                break;
              }
            }
            if(it == monitored_topics.end()){
              monitored_topics.emplace_front(q->name);
              monitored_topics.front().subscriber = n.subscribe(q->name, 1, &topicstats::TopicStats::callback, &monitored_topics.front());
              monitored_topics.front().subscribed = true;
            }
            else{
              it->subscriber = n.subscribe(q->name, 1, &topicstats::TopicStats::callback, &*it);
              it->subscribed = true;
            }
            ROS_INFO( "Subscribed to %s", q->name.c_str() );
          }
        }
      }

      rostune::MultipleTopicStats mts;
      mts.hostname = my_hostname;

      for (auto it = monitored_topics.begin(); it != monitored_topics.end(); it++) {
        rostune::SingleTopicStats sts;
        sts.name = it->topic_name;
        sts.num_of_messages = it->num_of_messages;
        sts.bytes_per_second = it->bytes_per_second;
        sts.avg_bytes_per_msg = it->avg_bytes_per_msg;
        sts.avg_msgs_per_sec = it->avg_msgs_per_sec;
        sts.total_bytes = it->total_bytes;
        mts.topics.push_back(sts);
      }

      mts.header.stamp = ros::Time::now();
      mts.topics = mts.topics;

      for (auto it = mts.topics.begin(); it != mts.topics.end(); it++){
        mts.total_num_of_messages += it->num_of_messages;
        mts.total_bytes_per_second += it->bytes_per_second;
        mts.total_avg_msgs_per_sec += it->avg_msgs_per_sec;
        mts.total_bytes_per_second += it->avg_bytes_per_msg;
        mts.total_bytes += it->total_bytes;
      }

      topicstats_pub.publish(mts);

    }

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
