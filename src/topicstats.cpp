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


#include "topicstats.h"

namespace topicstats {

  TopicStats::TopicStats(const std::string tn){
    topic_name = tn;
    avg_bytes_per_msg = 0;
    avg_msgs_per_sec = 0;
    bytes_per_second = 0;
    num_of_messages = 0;
    total_bytes = 0;
    start_time = 0;
    subscribed = false;
  }

  bool TopicStats::operator==(const TopicStats& ts) const{
    return topic_name.compare(ts.topic_name) == 0;
  }

  bool TopicStats::operator==(const std::string& tn) const{
    return topic_name.compare(tn) == 0;
  }

  void TopicStats::callback(const topic_tools::ShapeShifter::ConstPtr& msg) {
    if(start_time == 0) {
      start_time = ros::Time::now().toSec();
    }
    double curr_time = ros::Time::now().toSec();
    num_of_messages++;
    total_bytes += msg->size();
    avg_bytes_per_msg = total_bytes / num_of_messages;
    if(curr_time-start_time > 0){
      bytes_per_second = total_bytes / (curr_time-start_time);
      avg_msgs_per_sec = num_of_messages / (curr_time-start_time);
    }
  }

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
