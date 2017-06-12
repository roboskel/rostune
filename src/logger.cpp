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
#include "std_msgs/String.h"
#include "rostune/Logline.h"

void loggerCallback( const rostune::Logline::ConstPtr& msg )
{

  ROS_INFO( "Recv'ed: [%s, cputime: %lu, walltime: %lu + %lu, cputimediff: %lu, walltimediff: %lu, all mem: %lu, res mem: %lu]",
         msg->node_name.c_str(),
         msg->cputime, msg->header.stamp.sec, msg->header.stamp.nsec,
         msg->diffcputime, msg->diffwalltime,
         msg->all_memory, msg->resident_memory );
  
  float relcputimediff = ((float)msg->diffcputime) / msg->diffwalltime;

  ROS_INFO( "Computed: [%s, relative cputime diff: %0.6f, vmem: %luMB, resmem: %luMB]",
        msg->node_name.c_str(), relcputimediff,
        msg->all_memory/1024/2014, msg->resident_memory/1024/1024 );
}


int main( int argc, char **argv )
{
  ros::init( argc, argv, "logger" );
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe( "cpuload", 1000, loggerCallback );
  ros::spin();
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
