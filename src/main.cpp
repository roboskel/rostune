/*
 * Copyright (C) 2017, NCSR "Demokritos"
 *
 * BSD 3
 */
//
// %Tag(FULLTEXT)%
// %Tag(ROS_HEADER)%
#include "ros/ros.h"
// %EndTag(ROS_HEADER)%
// %Tag(MSG_HEADER)%
#include "std_msgs/String.h"
// %EndTag(MSG_HEADER)%

#include <sstream>


int main(int argc, char **argv)
{

  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
// %Tag(INIT)%
  ros::init(argc, argv, "talker");
// %EndTag(INIT)%

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
// %Tag(NODEHANDLE)%
  ros::NodeHandle n;
// %EndTag(NODEHANDLE)%

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
// %Tag(PUBLISHER)%
  ros::Publisher cputime_pub = n.advertise<std_msgs::String>( "cputime", 1000 );
// %EndTag(PUBLISHER)%

// %Tag(LOOP_RATE)%
  ros::Rate loop_rate(10);
// %EndTag(LOOP_RATE)%

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
// %Tag(ROS_OK)%
  int count = 0;
  while( ros::ok() )
  {
// %EndTag(ROS_OK)%
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
// %Tag(FILL_MESSAGE)%
    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();
// %EndTag(FILL_MESSAGE)%

// %Tag(ROSCONSOLE)%
    ROS_INFO("%s", msg.data.c_str());
// %EndTag(ROSCONSOLE)%

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
// %Tag(PUBLISH)%
    cputime_pub.publish( msg );
// %EndTag(PUBLISH)%

// %Tag(SPINONCE)%
    ros::spinOnce();
// %EndTag(SPINONCE)%

// %Tag(RATE_SLEEP)%
    loop_rate.sleep();
// %EndTag(RATE_SLEEP)%
    ++count;
  }


  return 0;
}
// %EndTag(FULLTEXT)%

/*
float cputime( int pid )
{
  FILE *fp;
  char buff[255];
  fp= fopen("/proc/123/stat", "r");
  if(fp == NULL) {
    
  }
  else {
    // Based on http://man7.org/linux/man-pages/man5/proc.5.html
    fscanf( fp, "%d %s %c %d %d %d %d %d %u %lu %lu %lu %lu %lu %lu %lu %lu",
	         &pid,
	            &filename,
	               &state,
	                  &ppid,
	                     &procgrp,
                                &sessionid,
                                   &tty,
                                      &termprocgrp,
                                         &kernelflags,
                                            &minorfaults,
                                                &childminorfaults,
                                                    &majorfaults,
                                                        &childmajorfaults,
                                                            &usertime,
                                                                &systime,
                                                                    &childrenusertime,
                                                                        &childrensystime
	    )
      fclose( fp );
  }

  
}
*/

/*

              (14) utime  %lu
                        Amount of time that this process has been scheduled
                        in user mode, measured in clock ticks (divide by
                        sysconf(_SC_CLK_TCK)).  This includes guest time,
                        guest_time (time spent running a virtual CPU, see
                        below), so that applications that are not aware of
                        the guest time field do not lose that time from
                        their calculations.

              (15) stime  %lu
                        Amount of time that this process has been scheduled
                        in kernel mode, measured in clock ticks (divide by
                        sysconf(_SC_CLK_TCK)).

              (16) cutime  %ld
                        Amount of time that this process's waited-for
                        children have been scheduled in user mode, measured
                        in clock ticks (divide by sysconf(_SC_CLK_TCK)).
                        (See also times(2).)  This includes guest time,
                        cguest_time (time spent running a virtual CPU, see
                        below).

              (17) cstime  %ld
                        Amount of time that this process's waited-for
                        children have been scheduled in kernel mode,
                        measured in clock ticks (divide by
                        sysconf(_SC_CLK_TCK)).

              (18) priority  %ld
                        (Explanation for Linux 2.6) For processes running a
                        real-time scheduling policy (policy below; see
                        sched_setscheduler(2)), this is the negated
                        scheduling priority, minus one; that is, a number in
                        the range -2 to -100, corresponding to real-time
                        priorities 1 to 99.  For processes running under a
                        non-real-time scheduling policy, this is the raw
                        nice value (setpriority(2)) as represented in the
                        kernel.  The kernel stores nice values as numbers in
                        the range 0 (high) to 39 (low), corresponding to the
                        user-visible nice range of -20 to 19.

                        Before Linux 2.6, this was a scaled value based on
                        the scheduler weighting given to this process.

              (19) nice  %ld
                        The nice value (see setpriority(2)), a value in the
                        range 19 (low priority) to -20 (high priority).

              (20) num_threads  %ld
                        Number of threads in this process (since Linux 2.6).
                        Before kernel 2.6, this field was hard coded to 0 as
                        a placeholder for an earlier removed field.

              (21) itrealvalue  %ld
                        The time in jiffies before the next SIGALRM is sent
                        to the process due to an interval timer.  Since
                        kernel 2.6.17, this field is no longer maintained,
                        and is hard coded as 0.

              (22) starttime  %llu
                        The time the process started after system boot.  In
                        kernels before Linux 2.6, this value was expressed
                        in jiffies.  Since Linux 2.6, the value is expressed
                        in clock ticks (divide by sysconf(_SC_CLK_TCK)).

                        The format for this field was %lu before Linux 2.6.

              (23) vsize  %lu
                        Virtual memory size in bytes.

              (24) rss  %ld
                        Resident Set Size: number of pages the process has
                        in real memory.  This is just the pages which count
                        toward text, data, or stack space.  This does not
                        include pages which have not been demand-loaded in,
                        or which are swapped out.

              (25) rsslim  %lu
                        Current soft limit in bytes on the rss of the
                        process; see the description of RLIMIT_RSS in
                        getrlimit(2).

              (26) startcode  %lu  [PT]
                        The address above which program text can run.

              (27) endcode  %lu  [PT]
                        The address below which program text can run.

              (28) startstack  %lu  [PT]
                        The address of the start (i.e., bottom) of the
                        stack.

              (29) kstkesp  %lu  [PT]
                        The current value of ESP (stack pointer), as found
                        in the kernel stack page for the process.

              (30) kstkeip  %lu  [PT]
                        The current EIP (instruction pointer).

              (31) signal  %lu
                        The bitmap of pending signals, displayed as a
                        decimal number.  Obsolete, because it does not
                        provide information on real-time signals; use
                        /proc/[pid]/status instead.

              (32) blocked  %lu
                        The bitmap of blocked signals, displayed as a
                        decimal number.  Obsolete, because it does not
                        provide information on real-time signals; use
                        /proc/[pid]/status instead.

              (33) sigignore  %lu
                        The bitmap of ignored signals, displayed as a
                        decimal number.  Obsolete, because it does not
                        provide information on real-time signals; use
                        /proc/[pid]/status instead.

              (34) sigcatch  %lu
                        The bitmap of caught signals, displayed as a decimal
                        number.  Obsolete, because it does not provide
                        information on real-time signals; use
                        /proc/[pid]/status instead.

              (35) wchan  %lu  [PT]
                        This is the "channel" in which the process is
                        waiting.  It is the address of a location in the
                        kernel where the process is sleeping.  The
                        corresponding symbolic name can be found in
                        /proc/[pid]/wchan.

              (36) nswap  %lu
                        Number of pages swapped (not maintained).

              (37) cnswap  %lu
                        Cumulative nswap for child processes (not
                        maintained).

              (38) exit_signal  %d  (since Linux 2.1.22)
                        Signal to be sent to parent when we die.

              (39) processor  %d  (since Linux 2.2.8)
                        CPU number last executed on.

              (40) rt_priority  %u  (since Linux 2.5.19)
                        Real-time scheduling priority, a number in the range
                        1 to 99 for processes scheduled under a real-time
                        policy, or 0, for non-real-time processes (see
                        sched_setscheduler(2)).

              (41) policy  %u  (since Linux 2.5.19)
                        Scheduling policy (see sched_setscheduler(2)).
                        Decode using the SCHED_* constants in linux/sched.h.

                        The format for this field was %lu before Linux
                        2.6.22.

              (42) delayacct_blkio_ticks  %llu  (since Linux 2.6.18)
                        Aggregated block I/O delays, measured in clock ticks
                        (centiseconds).

              (43) guest_time  %lu  (since Linux 2.6.24)
                        Guest time of the process (time spent running a
                        virtual CPU for a guest operating system), measured
                        in clock ticks (divide by sysconf(_SC_CLK_TCK)).

              (44) cguest_time  %ld  (since Linux 2.6.24)
                        Guest time of the process's children, measured in
                        clock ticks (divide by sysconf(_SC_CLK_TCK)).

              (45) start_data  %lu  (since Linux 3.3)  [PT]
                        Address above which program initialized and
                        uninitialized (BSS) data are placed.

              (46) end_data  %lu  (since Linux 3.3)  [PT]
                        Address below which program initialized and
                        uninitialized (BSS) data are placed.

              (47) start_brk  %lu  (since Linux 3.3)  [PT]
                        Address above which program heap can be expanded
                        with brk(2).

              (48) arg_start  %lu  (since Linux 3.5)  [PT]
                        Address above which program command-line arguments
                        (argv) are placed.

              (49) arg_end  %lu  (since Linux 3.5)  [PT]
                        Address below program command-line arguments (argv)
                        are placed.

              (50) env_start  %lu  (since Linux 3.5)  [PT]
                        Address above which program environment is placed.

              (51) env_end  %lu  (since Linux 3.5)  [PT]
                        Address below which program environment is placed.

              (52) exit_code  %d  (since Linux 3.5)  [PT]
                        The thread's exit status in the form reported by
                        waitpid(2).
*/
