#!/usr/bin/env python
import time
import rospy
import rosgraph
import subprocess, shlex

def init():
    rospy.init_node('rostune_messages')

    while not rospy.is_shutdown():
        '''
        command = "rostopic list"
        command = shlex.split(command)
        t_proc = subprocess.Popen(command, stdout=subprocess.PIPE)
        topics = t_proc.stdout.read()
        #print topics
        command = "rosnode list"
        command = shlex.split(command)
        n_proc = subprocess.Popen(command, stdout=subprocess.PIPE)
        nodes = n_proc.stdout.read()
        #print nodes
        nodes = nodes.split(' ')
        #print nodes
        for n in nodes:
            command = "rosnode info " + n
            command = shlex.split(command)
            n_proc = subprocess.Popen(command, stdout=subprocess.PIPE)
            info = n_proc.stdout.read()
            print info
        '''
        master = rosgraph.Master('/rostopic')
        state = master.getSystemState()
        pubs, subs, _ = state
        topics = list(set([t for t,_ in pubs] + [t for t,_ in subs])) 
        topics.sort()
        #print topics
        topic = rosgraph.names.script_resolve_name('rostopic', topics[0])
        print topic
        time.sleep(30)

if __name__ == '__main__':
    init()