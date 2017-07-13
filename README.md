# Description
Rostune is a tool that helps ROS developers distribute their nodes in the most effective way. It collects statistics for topics and nodes, such as CPU and network usage.

# How To Use rostune
- Run rostune on all of your machines. 
- Edit the `config/parameters.yaml` file based on your needs. 
- Using a ROS visualization tool like rqt_plot and (the even better) PlotJuggler, you can visualize the statistics you are interested in. 
- Move nodes to different machines, and observe which setup is the most effective, based on the statistics provided.

## PlotJuggler specific setup
In order for PlotJuggler to work as intended, you have to follow some simple steps:
- Run PlotJuggler.
- Navigate to Streaming -> Start: ROS Topic Streamer.
- In the popup dialog, tick on both tickboxes, and then click on the "Edit Rules" button.
- Open the provided `plotjuggler.rules.xml` file and paste its contents just before the `</SubstitutionRules>` tag in the second dialog that popped up when you clicked on "Edit Rules".
- Select "Confirm and Close".
- Select the rostune topic(s) that you want to suscribe to, and you are ready to go!

# ROS Indigo Build Status 
[![Build Status](http://build.ros.org/job/Isrc_uT__rostune__ubuntu_trusty__source/badge/icon)](http://build.ros.org/job/Isrc_uT__rostune__ubuntu_trusty__source)

# ROS Kinetic Build Status
[![Build Status](http://build.ros.org/job/Ksrc_uX__rostune__ubuntu_xenial__source/badge/icon)](http://build.ros.org/job/Ksrc_uX__rostune__ubuntu_xenial__source/)


