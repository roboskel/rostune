^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rostune
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Hopefully fixed dependency issues for all platforms
* Contributors: George Stavrinos

1.0.3 (2017-07-11)
------------------
* Added a dependency that was removed by mistake
* Contributors: George Stavrinos

1.0.2 (2017-07-11)
------------------
* Fixed dependencies
* Contributors: George Stavrinos

1.0.1 (2017-07-11)
------------------
* Release 1.0.0
* Fixed the mighty python nodes bug! yay!
* Fixed topic stats
* Each machine checks only for its own nodes, nodestats includes percentages.
* Added information about overall stats
* Statistics for all topics working. Now we need to filter them, based on the publisher.
* CPU usage and memory stats
  Unix-related code to get CPU usage and memory stats.
  Checking with master node to update list of nodes.
  Sample logger that just writes to ROSINFO.
* Contributors: George Stavrinos, Stasinos Konstantopoulos
