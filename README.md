# straf_recovery [![Build Status](https://travis-ci.org/PeterMitrano/straf_recovery.svg?branch=master)](https://travis-ci.org/PeterMitrano/straf_recovery)

A move_base recovery behavior plugin for omni directional robots.
It will drive away from the nearest obstacle in the local costmap, constantly checking for new nearest obstacles.
This often means lots of silly oscillation.
It will also straf directly towards the goal if it is within some threshold.

parameters:

 - go_to_goal_distance_threshold
 - minimum_translate_distance
 - maximum_translate_distance
 - straf_vel
 - timeout
 - frequency
 - xy_goal_tolerance

Timeout rules all. Then comes minimum and maximum distance. Straf velocity is constant.

See the ros wiki for possibly more details:
[http://wiki.ros.org/straf_recovery](http://wiki.ros.org/straf_recovery)
