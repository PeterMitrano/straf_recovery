# straf_recovery [![Build Status](https://travis-ci.org/PeterMitrano/straf_recovery.svg?branch=master)](https://travis-ci.org/PeterMitrano/straf_recovery)

A move_base recovery behavior plugin for omni directional robots.

parameters:

 - minimum_straf_distance
 - maximum_straf_distance
 - timeout
 - straf_vel
 
Timeout rules all. Then comes minimum and maximum distance. Straf velocity is constant.

# TODO

 - Use dynamic reconfigure
