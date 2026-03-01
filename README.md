## Team Table Tennis Tinkers
Team: Alex Chen, Connor Chai, Daniel Jenkins

## Purpose
This project was created out of interest to create a table-tennis serving robot and compete in the 2026 FAS Competition.

## Information about this repository
This repository contains code for two ESP32s, one 'master' and 'slave'
The ESP32s communicate via 2.4ghz wifi using MAC addresses, with the master sending packets containing button states 
and the slave moving servos and DC motors based on the given input provided by the payload packet. 

## Items Used
2x ESP-WROOM-32
1x SG90 servo motor (indexer)
2x MG90 servo motor (pitch & yaw control)
1x Motor Driver (SparkFun TB6612FNG)
Various 3D printed parts, jumper wires, buttons, etc
