intermediate.py in the scripts folder is the thing that does the communication between a ps5* controller and the arduino.

the program is pointed at an arduino at 192.168.0.57

a ros joy node with a controller must also be started to get the controller messages into ros.

*you might be able to use a ps4 controller since they have the same messaage map. 
