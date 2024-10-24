to run this build in the workspace with the terminal in this folder. run colcon build --packages-select shape_detector.

To run the code, run ros2 run shape_detector shape_detector which will run the code once and scan the image infront of the camera. If there is a shape it will write it to the text file in the folder. If it doesn't it will terminate and exit. 

Finally, to clear the text file, run ros2 run shape_detector shape_detector --clear to clear the text file. 

The code does need to be modularised and cleaned up and commented on but i will do that a bit later. 
