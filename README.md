To run the drone project follow these steps:

1. Open a terminal and start the roscore process by typing roscore.
2. Make sure the drone is connected to the computer via WiFi.
3. Open another terminal and type rosrun ardrone_autonomy ardrone_driver.
4. This should connect the computer to the drone and tell you information such as battery level.
5. Before taking off, make sure that the drone is on a flat surface.
6. Open another terminal and type rosservice call ardrone/flattrim
7. This recalibrates the sensors on the drone so it can hover during flight.
8. Open another terminal and type roscd drone_project/bin then press enter.
9. This is where the drone's executable is.
10. Type ./DroneProject to run the program, this should open up a window where you can control certain aspects of the drone.
11. When opening the executable using ./DroneProject there are several arguments you can input to make the program do different things.
	-r records the video feed to a file named video1.avi in the /bin folder.
	-v [input file] states what video to read from file.
	-m [input file] states what model to read from file.
12. To control the drone during flight, the main window must be focused and the following commands can be done via keyboard control:
	t - Take Off.
	l - Land.
	s - Stop learning.
	f - Begin automated flight tracking and following.
	h - Hover.
	SPACE - Emergency stop.
13. To start the tracking an initial bounding box must be specified, by dragging the mouse over the target object.
14. After an initial bounding box has been given the object tracking will begin its online learning.
15. Normal procedure to start tracking and following is:
	- Press T to take off.
	- Wait 5 seconds for the drone to calibrate.
	- Press F to begin tracking and targeting.
16 . Press L to land when done.
