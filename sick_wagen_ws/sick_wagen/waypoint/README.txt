It is noted that you should run "chmod +x -----.py" before you run python file.

This directory is for waypoint navigation. When you use waypoint for navigation, you should run program which run movebase, for example, "8-whill-tim2-movebase". 
Firstly, you make csv file which includes data for tracking. 
For this purpose, you can choose either "saver-waypoint.py" or "maker-waypoint.py.
Then you run "laoder-waypoint.py" and whill will move refering the csv file.
It is necessary to remove csv file every time you change the trajectory. 
(It should be modified)

FILE DESCRIPTION
"saver-waypoint.py"
	This file will save the position of whill every 5 seconds.
	You can change the interval in the program.

"maker-waypoint.py"
	This file will save the 2D-nav-goal you made.
	
"loader-waypoint.py" 
	This file will load every line of the csv file as 2D-nav-goal.
