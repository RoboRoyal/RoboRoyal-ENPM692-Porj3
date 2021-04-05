# RoboRoyal-ENPM692-Porj3

Project 3 Phase for ENPM692, Spring 2021

This project implements A* for a rigid robot with limited mobility and static obstacles.
The bot has a radius of 10 and a clearence of 5.
The robot can move in 5 direction, -60, -30, 0, +30, and +60 degrees from its current orientation in a single descrete step size. 

For this project, pygame is used to visualise the graph, along with the nodes explored and the final path found.

Pygame is the only library used that is not built in.

To run:
'''
python3 main.py
pygame 2.0.1 (SDL 2.0.14, Python 3.7.6)
Hello from the pygame community. https://www.pygame.org/contribute.html
Enter the <X> <Y> coordinates of the start point: 1 2 
Enter the <X> <Y> coordinates of the target point: 66 99
Point is inside an obstacle
Enter the <X> <Y> coordinates of the target point: 200 250
Point is inside an obstacle
Enter the <X> <Y> coordinates of the target point: 30 200
Finding path...
Found path
Visited:  350
Path:  22
Path length:  10
Done
'''
The program will ask for what type of algorithm to use, followed by the start and target location. If you enter an invalid point, it will re-prompt you.

In the shown map, white represents free space, black is for obstacles, cyan is explored nodes, and the final path is depicted in magenta.


Github repository: https://github.com/RoboRoyal/ENPM692-Porj3
