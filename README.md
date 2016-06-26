# Minimum-Time-Optimal-Control

This project solves the minimum time optimal control problem for a differential-drive mobile robot with a cruise-control system to go around a racetrack. The output are the optimal velocity and orientation commands. 

There is no need to install any special software. 

The free internet-based service for solving numerical optimization problems NEOS solver is used. You just need to go to https://neos-server.org/neos/ 
and then click on Submit a Job to NEOS under the NEOS Server title in the middle of the page. Next find Nonlinearly Constrained Optimization section and click on AMPL Input to the right of Knitro. 
The last step is to Upload the three files provided in this repository. Under Model file select RobotModel_MinTime.mod; upload RaceTrackData.dat to Data file; finally comm.run to the Commmands file

Once you are done click on Submit to NEOS on the bottom of the page. After that you should write down the Job number and password that will be displayed after. It will take 3-4 hours to finish the job. 
