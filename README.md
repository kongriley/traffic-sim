# traffic-sim
## Improving School Campus Traffic Flow Through Dynamic Simulation Modeling and Machine Learning

This is a series of programs designed to simulate a typical school day applied to different traffic protocols. It utilizes [SUMO](https://sumo.dlr.de/docs/index.html) as a simulation program.

## Structure 
* vcs3.py -- Main runner
* vcs2.net.xml -- Defines the layout of the traffic model
* vcs2.rou.xml -- Defines routes and flows
* vcs2.sumocfg -- SUMO configuration file
* baseline.npy -- Defines a baseline to compare the model to
* count-export.csv -- Real data about car flow

## Steps to run simulation (with GUI enabled)
1. Run vcs3.py to open SUMO-GUI. 
2. Adjust the delay to 50ms and change the visualization from "standard" to "real world". 
3. Click the play button to run the simulation!
