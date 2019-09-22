import os, sys
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

sumoBinary = "sumo-gui.exe"
sumoCmd = [sumoBinary, "-c", "vcs2.sumocfg"]

import traci

import matplotlib.pyplot as plt
import numpy as np

stop = 20
times = [15, 20, 30, 40, 60]
reds = [5, 5, 5, 5]
omega = 3
end = 1000

c = 0
sub = 0
pcount = 0

xs = np.repeat(np.arange(end)[:, np.newaxis], len(times), axis=1)
ys = np.zeros((end, len(times)))

index = 0
for time, red in zip(times, reds):
    traci.start(sumoCmd)
    traci.gui.setSchema(traci.gui.DEFAULT_VIEW, "real world")
    for step in range(end):
        traci.simulationStep()
        # print(str(step))

        # queues
        q1 = traci.lane.getLastStepVehicleNumber("drop1_1")
        q2 = traci.lane.getLastStepVehicleNumber("drop2_0")
        # print("drop 1: "+str(q1))
        # print("drop 2: "+str(q2))
        qp = sum(1 for p in traci.person.getIDList() if traci.person.getNextEdge(p) == ":jun_c0") * omega
        # print("drop p: "+str(qp))

        # cars currently deciding
        drops = traci.edge.getLastStepVehicleIDs("e13")
        # separate deciding by dropoff
        drops11 = [drop for drop in drops if "11" in traci.vehicle.getRouteID(drop)]
        drops12 = [drop for drop in drops if "12" in traci.vehicle.getRouteID(drop)]

        drops21 = [drop for drop in drops if "21" in traci.vehicle.getRouteID(drop)]
        drops22 = [drop for drop in drops if "22" in traci.vehicle.getRouteID(drop)]
        # print(len(drops11))

        if q1 > q2:
            # switch route 1 to route 2
            for car in drops11:
                d = traci.vehicle.getRouteID(car)[0]
                traci.vehicle.setRouteID(car, d+"21")
            for car in drops12:
                d = traci.vehicle.getRouteID(car)[0]
                traci.vehicle.setRouteID(car, d+"22")
        elif q1 < q2:
            # switch route 2 to route 1
            for car in drops21:
                d = traci.vehicle.getRouteID(car)[0]
                traci.vehicle.setRouteID(car, d+"11")
            for car in drops22:
                d = traci.vehicle.getRouteID(car)[0]
                traci.vehicle.setRouteID(car, d+"12")
        
        pot = [car for car in traci.vehicle.getIDList() if "drop" in traci.vehicle.getLaneID(car)]

        # drop off - add person
        for car in pot:
            # print(traci.vehicle.getColor(car))
            edge = traci.vehicle.getLaneID(car)[:-2]
            if traci.vehicle.getColor(car) == (255, 255, 0, 255):
                length = traci.lane.getLength(edge+"_0")
                traci.vehicle.setStop(car, edge, laneIndex=2-int(edge[-1:]), duration=stop, pos=length, startPos=0)
                traci.vehicle.setColor(car, (255, 0, 0))
            elif traci.vehicle.getColor(car) == (255, 0, 0, 255) and traci.vehicle.getStopState(car) == 1:
                traci.person.add(str(c), edge, traci.vehicle.getLanePosition(car), depart=step+5)
                traci.person.appendWalkingStage(str(c), [edge, "ped3"], 10)
                c += 1
                pcount += 1
                traci.vehicle.setColor(car, (0, 255, 0))

        # traffic light
        if (step + red) % time == 0:
            if q1 > q2:
                if q1 > qp:
                    traci.trafficlight.setPhase("jun", 1)
                else: 
                    traci.trafficlight.setPhase("jun", 3)
            else:
                if q2 > qp:
                    traci.trafficlight.setPhase("jun", 2)
                else: 
                    traci.trafficlight.setPhase("jun", 3)
        elif (step + red) % time == step:
            traci.trafficlight.setPhase("jun", 0)

        # first queue
        x = traci.lane.getLastStepVehicleNumber("park1_1")
        y = traci.lane.getLastStepVehicleNumber("park2a_1")+traci.lane.getLastStepVehicleNumber("park2b_1")+traci.lane.getLastStepVehicleNumber("park2c_1")
        
        parks = traci.edge.getLastStepVehicleIDs("e10")
        # separate deciding by dropoff
        parks11 = [park for park in parks if "11" in traci.vehicle.getRouteID(park)]
        parks12 = [park for park in parks if "12" in traci.vehicle.getRouteID(park)]

        parks21 = [park for park in parks if "21" in traci.vehicle.getRouteID(park)]
        parks22 = [park for park in parks if "22" in traci.vehicle.getRouteID(park)]
        
        if x > y:
            for car in parks11:
                d = traci.vehicle.getRouteID(car)[0]
                traci.vehicle.setRouteID(car, d+"12")
            for car in parks21:
                d = traci.vehicle.getRouteID(car)[0]
                traci.vehicle.setRouteID(car, d+"22")
        elif x < y:
            for car in parks12:
                d = traci.vehicle.getRouteID(car)[0]
                traci.vehicle.setRouteID(car, d+"11")
            for car in parks22:
                d = traci.vehicle.getRouteID(car)[0]
                traci.vehicle.setRouteID(car, d+"21")
        
        ys[step, index] = pcount
    traci.close()
    index += 1

index = 0
for x, y in zip(xs, ys):
    plt.plot(x, y, label='Duration of '+str(times[index])+'s')
    index += 1
plt.xlabel('Number of students dropped')
plt.ylabel('Time (s)')

plt.title("Total number of dropped students vs. time")
plt.legend()

plt.show()