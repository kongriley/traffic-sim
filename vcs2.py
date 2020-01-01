import os, sys
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

gui = False # use gui??

if gui:
    sumoBinary = "sumo-gui.exe"
else:
    sumoBinary = "sumo"
sumoCmd = [sumoBinary, "-c", "vcs2.sumocfg"]

import traci

import matplotlib.pyplot as plt
import numpy as np
import random
import csv

def key(x):
    return traci.vehicle.getLanePosition(x)

def first(ids):
    curr = None
    if len(ids) == 0:
        return 100000
    return traci.vehicle.getLanePosition(min(ids, key=key))

def setLight(q1, q2, qp):
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

stop = 20
omega = 3
end = 2000
trials = 50
cum_trials = 0

try:
    Q = np.load('q.npy')
except:
    Q = np.zeros((end, 3))
epsilon = 0.2
lr = 0.1
gamma = 0.9

xs = np.arange(end)
base_ys = np.zeros(end)
ys = np.zeros(end)

with open('count-export.csv') as f:
    read = list(csv.reader(f, delimiter=','))[1:]
    read = [x[2] for x in read]
    read = [s[4:] for s in read]
    read = [60*int(s[:2])+int(s[3:]) - 812 for s in read]

for t in range(trials+1): # number of trials, useful for randomness
    c = 0
    sub = 0
    pcount = 0
    carc = 0
    last_stop = 0

    prev_score = 0
    prev_action = 0

    # if gui:
    #     traci.gui.setSchema(traci.gui.DEFAULT_VIEW, "real world")
    traci.start(sumoCmd)
    car_list = read.copy()
    for step in range(end):
        rchoice = ['w11', 'w12', 'w21', 'w22', 'e11', 'e12', 'e21', 'e22', 's11', 's12', 's21', 's22']
        
        # add cars (data-based)
        if car_list[0] == step:
            route_name = random.choice(rchoice)
            traci.vehicle.add('car'+str(step), route_name, typeID='car')
            car_list = car_list[1:]
        
        # add cars (flow-based)
        # if step < 900:
        #     if step % 7 == 0:
        #         traci.vehicle.add('car'+str(carc), random.choice(rchoice), typeID='car')
        #         carc += 1
        # elif step < 1800:
        #     if step % 3 == 0:
        #         traci.vehicle.add('car'+str(carc), random.choice(rchoice), typeID='car')
        #         carc += 1
        # elif step < 2700:
        #     if step % 5 == 0:
        #         traci.vehicle.add('car'+str(carc), random.choice(rchoice), typeID='car')
        #         carc += 1
        
        # if step == 100:
        #     traci.vehicle.add('train', 'rt', typeID='t')

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

        id1 = traci.lane.getLastStepVehicleIDs("drop1_1")
        id2 = traci.lane.getLastStepVehicleIDs("drop2_0")
        if first(id1) < first(id2):
            # switch route 1 to route 2
            for car in drops11:
                d = traci.vehicle.getRouteID(car)[0]
                traci.vehicle.setRouteID(car, d+"21")
            for car in drops12:
                d = traci.vehicle.getRouteID(car)[0]
                traci.vehicle.setRouteID(car, d+"22")
        elif first(id1) > first(id2):
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
        
        
        if t == 0:
            setLight(q1, q2, qp)
        else: 
            # traffic light WITH Q-LEARNING!
            waiting_cars = traci.lane.getLastStepVehicleIDs("drop1_1")+traci.lane.getLastStepVehicleIDs("drop2_0")
            waiting_people = traci.edge.getLastStepPersonIDs("ped3")
            total_score = 0
            for curr in waiting_cars:
                total_score += traci.vehicle.getWaitingTime(curr)
            for curr in waiting_people:
                total_score += traci.person.getWaitingTime(curr)
            
            reward = -(total_score - prev_score) # reward = total waiting time of all people/cars
            if random.uniform(0, 1) < epsilon:
                action = random.randint(0, 2) # random action (explore)
            else:
                action = np.argmax(Q[step, :]) # best action (exploit)
            traci.trafficlight.setPhase("jun", action+1)
            if step > 0:
                # algorithm for Q-learning
                Q[step-1, prev_action] = Q[step-1, prev_action] + lr * (reward + gamma * np.max(Q[step, :]) - Q[step-1, prev_action])
            
            prev_action = action
            prev_score = total_score

        # first queue
        x = traci.lane.getLastStepVehicleIDs("park1_1")
        y = traci.lane.getLastStepVehicleIDs("park2a_1")
        
        parks = traci.edge.getLastStepVehicleIDs("e10")
        # separate deciding by dropoff
        parks11 = [park for park in parks if "11" in traci.vehicle.getRouteID(park)]
        parks12 = [park for park in parks if "12" in traci.vehicle.getRouteID(park)]

        parks21 = [park for park in parks if "21" in traci.vehicle.getRouteID(park)]
        parks22 = [park for park in parks if "22" in traci.vehicle.getRouteID(park)]
        
        if first(x) < first(y):
            for car in parks11:
                d = traci.vehicle.getRouteID(car)[0]
                traci.vehicle.setRouteID(car, d+"12")
            for car in parks21:
                d = traci.vehicle.getRouteID(car)[0]
                traci.vehicle.setRouteID(car, d+"22")
        elif first(x) > first(y):
            for car in parks12:
                d = traci.vehicle.getRouteID(car)[0]
                traci.vehicle.setRouteID(car, d+"11")
            for car in parks22:
                d = traci.vehicle.getRouteID(car)[0]
                traci.vehicle.setRouteID(car, d+"21")
        
        if t == 0:
            base_ys[step] = pcount
        else: 
            ys[step] = pcount
    traci.close()
    np.save('q', Q)

plt.plot(xs, base_ys, label='Baseline')
plt.plot(xs, ys, label='Q-learning ('+str(trials+cum_trials)+' episodes)')
plt.ylabel('Number of students dropped')
plt.xlabel('Time (s)')

plt.title("Results of Q-learning")
plt.legend()

plt.show()