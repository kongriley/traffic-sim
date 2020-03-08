import os, sys
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

gui = False # use gui??
testing = False
run_type = 3 # 1 - 3

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
import math

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

def getPed(nextEdge):
    total = 0
    for person in traci.person.getIDList():
        if traci.person.getSpeed(person) < 0.01 and traci.person.getNextEdge(person) == nextEdge:
            total += 1
    return total

def actionMax(total, arr):
    return np.max(arr[total[0], total[1], total[2], :])

def argActionMax(total, arr):
    return np.argmax(arr[total[0], total[1], total[2], :])

trials = 1 # number of episodes
cum_trials = 0

stop = 20
omega = 3
end = 3600
wmax = 80

try:
    Q = np.load('q.npy')
except:
    Q = np.random.rand(wmax, wmax, wmax, 4)
try:
    Q2 = np.load('q2.npy')
except:
    Q2 = np.random.rand(wmax, wmax, wmax, wmax, 4)
epsilon = 0.05
lr = 0.1
gamma = 0.8

xs = np.arange(end)
episode_xs = np.arange(trials)
base_ys = np.zeros(end)
enter_ys = np.zeros(end)
wait_ys = np.zeros(end)
car_ys = np.zeros(end)
reward_ys = np.zeros(trials)
wait2_ys = np.zeros(end)
reward2_ys = np.zeros(trials)
ys = np.zeros(end)

with open('count-export.csv') as f:
    read = list(csv.reader(f, delimiter=','))[1:]
    read = [x[2] for x in read]
    read = [s[4:] for s in read]
    read = [60*int(s[:2])+int(s[3:]) - 812 for s in read]

if not testing:
    trials += 2

for t in range(trials): # number of episodes
    # if t < 2:
    #     continue
    c = 0
    sub = 0
    pcount = 0
    carc = 0
    last_stop = 0
    cum_car = 0

    prev_state = (0, 0, 0, 0)
    prev_state2 = (0, 0, 0, 0, 0)

    # if gui:
    #     traci.gui.setSchema(traci.gui.DEFAULT_VIEW, "real world")
    traci.start(sumoCmd)
    state_lengths = [traci.lane.getLength("drop1_1"), traci.lane.getLength("drop2_0"), 1]
    state_lengths2 = [traci.lane.getLength("sin_0"),traci.lane.getLength("win_0"),traci.lane.getLength("ein_0"),traci.lane.getLength("e21_0")]

    car_list = read.copy()
    for step in range(end):
        real_car_num = traci.vehicle.getIDCount()
        rchoice = ['w', 'e', 's']
        

        # add cars (data-based)
        # if len(car_list) > 0:
        #     while car_list[0] == step:
        #         route_name = random.choices(rchoice, weights=[0.4, 0.1, 0.5])[0]
        #         route_name += random.choices(['11', '21', '12', '22'])[0]
        #         traci.vehicle.add('car'+str(cum_car), route_name, typeID='car')
        #         car_list = car_list[1:]
        #         if len(car_list) == 0: break
        #         cum_car += 1
        
        # add cars (flow-based)
        car_rand = random.random()
        if step < 900:
            if car_rand < 0.0675:
                route_name = random.choices(rchoice, weights=[0.4, 0.1, 0.5])[0]
                route_name += random.choices(['11', '21', '12', '22'])[0]
                traci.vehicle.add('car'+str(cum_car), route_name, typeID='car')
                cum_car += 1
        elif step < 1800:
            if car_rand < 0.145141667:
                route_name = random.choices(rchoice, weights=[0.4, 0.1, 0.5])[0]
                route_name += random.choices(['11', '21', '12', '22'])[0]
                traci.vehicle.add('car'+str(cum_car), route_name, typeID='car')
                cum_car += 1
        elif step < 2700:
            if car_rand < 0.33375:
                route_name = random.choices(rchoice, weights=[0.4, 0.1, 0.5])[0]
                route_name += random.choices(['11', '21', '12', '22'])[0]
                traci.vehicle.add('car'+str(cum_car), route_name, typeID='car')
                cum_car += 1
        else:
            if car_rand < 0.20305556:
                route_name = random.choices(rchoice, weights=[0.4, 0.1, 0.5])[0]
                route_name += random.choices(['11', '21', '12', '22'])[0]
                traci.vehicle.add('car'+str(cum_car), route_name, typeID='car')
                cum_car += 1
        
        
        # if step == 100:
        #     traci.vehicle.add('train', 'rt', typeID='t')

        traci.simulationStep()
        # print(str(step))

        # queues
        q1 = traci.lane.getLastStepVehicleNumber("drop1_1")
        q2 = traci.lane.getLastStepVehicleNumber("drop2_0")
        qp = len(traci.edge.getLastStepPersonIDs('ped3'))
        # print("drop 1: "+str(q1))
        # print("drop 2: "+str(q2))
        # print("drop p: "+str(qp))

        # cars currently deciding
        drops = traci.edge.getLastStepVehicleIDs("e13")+traci.edge.getLastStepVehicleIDs('e12')
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
        
        
        if t == 0 and testing == False:
            pass
            # setLight(q1, q2, qp)
            # if step % 60 == 0:
            #     traci.trafficlight.setPhase('jun', 1)
            # elif step % 60 == 20:
            #     traci.trafficlight.setPhase('jun', 2)
            # else:
            #     traci.trafficlight.setPhase('jun', 3)
        elif t == 1 and testing == False:
            setLight(q1, q2, qp)
        else: 
            # traffic light WITH Q-LEARNING!
            waiting_cars = traci.lane.getLastStepVehicleIDs("drop1_1")+traci.lane.getLastStepVehicleIDs("drop2_0")
            waiting_people = traci.edge.getLastStepPersonIDs("ped3")
            wait_time = 0
            for curr in waiting_cars:
                wait_time += traci.vehicle.getAccumulatedWaitingTime(curr)
            for curr in waiting_people:
                wait_time += traci.person.getWaitingTime(curr)
            
            num_cars = len(waiting_cars+waiting_people)
            if run_type == 1:
                reward = -wait_time
            elif run_type == 2:
                reward = -0.5*num_cars-0.7*wait_time
            elif run_type == 3:
                # num_cars2 /= (traci.lane.getLength("sin_0")+traci.lane.getLength("win_0")+traci.lane.getLength("ein_0")+traci.lane.getLength("e21_0"))
                reward = -(wait_time*num_cars) # reward = total waiting time of all people/cars

            state_tuple = [traci.lane.getLastStepVehicleNumber("drop1_1"), traci.lane.getLastStepVehicleNumber("drop2_0"), getPed(':jun_c0')]
            
            curr_state = [int(x/y) for x, y in zip(state_tuple, state_lengths)]
            # print(total_state)

            
            if random.uniform(0, 1) < epsilon:
                action = random.randint(0, 3) # random action (explore)
            else:
                action = np.argmax(Q[curr_state[0], curr_state[1], curr_state[2], :]) # best action (exploit)
            traci.trafficlight.setPhase("jun", action)

            total_state = (curr_state[0], curr_state[1], curr_state[2], action)

            if step > 0:
                # algorithm for Q-learning
                Q[prev_state] = Q[prev_state] + lr * (reward + gamma * np.max(Q[total_state[0], total_state[1], total_state[2], :]) - Q[prev_state])
            
            prev_state = total_state
            
            ### monterey ###

            waiting_cars2 = traci.edge.getLastStepVehicleIDs("sin")+traci.edge.getLastStepVehicleIDs("win")+traci.edge.getLastStepVehicleIDs("ein")+traci.edge.getLastStepVehicleIDs("e21")
            wait_time2 = 0
            for curr in waiting_cars2:
                wait_time2 += traci.vehicle.getAccumulatedWaitingTime(curr)

            num_cars2 = len(waiting_cars2)
            if run_type == 1:
                reward2 = -wait_time2
            elif run_type == 2:
                reward2 = -0.5*num_cars2-0.7*wait_time2
            elif run_type == 3:
                num_cars2 /= (traci.lane.getLength("sin_0")+traci.lane.getLength("win_0")+traci.lane.getLength("ein_0")+traci.lane.getLength("e21_0"))
                reward2 = -(wait_time2*num_cars2) # reward = total waiting time of all people/cars

            state_tuple2 = [traci.edge.getLastStepVehicleNumber("sin"),traci.edge.getLastStepVehicleNumber("win"),traci.edge.getLastStepVehicleNumber("ein"),traci.edge.getLastStepVehicleNumber("e21")]
            
            curr_state2 = [int(x/y) for x, y in zip(state_tuple2, state_lengths2)]

            if random.uniform(0, 1) < epsilon:
                action2 = random.randint(0, 3) # random action (explore)
            else:
                action2 = np.argmax(Q2[curr_state2[0], curr_state2[1], curr_state2[2], curr_state2[3], :]) # best action (exploit)
            traci.trafficlight.setPhase("mont", action2)

            total_state2 = (curr_state2[0], curr_state2[1], curr_state2[2], curr_state2[3], action)

            if step > 0:
                # algorithm for Q-learning
                Q2[prev_state2] = Q2[prev_state2] + lr * (reward + gamma * np.max(Q2[total_state2[0], total_state2[1], total_state2[2], total_state2[3] :]) - Q2[prev_state2])
            
            prev_state2 = total_state2

        # first queue
        x = traci.lane.getLastStepVehicleIDs("park1_0")
        y = traci.lane.getLastStepVehicleIDs("park2a_0")
        
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
        
        if t == 0 and testing == False:
            base_ys[step] = pcount
        elif t == 1 and testing == False:
            enter_ys[step] = pcount
        else: 
            # if t == trials:
            #     sumoCmd = ["sumo-gui.exe", "-c", "vcs2.sumocfg"]
            ys[step] = pcount
            if len(waiting_cars) > 0:
                wait_ys[step] = wait_time/len(waiting_cars)
            if len(waiting_cars2) > 0:
                wait2_ys[step] = wait_time2/len(waiting_cars2)
            car_ys[step] = cum_car

    if t > 1:
        pass
        # reward_ys[t-2] = reward
        # reward2_ys[t-2] = reward2
    
        np.save('q', Q)
        np.save('q2', Q2)

    traci.close()


fig, (ax1, ax2) = plt.subplots(2)
ax1.plot(xs, base_ys, label='Baseline')
ax1.plot(xs, ys, label='Q-learning')
ax1.set_title('Results of Q-learning')
ax1.set(ylabel='Number of cars',xlabel='Time (s)')
ax1.legend()

# ax2.plot(episode_xs, reward_ys, label='Reward of dropoff Q-table')
# ax2.plot(episode_xs, reward2_ys, label='Reward of Monterey Q-table')
# ax2.set_title('Reward over episodes')
# ax2.set(ylabel='Reward',xlabel='Episode')
# ax2.legend()

# fig, (ax1) = plt.subplots(1)

ax2.plot(xs, wait_ys, label='Wait time of dropoff Q-table')
ax2.plot(xs, wait2_ys, label='Wait time of Monterey Q-table')
ax2.set_title('Wait time over episodes')
ax2.set(ylabel='Wait time (s)',xlabel='Episode')
ax2.legend()

# plt.plot(xs, car_ys)
# # plt.plot(xs, enter_ys, label='Naive')
# # plt.plot(xs, ys, label='Car distribution')
# plt.ylabel('Number')
# plt.xlabel('Time (s)')

# plt.title("Results of Q-learning")
# plt.legend()

plt.show()