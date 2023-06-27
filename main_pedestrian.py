import os
import sys
import traceback
import json
import optparse
import random
from pprint import pprint
from config import *
from route_file_generation import generate_routefile, generate_routefile_Veberod, generate_routefile_pedestrian, \
    generate_routefile_Veberod_uni, generate_routefile_pedestrian_uni
import matplotlib.pyplot as plt
from kafka import  KafkaProducer

# we need to import python modules from the $SUMO_HOME/tools directory
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

from sumolib import checkBinary  # noqa
import traci  # noqa
import numpy as np


def get_jam_veh_ids(veh_id_list, jam_length):
    veh_ids = sorted([veh_id.split("_")[1] for veh_id in veh_id_list], key=int)
    return veh_ids[:jam_length]


# get the number of vehicles joining the queue through the vehicle id list
def get_increased_jam_num(veh_ids_list):
    num = 0
    if len(veh_ids_list) > 10:
        num = len(veh_ids_list[0])
    return len(set(sum(veh_ids_list, []))) - num


def pedestrian_baseline_test():
    run_time = 1000
    step_size = 0.2

    veh_lam_1 = 0.11
    veh_lam_2 = 0.125
    ped_lam_1 = 0.01
    ped_lam_2 = 0.01

    mean_waiting_time_1_list = []
    mean_waiting_time_2_list = []
    mean_veh_waiting_time_list = []
    mean_ped_waiting_time_list = []
    mean_waiting_time_all_list = []

    for demand_scale in np.arange(1, 1.5, 1):
        tmp_1 = []
        tmp_2 = []
        tmp_3 = []
        tmp_4 = []
        tmp_5 = []

        # run 20 times per par
        for iter_num in range(5):
            mean_waiting_time_1, mean_waiting_time_2, mean_veh_waiting_time, mean_ped_waiting_time, mean_waiting_time_all \
                = one_iter_ped_fix_policy(run_time, step_size, [veh_lam_1, veh_lam_2, ped_lam_1, ped_lam_2],
                                          demand_scale)

            tmp_1.append(mean_waiting_time_1)
            tmp_2.append(mean_waiting_time_2)
            tmp_3.append(mean_veh_waiting_time)
            tmp_4.append(mean_ped_waiting_time)
            tmp_5.append(mean_waiting_time_all)

        mean_waiting_time_1_list.append(np.mean(tmp_1))
        mean_waiting_time_2_list.append(np.mean(tmp_2))
        mean_veh_waiting_time_list.append(np.mean(tmp_3))
        mean_ped_waiting_time_list.append(np.mean(tmp_4))
        mean_waiting_time_all_list.append(np.mean(tmp_5))

        print("for demand scale " + str(demand_scale))
        print("mean_waiting_time_veh_1: " + str(mean_waiting_time_1_list))
        print("mean_waiting_time_veh_2: " + str(mean_waiting_time_2_list))
        print("mean_veh_waiting_time: " + str(mean_veh_waiting_time_list))
        print("mean_ped_waiting_time: " + str(mean_ped_waiting_time_list))
        print("mean_waiting_time: " + str(mean_waiting_time_all_list))
        print("===================================")


# baseline policy: default no traffic light(horizontal with higher priority);
# any person wait for more than 1 sec, light would on
def check_baseline_policy():
    wait = [False, False]
    for wa in WALKINGAREAS:
        peds = traci.edge.getLastStepPersonIDs(wa)
        for ped in peds:
            if traci.person.getWaitingTime(ped) >= 1:
                if (not wait[0]) and (traci.person.getNextEdge(ped) in CROSSING_13):
                    numWaiting = traci.trafficlight.getServedPersonCount(TLID, PEDESTRIAN_GREEN_PHASE_13)
                    # print("%s: pedestrian %s pushes the button to cross 13 (waiting: %s)" %
                    #       (traci.simulation.getTime(), ped, numWaiting))
                    wait[0] = True

                elif (not wait[1]) and (traci.person.getNextEdge(ped) in CROSSING_24):
                    numWaiting = traci.trafficlight.getServedPersonCount(TLID, PEDESTRIAN_GREEN_PHASE_24)
                    # print("%s: pedestrian %s pushes the button to cross 24 (waiting: %s)" %
                    #       (traci.simulation.getTime(), ped, numWaiting))
                    wait[1] = True

                elif wait[0] and wait[1]:
                    return wait
    return wait


# policy 1: push the button when either queue larger than threshold(s_3/s_4) or waiting longer than (theta_3/theta_4) sec
def check_policy_1(s_3, s_4, theta_3, theta_4, print_mode=False):
    waiting_num_13 = 0
    waiting_num_24 = 0

    wait = [False, False]
    numWaiting_13 = traci.trafficlight.getServedPersonCount(TLID, PEDESTRIAN_GREEN_PHASE_13)
    numWaiting_24 = traci.trafficlight.getServedPersonCount(TLID, PEDESTRIAN_GREEN_PHASE_24)
    for wa in WALKINGAREAS:
        peds = traci.edge.getLastStepPersonIDs(wa)
        for ped in peds:
            # ped is to cross 13
            if traci.person.getNextEdge(ped) in CROSSING_13:
                if traci.person.getWaitingTime(ped) >= theta_3:
                    if print_mode:
                        print(
                            "%s: pedestrian %s pushes the button to cross 13 when MAX_WAITING_TIME occurs(waiting: %s)" %
                            (traci.simulation.getTime(), ped, numWaiting_13))
                    return [True, False]
                else:
                    waiting_num_13 += 1
                    if waiting_num_13 > s_3:
                        if print_mode:
                            print(
                                "%s: pedestrian %s pushes the button to cross 13 when LONG_QUEUE occurs(waiting: %s)" %
                                (traci.simulation.getTime(), ped, numWaiting_13))
                        return [True, False]

            # ped is to cross 24
            elif traci.person.getNextEdge(ped) in CROSSING_24:
                if traci.person.getWaitingTime(ped) >= theta_4:
                    if print_mode:
                        print(
                            "%s: pedestrian %s pushes the button to cross 24 when MAX_WAITING_TIME occurs(waiting: %s)" %
                            (traci.simulation.getTime(), ped, numWaiting_24))
                    return [False, True]
                else:
                    waiting_num_24 += 1
                    if waiting_num_24 > s_4:
                        if print_mode:
                            print(
                                "%s: pedestrian %s pushes the button to cross 24 when LONG_QUEUE occurs(waiting: %s)" %
                                (traci.simulation.getTime(), ped, numWaiting_24))
                        return [False, True]

    return wait


def one_iter_ped_fix_policy(run_time, step_size, lam, demand_scale):
    generate_routefile_Veberod("./input/Veberod_intersection_pedestrian.rou.xml", run_time, demand_scale * lam[0],
                               demand_scale * lam[1], False)
    generate_routefile_pedestrian("./input/Veberod_intersection_pedestrian.trip.xml", run_time, demand_scale * lam[2],
                                  demand_scale * lam[3], False)

    queue_length_1 = [0]
    queue_length_2 = [0]
    depart_veh_num_1 = [0]
    depart_veh_num_2 = [0]
    beta_1 = []
    beta_2 = []

    traci.start(
        [sumoBinary, "-c", "./input/Veberod_intersection_pedestrian.sumocfg", "--step-length", str(step_size),
         "--fcd-output.geo", "true", "--fcd-output", "./output/veberod-fcd.xml", "--no-step-log", "--no-warnings"])

    # we start with phase 0 --- 42 green (road 2)
    # <phase duration="200" state="GrGr"/>
    traci.trafficlight.setProgram(TLID, "0")
    traci.trafficlight.setPhase(TLID, 0)
    step = 0.
    total_ped_waiting_time = 0.
    ped_id_set = set()
    wait_condition = [False, False]
    while step < run_time:
        step += step_size
        traci.simulationStep()

        # for wa in WALKINGAREAS:
        #     peds = traci.edge.getLastStepPersonIDs(wa)
        #     for ped in peds:
        #         print(ped)
        #         print(traci.person.getStage(ped))
        #         print(traci.person.getNextEdge(ped))
        #         print(traci.person.getEdges(ped))
        #         print(traci.person.getLaneID(ped))
        #         print(traci.person.getLanePosition(ped))
        #         print('==================')

        # for edge in CROSSING_24:
        #     print(traci.edge.getLastStepPersonIDs(edge))
        #
        # for edge in CROSSING_13:
        #     print(traci.edge.getLastStepPersonIDs(edge))

        # update pedestrian waiting time
        if step > 100:
            # print("ped_waiting_id: " + str(get_waiting_ped_id()))
            total_ped_waiting_time += step_size * sum(get_waiting_ped_num())
            ped_id_set = update_ped_id_set(ped_id_set)

        # update vehicle waiting time
        queue_length_1.append(traci.lanearea.getJamLengthVehicle("det_13"))
        queue_length_2.append(traci.lanearea.getJamLengthVehicle("det_42"))
        # print("queue_length: " + str(queue_length_1[-1]) + "    " + str(queue_length_2[-1]))

        # update depart vehicle number
        tmp1 = len(set(sum(beta_1, [])))
        tmp2 = len(set(sum(beta_2, [])))
        if traci.trafficlight.getPhase(TLID) in VEHICLE_GREEN_PHASE_1:
            beta_1.append(list(traci.inductionloop.getLastStepVehicleIDs("det_13_0")))
            beta_2 = []
            depart_veh_num_1.append(len(set(sum(beta_1, []))) - tmp1)
            depart_veh_num_2.append(0)

        elif traci.trafficlight.getPhase(TLID) in VEHICLE_GREEN_PHASE_2:
            beta_1 = []
            beta_2.append(list(traci.inductionloop.getLastStepVehicleIDs("det_42_0")))
            depart_veh_num_2.append(len(set(sum(beta_2, []))) - tmp2)
            depart_veh_num_1.append(0)

        else:
            beta_1.append(list(traci.inductionloop.getLastStepVehicleIDs("det_13_0")))
            beta_2.append(list(traci.inductionloop.getLastStepVehicleIDs("det_42_0")))
            depart_veh_num_1.append(len(set(sum(beta_1, []))) - tmp1)
            depart_veh_num_2.append(len(set(sum(beta_2, []))) - tmp2)

        if traci.trafficlight.getPhase(TLID) not in [1, 4, 7]:
            wait_condition = check_baseline_policy()
            # wait_condition = check_policy_1(3, 3, 5, 5)
            if wait_condition[0] and wait_condition[1]:
                traci.trafficlight.setPhase(TLID, PEDESTRIAN_GREEN_PHASE_all)
            elif wait_condition[0]:
                traci.trafficlight.setPhase(TLID, PEDESTRIAN_GREEN_PHASE_13)
            elif wait_condition[1]:
                traci.trafficlight.setPhase(TLID, PEDESTRIAN_GREEN_PHASE_24)

    traci.close()
    sys.stdout.flush()

    mean_waiting_time_1 = step_size * sum(queue_length_1[100:]) / (sum(depart_veh_num_1[100:]))
    mean_waiting_time_2 = step_size * sum(queue_length_2[100:]) / (sum(depart_veh_num_2[100:]))

    mean_veh_waiting_time = step_size * (sum(queue_length_1[100:]) + sum(queue_length_2[100:])) / (
            sum(depart_veh_num_1[100:]) + sum(depart_veh_num_2[100:]))
    mean_ped_waiting_time = total_ped_waiting_time / len(ped_id_set)
    mean_waiting_time_all = (total_ped_waiting_time + step_size * sum(queue_length_1[100:]) + step_size * sum(
        queue_length_2[100:])) / (len(ped_id_set) + sum(depart_veh_num_1[100:]) + sum(depart_veh_num_2[100:]))

    # print("ped info: " + str(total_ped_waiting_time) + "   " + str(len(ped_id_set)))
    # print("veh info_1: " + str(step_size * sum(queue_length_1[100:])) + "    " + str(sum(depart_veh_num_1[100:])))
    # print("veh info_2: " + str(step_size * sum(queue_length_2[100:])) + "    " + str(sum(depart_veh_num_2[100:])))

    return mean_waiting_time_1, mean_waiting_time_2, mean_veh_waiting_time, mean_ped_waiting_time, mean_waiting_time_all


def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option("--nogui", action="store_true",
                         default=False, help="run the commandline version of sumo")
    options, args = optParser.parse_args()
    return options


def update_ped_id_set(ped_id_set):
    for wa in WALKINGAREAS:
        peds = traci.edge.getLastStepPersonIDs(wa)
        ped_id_set.update(peds)
    return ped_id_set


def get_largest_ped_id(largest_ped_id):
    for wa in WALKINGAREAS:
        peds = traci.edge.getLastStepPersonIDs(wa)
        for ped in peds:
            tmp = int(ped.split("_")[1])
            if tmp > largest_ped_id:
                largest_ped_id = tmp
    return largest_ped_id


def get_waiting_ped_num():
    total_waiting_num = [0, 0]
    for wa in WALKINGAREAS:
        peds = traci.edge.getLastStepPersonIDs(wa)
        for ped in peds:
            if traci.person.getWaitingTime(ped) > 0:
                if traci.person.getNextEdge(ped) in CROSSING_13:
                    total_waiting_num[0] += 1
                elif traci.person.getNextEdge(ped) in CROSSING_24:
                    total_waiting_num[1] += 1
    return total_waiting_num


def get_waiting_ped_id():
    waiting_ped_id = [[], []]
    for wa in WALKINGAREAS:
        peds = traci.edge.getLastStepPersonIDs(wa)
        for ped in peds:
            if traci.person.getWaitingTime(ped) > 0:
                if traci.person.getNextEdge(ped) in CROSSING_13:
                    waiting_ped_id[0].append(ped)
                elif traci.person.getNextEdge(ped) in CROSSING_24:
                    waiting_ped_id[1].append(ped)
    return waiting_ped_id


def get_crossing_ped_id():
    crossing_ped_id = [[], []]
    for edge in CROSSING_13:
        crossing_ped_id[0] = crossing_ped_id[0] + list(traci.edge.getLastStepPersonIDs(edge))
    for edge in CROSSING_24:
        crossing_ped_id[1] = crossing_ped_id[1] + list(traci.edge.getLastStepPersonIDs(edge))
    return crossing_ped_id


def update_event_time_derivative(idx, step_size, det_veh_num, queue_length, f, alpha_rate, h, theta, d_x, d_tau, \
                                 last_switch, last_push_button, print_mode=False):
    light_switch = False
    d_tau = [0] * len(theta)
    idx_ = 0
    if idx == 0:
        idx_ = 1

    # alpha1=0
    if det_veh_num[idx][-2] > 0 and det_veh_num[idx][-1] == 0 and \
            ((queue_length[idx_][-1] > 0 and not f[idx_]) or
             (queue_length[idx_][-1] == 0 and f[idx] and not f[idx_])):
        light_switch = True
        d_tau = [0] * len(theta)
        if print_mode:
            print("alpha{}=0".format(idx))

    # z1=theta_1_max
    elif last_switch >= theta[idx * 2 + 1] and \
            queue_length[idx_][-1] > 0:
        light_switch = True
        d_tau[idx * 2 + 1] = d_tau[idx * 2 + 1] + 1
        events_count_list[idx * 2 + 1] += 1
        if print_mode:
            print("z{0}=theta_{0}_max".format(idx + 1))

    # z1=theta_1_min
    elif last_switch >= theta[idx * 2] > (last_switch - 1) and \
            ((f[idx] and not f[idx_]) or (
                    queue_length[idx][-1] < theta[6 + idx] and queue_length[idx_][-1] >= theta[6 + idx_])):
        light_switch = True

        d_tau[idx * 2] = d_tau[idx * 2] + 1
        events_count_list[idx * 2] += 1
        if print_mode:
            print("z{0}=theta_{0}_min".format(idx + 1))

    # x1 = s1(a)
    elif queue_length[idx][-2] > theta[idx + 6] >= queue_length[idx][-1] and \
            (last_switch >= theta[idx * 2] and queue_length[idx_][-1] >= theta[6 + idx_]):
        light_switch = True

        if alpha_rate[idx] - h[idx] == 0:
            print("WARNING: alpha_rate_{0} - h{0} == 0 when x{0} = s{0}(a)/ G2R1".format(idx + 1))
        else:
            tmp = -np.array(d_x[idx])
            tmp[6 + idx] += 1
            d_tau = tmp / (alpha_rate[idx] - h[idx])

        events_count_list[idx + 6] += 1
        if print_mode:
            print("x{0} = s{0}(a) G2R{0}".format(idx + 1))

    # x2 = s2(b)
    elif queue_length[idx_][-2] < theta[idx_ + 6] <= queue_length[idx_][-1] and \
            (last_switch >= theta[idx * 2] and queue_length[idx][-1] < theta[idx + 6]):
        light_switch = True

        if alpha_rate[idx_] == 0:
            alpha_rate[idx_] = 0.1
            if print_mode:
                print("WARING: alpha_rate{0}==0 when x{0} = s{0}(b), use alternative {1}".format(idx_ + 1, 0.1))
        tmp = - np.array(d_x[idx_])
        tmp[6 + idx_] += 1
        d_tau = tmp / alpha_rate[idx_]
        events_count_list[6 + idx_] += 1
        if print_mode:
            print("x{0} = s{0}(b) G2R{0}".format(idx_ + 1))

    # x4=s4(a) --> f2=0
    elif queue_length[idx_ + 2][-2] > theta[idx_ + 8] >= queue_length[idx_ + 2][-1] and \
            ((det_veh_num[idx][-1] == 0 and queue_length[idx_][-1] == 0 and f[idx]) or \
             (det_veh_num[idx][-1] == 0 and queue_length[idx_][-1] > 0) or \
             (det_veh_num[idx][-1] > 0 and queue_length[idx_][-1] > 0 and last_switch > theta[idx * 2] and not f[
                 idx])):
        light_switch = True
        if alpha_rate[idx_ + 2] - h[idx_ + 2] == 0:
            print("WARNING: alpha_rate{0} - h{0} == 0 when x{0} = s{0}(a)/ G2R{1}".format(idx_ + 3, idx + 1))
        else:
            tmp = - np.array(d_x[idx_ + 2])
            tmp[idx_ + 8] += 1
            d_tau = tmp / (alpha_rate[idx_ + 2] - h[idx_ + 2])

        events_count_list[idx_ + 8] += 1
        if print_mode:
            print("x{0} = s{0}(a) G2R{1}".format(idx_ + 3, idx + 1))

    # x3 = s3(b) --> f1=1
    elif queue_length[idx + 2][-2] < theta[idx + 8] <= queue_length[idx + 2][-1] and \
            ((det_veh_num[idx][-1] > 0 and last_switch >= theta[idx * 2] and not f[idx_]) or \
             (det_veh_num[idx][-1] == 0 and queue_length[idx_][-1] == 0 and not f[idx_])):
        light_switch = True

        if alpha_rate[idx + 2] == 0:
            alpha_rate[idx + 2] = 0.1
            if print_mode:
                print("WARING: alpha_rate_{0}==0 when x{0} = s{0}(b), use alternative 0.1".format(idx + 3))

        tmp = - np.array(d_x[idx + 2])
        tmp[idx + 8] += 1
        d_tau = tmp / alpha_rate[idx + 2]
        events_count_list[idx + 8] += 1
        if print_mode:
            print("x{0} = s{0}(b) G2R{1}".format(idx + 3, idx + 1))

    #   y1 = theta3 --> f1 = 1
    elif last_push_button >= theta[idx + 4] > (last_push_button - step_size) and \
            ((det_veh_num[idx][-1] > 0 and last_switch >= theta[idx * 2] and not f[idx_]) or \
             (det_veh_num[idx][-1] == 0 and queue_length[idx_][-1] == 0 and not f[idx_])):

        light_switch = True
        d_tau = np.zeros(len(theta))
        d_tau[idx + 4] = 1
        events_count_list[idx + 4] += 1
        if print_mode:
            print("y{0} = theta{1}  G2R{0}".format(idx + 1, idx + 3))

    else:
        d_tau = np.zeros(len(theta))

    return light_switch, d_tau


def update_state_derivative(green_1, step_size, last_switch, NEP, queue_length, alpha_rate, d_x, d_tau, \
                            last_switch_dtau, h, push_button, print_mode=False):
    green = [green_1, not green_1, not green_1, green_1]

    for idx in range(4):
        # S_1
        if last_switch <= step_size and (not green[idx]) and (not NEP[idx]) and queue_length[idx][-1] > 0:
            NEP[idx] = True
            if print_mode:
                print("event at S_{0} with G2R_{0}".format(idx + 1))
            d_x[idx] = - alpha_rate[idx] * np.array(last_switch_dtau)
            if idx >= 2:
                push_button[idx - 2] = True

        elif last_switch != 0. and (not NEP[idx]) and queue_length[idx][-1] > 0:
            NEP[idx] = True
            d_x[idx] = [0] * 10
            if print_mode:
                print("event at S_{} NOT with light switch".format(idx + 1))
            if idx >= 2 and not green[idx]:
                push_button[idx - 2] = True

        # E_1
        elif NEP[idx] and (queue_length[idx][-1] == 0):
            NEP[idx] = False
            d_x[idx] = [0] * 10
            if print_mode:
                print("event at E_{}".format(idx + 1))

        # EP_1
        elif (not NEP[idx]) and queue_length[idx][-1] == 0:
            d_x[idx] = [0] * 10
            if print_mode:
                print("event at EP_{}".format(idx + 1))

        # G2R1
        elif last_switch == 0 and not green[idx]:
            d_x[idx] = d_x[idx] - h[idx] * np.array(d_tau)
            if print_mode:
                print("event at G2R_{}".format(idx + 1))

        # R2G1
        elif last_switch == 0 and green[idx]:
            d_x[idx] = d_x[idx] + h[idx] * np.array(d_tau)
            if print_mode:
                print("event at R2G_{}".format(idx + 1))

        else:
            if print_mode:
                if NEP[idx]:
                    print("d_x{0} not change, in NEP_{0}".format(idx + 1))
                else:
                    print("d_x{0} not change, in EP_{0}".format(idx + 1))

    return d_x, NEP, push_button


def update_result(d_L_list, performance_list, d_L, queue_length, depart_veh_num, depart_ped_num, par_list,
                  step_per_iter, last_par_update_step_size, iter_num, max_delta_theta, max_delta_s,print_mode):
    '''par_list = [[i] for i in initial_par]'''

    mean_waiting_time_1 = sum(queue_length[0][-step_per_iter:]) / sum(depart_veh_num[0][-step_per_iter:])
    mean_waiting_time_2 = sum(queue_length[1][-step_per_iter:]) / sum(depart_veh_num[1][-step_per_iter:])
    mean_veh_waiting_time = (sum(queue_length[0][-step_per_iter:]) + sum(queue_length[1][-step_per_iter:])) / \
                            (sum(depart_veh_num[0][-step_per_iter:]) + sum(depart_veh_num[1][-step_per_iter:]))
    mean_ped_waiting_time = (sum(queue_length[2][-step_per_iter:]) + sum(queue_length[3][-step_per_iter:])) / \
                            (sum(depart_ped_num[0][-step_per_iter:]) + sum(depart_ped_num[1][-step_per_iter:]))
    mean_waiting_time_total = sum([sum(i[-step_per_iter:]) for i in queue_length]) / \
                              (sum([sum(i[-step_per_iter:]) for i in depart_veh_num]) + sum(
                                  [sum(i[-step_per_iter:]) for i in depart_ped_num]))

    performance_list.append(
        [mean_waiting_time_1, mean_waiting_time_2, mean_veh_waiting_time, mean_ped_waiting_time,
         mean_waiting_time_total])

    d_L_list.append(d_L)


    # update parameters
    delta_theta_max = max_delta_theta * (1/iter_num) ** 0.3
    delta_s_max = max_delta_s * (1/iter_num) ** 0.3
    par_update_step_size = get_par_update_step_size(last_par_update_step_size, d_L_list, 0.1, delta_theta_max, 0.1, delta_s_max)

    # print("par_update_step_size: " + str(par_update_step_size))
    # theta_1_min
    par_list[0].append(max(0.1, par_list[0][-1] - par_update_step_size * d_L[0]))
    # theta_1_max
    par_list[1].append(max(par_list[0][-1], par_list[1][-1] - par_update_step_size * d_L[1]))
    # theta_2_min
    par_list[2].append(max(0.1, par_list[2][-1] - par_update_step_size * d_L[2]))
    # theta_2_max
    par_list[3].append(max(par_list[2][-1], par_list[3][-1] - par_update_step_size * d_L[3]))
    # theta_3
    par_list[4].append(max(0.1, par_list[4][-1] - par_update_step_size * d_L[4]))
    # theta_4
    par_list[5].append(max(0.1, par_list[5][-1] - par_update_step_size * d_L[5]))
    # s_1
    par_list[6].append(max(0.1, par_list[6][-1] - par_update_step_size * d_L[6]))
    # s_2
    par_list[7].append(max(0.1, par_list[7][-1] - par_update_step_size * d_L[7]))
    # s_3
    par_list[8].append(max(0.1, par_list[8][-1] - par_update_step_size * d_L[8]))
    # s_4
    par_list[9].append(max(0.1, par_list[9][-1] - par_update_step_size * d_L[9]))

    theta = [par[-1] for par in par_list]

    return theta, d_L_list, performance_list, par_list, par_update_step_size


#  check whether to update parameter; update only if gradient_flag and (queue_length_flag or alpha_rate_flag )
def update_par_check(alpha_rate_list, queue_length, step_per_iter, step_size, gradient_stable_flag=True):
    gradient_flag = not gradient_stable_flag
    queue_length_flag = False
    alpha_rate_flag = False

    if step_per_iter < 360 / step_size:
        return False
    if step_per_iter > 3600 / step_size:
        return True

    # compare queue length of last 6-3 min, and last 3 min
    ql_tmp_pre = sum([sum(l[int(-(60 * 6) / step_size):int(-(60 * 3) / step_size)]) for l in queue_length])
    ql_tmp_cur = sum([sum(l[int(-(60 * 3) / step_size):]) for l in queue_length])  # total queue num for last minute
    if (ql_tmp_cur - ql_tmp_pre) / ql_tmp_pre > 0.5:
        queue_length_flag = True

    # compare arrival rate of each flow
    for i in range(len(alpha_rate_list[0])):
        ar_tmp_pre = np.mean(
            [alpha[i] for alpha in alpha_rate_list[int(-(60 * 6) / step_size):int(-(60 * 3) / step_size)]])
        ar_tmp_cur = np.mean([alpha[i] for alpha in alpha_rate_list[int(-(60 * 3) / step_size):]])
        if abs(ar_tmp_cur - ar_tmp_pre) / ar_tmp_pre > 0.5:
            alpha_rate_flag = True
            break

    update_par_flag = gradient_flag and (queue_length_flag or alpha_rate_flag)

    return update_par_flag


def check_gradient_stability(d_L_list):
    """stable when last 3 gradient of each par contain sign change or avg of last 3 gradient small enough"""
    # gradient_stable_flag = False
    flag = 0
    if len(d_L_list) < 3:
        return False

    transposed_d_L_list = [list(i) for i in zip(*d_L_list)]
    for idx in range(len(transposed_d_L_list)):
        d_par = transposed_d_L_list[idx]
        # if exist 3 par that does not contain sign change and not small enouch -->not stable
        if (all(i < 0 for i in d_par[-3:]) or all(i > 0 for i in d_par[-3:])) and np.mean(d_par[-3:]) > 0.01:
            flag += 1
        if flag > 2:
            return False
    return True


def get_par_update_step_size(last_step_size, d_L_list, delta_theta_min, delta_theta_max, delta_s_min, delta_s_max):
    tmp = float('inf')
    transposed_d_L_list = [list(i) for i in zip(*d_L_list)]

    d_L_theta = np.array(d_L_list[-1][0:6])
    d_L_s = np.array(d_L_list[-1][6:])

    max_dL_theta = abs(max(d_L_theta, key=abs))
    # min_dL_theta = abs(min(d_L_theta[d_L_theta != 0], key=abs))
    max_dL_s = abs(max(d_L_s, key=abs))
    # min_dL_s = abs(min(d_L_s[d_L_s != 0], key=abs))
    # analyze for each parameter
    for idx in range(len(transposed_d_L_list)):
        d_par = transposed_d_L_list[idx]
        if d_par[-1] == 0:
            continue
        # contains sign change in last 3
        if any(i < 0 for i in d_par[-4:]) and any(i > 0 for i in d_par[-4:]):
            tmp = last_step_size / 1.5
            # make sure not greater than max allowable par change
            if max_dL_theta * tmp > delta_theta_max:
                tmp = min(tmp, delta_theta_max / max_dL_theta)

            if max_dL_s * tmp > delta_s_max:
                tmp = min(tmp, delta_s_max / max_dL_s)

            # tmp = min(delta_theta_min / max_dL_theta, delta_s_min / max_dL_s)
            return tmp


    #   if no sign change in last 3 for current par (continue optimization)
    if max_dL_s != 0:
        tmp = min(delta_theta_max/max_dL_theta, delta_s_max/max_dL_s)
    else:
        tmp = delta_theta_max / max_dL_theta

    #     # for theta parameter (green time threshold) [theta_min, theta_max]s
    #     if idx <= 5:
    #         if abs(last_step_size * d_par[-1]) < delta_theta_min:
    #             tmp = min(tmp, abs(delta_theta_min / d_par[-1]))
    #         elif abs(last_step_size * d_par[-1]) > delta_theta_max:
    #             tmp = min(tmp, abs(delta_theta_max / d_par[-1]))
    #     else:
    #         if abs(last_step_size * d_par[-1]) < delta_s_min:
    #             tmp = min(tmp, abs(delta_s_min / d_par[-1]))
    #         elif abs(last_step_size * d_par[-1]) > delta_s_max:
    #             tmp = min(tmp, abs(delta_s_max / d_par[-1]))
    # if tmp == float('inf'):
    #     tmp = last_step_size
    return tmp


# time driven
def one_iter_ped_adaptive(theta, lam, demand_scale, step_size, run_time, fix_seed, print_mode, sumoBinary):
    """
         theta: [theta_1_min, theta_1_max, theta_2_min, theta_2_max, theta_3, theta_4, s_1, s_2, s_3, s_4]
         lam: [lam_1_veh, lam_2_veh, lam_1_ped, lam_2_ped]
         step_size: time passed for each simulation step
         """

    generate_routefile_Veberod("./input/Veberod_intersection_pedestrian.rou.xml", run_time, demand_scale[0] * lam[0],
                               demand_scale[0] * lam[1], fix_seed)
    generate_routefile_pedestrian("./input/Veberod_intersection_pedestrian.trip.xml", run_time, demand_scale[1] * lam[2],
                                  demand_scale[1] * lam[3], fix_seed)

    # one run renewal
    last_switch_dtau = [0] * len(theta)

    # derivatives
    d_x = [[0] * len(theta) for _ in range(4)]  # d_x[0] = [0,0,0,0,0..]
    d_x_total = [[0] * len(theta) for _ in range(4)]
    d_tau = [0] * len(theta)
    d_L = [0] * len(theta)

    NEP = [False] * 4

    # arrival and deprat rate
    alpha = [[] for _ in range(4)]  # list of veh/ped ids enter this road in the last time step [[],[],[],[]]
    beta = [[] for _ in range(4)]  # list of veh/ped ids passing the cross in the last time step
    jam = [[] for _ in range(4)]  # list of vehicle/ped ids in the queue in the last time step
    alpha_rate = [0] * 4
    beta_rate = [0] * 4

    # free departure rate
    h = [0.55] * 4

    det_veh_num = [[0] for _ in range(2)]  # vehicle num at each time step  [[0], [0]]
    queue_length = [[0] for _ in range(4)]  # [[0], [0], [0], [0]]
    depart_veh_num = [[0] for _ in range(2)]
    depart_ped_num = [[0] for _ in range(2)]

    traci.start(
        [sumoBinary, "-c", "./input/Veberod_intersection_pedestrian.sumocfg", "--step-length", str(step_size),
         "--fcd-output.geo", "true", "--fcd-output", "./output/veberod-fcd.xml", "--no-step-log", "--no-warnings"])

    # we start with phase 0 --- 42 green (road 2)
    # <phase duration="200" state="grgrrGrG"/>
    traci.trafficlight.setProgram(TLID, "1")
    traci.trafficlight.setPhase(TLID, 0)
    green_1 = False
    push_button = [False, False]

    step = 0.
    last_switch = 0.  # time passed since last switch
    last_push_button = 0.  # time since last push of button

    while step < run_time:
        traci.simulationStep()
        last_switch += step_size
        step += step_size
        if True in push_button:
            last_push_button += step_size

        # pedestrian condition
        waiting_ped_id = get_waiting_ped_id()
        crossing_ped_id = get_crossing_ped_id()

        # update state list
        det_veh_num[0].append(traci.lanearea.getLastStepVehicleNumber("det_13"))
        det_veh_num[1].append(traci.lanearea.getLastStepVehicleNumber("det_42"))
        queue_length[0].append(traci.lanearea.getJamLengthVehicle("det_13"))
        queue_length[1].append(traci.lanearea.getJamLengthVehicle("det_42"))
        queue_length[2].append(len(waiting_ped_id[0]))
        queue_length[3].append(len(waiting_ped_id[1]))
        # print([queue[-1] for queue in queue_length])

        # update pedestrian state
        f = [False, False]
        if queue_length[2][-1] >= theta[8]:
            f[0] = True
        if queue_length[3][-1] >= theta[9]:
            f[1] = True
        if green_1 and last_push_button >= theta[4]:
            f[0] = True
        if (not green_1) and last_push_button >= theta[5]:
            f[1] = True

        # update departure rate
        tmp = [len(set(sum(i, []))) for i in beta]
        if green_1:
            beta[0].append(list(traci.inductionloop.getLastStepVehicleIDs("det_13_0")))
            beta[1] = []
            beta[2] = []
            beta[3].append(crossing_ped_id[1])
            depart_veh_num[0].append(len(set(sum(beta[0], []))) - tmp[0])
            depart_veh_num[1].append(0)
            depart_ped_num[0].append(0)
            depart_ped_num[1].append(len(set(sum(beta[3], []))) - tmp[3])

        else:
            beta[0] = []
            beta[1].append(list(traci.inductionloop.getLastStepVehicleIDs("det_42_0")))
            beta[2].append(crossing_ped_id[0])
            beta[3] = []
            depart_veh_num[0].append(0)
            depart_veh_num[1].append(len(set(sum(beta[1], []))) - tmp[1])
            depart_ped_num[0].append(len(set(sum(beta[2], []))) - tmp[2])
            depart_ped_num[1].append(0)

        if len(beta[0]) > 10. / step_size:
            beta[0].pop(0)
            beta[3].pop(0)
        if len(beta[1]) > 10. / step_size:
            beta[1].pop(0)
            beta[2].pop(0)

        beta_rate = [len(set(sum(i, []))) / (10. / step_size) for i in beta]

        # update arrival rate

        # alpha rate = the rate of join queue rather than the rate of appearing on the road
        if queue_length[0][-1] > 0:
            pure_veh_id_13 = list(set(traci.lanearea.getLastStepVehicleIDs("det_13")) - set((beta[0] or ["any"])[-1]))
            jam[0].append(get_jam_veh_ids(pure_veh_id_13, queue_length[0][-1]))
            if len(jam[0]) > 11. / step_size:
                jam[0].pop(0)
            alpha_rate[0] = get_increased_jam_num(jam[0]) / (10. / step_size)
        else:
            jam[0] = []
            alpha_rate[0] = beta_rate[0]

        if queue_length[1][-1] > 0:
            pure_veh_id_42 = list(set(traci.lanearea.getLastStepVehicleIDs("det_42")) - set((beta[1] or ["any"])[-1]))
            jam[1].append(get_jam_veh_ids(pure_veh_id_42, queue_length[1][-1]))
            if len(jam[1]) > 11. / step_size:
                jam[1].pop(0)
            alpha_rate[1] = get_increased_jam_num(jam[1]) / (10. / step_size)
        else:
            jam[1] = []
            alpha_rate[1] = beta_rate[1]

        if queue_length[2][-1] > 0:
            jam[2].append(waiting_ped_id[0])
            if len(jam[2]) > (11. / step_size):
                jam[2].pop(0)
            alpha_rate[2] = get_increased_jam_num(jam[2]) / (10. / step_size)
        else:
            jam[2] = []
            alpha_rate[2] = beta_rate[2]

        if queue_length[3][-1] > 0:
            jam[3].append(waiting_ped_id[1])
            if len(jam[3]) > 11. / step_size:
                jam[3].pop(0)
            alpha_rate[3] = get_increased_jam_num(jam[3]) / (10. / step_size)
        else:
            jam[3] = []
            alpha_rate[3] = beta_rate[3]


        # update event time derivative
        idx = int(not green_1)
        light_switch, d_tau = update_event_time_derivative(idx, step_size, det_veh_num, queue_length, f, alpha_rate, h,
                                                           theta, d_x, d_tau, last_switch, last_push_button, print_mode)

        if light_switch:
            traci.trafficlight.setPhase(TLID, idx)
            last_switch = 0.
            last_push_button = 0.
            push_button = [False, False]
            green_1 = not green_1

        if not np.all(np.array(d_tau) == 0):
            last_switch_dtau = d_tau

        if print_mode:
            print("time:" + str(step))
            print("last_switch:" + str(last_switch))
            print("last_push_button: " + str(last_push_button))
            print("green in 13:" + str(green_1))
            # print("jam_length_vehicle:  " + str(traci.lanearea.getJamLengthVehicle("det_13")))
            # print("last_step_veh_num:  " + str(traci.lanearea.getLastStepVehicleNumber("det_13")))
            # print("alpha_1: "+str(np.mean(alpha_1)))
            # print("beta_1: "+str(np.mean(beta_1)))
            print("d_tau:" + str(d_tau))

        # update state derivative
        d_x, NEP, push_button = update_state_derivative(green_1, step_size, last_switch, NEP, queue_length, alpha_rate,
                                                        d_x, d_tau, last_switch_dtau, h, push_button, print_mode=False)

        d_L = d_L + np.sum(np.array(d_x), 0)
        d_x_total = np.array(d_x_total) + np.array(d_x)
        if print_mode:
            print('----------------')
            print("d_x: " + str(d_x))
            print("d_x_total: " + str(d_x_total))
            print('===========================================================================================')

    mean_waiting_time_1 = sum(queue_length[0][100:]) / sum(depart_veh_num[0][100:])
    mean_waiting_time_2 = sum(queue_length[1][100:]) / sum(depart_veh_num[1][100:])
    mean_veh_waiting_time = (sum(queue_length[0][100:]) + sum(queue_length[1][100:])) / \
                            (sum(depart_veh_num[0][100:]) + sum(depart_veh_num[1][100:]))
    mean_ped_waiting_time = (sum(queue_length[2][100:]) + sum(queue_length[3][100:])) / \
                            (sum(depart_ped_num[0][100:]) + sum(depart_ped_num[1][100:]))
    mean_waiting_time_total = sum([sum(i[100:]) for i in queue_length]) / \
                              (sum([sum(i[100:]) for i in depart_veh_num]) + sum(
                                  [sum(i[100:]) for i in depart_ped_num]))

    traci.close()
    sys.stdout.flush()

    return np.array(d_L) * 1.0 / run_time, mean_waiting_time_1, mean_waiting_time_2, \
           mean_veh_waiting_time, mean_ped_waiting_time, mean_waiting_time_total


# time driven
def repeat_iters(zk, simulation_report, par, lam, demand_scale, step_size, run_time, iters_per_par, fix_seed=False, print_mode=False):
    """repeat the test with same parameter, but different random seed"""

    d_L_list = []
    performance_list = []

    # fix_seed = False
    # if iters_per_par == 1:
    #     fix_seed = True

    iters = 0
    while iters < iters_per_par:

        (zookeeper_report_status, node_stat) = zk.get("TLC/SimulationReport/%s/reportStatus" % simulation_report.get("pk"))
        print("zookeeper_report_status: %s" % zookeeper_report_status)

        if zookeeper_report_status != 'Stop':
            iters += 1
            d_L, mean_waiting_time_1, mean_waiting_time_2, mean_veh_waiting_time, mean_ped_waiting_time, \
            mean_waiting_time_total = one_iter_ped_adaptive(par, lam, demand_scale, step_size, run_time, fix_seed,
                                                            print_mode, sumoBinary)
            # exclude the outliers
            if np.any(np.array(d_L) > 1000) or np.any(np.array(d_L) < -1000):
                print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                print("extreme d_L  " + str(d_L) + " ----- delete")
                print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                iters -= 1
                continue
            d_L_list.append(d_L)
            performance_list.append([mean_waiting_time_1, mean_waiting_time_2, mean_veh_waiting_time, mean_ped_waiting_time,
                                     mean_waiting_time_total])
    
            # print("d_L:")
            # pprint(d_L_list[-1])
            # print("for par " + str(par) + ":")
            # print(performance_list[-1])
            # if iters == iters_per_par:
            #     print("mean_waiting_time_total ----" + str([i[-1] for i in performance_list]))
            #     print("avg: " + str(np.mean([i[-1] for i in performance_list])))
            #     print("==============")

    return np.mean(d_L_list, 0), np.mean(performance_list, 0)


# time driven main function: time driven parameter updating with parallel running
def ipa_gradient_method_pedestrian(zk, producer, simulation_report, initial_par, lam, demand_scale, step_size, run_time,
                                   total_iter_num, iters_per_par, print_mode):
    par_list_output = None
    performance_list_output = None

    kafka_topic_sumo_run_report = os.environ.get('KAFKA_TOPIC_SUMO_RUN_REPORT') or "smartvillage-sumo-run-report"
    kafka_topic_sumo_run_iter = os.environ.get('KAFKA_TOPIC_SUMO_RUN_ITER') or "smartvillage-sumo-run-iter"
    try:
    
        global events_count_list
        avg_events_count_list = []
        par_update_step_size = 30
        """ run ipa method to update par"""
        lam = np.array(lam)/60.
        par_list = [[i] for i in initial_par]
    
        d_L_list = []
        performance_list = []
    
        iter_num = 0
    
        while iter_num < total_iter_num:
            body = { "iter_num": iter_num, "simulation_report": simulation_report }
            producer.send(kafka_topic_sumo_run_iter, json.dumps(body).encode('utf-8'))
            iter_num += 1


# time driven function per iter: time driven parameter updating with parallel running
def ipa_gradient_method_pedestrian_iter(zk, producer, iter_num, simulation_report, initial_par, lam, demand_scale, step_size, run_time,
                                   total_iter_num, iters_per_par, print_mode):
    try:
        (zookeeper_report_status, node_stat) = zk.get("TLC/SimulationReport/%s/reportStatus" % simulation_report.get("pk"))
        print("zookeeper_report_status: %s" % zookeeper_report_status)

        if zookeeper_report_status != 'Stop':
    
            d_L, performance = repeat_iters(zk, simulation_report, [par[-1] for par in par_list], lam, demand_scale, step_size, run_time,
                                            iters_per_par)
    
            # update performance
            performance_list.append(performance)
            d_L_list.append(d_L)
            print('****************************************************************************')
            # print("dL: " + str(d_L))
            print("parameters:")
            par_list_output = []
            for par in par_list:
                par_list_output.append([round(i, 3) for i in par])
                # print(str([round(i, 3) for i in par]) + ",")
            print(par_list_output)
    
            print("performance:")
            # print([round(i, 3) for i in [i[-1] for i in performance_list]])
            performance_list_output = []
            for j in range(5):
                performance_list_output.append([round(i, 3) for i in [i[j] for i in performance_list]])
            print(performance_list_output)
    
    
            print("events count:")
            print(events_count_list)
            avg_events_count_list.append(events_count_list)
            events_count_list = [0]*10
    
            generate_plots(par_list_output, performance_list_output)
    
            # update parameters
            par_update_step_size = get_par_update_step_size(par_update_step_size, d_L_list, 1., 4., 0.5, 1.5)
            # print("par_update_step_size: " + str(par_update_step_size))
    
            # theta_1_min
            par_list[0].append(max(0.1, par_list[0][-1] - par_update_step_size * d_L[0]))
            # theta_1_max
            par_list[1].append(max(par_list[0][-1], par_list[1][-1] - par_update_step_size * d_L[1]))
            # theta_2_min
            par_list[2].append(max(0.1, par_list[2][-1] - par_update_step_size * d_L[2]))
            # theta_2_max
            par_list[3].append(max(par_list[2][-1], par_list[3][-1] - par_update_step_size * d_L[3]))
            # theta_3
            par_list[4].append(max(0.1, par_list[4][-1] - par_update_step_size * d_L[4]))
            # theta_4
            par_list[5].append(max(0.1, par_list[5][-1] - par_update_step_size * d_L[5]))
            # s_1
            par_list[6].append(max(0.1, par_list[6][-1] - par_update_step_size * d_L[6]))
            # s_2
            par_list[7].append(max(0.1, par_list[7][-1] - par_update_step_size * d_L[7]))
            # s_3
            par_list[8].append(max(0.1, par_list[8][-1] - par_update_step_size * d_L[8]))
            # s_4
            par_list[9].append(max(0.1, par_list[9][-1] - par_update_step_size * d_L[9]))
    
            print('****************************************************************************')
            result = { "pk": simulation_report.get("pk"), "setUpdatedParameters": par_list_output, "setUpdatedPerformance": performance_list_output, "setReportStatus": "Running" }
            producer.send(kafka_topic_sumo_run_report, json.dumps(result).encode('utf-8'))
        print('avg event counts:')
        print(np.mean(avg_events_count_list,0))
    except Exception as e:
        ex_type, ex_value, ex_traceback = sys.exc_info()
        # Extract unformatter stack traces as tuples
        trace_back = traceback.extract_tb(ex_traceback)
    
        # Format stacktrace
        stack_trace = list()
    
        for trace in trace_back:
            stack_trace.append("File : %s , Line : %d, Func.Name : %s, Message : %s" % (trace[0], trace[1], trace[2], trace[3]))
    
        print("%s occured processing a message on %s topic: %s\n%s" % (ex_type.__name__, kafka_topic_sumo_run_report, ex_value, '\n'.join(stack_trace)))
    return par_list_output, performance_list_output


def reset_par(theta, det_veh_num, queue_length, depart_veh_num, depart_ped_num):
    last_switch_dtau = [0] * len(theta)
    # d_x = [[0] * len(theta) for _ in range(4)]  # d_x[0] = [0,0,0,0,0..] , 4 flows
    # d_x_total = []  # d_L of each step; sum(d_x_total)=d_L
    d_tau = [0] * len(theta)
    d_L = [0] * len(theta)
    alpha_rate_list = []
    det_veh_num = [i[-5:] for i in det_veh_num]
    queue_length = [i[-5:] for i in queue_length]
    depart_veh_num = [i[-5:] for i in depart_veh_num]
    depart_ped_num = [i[-5:] for i in depart_ped_num]
    return last_switch_dtau, d_tau, d_L, alpha_rate_list, det_veh_num, queue_length, depart_veh_num, depart_ped_num


# event driven sequential: whole time
def ped_adaptive_sequential(initial_par, step_size, run_time, print_mode):
    # initialization
    d_L_list = []  # append d_L at each iteration
    d_x_total = []  # d_L of each step; sum(d_x_total)=d_L
    performance_list = []  # append[mean_waiting_time_1,mean_waiting_time_2,...] at each iteration
    par_list = [[i] for i in initial_par]
    theta = initial_par
    update_time = []
    max_change_list = []
    NEP = [False] * 4

    # arrival and deprat rate
    alpha = [[] for _ in range(4)]  # list of veh/ped ids enter this road in the last time step [[],[],[],[]]
    beta = [[] for _ in range(4)]  # list of veh/ped ids passing the cross in the last time step
    jam = [[] for _ in range(4)]  # list of vehicle/ped ids in the queue in the last time step
    alpha_rate = [0] * 4
    beta_rate = [0] * 4
    h = [0.55] * 4  # free departure rate
    d_x = [[0] * len(theta) for _ in range(4)]  # d_x[0] = [0,0,0,0,0..] , 4 flows

    det_veh_num = [[0] for _ in range(2)]  # vehicle num at each time step  [[0], [0]]
    queue_length = [[0] for _ in range(4)]  # [[0], [0], [0], [0]]
    depart_veh_num = [[0] for _ in range(2)]
    depart_ped_num = [[0] for _ in range(2)]

    # one run renewal
    last_switch_dtau, d_tau, d_L, alpha_rate_list, det_veh_num, queue_length, depart_veh_num, \
    depart_ped_num = reset_par(theta, det_veh_num, queue_length, depart_veh_num, depart_ped_num)

    traci.start(
        [sumoBinary, "-c", "./input/Veberod_intersection_pedestrian.sumocfg", "--step-length", str(step_size),
         "--fcd-output.geo", "true", "--fcd-output", "./output/veberod-fcd.xml", "--no-step-log", "--no-warnings"])

    # iteration initialization
    # we start with phase 0 --- 42 green (road 2)
    # <phase duration="200" state="grgrrGrG"/>
    traci.trafficlight.setProgram(TLID, "1")
    traci.trafficlight.setPhase(TLID, 0)
    green_1 = False
    push_button = [False, False]
    step = 0.
    iter_num = 0

    last_switch = 0.  # time passed since last switch
    last_push_button = 0.  # time since last push of button
    gradient_stable_flag = False
    step_per_iter = 0  # update 1 when one data slice recorded
    par_update_step_size = 30
    max_delta_theta = 3
    max_delta_s = 2

    while step < run_time:
        traci.simulationStep()
        last_switch += step_size
        step += step_size
        step_per_iter += 1
        if True in push_button:
            last_push_button += step_size

        # pedestrian condition
        waiting_ped_id = get_waiting_ped_id()
        crossing_ped_id = get_crossing_ped_id()

        # update state list
        det_veh_num[0].append(traci.lanearea.getLastStepVehicleNumber("det_13"))
        det_veh_num[1].append(traci.lanearea.getLastStepVehicleNumber("det_42"))
        queue_length[0].append(traci.lanearea.getJamLengthVehicle("det_13"))
        queue_length[1].append(traci.lanearea.getJamLengthVehicle("det_42"))
        queue_length[2].append(len(waiting_ped_id[0]))
        queue_length[3].append(len(waiting_ped_id[1]))
        # print([queue[-1] for queue in queue_length])

        # update pedestrian state
        f = [False, False]
        if green_1 and (queue_length[2][-1] >= theta[8] or last_push_button >= theta[4]):
            f[0] = True
        if (not green_1) and (queue_length[3][-1] >= theta[9] or last_push_button >= theta[5]):
            f[1] = True

        # update departure rate
        tmp = [len(set(sum(i, []))) for i in beta]
        if green_1:
            beta[0].append(list(traci.inductionloop.getLastStepVehicleIDs("det_13_0")))
            beta[1] = []
            beta[2] = []
            beta[3].append(crossing_ped_id[1])
            depart_veh_num[0].append(len(set(sum(beta[0], []))) - tmp[0])
            depart_veh_num[1].append(0)
            depart_ped_num[0].append(0)
            depart_ped_num[1].append(len(set(sum(beta[3], []))) - tmp[3])
        else:
            beta[0] = []
            beta[1].append(list(traci.inductionloop.getLastStepVehicleIDs("det_42_0")))
            beta[2].append(crossing_ped_id[0])
            beta[3] = []
            depart_veh_num[0].append(0)
            depart_veh_num[1].append(len(set(sum(beta[1], []))) - tmp[1])
            depart_ped_num[0].append(len(set(sum(beta[2], []))) - tmp[2])
            depart_ped_num[1].append(0)

        if len(beta[0]) > 10. / step_size:
            beta[0].pop(0)
            beta[3].pop(0)
        if len(beta[1]) > 10. / step_size:
            beta[1].pop(0)
            beta[2].pop(0)

        beta_rate = [len(set(sum(i, []))) / (10. / step_size) for i in beta]

        # update arrival rate
        # alpha rate = the rate of join queue rather than the rate of appearing on the road
        if queue_length[0][-1] > 0:
            pure_veh_id_13 = list(set(traci.lanearea.getLastStepVehicleIDs("det_13")) - set((beta[0] or ["any"])[-1]))
            jam[0].append(get_jam_veh_ids(pure_veh_id_13, queue_length[0][-1]))
            if len(jam[0]) > 11. / step_size:
                jam[0].pop(0)
            alpha_rate[0] = get_increased_jam_num(jam[0]) / (10. / step_size)
        else:
            jam[0] = []
            alpha_rate[0] = beta_rate[0]

        if queue_length[1][-1] > 0:
            pure_veh_id_42 = list(set(traci.lanearea.getLastStepVehicleIDs("det_42")) - set((beta[1] or ["any"])[-1]))
            jam[1].append(get_jam_veh_ids(pure_veh_id_42, queue_length[1][-1]))
            if len(jam[1]) > 11. / step_size:
                jam[1].pop(0)
            alpha_rate[1] = get_increased_jam_num(jam[1]) / (10. / step_size)
        else:
            jam[1] = []
            alpha_rate[1] = beta_rate[1]

        if queue_length[2][-1] > 0:
            jam[2].append(waiting_ped_id[0])
            if len(jam[2]) > (11. / step_size):
                jam[2].pop(0)
            alpha_rate[2] = get_increased_jam_num(jam[2]) / (10. / step_size)
        else:
            jam[2] = []
            alpha_rate[2] = beta_rate[2]

        if queue_length[3][-1] > 0:
            jam[3].append(waiting_ped_id[1])
            if len(jam[3]) > 11. / step_size:
                jam[3].pop(0)
            alpha_rate[3] = get_increased_jam_num(jam[3]) / (10. / step_size)
        else:
            jam[3] = []
            alpha_rate[3] = beta_rate[3]

        alpha_rate_list.append(alpha_rate)

        # update event time derivative
        idx = int(not green_1)
        light_switch, d_tau = update_event_time_derivative(idx, step_size, det_veh_num, queue_length, f, alpha_rate, h,
                                                           theta, d_x, d_tau, last_switch, last_push_button, print_mode)
        if light_switch:
            traci.trafficlight.setPhase(TLID, idx)
            last_switch = 0.
            last_push_button = 0.
            push_button = [False, False]
            green_1 = not green_1
        if not np.all(np.array(d_tau) == 0):
            last_switch_dtau = d_tau

        # update state derivative
        d_x, NEP, push_button = update_state_derivative(green_1, step_size, last_switch, NEP, queue_length, alpha_rate,
                                                        d_x, d_tau, last_switch_dtau, h, push_button, print_mode=False)
        # d_L = d_L + np.sum(np.array(d_x), 0)
        d_x_total.append(np.sum(np.array(d_x), 0))


        # check whether to update parameters
        # update_par_flag = update_par_check(alpha_rate_list, queue_length, step_per_iter, step_size,
        #                                    gradient_stable_flag)
        update_par_flag = False
        update_interval = 1200
        if step_per_iter == update_interval:
            update_par_flag = True
        if update_par_flag:

            # different ways of calculating dL:
            # 1. only use interval data
            d_L = np.array(sum(d_x_total[-update_interval:])) / step_per_iter

            # 2. use all historical data, update every fixed interval
            # d_L = np.array(sum(d_x_total))/step

            # 3. use all historical data, but give more weight to recent one: a * dL_last_interval + (1-a)* dL_history
            # if step > update_interval:
            #     d_L = 0.7 * sum(d_x_total[-update_interval:])/update_interval + \
            #             0.3 * sum(d_x_total[:-update_interval])/(step-update_interval)
            # else:
            #     d_L = np.array(sum(d_x_total)) / step

            # 4. use all historical data, exponential weight
            # if len(d_L_list) > 0:
            #     d_L = a * sum(d_x_total[-update_interval:])/update_interval + (1-a) * d_L_list[-1]
            # else:
            #     d_L = sum(d_x_total) / step
            # tmp = sum(d_x_total[:update_interval])/update_interval
            # for i in range(round(step/update_interval)-1):
            #     tmp = 0.5 * tmp + 0.5 * sum(d_x_total[(i+1)*update_interval:(i+2)*update_interval])/update_interval
            # dL = tmp

            # 5. use a * interval of data, update every fixed interval
            # d_L = sum(d_x_total[-(4*update_interval):])/(4*update_interval)

            # 5. use last several interval of data, but give more weight to recent one
            # d_L = 0.6 * sum(d_x_total[-update_interval:])/update_interval + \
            #       0.4 * sum(d_x_total[-(2 * update_interval):-update_interval])/update_interval
                  # 0.05 * sum(d_x_total[-(3 * update_interval):-(2 * update_interval)]) / update_interval + \
                  # 0.05 * sum(d_x_total[-(4 * update_interval):-(3 * update_interval)]) / update_interval




            # d_L = np.array(d_L) * 1.0 / step_per_iter
            # iter_num = round(step/update_interval)
            iter_num += 1
            # if step == 21600:
            #     iter_num = 1
            #     max_delta_theta = 2
            #     max_delta_s = 1
            # if step == 36000:
            #     iter_num = 1
            theta, d_L_list, performance_list, par_list, par_update_step_size = \
                update_result(d_L_list, performance_list, d_L, queue_length, depart_veh_num, depart_ped_num, par_list, \
                              step_per_iter, par_update_step_size, iter_num, max_delta_theta, max_delta_s, print_mode)
            # gradient_stable_flag = check_gradient_stability(d_L_list)
            update_time.append(step)
            max_change_list.append(max(par_update_step_size * d_L_list[-1], key=abs))
            # for j in range(5):
            #     print(str([round(i, 3) for i in [i[j] for i in performance_list]]) + ",")
            # print("------")
            # for i in range(len(d_L_list[0])):
            #     print(str([round(dl[i], 3) for dl in d_L_list]) + ",")
            # print(update_time)
            # print(max_change_list)
            # one run renewal
            last_switch_dtau, d_tau, d_L, alpha_rate_list, det_veh_num, queue_length, depart_veh_num, \
            depart_ped_num = reset_par(theta, det_veh_num, queue_length, depart_veh_num, depart_ped_num)

            step_per_iter = 0
    print(max_change_list)

    traci.close()
    sys.stdout.flush()

    return d_L_list, performance_list, par_list, update_time


# main function: event driven parameter updating, with sequential running
def ipa_gradient_method_pedestrian_sequential(initial_par, lam, demand_scale, step_size, run_time, iter_num,
                                             print_mode=False):

    var_list = []
    avg_list = []
    for iter in range(iter_num):
        generate_routefile_Veberod("./input/Veberod_intersection_pedestrian.rou.xml", run_time, demand_scale[0] * lam[0],
                                   demand_scale[0] * lam[1], False)
        generate_routefile_pedestrian("./input/Veberod_intersection_pedestrian.trip.xml", run_time,
                                      demand_scale[1] * lam[2],
                                      demand_scale[1] * lam[3], False)


        # generate_routefile_Veberod_uni("./input/Veberod_intersection_pedestrian.rou.xml", run_time,
        #                                [demand_scale[0] * lam[0], 0.62],
        #                                [demand_scale[0] * lam[1], 0.67], False)
        # generate_routefile_pedestrian_uni("./input/Veberod_intersection_pedestrian.trip.xml", run_time,
        #                               [demand_scale[1] * lam[2], 0.19],
        #                               [demand_scale[1] * lam[3], 0.19], False)

        d_L_list, performance_list, par_list, update_time = ped_adaptive_sequential(initial_par, step_size, run_time, print_mode)
        waiting_time_list = np.transpose(performance_list)[-1][15:]
        var_list.append(np.var(waiting_time_list))
        avg_list.append(np.mean(waiting_time_list))

        print('===================================================')
        for j in range(5):
            print(str([round(i, 3) for i in [i[j] for i in performance_list]]) + ",")
        print("------")
        for i in range(len(d_L_list[0])):
            print(str([round(dl[i], 3) for dl in d_L_list]) + ",")
        print("------")
        for i in range(len(par_list)):
            print(str([round(p,3) for p in par_list[i][:-1]]) + ",")
        print("------")

        print(update_time)

    print('var: ' + str(np.mean(var_list)))
    print(var_list)
    print('avg: ' + str(np.mean(avg_list)))
    print(avg_list)
    print('*******************************************************************************')


# event driven: one iter
def one_iter_ped_adaptive_event_driven(theta, lam, demand_scale, step_size, run_time, fix_seed, gradient_stable_flag,
                                       print_mode, sumoBinary):
    """
         theta: [theta_1_min, theta_1_max, theta_2_min, theta_2_max, theta_3, theta_4, s_1, s_2, s_3, s_4]
         lam: [lam_1_veh, lam_2_veh, lam_1_ped, lam_2_ped]
         step_size: time passed for each simulation step
         """

    generate_routefile_Veberod("./input/Veberod_intersection_pedestrian.rou.xml", run_time, demand_scale * lam[0],
                               demand_scale * lam[1], fix_seed)
    generate_routefile_pedestrian("./input/Veberod_intersection_pedestrian.trip.xml", run_time, demand_scale * lam[2],
                                  demand_scale * lam[3], fix_seed)

    # one run renewal
    last_switch_dtau = [0] * len(theta)

    # derivatives
    d_x = [[0] * len(theta) for _ in range(4)]  # d_x[0] = [0,0,0,0,0..]
    d_x_total = [[0] * len(theta) for _ in range(4)]
    d_tau = [0] * len(theta)
    d_L = [0] * len(theta)

    NEP = [False] * 4

    # arrival and deprat rate
    alpha = [[] for _ in range(4)]  # list of veh/ped ids enter this road in the last time step [[],[],[],[]]
    beta = [[] for _ in range(4)]  # list of veh/ped ids passing the cross in the last time step
    jam = [[] for _ in range(4)]  # list of vehicle/ped ids in the queue in the last time step
    alpha_rate = [0] * 4
    beta_rate = [0] * 4
    alpha_rate_list = []

    # free departure rate
    h = [0.55] * 4

    det_veh_num = [[0] for _ in range(2)]  # vehicle num at each time step  [[0], [0]]
    queue_length = [[0] for _ in range(4)]  # [[0], [0], [0], [0]]
    depart_veh_num = [[0] for _ in range(2)]
    depart_ped_num = [[0] for _ in range(2)]

    traci.start(
        [sumoBinary, "-c", "./input/Veberod_intersection_pedestrian.sumocfg", "--step-length", str(step_size),
         "--fcd-output.geo", "true", "--fcd-output", "./output/veberod-fcd.xml", "--no-step-log", "--no-warnings"])

    # we start with phase 0 --- 42 green (road 2)
    # <phase duration="200" state="grgrrGrG"/>
    traci.trafficlight.setProgram(TLID, "1")
    traci.trafficlight.setPhase(TLID, 0)
    green_1 = False
    push_button = [False, False]

    step = 0.
    last_switch = 0.  # time passed since last switch
    last_push_button = 0.  # time since last push of button
    update_par_flag = False
    while not update_par_flag:
        traci.simulationStep()
        last_switch += step_size
        step += step_size
        if True in push_button:
            last_push_button += step_size

        # pedestrian condition
        waiting_ped_id = get_waiting_ped_id()
        crossing_ped_id = get_crossing_ped_id()

        # update state list
        det_veh_num[0].append(traci.lanearea.getLastStepVehicleNumber("det_13"))
        det_veh_num[1].append(traci.lanearea.getLastStepVehicleNumber("det_42"))
        queue_length[0].append(traci.lanearea.getJamLengthVehicle("det_13"))
        queue_length[1].append(traci.lanearea.getJamLengthVehicle("det_42"))
        queue_length[2].append(len(waiting_ped_id[0]))
        queue_length[3].append(len(waiting_ped_id[1]))
        # print([queue[-1] for queue in queue_length])

        # update pedestrian state
        f = [False, False]
        if green_1 and (queue_length[2][-1] >= theta[8] or last_push_button >= theta[4]):
            f[0] = True
        if (not green_1) and (queue_length[3][-1] >= theta[9] or last_push_button >= theta[5]):
            f[1] = True

        if print_mode:
            print("current detected vehicle number: " + str(det_veh_num[0][-1]) + "  " + str(det_veh_num[1][-1]))
            print("jam length: " + str(queue_length[0][-1]) + "  " + str(queue_length[1][-1]))

        # update departure rate
        tmp = [len(set(sum(i, []))) for i in beta]
        if green_1:
            beta[0].append(list(traci.inductionloop.getLastStepVehicleIDs("det_13_0")))
            beta[1] = []
            beta[2] = []
            beta[3].append(crossing_ped_id[1])
            depart_veh_num[0].append(len(set(sum(beta[0], []))) - tmp[0])
            depart_veh_num[1].append(0)
            depart_ped_num[0].append(0)
            depart_ped_num[1].append(len(set(sum(beta[3], []))) - tmp[3])

        else:
            beta[0] = []
            beta[1].append(list(traci.inductionloop.getLastStepVehicleIDs("det_42_0")))
            beta[2].append(crossing_ped_id[0])
            beta[3] = []
            depart_veh_num[0].append(0)
            depart_veh_num[1].append(len(set(sum(beta[1], []))) - tmp[1])
            depart_ped_num[0].append(len(set(sum(beta[2], []))) - tmp[2])
            depart_ped_num[1].append(0)

        if len(beta[0]) > 10. / step_size:
            beta[0].pop(0)
            beta[3].pop(0)
        if len(beta[1]) > 10. / step_size:
            beta[1].pop(0)
            beta[2].pop(0)

        beta_rate = [len(set(sum(i, []))) / (10. / step_size) for i in beta]

        # update arrival rate

        # alpha rate = the rate of join queue rather than the rate of appearing on the road
        if queue_length[0][-1] > 0:
            pure_veh_id_13 = list(set(traci.lanearea.getLastStepVehicleIDs("det_13")) - set((beta[0] or ["any"])[-1]))
            jam[0].append(get_jam_veh_ids(pure_veh_id_13, queue_length[0][-1]))
            if len(jam[0]) > 11. / step_size:
                jam[0].pop(0)
            alpha_rate[0] = get_increased_jam_num(jam[0]) / (10. / step_size)
        else:
            jam[0] = []
            alpha_rate[0] = beta_rate[0]

        if queue_length[1][-1] > 0:
            pure_veh_id_42 = list(set(traci.lanearea.getLastStepVehicleIDs("det_42")) - set((beta[1] or ["any"])[-1]))
            jam[1].append(get_jam_veh_ids(pure_veh_id_42, queue_length[1][-1]))
            if len(jam[1]) > 11. / step_size:
                jam[1].pop(0)
            alpha_rate[1] = get_increased_jam_num(jam[1]) / (10. / step_size)
        else:
            jam[1] = []
            alpha_rate[1] = beta_rate[1]

        if queue_length[2][-1] > 0:
            jam[2].append(waiting_ped_id[0])
            if len(jam[2]) > (11. / step_size):
                jam[2].pop(0)
            alpha_rate[2] = get_increased_jam_num(jam[2]) / (10. / step_size)
        else:
            jam[2] = []
            alpha_rate[2] = beta_rate[2]

        if queue_length[3][-1] > 0:
            jam[3].append(waiting_ped_id[1])
            if len(jam[3]) > 11. / step_size:
                jam[3].pop(0)
            alpha_rate[3] = get_increased_jam_num(jam[3]) / (10. / step_size)
        else:
            jam[3] = []
            alpha_rate[3] = beta_rate[3]

        alpha_rate_list.append(alpha_rate)

        # update event time derivative
        idx = int(not green_1)
        light_switch, d_tau = update_event_time_derivative(idx, step_size, det_veh_num, queue_length, f, alpha_rate, h,
                                                           theta, d_x, d_tau, last_switch, last_push_button, print_mode)
        if light_switch:
            traci.trafficlight.setPhase(TLID, idx)
            last_switch = 0.
            last_push_button = 0.
            push_button = [False, False]
            green_1 = not green_1
        if not np.all(np.array(d_tau) == 0):
            last_switch_dtau = d_tau

        # update state derivative
        d_x, NEP, push_button = update_state_derivative(green_1, step_size, last_switch, NEP, queue_length, alpha_rate,
                                                        d_x, d_tau, last_switch_dtau, h, push_button, print_mode=False)

        d_L = d_L + np.sum(np.array(d_x), 0)
        d_x_total = np.array(d_x_total) + np.array(d_x)

        update_par_flag = update_par_check([alpha[100:] for alpha in alpha_rate_list], [l[100:] for l in queue_length],
                                           step - 100, step_size, gradient_stable_flag)

    traci.close()
    sys.stdout.flush()

    return step, np.array(d_L) * 1.0 / step, queue_length, depart_veh_num, depart_ped_num


# event driven
def repeat_iters_event_driven(par, lam, demand_scale, step_size, run_time, iters_per_par, gradient_stable_flag,
                              print_mode=False):
    """repeat the test with same parameter, but different random seed"""

    d_L_list = []
    performance_list = []
    update_time_list = []

    fix_seed = False
    if iters_per_par == 1:
        fix_seed = True

    iters = 0
    extreme_iter = 0
    extreme_threshold = 100
    while iters < iters_per_par:
        iters += 1
        update_time, d_L, queue_length, depart_veh_num, depart_ped_num = \
            one_iter_ped_adaptive_event_driven(par, lam, demand_scale, step_size, run_time, fix_seed,
                                               gradient_stable_flag, print_mode, sumoBinary)
        # exclude the outliers
        if np.any(np.array(d_L) > extreme_threshold) or np.any(np.array(d_L) < -extreme_threshold):
            print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            print("extreme d_L  " + str(d_L) + " ----- delete")
            print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            extreme_iter += 1
            iters -= 1
            extreme_threshold = 2 ** (extreme_iter) * 100
            print("extreme_threshold: " + str(extreme_threshold))

            continue

        start_record_time = 100  # in order to eliminating the unstable simulation starting range
        mean_waiting_time_1 = sum(queue_length[0][start_record_time:]) / sum(depart_veh_num[0][start_record_time:])
        mean_waiting_time_2 = sum(queue_length[1][start_record_time:]) / sum(depart_veh_num[1][start_record_time:])
        mean_veh_waiting_time = (sum(queue_length[0][start_record_time:]) + sum(queue_length[1][start_record_time:])) / \
                                (sum(depart_veh_num[0][start_record_time:]) + sum(
                                    depart_veh_num[1][start_record_time:]))
        mean_ped_waiting_time = (sum(queue_length[2][start_record_time:]) + sum(queue_length[3][start_record_time:])) / \
                                (sum(depart_ped_num[0][start_record_time:]) + sum(
                                    depart_ped_num[1][start_record_time:]))
        mean_waiting_time_total = sum([sum(i[start_record_time:]) for i in queue_length]) / \
                                  (sum([sum(i[start_record_time:]) for i in depart_veh_num]) + sum(
                                      [sum(i[start_record_time:]) for i in depart_ped_num]))

        performance_list.append(
            [mean_waiting_time_1, mean_waiting_time_2, mean_veh_waiting_time, mean_ped_waiting_time,
             mean_waiting_time_total])
        d_L_list.append(d_L)
        update_time_list.append(update_time)

        # if iters == iters_per_par:
        #     print("mean_waiting_time_total ----" + str([i[-1] for i in performance_list]))
        #     print("avg: " + str(np.mean([i[-1] for i in performance_list])))
        #     print("==============")

    return np.mean(d_L_list, 0), np.mean(performance_list, 0), np.mean(update_time_list)


# main function: event driven parameter updating, with parallel running
def ipa_gradient_method_pedestrian_event_driven(initial_par, lam, demand_scale, step_size, run_time, iters_per_par,
                                                print_mode):
    """ run ipa method to update par"""

    par_list = [[i] for i in initial_par]

    d_L_list = []
    performance_list = []
    update_time_list = [0]

    gradient_stable_flag = False
    disturb_flag = False
    disturb_time = 0
    recover_time = 0
    iter_num = 0
    par_update_step_size = 30
    while update_time_list[-1] < run_time:
        if not disturb_flag and recover_time == 0 and update_time_list[-1] > 10000:
            lam[1] = lam[1] * 1.5
            disturb_flag = True
            disturb_time = update_time_list[-1]
        if disturb_flag and update_time_list[-1] - disturb_time > 20000:
            lam[1] = lam[1] / 1.5
            disturb_flag = False
            recover_time = update_time_list[-1]

        iter_num += 1
        d_L, performance, update_time = repeat_iters_event_driven([par[-1] for par in par_list], lam, demand_scale,
                                                                  step_size, run_time + 1000, iters_per_par,
                                                                  gradient_stable_flag, print_mode)
        performance_list.append(performance)
        d_L_list.append(d_L)
        gradient_stable_flag = check_gradient_stability(d_L_list)
        # gradient_stable_flag = True

        update_time_list.append(update_time_list[-1] + round(update_time))
        print('****************************************************************************')
        print("dL: " + str(d_L))
        for par in par_list:
            print(str([round(i, 3) for i in par]) + ",")

        print("------")
        for j in range(5):
            print(str([round(i, 3) for i in [i[j] for i in performance_list]]) + ",")
        print("------")
        for i in range(len(d_L)):
            print(str([round(dl[i], 3) for dl in d_L_list]) + ",")
        print("------")
        print(update_time_list[1:])

        # update parameters
        par_update_step_size = get_par_update_step_size(par_update_step_size, d_L_list, 0.8, 3., 0.5, 1.)
        print("par_update_step_size: " + str(par_update_step_size))
        # theta_1_min
        par_list[0].append(max(0.1, par_list[0][-1] - par_update_step_size * d_L[0]))
        # theta_1_max
        par_list[1].append(max(par_list[0][-1], par_list[1][-1] - par_update_step_size * d_L[1]))
        # theta_2_min
        par_list[2].append(max(0.1, par_list[2][-1] - par_update_step_size * d_L[2]))
        # theta_2_max
        par_list[3].append(max(par_list[2][-1], par_list[3][-1] - par_update_step_size * d_L[3]))
        # theta_3
        par_list[4].append(max(0.1, par_list[4][-1] - par_update_step_size * d_L[4]))
        # theta_4
        par_list[5].append(max(0.1, par_list[5][-1] - par_update_step_size * d_L[5]))
        # s_1
        par_list[6].append(max(0.1, par_list[6][-1] - par_update_step_size * d_L[6]))
        # s_2
        par_list[7].append(max(0.1, par_list[7][-1] - par_update_step_size * d_L[7]))
        # s_3
        par_list[8].append(max(0.1, par_list[8][-1] - par_update_step_size * d_L[8]))
        # s_4
        par_list[9].append(max(0.1, par_list[9][-1] - par_update_step_size * d_L[9]))

        print('****************************************************************************')

    print("disturb time: " + str(disturb_time))
    print("recover time: " + str(recover_time))


def generate_plots(par, performance):
    x = np.arange(len(performance[0]))
#    performance plot
    plt.figure()
    plt.plot(x, performance[-1], label='vehicle and pedestrian', marker='.')
    plt.plot(x, performance[-3], label='only vehicle', marker='v')
    plt.plot(x, performance[-2], label='only pedestrian', marker='+')
    plt.xlabel('iterations')
    plt.ylabel("average waiting time(s)")
    # plt.xticks(np.arange(0,20,2))
    # plt.yticks(np.arange(0, 26, 2))
    # plt.grid()
    plt.legend()
    plt.savefig("./output/performance.png")
    # plt.show()

#     parameter plot
    plt.figure()
    plt.plot(x, par[0], "r--", label = r'$\theta_1^{min}$')
    plt.plot(x, par[1], 'r', label = r'$\theta_1^{max}$')
    plt.plot(x, par[2], 'g--', label = r'$\theta_2^{min}$', marker = '+')
    plt.plot(x, par[3], 'g', label = r'$\theta_2^{max}$', marker = '+')
    plt.xlabel('iterations')
    plt.ylabel("GREEN length threshold")
    # plt.xticks(np.arange(0,20,2))
    # plt.yticks(np.arange(0, 26, 2))
    # plt.grid()
    plt.legend()
    plt.savefig("./output/GREEN_length_threshold_vehicle.png")
    # plt.show()

    # plt.figure(figsize=(4, 6))
    plt.figure()
    plt.plot(x, par[4], label=r'$\theta_3$', marker='.')
    plt.plot(x, par[5], label=r"$\theta_4$", marker='v')
    plt.xlabel('iterations')
    plt.ylabel("GREEN length threshold")
    # plt.xticks(np.arange(0,20,2))
    # plt.yticks(np.arange(0, 26, 2))
    # plt.grid()
    plt.legend()
    plt.savefig("./output/GREEN_length_threshold_pedestrian.png")
    # plt.show()

    plt.figure()
    plt.plot(x, par[6], label = "$s_1$", marker = '.')
    plt.plot(x, par[7], label = "$s_2$", marker = '+')
    plt.plot(x, par[8], label = "$s_3$", marker = 'v')
    plt.plot(x, par[9], label = "$s_4$", marker = 0)
    plt.xlabel('iterations')
    plt.ylabel("queue content threshold")
    # plt.grid()
    plt.legend()
    plt.savefig("./output/queue_length_threshold.png")
    # plt.show()
    plt.close('all')


sumoBinary = checkBinary('sumo')
if __name__ == "__main__":
    sumoBinary = checkBinary('sumo')
    # sumoBinary = checkBinary('sumo-gui')

    # pedestrian_baseline_test()

#    ipa_gradient_method_pedestrian(initial_par=[30, 35, 15, 20, 10, 100, 5, 8, 3, 3], lam=[10, 10, 5, 5],
#                                   demand_scale=[1, 1], step_size=1, run_time=1000,
#                                   total_iter_num=10, iters_per_par=10, print_mode=False)


    # sequential one time
    ipa_gradient_method_pedestrian_sequential(
        initial_par=[10, 20, 30, 50, 10, 10, 8, 8, 5, 5],
        lam=[0.11, 0.125, 0.01, 0.01],
        demand_scale=[1.4, 1.4], step_size=1, run_time=43200, iter_num=1,  print_mode=False)



    # event driven + parallel repeat
    # ipa_gradient_method_pedestrian_event_driven(initial_par=[10, 20, 30, 50, 10, 10, 8, 8, 8, 8],
    #                                             lam=[0.11, 0.125, 0.01, 0.01],
    #                                             demand_scale=1.2, step_size=1, run_time=50000, iters_per_par=50,
    #                                             print_mode=False)
