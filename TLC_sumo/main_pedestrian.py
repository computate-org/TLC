import os
import sys
import optparse
import random
from pprint import pprint
from config import *
from route_file_generation import generate_routefile, generate_routefile_Veberod, generate_routefile_pedestrian

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
        for iter_num in range(1):
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
    idx_ = 0
    if idx == 0:
        idx_ = 1

    # alpha1=0
    if det_veh_num[idx][-2] > 0 and det_veh_num[idx][-1] == 0:
        if (queue_length[idx_][-1] > 0 and not f[idx_]) or \
                (queue_length[idx_][-1] == 0 and f[idx] and not f[idx_]):

            light_switch = True
            d_tau = [0] * len(theta)
            if print_mode:
                print("alpha{}=0".format(idx))

    # z1=theta_1_max
    elif last_switch >= theta[idx * 2 + 1] > (last_switch - step_size) and queue_length[idx_][-1] > 0:
        light_switch = True
        d_tau[idx * 2 + 1] = d_tau[idx * 2 + 1] + 1
        if print_mode:
            print("z{0}=theta_{0}_max".format(idx + 1))

    # z1=theta_1_min
    elif last_switch >= theta[idx] > (last_switch - 1):
        if (f[idx] and not f[idx_]) or (
                queue_length[idx][-1] < theta[6 + idx] and queue_length[idx_][-1] >= theta[6 + idx_]):
            light_switch = True

            d_tau[idx * 2] = d_tau[idx * 2] + 1
            if print_mode:
                print("z{0}=theta_{0}_min".format(idx + 1))

    # x1 = s1(a)
    elif queue_length[idx][-2] > theta[idx + 6] >= queue_length[idx][-1]:
        if last_switch >= theta[idx * 2] and queue_length[idx_][-1] >= theta[6 + idx_]:
            light_switch = True

            if alpha_rate[idx] - h[idx] == 0:
                print("WARNING: alpha_rate_{0} - h{0} == 0 when x{0} = s{0}(a)/ G2R1".format(idx + 1))
            else:
                tmp = -np.array(d_x[idx])
                tmp[6 + idx] += 1
                d_tau = tmp / (alpha_rate[idx] - h[idx])
            if print_mode:
                print("x{0} = s{0}(a) G2R{0}".format(idx + 1))

    # x2 = s2(b)
    elif queue_length[idx_][-2] < theta[idx_ + 6] <= queue_length[idx_][-1]:
        if last_switch >= theta[idx * 2] and queue_length[idx][-1] < theta[idx + 6]:
            light_switch = True

            if alpha_rate[idx_] == 0:
                alpha_rate[idx_] = 0.1
                if print_mode:
                    print("WARING: alpha_rate{0}==0 when x{0} = s{0}(b), use alternative {1}".format(idx_ + 1, 0.1))
            tmp = - np.array(d_x[idx_])
            tmp[6 + idx_] += 1
            d_tau = tmp / alpha_rate[idx_]
            if print_mode:
                print("x{0} = s{0}(b) G2R{0}".format(idx_ + 1))

    # x4=s4(a) --> f2=0
    elif queue_length[idx_ + 2][-2] > theta[idx_ + 8] >= queue_length[idx_ + 2][-1]:
        if (det_veh_num[idx][-1] == 0 and queue_length[idx_][-1] == 0 and f[idx]) or \
                (det_veh_num[idx][-1] == 0 and queue_length[idx_][-1] > 0) or \
                (det_veh_num[idx][-1] > 0 and queue_length[idx_][-1] > 0 and last_switch > theta[idx * 2] and not f[
                    idx]):
            light_switch = True
            if alpha_rate[idx_ + 2] - h[idx_ + 2] == 0:
                print("WARNING: alpha_rate{0} - h{0} == 0 when x{0} = s{0}(a)/ G2R{1}".format(idx_ + 3, idx + 1))
            else:
                tmp = - np.array(d_x[idx_ + 2])
                tmp[idx_ + 8] += 1
                d_tau = tmp / (alpha_rate[idx_ + 2] - h[idx_ + 2])
            if print_mode:
                print("x{0} = s{0}(a) G2R{1}".format(idx_ + 3, idx + 1))

    # x3 = s3(b) --> f1=1
    elif queue_length[idx + 2][-2] < theta[idx + 8] <= queue_length[idx + 2][-1]:
        if (det_veh_num[idx][-1] > 0 and last_switch >= theta[idx * 2] and not f[idx_]) or \
                (det_veh_num[idx][-1] == 0 and queue_length[idx_][-1] == 0 and not f[idx_]):
            light_switch = True

            if alpha_rate[idx + 2] == 0:
                alpha_rate[idx + 2] = 0.1
                if print_mode:
                    print("WARING: alpha_rate_{0}==0 when x{0} = s{0}(b), use alternative 0.1".format(idx + 3))

            tmp = - np.array(d_x[idx + 2])
            tmp[idx + 8] += 1
            d_tau = tmp / alpha_rate[idx + 2]
            if print_mode:
                print("x{0} = s{0}(b) G2R{1}".format(idx + 3, idx + 1))

    #   y1 = theta3 --> f1 = 1
    elif last_push_button >= theta[idx + 4] > (last_push_button - step_size):
        if (det_veh_num[idx][-1] > 0 and last_switch >= theta[idx * 2] and not f[idx_]) or \
                (det_veh_num[idx][-1] == 0 and queue_length[idx_][-1] == 0 and not f[idx_]):

            light_switch = True
            d_tau = np.zeros(len(theta))
            d_tau[idx + 4] = 1
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


def one_iter_ped_adaptive(theta, lam, demand_scale, step_size, run_time, fix_seed, print_mode, sumoBinary):
    """
         theta: [theta_1_min, theta_1_max, theta_2_min, theta_2_max, theta_3, theta_4, s_1, s_2, s_3, s_4]
         lam: [lam_1_veh, lam_2_veh, lam_1_ped, lam_2_ped]
         step_size: time passed for each simulation step
         """

    generate_routefile_Veberod("Veberod_intersection_pedestrian.rou.xml", run_time, demand_scale * lam[0],
                               demand_scale * lam[1], fix_seed)
    generate_routefile_pedestrian("Veberod_intersection_pedestrian.trip.xml", run_time, demand_scale * lam[2],
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

        if print_mode:
            print('------------------------------------')
            # print("alpha:" + str(alpha_1) + ">>>>" + str(alpha_2))
            # print("veh_ids: " + str(traci.lanearea.getLastStepVehicleIDs("det_13")) + ">>>>" + str(traci.lanearea.getLastStepVehicleIDs("det_42")))
            print("jam:" + str(np.array(jam)))
            print("arrival rate: " + str(alpha_rate))
            print("beta:" + str(np.array(beta)))
            print("departure rate: " + str(beta_rate))
            print("depart_veh_num: " + str(depart_veh_num[0][-1]) + ">>>>" + str(depart_veh_num[1][-1]))
            print("depart_ped_num: " + str(depart_ped_num[0][-1]) + ">>>>" + str(depart_ped_num[1][-1]))
            # print('tmp: ' + str(tmp1)+ ">>>>" + str(tmp2))
            print('-------------------------------------')

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


def repeat_iters(par, lam, demand_scale, step_size, run_time, iters_per_par, print_mode=False):
    """repeat the test with same parameter, but different random seed"""

    d_L_list = []
    performance_list = []

    fix_seed = False
    if iters_per_par == 1:
        fix_seed = True

    iters = 0
    while iters < iters_per_par:
        iters += 1
        d_L, mean_waiting_time_1, mean_waiting_time_2, mean_veh_waiting_time, mean_ped_waiting_time, \
            mean_waiting_time_total = one_iter_ped_adaptive(par, lam, demand_scale, step_size, run_time, fix_seed,
                                                        print_mode, sumoBinary)
        # exclude the outliers
        if np.any(np.array(d_L) > 100) or np.any(np.array(d_L) < -100):
            print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            print("extreme d_L  " + str(d_L) + " ----- delete")
            print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            continue
        d_L_list.append(d_L)
        performance_list.append([mean_waiting_time_1, mean_waiting_time_2, mean_veh_waiting_time, mean_ped_waiting_time, mean_waiting_time_total])

        print("d_L:")
        pprint(d_L)

        if print_mode:
            print("d_L:")
            pprint(d_L_list)
            print("==================================================================================================")
            print("for par " + str(par) + ":")
            print("mean_waiting_time_total ----" + str([i[-1] for i in performance_list]))
            print("==================================================================================================")

    return np.mean(d_L_list, 0), np.mean(performance_list, 1)


def ipa_gradient_method_pedestrian(initial_par, lam, demand_scale, step_size, par_update_step_size, run_time, total_iter_num, iters_per_par):
    """ run ipa method to update par"""

    par_list = [[i] for i in initial_par]

    d_L_list = []
    performance_list = []

    iter_num = 0

    while iter_num < total_iter_num:
        iter_num += 1

        # if iter_num == 30:
        #     stepsize = stepsize / 2.
        # if iter_num == 50:
        #     stepsize = stepsize / 2.

        d_L, performance = repeat_iters([par[-1] for par in par_list], lam, demand_scale, step_size, run_time,
                                        iters_per_par,  print_mode=True)

        # update performance
        performance_list.append(performance)
        d_L_list.append(d_L)
        print('****************************************************************************')
        print("dL: " + str(d_L))
        for par in par_list:
            print([round(i, 3) for i in par])

        print("------")
        print([round(i, 3) for i in [i[-1] for i in performance_list]])

        print("------")
        for i in range(len(d_L)):
            print([round(dl[i], 3) for dl in d_L_list])

        print('****************************************************************************')

        # update parameters

        # theta_1_min
        par_list[0].append(max(0.1, par_list[0][-1] - par_update_step_size * d_L[0]))
        # theta_1_max
        par_list[1].append(max(par_list[0][-1], par_list[1][-1] - par_update_step_size * d_L[1]))
        # theta_2_min
        par_list[2].append(max(0.1, par_list[2][-1] - par_update_step_size * d_L[2]))
        # theta_2_max
        par_list[3].append(max(par_list[2][-1], par_list[3][-1] - par_update_step_size * d_L[3]))
        # theta_3
        par_list[4].append(par_list[4][-1] - par_update_step_size * d_L[4])
        # theta_4
        par_list[5].append(par_list[5][-1] - par_update_step_size * d_L[5])
        # s_1
        par_list[6].append(max(0.1, par_list[6][-1] - par_update_step_size * d_L[6]))
        # s_2
        par_list[7].append(max(0.1, par_list[7][-1] - par_update_step_size * d_L[7]))
        # s_3
        par_list[8].append(max(0.1, par_list[8][-1] - par_update_step_size * d_L[8]))
        # s_4
        par_list[9].append(max(0.1, par_list[9][-1] - par_update_step_size * d_L[9]))


if __name__ == "__main__":
    # sumoBinary = checkBinary('sumo')
    sumoBinary = checkBinary('sumo-gui')

    # pedestrian_baseline_test()
    ipa_gradient_method_pedestrian(initial_par=[1,20,1,40,100,100,100,100,100,100], lam=[1/4,1/6,1/10,1/10],
                                   demand_scale=1, step_size=1, par_update_step_size=50, run_time=2000,
                                   total_iter_num=20, iters_per_par=1)

