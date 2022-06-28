import os
import sys
import optparse
import random
from pprint import pprint

# we need to import python modules from the $SUMO_HOME/tools directory
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

from sumolib import checkBinary  # noqa
import traci  # noqa
import numpy as np
import pandas as pd
import random
import math


def expo_gen(lam):
    return -1 / lam * math.log(1 - random.random())


def generate_routefile_Veberod(run_time, lam1, lam2, fix_seed):

    seed = 3570926575676058649
    if not fix_seed:
        seed = random.randrange(sys.maxsize)
    random.seed(seed)

    print("Seed was:", seed)

    N = run_time  # number of time steps
    # demand per second from different directions
    lam13 = lam1
    lam42 = lam2

    next_13 = expo_gen(lam13)
    next_42 = expo_gen(lam42)

    with open("Veberod_intersection.rou.xml", "w") as routes:
        print("""<routes>
        <vType id="v1" accel="2.6" decel="4.5" sigma="0.5" length="5" minGap="2.5" maxSpeed="55" \
guiShape="passenger"/>

        <route id="r13" edges="96027913#0 96027913#2 30186293 311389510#1" />
        <route id="r42" edges="-355113043#1 -24626686#2 -24626686#1 -34992983#3 -34992983#1" />""", file=routes)
        vehNr = 0
        for i in range(N):
            if i > next_13:
                print('    <vehicle id="right_%i" type="v1" route="r13" depart="%i" />' % (
                    vehNr, i), file=routes)
                vehNr += 1
                next_13 += expo_gen(lam13)

            if i > next_42:
                print('    <vehicle id="up_%i" type="v1" route="r42" depart="%i" />' % (
                    vehNr, i), file=routes)
                vehNr += 1
                next_42 += expo_gen(lam42)

        print("</routes>", file=routes)


def generate_routefile(run_time, lam1, lam2, fix_seed):
    if fix_seed:
        random.seed(22)  # make tests reproducible
    else:
        seed = random.randrange(sys.maxsize)
        random.seed(seed)

    print("Seed was:", seed)

    N = run_time  # number of time steps
    # demand per second from different directions
    lam13 = lam1
    lam42 = lam2

    next_13 = expo_gen(lam13)
    next_42 = expo_gen(lam42)

    with open("tlc_single_straight.rou.xml", "w") as routes:
        print("""<routes>
        <vType id="v1" accel="2.6" decel="4.5" sigma="0.5" length="5" minGap="2.5" maxSpeed="55" \
guiShape="passenger"/>

        <route id="r13" edges="e10 e03" />
        <route id="r42" edges="e40 e02" />""", file=routes)
        vehNr = 0
        for i in range(N):
            if i > next_13:
                print('    <vehicle id="right_%i" type="v1" route="r13" depart="%i" />' % (
                    vehNr, i), file=routes)
                vehNr += 1
                next_13 += expo_gen(lam13)

            if i > next_42:
                print('    <vehicle id="up_%i" type="v1" route="r42" depart="%i" />' % (
                    vehNr, i), file=routes)
                vehNr += 1
                next_42 += expo_gen(lam42)

        print("</routes>", file=routes)


# get the vehicle id list of the queue
def get_jam_veh_ids(veh_id_list, jam_length):
    veh_ids = sorted([veh_id.split("_")[1] for veh_id in veh_id_list], key=int)
    return veh_ids[:jam_length]


# get the number of vehicles joining the queue through the vehicle id list
def get_increased_jam_num(veh_ids_list):
    num = 0
    if len(veh_ids_list) > 10:
        num = len(veh_ids_list[0])
    return len(set(sum(veh_ids_list, [])))-num


def one_iter(theta_1_min, theta_1_max, theta_2_min, theta_2_max, s_1, s_2, lam_1, lam_2, run_time, fix_seed, print_mode,
             sumoBinary):
    # one run renewal
    total_length = 0
    par_direct = [0] * 6
    step = 0
    last_switch_dtau = [0]*6

    # derivatives
    d_tau = [0] * 6
    d_x1 = [0] * 6
    d_x2 = [0] * 6
    d_L = [0] * 6
    d_x1_total = [0] * 6
    d_x2_total = [0] * 6
    NEP_1 = False
    NEP_2 = False

    # arrival and deprat rate
    alpha_1 = []  # list of vehicle ids enter this road in the last time step
    beta_1 = []  # list of vehicle ids pass the cross in the last time step
    jam_1 = []  # list of vehicle ids in the queue in the last time step
    alpha_2 = []
    beta_2 = []
    jam_2 = []

    h1 = 0.55
    h2 = 0.55

    det_veh_num_1 = [0]  # vehicle num at each time step
    det_veh_num_2 = [0]
    queue_length_1 = [0]
    queue_length_2 = [0]
    depart_veh_num_1 = [0]
    depart_veh_num_2 = [0]


    # generate_routefile(run_time+100, lam_1, lam_2, fix_seed)
    generate_routefile_Veberod(run_time + 100, lam_1, lam_2, fix_seed)

    tl_id = "267701936"
    # tl_id = "n1"
    # traci.start([sumoBinary,  "-c", "tlc_single_straight.sumocfg", "--fcd-output.geo", "--no-step-log", "--no-warnings"])
    traci.start(
        [sumoBinary, "-c", "tlc_single_straight.sumocfg", "--fcd-output.geo", "true", "--fcd-output", "veberod-fcd.xml",
         "--no-step-log", "--no-warnings"])
    # we start with phase 0 --- 42 green (road 2)
    # <phase duration="200" state="GrGr"/>
    traci.trafficlight.setPhase(tl_id, 0)
    last_switch = 0
    green_1 = False
    green_1_length = 0
    green_2_length = 0
    while step < run_time:
        traci.simulationStep()
        last_switch += 1
        step += 1
        # update det_veh_num list
        det_veh_num_1.append(traci.lanearea.getLastStepVehicleNumber("det_13"))
        det_veh_num_2.append(traci.lanearea.getLastStepVehicleNumber("det_42"))
        queue_length_1.append(traci.lanearea.getJamLengthVehicle("det_13"))
        queue_length_2.append(traci.lanearea.getJamLengthVehicle("det_42"))
        if print_mode:
            print("current detected vehicle number: " + str(det_veh_num_1[-1]) + "  " + str(det_veh_num_2[-1]))
            print("jam length: " + str(queue_length_1[-1]) + "  " + str(queue_length_2[-1]))


        # update departure rate
        tmp1 = len(set(sum(beta_1, [])))
        tmp2 = len(set(sum(beta_2, [])))
        if green_1:
            beta_1.append(list(traci.inductionloop.getLastStepVehicleIDs("det_13_0")))
            beta_2 = []
            depart_veh_num_1.append(len(set(sum(beta_1, []))) - tmp1)
            depart_veh_num_2.append(0)
        else:
            beta_1 = []
            beta_2.append(list(traci.inductionloop.getLastStepVehicleIDs("det_42_0")))
            depart_veh_num_2.append(len(set(sum(beta_2, []))) - tmp2)
            depart_veh_num_1.append(0)

        if len(beta_1) > 10:
            beta_1.pop(0)
        if len(beta_2) > 10:
            beta_2.pop(0)

        beta_1_rate = len(set(sum(beta_1, []))) / 10
        beta_2_rate = len(set(sum(beta_2, []))) / 10


        # update arrival rate
        alpha_1.append(list(traci.inductionloop.getLastStepVehicleIDs("det_13_1")))
        alpha_2.append(list(traci.inductionloop.getLastStepVehicleIDs("det_42_4")))
        if len(alpha_1) > 10:
            alpha_1.pop(0)
            alpha_2.pop(0)

        if queue_length_1[-1] > 0:
            pure_veh_id_13 = list(set(traci.lanearea.getLastStepVehicleIDs("det_13")) - set((beta_1 or ["any"])[-1]))
            jam_1.append(get_jam_veh_ids(pure_veh_id_13 , queue_length_1[-1]))
            if len(jam_1) > 11:
                jam_1.pop(0)
            alpha_1_rate = get_increased_jam_num(jam_1) / 10.
        else:
            jam_1 = []
            # alpha_1_rate = len(set(sum(alpha_1, []))) / 10.
            alpha_1_rate = beta_1_rate


        if queue_length_2[-1] > 0:
            pure_veh_id_42 = list(set(traci.lanearea.getLastStepVehicleIDs("det_42")) - set((beta_2 or ["any"])[-1]))
            jam_2.append(get_jam_veh_ids(pure_veh_id_42, queue_length_2[-1]))
            if len(jam_2) > 11:
                jam_2.pop(0)
            alpha_2_rate = get_increased_jam_num(jam_2) / 10.

        else:
            jam_2 = []
            # alpha_2_rate = len(set(sum(alpha_2, []))) / 10.
            alpha_2_rate = beta_2_rate

        if print_mode:
            print('------------------------------------')
            print("alpha:" + str(alpha_1) + ">>>>" + str(alpha_2))
            print("veh_ids: " + str(traci.lanearea.getLastStepVehicleIDs("det_13")) + ">>>>" + str(traci.lanearea.getLastStepVehicleIDs("det_42")))
            print("jam:" + str(jam_1) + ">>>>" + str(jam_2))
            print("arrival rate: " + str(alpha_1_rate) + ">>>>" + str(alpha_2_rate))
            print("beta:" + str(beta_1) + ">>>>" + str(beta_2))
            print("departure rate: " + str(beta_1_rate) + ">>>>" + str(beta_2_rate))
            print("depart_veh_num: " + str(depart_veh_num_1[-1]) + ">>>>" + str(depart_veh_num_2[-1]))
            print('tmp: ' + str(tmp1)+ ">>>>" + str(tmp2))
            print('-------------------------------------')

        # update event time derivative
        if green_1:
            green_1_length += 1
            if queue_length_2[-1] == 0:
                d_tau = [0, 0, 0, 0, 0, 0]
                if print_mode:
                    print("alpha2=0")

            # alpha1=0, alpha2>0
            elif det_veh_num_1[-1] == 0:
                # switch light
                traci.trafficlight.setPhase(tl_id, 0)
                last_switch = 0
                green_1 = False

                d_tau = [0, 0, 0, 0, 0, 0]
                if print_mode:
                    print("alpha1=0, alpha2>0")

            # z1=theta_1_max
            elif last_switch >= theta_1_max:
                # switch light
                traci.trafficlight.setPhase(tl_id, 0)
                last_switch = 0
                green_1 = False

                # update event time derivative
                d_tau[1] = d_tau[1] + 1
                if print_mode:
                    print("z1=theta_1_max")

            # z1=theta_1_min
            elif last_switch >= theta_1_min > (last_switch - 1) \
                    and queue_length_2[-1] >= s_2 and queue_length_1[-1] < s_1:
                # switch light
                traci.trafficlight.setPhase(tl_id, 0)
                last_switch = 0
                green_1 = False
                # update event time derivative
                d_tau[0] = d_tau[0] + 1
                if print_mode:
                    print("z1=theta_1_min")

            # x1 = s1(a)/ G2R1
            elif last_switch >= theta_1_min and queue_length_2[-1] >= s_2 \
                    and queue_length_1[-2] > s_1 >= queue_length_1[-1]:
                # switch light
                traci.trafficlight.setPhase(tl_id, 0)
                green_1 = False
                last_switch = 0
                # update event time derivative
                if alpha_1_rate - h1 == 0:
                    # if true_dyn_1 == 0:
                    print("WARNING: alpha_1_rate - h1 == 0 when x1 = s1(a)/ G2R1")
                else:
                    d_tau = (np.array([0, 0, 0, 0, 1, 0]) - np.array(d_x1)) / (alpha_1_rate - h1)
                if print_mode:
                    print("x1 = s1(a) G2R1")

            # x2 = s2(b)
            elif last_switch >= theta_1_min and queue_length_1[-1] < s_1 \
                    and queue_length_2[-2] < s_2 <= queue_length_2[-1]:
                # switch light
                traci.trafficlight.setPhase(tl_id, 0)
                green_1 = False
                last_switch = 0
                # update event time derivative
                if alpha_1_rate == 0:
                    alpha_1_rate = max(len(set(sum(alpha_1, []))) / 10., 0.1)
                    if print_mode:
                        print("WARING: alpha_1_rate==0 when x2 = s2(b), use alternative " + str(alpha_1_rate))
                else:
                    d_tau = (np.array([0, 0, 0, 0, 0, 1]) - np.array(d_x1)) / alpha_1_rate
                if print_mode:
                    print("x2 = s2(b) G2R1")

            else:
                d_tau = [0, 0, 0, 0, 0, 0]


        else:
            green_2_length += 1
            if queue_length_1[-1] == 0:
                d_tau = [0, 0, 0, 0, 0, 0]
                if print_mode:
                    print("alpha1=0")

            # alpha2=0, alpha1>0
            elif det_veh_num_2[-1] == 0:
                # switch light
                traci.trafficlight.setPhase(tl_id, 1)
                last_switch = 0
                green_1 = True
                d_tau = [0, 0, 0, 0, 0, 0]
                if print_mode:
                    print("alpha2=0, alpha1>0")

            # z2=theta_2_max
            elif last_switch >= theta_2_max:
                # switch light
                traci.trafficlight.setPhase(tl_id, 1)
                last_switch = 0
                green_1 = True

                # update event time derivative
                d_tau[3] = d_tau[3] + 1
                if print_mode:
                    print("z2=theta_2_max")

            # z2=theta_2_min
            elif last_switch >= theta_2_min > (last_switch - 1) \
                    and queue_length_1[-1] >= s_1 and queue_length_2[-1] < s_2:
                # switch light
                traci.trafficlight.setPhase(tl_id, 1)
                last_switch = 0
                green_1 = True
                # update event time derivative
                d_tau[2] = d_tau[2] + 1
                if print_mode:
                    print("z2=theta_2_min")

            # x2 = s2(a)/ G2R2
            elif last_switch >= theta_2_min and queue_length_1[-1] >= s_1 \
                    and queue_length_2[-2] > s_2 >= queue_length_2[-1]:
                # switch light
                traci.trafficlight.setPhase(tl_id, 1)
                green_1 = True
                last_switch = 0
                # update event time derivative
                if alpha_2_rate - h2 == 0:
                    print("WARNING: alpha_2_rate - h2 == 0 when x2 = s2(a)/ G2R2")
                else:
                    d_tau = (np.array([0, 0, 0, 0, 0, 1]) - np.array(d_x2)) / (alpha_2_rate - h2)
                if print_mode:
                    print("x2 = s2(a) G2R2")



            # x1 = s1(b)
            elif last_switch >= theta_2_min and queue_length_2[-1] < s_2 \
                    and queue_length_1[-2] < s_1 <= queue_length_1[-1]:
                # switch light
                traci.trafficlight.setPhase(tl_id, 1)
                green_1 = True
                last_switch = 0
                # update event time derivative
                if alpha_2_rate == 0:
                    alpha_2_rate = max(len(set(sum(alpha_2, []))) / 10., 0.1)

                    if print_mode:
                        print("WARING: alpha_2_rate==0 when x1 = s1(b), use alternative " + str(alpha_2_rate))

                d_tau = (np.array([0, 0, 0, 0, 1, 0]) - np.array(d_x2)) / alpha_2_rate
                if print_mode:
                    print("x1 = s1(b) G2R2")

            else:
                d_tau = [0, 0, 0, 0, 0, 0]

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

        # first consider road 13
        if print_mode:
            print("------------")

        # S_1
        if last_switch <= 1 and not green_1 and not NEP_1 and queue_length_1[-1] > 0:
            NEP_1 = True
            if print_mode:
                print("event at S_1 with G2R_1")
            d_x1 = - alpha_1_rate * np.array(last_switch_dtau)
        elif last_switch != 0 and (not NEP_1) and queue_length_1[-1] > 0:
            NEP_1 = True
            d_x1 = [0, 0, 0, 0, 0, 0]
            if print_mode:
                print("event at S_1 NOT with light switch")


        # E_1
        elif NEP_1 and queue_length_1[-1] == 0:
            NEP_1 = False
            d_x1 = [0, 0, 0, 0, 0, 0]
            if print_mode:
                print("event at E_1")

        # EP
        # elif (not NEP_1) and det_veh_num_1[-1] == 0 and not green_1:
        #     d_x1 = [0, 0, 0, 0, 0, 0]
        #     if print_mode:
        #         print("event at EP_1")
        # elif (not NEP_1) and queue_length_1[-1] == 0 and green_1:
        #     d_x1 = [0, 0, 0, 0, 0, 0]
        #     if print_mode:
        #         print("event at EP_1")
        elif (not NEP_1) and queue_length_1[-1] == 0:
            d_x1 = [0, 0, 0, 0, 0, 0]
            if print_mode:
                print("event at EP_1")

        # G2R1
        elif last_switch == 0 and not green_1:
            d_x1 = d_x1 - h1 * np.array(d_tau)
            if print_mode:
                print("event at G2R_1")

            # R2G1
        elif last_switch == 0 and green_1:
            d_x1 = d_x1 + h1 * np.array(d_tau)
            if print_mode:
                print("event at R2G_1")



        else:
            if print_mode:
                if NEP_1:
                    print("d_x1 not change, in NEP_1")
                else:
                    print("d_x1 not change, in EP_1")



        # for road 42

        # S_2
        if last_switch <= 1 and green_1 and not NEP_2 and queue_length_2[-1] > 0:
            NEP_2 = True
            d_x2 = - alpha_2_rate * np.array(last_switch_dtau)
            if print_mode:
                print("event at S_2 with G2R_2")

        elif last_switch != 0 and (not NEP_2) and queue_length_2[-1] > 0:
            NEP_2 = True
            d_x2 = [0, 0, 0, 0, 0, 0]
            if print_mode:
                print("event at S_2 NOT with light switch")


        # E_2
        elif NEP_2 and queue_length_2[-1] == 0:
            NEP_2 = False
            d_x2 = [0, 0, 0, 0, 0, 0]
            if print_mode:
                print("event at E_2")

        # EP
        elif not NEP_2 and queue_length_2[-1] == 0:
            d_x2 = [0, 0, 0, 0, 0, 0]
            if print_mode:
                print("event at EP_2")


        # G2R2
        elif last_switch == 0 and green_1:
            d_x2 = d_x2 - h2 * np.array(d_tau)
            if print_mode:
                print("event at G2R_2")

        # R2G2
        elif last_switch == 0 and not green_1:
            d_x2 = d_x2 + h2 * np.array(d_tau)
            if print_mode:
                print("event at R2G_2 ")



        else:
            if print_mode:
                if NEP_2:
                    print("d_x2 not change, in NEP_2")
                else:
                    print("d_x2 not change, in EP_2")

        #
        # if not NEP_2 and queue_length_2[-1] > 0:
        #     NEP_2 = True
        #     # induced by G2R2
        #     if last_switch == 0 and green_1:
        #         if print_mode:
        #             print("event at S_2 with light switch")
        #         d_x2 = - alpha_2_rate * np.array(d_tau)
        #     else:
        #         d_x2 = [0, 0, 0, 0, 0, 0]
        #         if print_mode:
        #             print("event at S_2 NOT with light switch")

        d_L = d_L + np.array(d_x1) + np.array(d_x2)
        d_x1_total = d_x1_total + np.array(d_x1)
        d_x2_total = d_x2_total + np.array(d_x2)
        if print_mode:
            print('----------------')
            print("d_x1: " + str(d_x1))
            print("d_x2: " + str(d_x2))
            print("d_x1_total: " + str(d_x1_total))
            print("d_x2_total: " + str(d_x2_total))
            print('===========================================================================================')
    try:
        mean_waiting_time_total = (sum(queue_length_1[100:]) + sum(queue_length_2[100:]))/(sum(depart_veh_num_1[100:])+sum(depart_veh_num_2[100:]))
        mean_waiting_time_1 = sum(queue_length_1[100:])/(sum(depart_veh_num_1[100:]))
        mean_waiting_time_2 = sum(queue_length_2[100:])/(sum(depart_veh_num_2[100:]))
    except:
        print(depart_veh_num_1)
        print(depart_veh_num_2)
        print(queue_length_1)
        print(queue_length_2)
        print("=")
    traci.close()
    sys.stdout.flush()
    # print("green_1_length: " + str(green_1_length))
    # print("green_2_length: " + str(green_2_length))
    # return np.array(d_L) * 1.0 / run_time, (sum(queue_length_1[100:]) + sum(queue_length_2[100:])) * 1.0 / (run_time - 100), \
    #        sum(queue_length_1[100:]) * 1.0 / (run_time - 100), sum(queue_length_2[100:]) * 1.0 / (run_time - 100)
    return np.array(d_L) * 1.0 / run_time, mean_waiting_time_total, mean_waiting_time_1, mean_waiting_time_2


def run(par, lam_1, lam_2, run_time, iters_per_par, sumoBinary, print_mode=False):
    """execute the TraCI control loop"""
    theta_1_min = par[0]
    theta_1_max = par[1]
    theta_2_min = par[2]
    theta_2_max = par[3]
    s_1 = par[4]
    s_2 = par[5]

    d_L_list = []
    mean_queue_length_list = []
    output1 = []
    output2 = []
    fix_seed = False
    if iters_per_par == 1:
        fix_seed = True

    iters = 0
    while iters < iters_per_par:
        iters += 1
        d_L, mean_queue_length, op1, op2 = one_iter(theta_1_min, theta_1_max, theta_2_min, theta_2_max, s_1, s_2,
                                                    lam_1, lam_2, run_time, fix_seed, print_mode, sumoBinary)
        #exclude the outliers
        if np.any(np.array(d_L) > 100) or np.any(np.array(d_L) < -100):
            print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            print("extreme d_L  " + str(d_L) + " ----- delete")
            print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            continue
        d_L_list.append(d_L)

        mean_queue_length_list.append(mean_queue_length)
        output1.append(op1)
        output2.append(op2)
        print("d_L:")
        pprint(d_L)

        if print_mode:
            print("d_L:")
            pprint(d_L_list)
            print("==================================================================================================")
            print("for par " + str(par) + ":")
            print("mean_queue_length ----" + str(mean_queue_length_list))
            print("==================================================================================================")

    return np.mean(d_L_list, 0), np.mean(mean_queue_length_list), np.mean(output1), np.mean(output2)


def tl_test():
    traci.trafficlight.setPhase("n1", 0)
    step = 0
    while step < 1000:
        traci.simulationStep()
        step += 1
        if step % 10 == 0:
            traci.trafficlight.setPhase("n1", 1)
        if step % 20 == 0:
            traci.trafficlight.setPhase("n1", 0)
        print("step:" + str(step))
    traci.close()
    sys.stdout.flush()


def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option("--nogui", action="store_true",
                         default=False, help="run the commandline version of sumo")
    options, args = optParser.parse_args()
    return options


def ipa_gradient_mehtod(initial_par, lam_1, lam_2, run_time, iters_per_par, total_iter_num, stepsize, sumoBinary,
                        print_mode):

    par_list = [[i] for i in initial_par]

    mean_queue_length_list = []
    mean_queue_length_1_list = []
    mean_queue_length_2_list = []
    d_L_list = []
    iter_num = 0

    while iter_num < total_iter_num:
        iter_num += 1

        if iter_num==10:
            stepsize = stepsize/2.
        if iter_num==20:
            stepsize = stepsize/2.
    #     if iter_num == 50:
    #         stepsize = stepsize / 2.
    #     if iter_num==100:
    #         stepsize = stepsize/2.
    #     if iter_num==130:
    #         stepsize = stepsize/2.


        # stepsize = (1/iter_num*1.0)**1.5

        d_L, mean_queue_length, output1, output2 = run([par[-1] for par in par_list], lam_1, lam_2,
                                                       run_time, iters_per_par, sumoBinary, print_mode)

        # update performance
        mean_queue_length_list.append(mean_queue_length)
        mean_queue_length_1_list.append(output1)
        mean_queue_length_2_list.append(output2)
        d_L_list.append(d_L)
        print('****************************************************************************')
        print("dL: " + str(d_L))
        for par in par_list:
            print([round(i, 3) for i in par])

        print("------")
        print([round(i, 3) for i in mean_queue_length_list])
        print([round(i, 3) for i in mean_queue_length_1_list])
        print([round(i, 3) for i in mean_queue_length_2_list])
        # print([round(i, 3) for i in dL_dtheta1_list])
        # print([round(i, 3) for i in dL_dtheta2_list])
        print("------")
        for i in range(len(d_L)):
            print([round(dl[i], 3) for dl in d_L_list])

        print('****************************************************************************')

        # update parameters

        #theta_1_min
        par_list[0].append(max(0.1, par_list[0][-1] - stepsize * d_L[0]))
        #theta_1_max
        par_list[1].append(max(par_list[0][-1], par_list[1][-1] - stepsize * d_L[1]))
        #theta_2_min
        par_list[2].append(max(0.1, par_list[2][-1] - stepsize * d_L[2]))
        #theta_2_max
        par_list[3].append(max(par_list[2][-1], par_list[3][-1] - stepsize * d_L[3]))
        #s_1
        par_list[4].append(max(0.1, par_list[4][-1] - stepsize * d_L[4]))
        #s_2
        par_list[5].append(max(0.1, par_list[5][-1] - stepsize * d_L[5]))


def brute_force_mehtod(initial_par, lam_1, lam_2, run_time, iters_per_par, total_iter_num, par_change_idx, stepsize,
                       sumoBinary, print_mode):
    mean_queue_length_list = []
    queue_length_1_list = []
    queue_length_2_list = []
    dL_dtheta1_list = []
    dL_dtheta2_list = []

    iter_num = 0
    updated_par = initial_par

    while iter_num < total_iter_num:
        iter_num += 1
        print('****************************************************************************')
        print(updated_par)
        d_L, mean_queue_length, queue_length_1, queue_length_2 = run(updated_par, lam_1, lam_2, run_time, iters_per_par,
                                                                     sumoBinary, print_mode)
        # print("d_L:"+str(d_L))
        updated_par[par_change_idx] += stepsize
        # update performance
        mean_queue_length_list.append(round(mean_queue_length, 3))
        queue_length_1_list.append(round(queue_length_1, 3))
        queue_length_2_list.append(round(queue_length_2, 3))
        dL_dtheta1_list.append(round(d_L[1], 3))
        dL_dtheta2_list.append(round(d_L[3], 3))

        print(mean_queue_length_list)
        print(queue_length_1_list)
        print(queue_length_2_list)
        # print(dL_dtheta1_list)
        # print(dL_dtheta2_list)
        print('****************************************************************************')
    return mean_queue_length_list, queue_length_1_list, queue_length_2_list, dL_dtheta1_list, dL_dtheta2_list


if __name__ == "__main__":
    sumoBinary = checkBinary('sumo')
    # sumoBinary = checkBinary('sumo-gui')

    ipa_gradient_mehtod(initial_par=[5, 10, 10, 40, 3, 4], lam_1=1/6., lam_2=1/10., run_time=1000, iters_per_par=20,
                        total_iter_num=50, stepsize=10, sumoBinary=sumoBinary, print_mode=False)

    # brute_force_mehtod(initial_par=[10, 20, 30, 40, 100, 100], lam_1=1/2., lam_2=1/5., run_time=2000, iters_per_par=10,
    #                    total_iter_num=20, par_change_idx=1, stepsize=1, sumoBinary=sumoBinary, print_mode=False)

    # bf_chart = pd.DataFrame(
    #     columns=["theta_1_max", "theta_2_max", "total_queue_length", "queue_length_1", "queue_length_2", "dLdtheta_1",
    #              "dLdtheta_2"])
    #
    # for theta_1 in range(10, 40):
    #     mean_queue_length_list, queue_length_1_list, queue_length_2_list, dL_dtheta1_list, dL_dtheta2_list = \
    #         brute_force_mehtod(initial_par=[1, theta_1, 1, 10, 100, 100], lam_1=1 / 5., lam_2=1 / 5., run_time=2000,
    #                            iters_per_par=1, total_iter_num=30, par_change_idx=3, stepsize=1, sumoBinary=sumoBinary,
    #                            print_mode=True)
    #     bf_chart_ = pd.DataFrame(
    #         columns=["theta_1_max", "theta_2_max", "total_queue_length", "queue_length_1", "queue_length_2",
    #                  "dLdtheta_1", "dLdtheta_2"])
    #     bf_chart_["theta_1_max"] = [theta_1] * len(mean_queue_length_list)
    #     bf_chart_["theta_2_max"] = range(10, 40)
    #     bf_chart_["total_queue_length"] = mean_queue_length_list
    #     bf_chart_["queue_length_1"] = queue_length_1_list
    #     bf_chart_["queue_length_2"] = queue_length_2_list
    #     bf_chart_["dLdtheta_1"] = dL_dtheta1_list
    #     bf_chart_["dLdtheta_2"] = dL_dtheta2_list
    #
    #     bf_chart = pd.concat([bf_chart, bf_chart_])
    #
    # bf_chart.to_csv('bf_chart.csv', index=False)
