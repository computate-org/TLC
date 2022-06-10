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
import random
import math


def expo_gen(lam):
    return -1 / lam * math.log(1 - random.random())


def generate_routefile(run_time, lam1, lam2):
    # random.seed(22)  # make tests reproducible
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


def one_iter(theta_1_min, theta_1_max, theta_2_min, theta_2_max, s_1, s_2, lam_1, lam_2, run_time, print_mode,
             sumoBinary):
    generate_routefile(run_time, lam_1, lam_2)
    # one run renewal
    total_length = 0
    par_direct = [0] * 6
    step = 0

    # derivatives
    d_tau = [0] * 6
    d_x1 = [0] * 6
    d_x2 = [0] * 6
    d_L = [0] * 6
    NEP_1 = False
    NEP_2 = False

    # arrival and deprat rate
    alpha_1 = []  # number of vehicle enter this road in the last time step)
    beta_1 = []  # number of vehicle pass the cross in the last time step)
    alpha_2 = []
    beta_2 = []

    h1 = 0.8
    h2 = 0.8

    true_dyn_1 = 0
    true_dyn_2 = 0

    norm_coeff = 1.5  # a vehicle might take several time units to pass a detector

    det_veh_num_1 = [0]  # vehicle num at each time step
    det_veh_num_2 = [0]

    traci.start([sumoBinary, "-c", "tlc_single_straight.sumocfg", "--no-step-log", "--no-warnings"])
    # we start with phase 0 --- 42 green (road 2)
    # <phase duration="200" state="GrGr"/>
    traci.trafficlight.setPhase("n1", 0)
    last_switch = 0
    green_1 = False

    while step < run_time - 100:
        traci.simulationStep()
        last_switch += 1
        step += 1
        # update det_veh_num list
        det_veh_num_1.append(traci.lanearea.getLastStepVehicleNumber("det_13"))
        det_veh_num_2.append(traci.lanearea.getLastStepVehicleNumber("det_42"))
        if print_mode:
            print("current detected vehicle number: " + str(det_veh_num_1[-1]) + "  " + str(det_veh_num_2[-1]))
            print("jam length: " + str(traci.lanearea.getJamLengthVehicle("det_13")) + "  " + str(
                traci.lanearea.getJamLengthVehicle("det_42")))

        # update arrival & departure rate
        if green_1:
            beta_1.append(traci.inductionloop.getLastStepVehicleNumber("det_13_0"))
            beta_2 = [0]
        else:
            beta_1 = [0]
            beta_2.append(traci.inductionloop.getLastStepVehicleNumber("det_42_0"))

        alpha_1.append(traci.inductionloop.getLastStepVehicleNumber("det_13_1"))
        alpha_2.append(traci.inductionloop.getLastStepVehicleNumber("det_42_4"))
        if len(alpha_1) > 10:
            alpha_1.pop(0)
            alpha_2.pop(0)

        if len(beta_1) > 10:
            beta_1.pop(0)
        if len(beta_2) > 10:
            beta_2.pop(0)
        alpha_1_rate = 0.7 * np.mean(alpha_1) / norm_coeff + 0.3 * lam_1
        alpha_2_rate = 0.7 * np.mean(alpha_2) / norm_coeff + 0.3 * lam_2
        if beta_1 == [0]:
            beta_1_rate = 0
        else:
            beta_1_rate = 0.7 * np.mean(beta_1) / norm_coeff + 0.3 * h1
        if beta_2 == [0]:
            beta_2_rate = 0
        else:
            beta_2_rate = 0.7 * np.mean(beta_2) / norm_coeff + 0.3 * h2
        true_dyn_1 = np.mean(alpha_1) - np.mean(beta_1)
        true_dyn_2 = np.mean(alpha_2) - np.mean(beta_2)

        if print_mode:
            print('------------------------------------')
            print("alpha:" + str(alpha_1) + "  " + str(alpha_2))
            print("arrival rate: " + str(alpha_1_rate) + "  " + str(alpha_2_rate))
            print("beta:" + str(beta_1) + "  " + str(beta_2))
            print("departure rate: " + str(beta_1_rate) + "  " + str(beta_2_rate))
            print("true dynamics: " + str(true_dyn_1) + "  " + str(true_dyn_2))
            print('-------------------------------------')

        # update event time derivative
        if green_1:
            if det_veh_num_2[-1] == 0:
                d_tau = [0, 0, 0, 0, 0, 0]
                # if print_mode:
                #     print("alpha2=0")

            # alpha1=0, alpha2>0
            elif det_veh_num_1[-1] == 0:
                # switch light
                traci.trafficlight.setPhase("n1", 0)
                last_switch = 0
                green_1 = False

                d_tau = [0, 0, 0, 0, 0, 0]
                if print_mode:
                    print("alpha1=0, alpha2>0")

            # z1=theta_1_max
            elif last_switch == theta_1_max:
                # switch light
                traci.trafficlight.setPhase("n1", 0)
                last_switch = 0
                green_1 = False

                # update event time derivative
                d_tau[1] = d_tau[1] + 1
                if print_mode:
                    print("z1=theta_1_max")

            # z1=theta_1_min
            elif last_switch == theta_1_min \
                    and det_veh_num_2[-1] >= s_2 and det_veh_num_1[-1] < s_1:
                # switch light
                traci.trafficlight.setPhase("n1", 0)
                last_switch = 0
                green_1 = False
                # update event time derivative
                d_tau[0] = d_tau[0] + 1
                if print_mode:
                    print("z1=theta_1_min")

            # x1 = s1(a)/ G2R1
            elif last_switch >= theta_1_min and det_veh_num_2[-1] >= s_2 \
                    and det_veh_num_1[-2] > s_1 >= det_veh_num_1[-1]:
                # switch light
                traci.trafficlight.setPhase("n1", 0)
                green_1 = False
                last_switch = 0
                # update event time derivative
                # if alpha_1_rate - beta_1_rate == 0:
                if true_dyn_1 == 0:
                    continue
                else:
                    d_tau = (np.array([0, 0, 0, 0, 1, 0]) - np.array(d_x1)) / (true_dyn_1)
                if print_mode:
                    print("x1 = s1(a) G2R1")

            # x2 = s2(b)
            elif last_switch >= theta_1_min and det_veh_num_1[-1] < s_1 \
                    and det_veh_num_2[-2] < s_2 <= det_veh_num_2[-1]:
                # switch light
                traci.trafficlight.setPhase("n1", 0)
                green_1 = False
                last_switch = 0
                # update event time derivative
                if alpha_1_rate == 0:
                    continue
                else:
                    d_tau = (np.array([0, 0, 0, 0, 0, 1]) - np.array(d_x1)) / alpha_1_rate
                if print_mode:
                    print("x2 = s2(b) G2R1")

            else:
                d_tau = [0, 0, 0, 0, 0, 0]


        else:

            if det_veh_num_1[-1] == 0:
                d_tau = [0, 0, 0, 0, 0, 0]
                # if print_mode:
                #     print("alpha1=0")

            # alpha2=0, alpha1>0
            elif det_veh_num_2[-1] == 0:
                # switch light
                traci.trafficlight.setPhase("n1", 1)
                last_switch = 0
                green_1 = True
                d_tau = [0, 0, 0, 0, 0, 0]
                if print_mode:
                    print("alpha2=0, alpha1>0")

            # z2=theta_2_max
            elif last_switch == theta_2_max:
                # switch light
                traci.trafficlight.setPhase("n1", 1)
                last_switch = 0
                green_1 = True

                # update event time derivative
                d_tau[3] = d_tau[3] + 1
                if print_mode:
                    print("z2=theta_2_max")

            # z2=theta_2_min
            elif last_switch == theta_2_min \
                    and det_veh_num_1[-1] >= s_1 and det_veh_num_2[-1] < s_2:
                # switch light
                traci.trafficlight.setPhase("n1", 1)
                last_switch = 0
                green_1 = True
                # update event time derivative
                d_tau[2] = d_tau[2] + 1
                if print_mode:
                    print("z2=theta_2_min")

            # x2 = s2(a)/ G2R2
            elif last_switch >= theta_2_min and det_veh_num_1[-1] >= s_1 \
                    and det_veh_num_2[-2] > s_2 >= det_veh_num_2[-1]:
                # switch light
                traci.trafficlight.setPhase("n1", 1)
                green_1 = True
                last_switch = 0
                # update event time derivative
                # if alpha_2_rate - beta_2_rate == 0:
                if true_dyn_2 == 0:
                    continue
                else:
                    d_tau = (np.array([0, 0, 0, 0, 0, 1]) - np.array(d_x2)) / (true_dyn_2)
                if print_mode:
                    print("x2 = s2(a) G2R2")



            # x1 = s1(b)
            elif last_switch >= theta_2_min and det_veh_num_2[-1] < s_2 \
                    and det_veh_num_1[-2] < s_1 <= det_veh_num_1[-1]:
                # switch light
                traci.trafficlight.setPhase("n1", 1)
                green_1 = True
                last_switch = 0
                # update event time derivative
                if alpha_2_rate == 0:
                    continue
                else:
                    d_tau = (np.array([0, 0, 0, 0, 1, 0]) - np.array(d_x2)) / alpha_2_rate
                if print_mode:
                    print("x1 = s1(b) G2R2")

            else:
                d_tau = [0, 0, 0, 0, 0, 0]

        if print_mode:
            print("time:" + str(step))
            print("last_switch:" + str(last_switch))
            print("green in 13:" + str(green_1))
            # print("jam_length_vehilce:  " + str(traci.lanearea.getJamLengthVehicle("det_13")))
            # print("last_step_veh_num:  " + str(traci.lanearea.getLastStepVehicleNumber("det_13")))
            # print("alpha_1: "+str(np.mean(alpha_1)))
            # print("beta_1: "+str(np.mean(beta_1)))
            print("d_tau:" + str(d_tau))

        # update state derivative

        # first consider road 13

        # EP
        if print_mode:
            print("------------")
        if (not NEP_1) and det_veh_num_1[-1] == 0:
            d_x1 = [0, 0, 0, 0, 0, 0]
            if print_mode:
                print("event at EP_1")
        # S_1
        elif (not NEP_1) and det_veh_num_1[-1] > 0:
            NEP_1 = True
            # # induced by G2R1
            # if last_switch == 0 and not green_1:
            #     d_x1 = - alpha_1_rate * np.array(d_tau)
            #     if print_mode:
            #         print("event at S_1 with light switch")

            # else:
            d_x1 = [0, 0, 0, 0, 0, 0]
            if print_mode:
                print("event at S_1 NOT with light switch")
        # E_1
        elif NEP_1 and det_veh_num_1[-1] == 0:
            NEP_1 = False
            d_x1 = [0, 0, 0, 0, 0, 0]
            if print_mode:
                print("event at E_1")

        # G2R1
        elif last_switch == 0 and not green_1:
            d_x1 = d_x1 - h1 * np.array(d_tau)
            if print_mode:
                print("event at G2R_1")

        # R2G1
        elif last_switch == 0 and green_1:
            d_x1 = d_x1 + h2 * np.array(d_tau)
            if print_mode:
                print("event at R2G_1")

        # for road 42
        # EP
        if not NEP_2 and det_veh_num_2[-1] == 0:
            if print_mode:
                print("event at EP_2")

            d_x2 = [0, 0, 0, 0, 0, 0]
        # S_2
        elif not NEP_2 and det_veh_num_2[-1] > 0:
            NEP_2 = True
            # # induced by G2R2
            # if last_switch == 0 and green_1:
            #     if print_mode:
            #         print("event at S_2 with light switch")
            #     d_x2 = - alpha_2_rate * np.array(d_tau)
            # else:
            d_x2 = [0, 0, 0, 0, 0, 0]
            if print_mode:
                print("event at S_2 NOT with light switch")
        # E_2
        elif NEP_2 and det_veh_num_2[-1] == 0:
            NEP_2 = False
            d_x2 = [0, 0, 0, 0, 0, 0]
            if print_mode:
                print("event at E_2")

        # G2R2
        elif last_switch == 0 and green_1:
            d_x2 = d_x2 - h2 * np.array(d_tau)
            if print_mode:
                print("event at G2R_2")

        # R2G2
        elif last_switch == 0 and not green_1:
            d_x2 = d_x2 + h2 * np.array(d_tau)
            if print_mode:
                print("event at R2G_2 by light switch")

        d_L = d_L + np.array(d_x1) + np.array(d_x2)
        if print_mode:
            print('----------------')
            print("d_x1: " + str(d_x1))
            print("d_x2: " + str(d_x2))
            print('===========================================================================================')

    traci.close()
    sys.stdout.flush()
    return np.array(d_L) * 1.0 / run_time, (sum(det_veh_num_1[100:]) + sum(det_veh_num_2[100:])) * 1.0 / (
            run_time - 200)


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

    iters = 0
    while iters < iters_per_par:
        iters += 1
        d_L, mean_queue_length = one_iter(theta_1_min, theta_1_max, theta_2_min, theta_2_max, s_1, s_2, lam_1, lam_2,
                                          run_time, print_mode, sumoBinary)
        d_L_list.append(d_L)
        mean_queue_length_list.append(mean_queue_length)

        if print_mode:
            print("d_L:")
            pprint(d_L_list)
    print("==================================================================================================")
    print("mean_queue_length with par" + str(par) + "----" + str(mean_queue_length_list))
    print("==================================================================================================")

    return np.mean(d_L_list, 0), np.mean(mean_queue_length_list)


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
    theta_1_min_list = [initial_par[0]]
    theta_1_max_list = [initial_par[1]]
    theta_2_min_list = [initial_par[2]]
    theta_2_max_list = [initial_par[3]]
    s_1_list = [initial_par[4]]
    s_2_list = [initial_par[5]]

    mean_queue_length_list = []
    iter_num = 0

    while iter_num < total_iter_num:
        iter_num += 1

        d_L, mean_queue_length = run(
            [theta_1_min_list[-1], theta_1_max_list[-1], theta_2_min_list[-1], theta_2_max_list[-1],
             s_1_list[-1], s_2_list[-1]], lam_1, lam_2, run_time, iters_per_par,
            sumoBinary, print_mode)
        # update parameters
        theta_1_min_list.append(max(0.1, theta_1_min_list[-1] - stepsize * d_L[0]))
        theta_1_max_list.append(max(theta_1_min_list[-1], theta_1_max_list[-1] - stepsize * d_L[1]))
        theta_2_min_list.append(max(0.1, theta_2_min_list[-1] - stepsize * d_L[2]))
        theta_2_max_list.append(max(theta_2_min_list[-1], theta_2_max_list[-1] - stepsize * d_L[3]))
        s_1_list.append(max(0.1, s_1_list[-1] - stepsize * d_L[4]))
        s_2_list.append(max(0.1, s_2_list[-1] - stepsize * d_L[5]))

        # update performance
        mean_queue_length_list.append(mean_queue_length)
        print('****************************************************************************')
        print("dL: " + str(d_L))
        print([round(i, 3) for i in theta_1_min_list])
        print([round(i, 3) for i in theta_1_max_list])
        print([round(i, 3) for i in theta_2_min_list])
        print([round(i, 3) for i in theta_2_max_list])
        print([round(i, 3) for i in s_1_list])
        print([round(i, 3) for i in s_2_list])
        print([round(i, 3) for i in mean_queue_length_list])
        print('****************************************************************************')


def brute_force_mehtod(initial_par, lam_1, lam_2, run_time, iters_per_par, total_iter_num, par_change_idx, stepsize,
                       sumoBinary, print_mode):
    mean_queue_length_list = []
    iter_num = 0
    updated_par = initial_par

    while iter_num < total_iter_num:
        iter_num += 1
        print('****************************************************************************')
        print(updated_par)

        d_L, mean_queue_length = run(updated_par, lam_1, lam_2, run_time, iters_per_par,
                                     sumoBinary, print_mode)

        updated_par[par_change_idx] += stepsize
        # update performance
        mean_queue_length_list.append(mean_queue_length)

        print([round(i, 3) for i in mean_queue_length_list])
        print('****************************************************************************')


if __name__ == "__main__":
    sumoBinary = checkBinary('sumo')
    # sumoBinary = checkBinary('sumo-gui')
    # generate_routefile(3600, 1/1., 1./10)

    # ipa_gradient_mehtod(initial_par=[20, 40, 40, 60, 4, 4], lam_1=1/3., lam_2=1/5., run_time=600, iters_per_par=1,
    #                     total_iter_num=20, stepsize=0.1, sumoBinary=sumoBinary, print_mode=False)

    brute_force_mehtod(initial_par=[20, 40, 40, 60, 4, 4], lam_1=1 / 3., lam_2=1 / 5., run_time=600, iters_per_par=10,
                       total_iter_num=20, par_change_idx=0, stepsize=1, sumoBinary=sumoBinary, print_mode=False)
