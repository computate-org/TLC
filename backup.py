# alpha1=0
            if det_veh_num[0][-2] > 0 and det_veh_num[0][-1] == 0:
                if (queue_length_2[-1] > 0 and not f2) or (queue_length_2[-1] == 0 and f1 and not f2):
                    # switch light
                    traci.trafficlight.setPhase(TLID, 0)
                    last_switch = 0.
                    last_push_button = 0.
                    green_1 = False

                    d_tau = [0] * len(theta)
                    if print_mode:
                        print("alpha1=0")

            # z1=theta_1_max
            elif last_switch >= theta[1] > (last_switch - step_size) and queue_length_2[-1] > 0:
                # switch light
                traci.trafficlight.setPhase(TLID, 0)
                last_switch = 0.
                last_push_button = 0.
                green_1 = False

                # update event time derivative
                d_tau[1] = d_tau[1] + 1
                if print_mode:
                    print("z1=theta_1_max")

            # z1=theta_1_min
            elif last_switch >= theta[0] > (last_switch - 1):
                if (f1 and not f2) or (queue_length_1[-1] < theta[6] and queue_length_2[-1] >= theta[7]):
                    # switch light
                    traci.trafficlight.setPhase(TLID, 0)
                    last_switch = 0.
                    last_push_button = 0.
                    green_1 = False
                    # update event time derivative
                    d_tau[0] = d_tau[0] + 1
                    if print_mode:
                        print("z1=theta_1_min")

            # x1 = s1(a)
            elif queue_length_1[-2] > theta[6] >= queue_length_1[-1]:
                if last_switch >= theta[0] and queue_length_2[-1] >= theta[7]:
                    # switch light
                    traci.trafficlight.setPhase(TLID, 0)
                    green_1 = False
                    last_switch = 0.
                    last_push_button = 0.
                    # update event time derivative
                    if alpha_1_rate - h1 == 0:
                        print("WARNING: alpha_1_rate - h1 == 0 when x1 = s1(a)/ G2R1")
                    else:
                        d_tau = (np.array([0, 0, 0, 0, 0, 0, 1, 0, 0, 0]) - np.array(d_x1)) / (alpha_1_rate - h1)
                    if print_mode:
                        print("x1 = s1(a) G2R1")

            # x2 = s2(b)
            elif queue_length_2[-2] < theta[7] <= queue_length_2[-1]:
                if last_switch >= theta[0] and queue_length_1[-1] < theta[6]:
                    # switch light
                    traci.trafficlight.setPhase(TLID, 0)
                    green_1 = False
                    last_switch = 0.
                    last_push_button = 0.
                    # update event time derivative
                    if alpha_2_rate == 0:
                        alpha_2_rate = max(len(set(sum(alpha_2, []))) / (10. / step_size), 0.1)
                        if print_mode:
                            print("WARING: alpha_2_rate==0 when x2 = s2(b), use alternative " + str(alpha_2_rate))
                    else:
                        d_tau = (np.array([0, 0, 0, 0, 0, 0, 0, 1, 0, 0]) - np.array(d_x2)) / alpha_2_rate
                    if print_mode:
                        print("x2 = s2(b) G2R1")

            # x4=s4(a) --> f2=0
            elif queue_length_4[-2] > theta[9] >= queue_length_4[-1]:
                if (det_veh_num_1[-1] == 0 and queue_length_2[-1] == 0 and f1) or \
                        (det_veh_num_1[-1] == 0 and queue_length_2[-1] > 0) or \
                        (det_veh_num_1[-1] > 0 and queue_length_2[-1] > 0 and last_switch > theta[0] and not f1):
                    # switch light
                    traci.trafficlight.setPhase(TLID, 0)
                    green_1 = False
                    last_switch = 0.
                    last_push_button = 0.
                    if alpha_4_rate - h4 == 0:
                        print("WARNING: alpha_4_rate - h4 == 0 when x4 = s4(a)/ G2R1")
                    else:
                        d_tau = (np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 1]) - np.array(d_x4)) / (alpha_4_rate - h4)
                    if print_mode:
                        print("x4 = s4(a) G2R1")

            # x3 = s3(b) --> f1=1
            elif queue_length_3[-2] < theta[8] <= queue_length_3[-1]:
                if (det_veh_num_1[-1] > 0 and last_switch >= theta[0] and not f2) or \
                        (det_veh_num_1[-1] == 0 and queue_length_2[-1] == 0 and not f2):
                    traci.trafficlight.setPhase(TLID, 0)
                    green_1 = False
                    last_switch = 0.
                    last_push_button = 0.
                    # update event time derivative
                    if alpha_3_rate == 0:
                        alpha_3_rate = max(len(set(sum(alpha_3, []))) / (10. / step_size), 0.1)
                        if print_mode:
                            print("WARING: alpha_3_rate==0 when x3 = s3(b), use alternative " + str(alpha_3_rate))
                    else:
                        d_tau = (np.array([0, 0, 0, 0, 0, 0, 0, 0, 1, 0]) - np.array(d_x3)) / alpha_3_rate
                    if print_mode:
                        print("x3 = s3(b) G2R1")

            #   y1 = theta3 --> f1 = 1
            elif last_push_button >= theta[4] > (last_push_button - step_size):
                if (det_veh_num_1[-1] > 0 and last_switch >= theta[0] and not f2) or \
                        (det_veh_num_1[-1] == 0 and queue_length_2[-1] == 0 and not f2):

                    traci.trafficlight.setPhase(TLID, 0)
                    green_1 = False
                    last_switch = 0.
                    last_push_button = 0.
                    d_tau = [0, 0, 0, 0, 1, 0, 0, 0, 0, 0]
                    if print_mode:
                        print("y1 = theta3  G2R1")

            else:
                d_tau = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]


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
                traci.trafficlight.setPhase(TLID, 1)
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
                traci.trafficlight.setPhase(TLID, 1)
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





# state derivative
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
