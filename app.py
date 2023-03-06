from flask import Flask
from threading import Event
from flask_kafka import FlaskKafka
from kafka import  KafkaProducer
from sumolib import checkBinary  # noqa
import json
import signal
import os
import main_pedestrian

app = Flask(__name__)

INTERRUPT_EVENT = Event()

kafka_brokers = os.environ.get('KAFKA_BROKERS') or "smartvillage-kafka-kafka-brokers.smartabyar-smartvillage.svc.cluster.local:9092"
kafka_group = os.environ.get('KAFKA_GROUP') or "smartvillage-kafka-group"
kafka_topic_sumo_run = os.environ.get('KAFKA_TOPIC_SUMO_RUN') or "smartvillage-sumo-run"
kafka_topic_sumo_run_report = os.environ.get('KAFKA_TOPIC_SUMO_RUN_REPORT') or "smartvillage-sumo-run-report"
bus = FlaskKafka(INTERRUPT_EVENT,
         bootstrap_servers=",".join([kafka_brokers]),
         group_id=kafka_group
         )

@bus.handle(kafka_topic_sumo_run)
def test_topic_handler(msg):
    print("received message from %s topic: %s" % (kafka_topic_sumo_run, msg))
    sumoBinary = checkBinary('sumo')
    body = json.loads(msg.value)

    initial_par = body.get('paramInitialPar', [10, 20, 30, 50, 10, 10, 8, 8, 5, 5])
    lam = body.get('paramLam', [10, 10, 6, 6])
    demand_scale = body.get('paramDemandScale', 1)
    step_size = body.get('paramStepSize', 1)
    par_update_step_size = body.get('paramUpdateStepSize', 30)
    run_time = body.get('paramRunTime', 1000)
    total_iter_num = body.get('paramTotalIterNum', 10)
    iters_per_par = body.get('paramItersPerPar', 5)

    main_pedestrian.ipa_gradient_method_pedestrian(
            initial_par=initial_par
            , lam=lam
            , demand_scale=demand_scale
            , step_size=step_size
            , par_update_step_size=par_update_step_size
            , run_time=run_time
            , total_iter_num=total_iter_num
            , iters_per_par=iters_per_par
            , print_mode=False)

def listen_kill_server():
    signal.signal(signal.SIGTERM, bus.interrupted_process)
    signal.signal(signal.SIGINT, bus.interrupted_process)
    signal.signal(signal.SIGQUIT, bus.interrupted_process)
    signal.signal(signal.SIGHUP, bus.interrupted_process)

# main function: time driven parameter updating with parallel running
def ipa_gradient_method_pedestrian(initial_par, lam, demand_scale, step_size, par_update_step_size, run_time,
                                   total_iter_num, iters_per_par, print_mode):
    """ run ipa method to update par"""

    par_list = [[i] for i in initial_par]

    d_L_list = []
    performance_list = []

    iter_num = 0

    while iter_num < total_iter_num:
        iter_num += 1

        d_L, performance = repeat_iters([par[-1] for par in par_list], lam, demand_scale, step_size, run_time,
                                        iters_per_par, print_mode)

        # update performance
        performance_list.append(performance)
        d_L_list.append(d_L)
        for par in par_list:
            print(str([round(i, 3) for i in par]) + ",")

        print("------")
        for j in range(5):
            print(str([round(i, 3) for i in [i[j] for i in performance_list]]) + ",")
        print("------")
        for i in range(len(d_L)):
            print(str([round(dl[i], 3) for dl in d_L_list]) + ",")

        # update parameters
        par_update_step_size = get_par_update_step_size(par_update_step_size, d_L_list, 1., 4., 0.5, 1.5)
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

@app.route('/')
def hello():
    # producer = KafkaProducer(bootstrap_servers=kafka_brokers)
    # producer.send(kafka_topic_sumo_run, b'{ "paramTotalIterNum": 1, "paramItersPerPar": 1 }')
    return "Hello World!"

def start():
    flask_port = os.environ.get('FLASK_PORT') or 8081
    flask_port = int(flask_port)

    bus.run()
    listen_kill_server()
    app.run(port=flask_port, host='0.0.0.0')

if __name__ == "__main__":
    start()

