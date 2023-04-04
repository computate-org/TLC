from flask import Flask
from threading import Event
from consumer import FlaskKafka
from kafka import  KafkaProducer
from sumolib import checkBinary  # noqa
import json
import signal
import os
import sys
import traceback
import main_pedestrian

app = Flask(__name__)

INTERRUPT_EVENT = Event()

kafka_brokers = os.environ.get('KAFKA_BROKERS') or "kafka0.apps-crc.testing:32000"
kafka_group = os.environ.get('KAFKA_GROUP') or "smartvillage-kafka-group"
kafka_topic_sumo_run = os.environ.get('KAFKA_TOPIC_SUMO_RUN') or "smartvillage-sumo-run"
kafka_topic_sumo_run_report = os.environ.get('KAFKA_TOPIC_SUMO_RUN_REPORT') or "smartvillage-sumo-run-report"
kafka_security_protocol = os.environ.get('KAFKA_SECURITY_PROTOCOL') or "SSL"
# Run: oc -n smart-village-view get secret/smartvillage-kafka-cluster-ca-cert -o jsonpath="{.data.ca\.crt}"
kafka_ssl_cafile = os.environ.get('KAFKA_SSL_CAFILE') or "/usr/local/src/TLC/ca.crt"
# Run: oc -n smart-village-view get secret/smartvillage-kafka-cluster-ca-cert -o jsonpath="{.data.ca\.crt}"
kafka_ssl_certfile = os.environ.get('KAFKA_SSL_CERTFILE') or "/usr/local/src/TLC/user.crt"
# Run: oc -n smart-village-view get secret/smartvillage-kafka-cluster-ca-cert -o jsonpath="{.data.ca\.password}"
kafka_ssl_keyfile = os.environ.get('KAFKA_SSL_KEYFILE') or "/usr/local/src/TLC/user.key"
kafka_max_poll_records = int(os.environ.get('KAFKA_MAX_POLL_RECORDS') or "1")
kafka_max_poll_interval_ms = int(os.environ.get('KAFKA_MAX_POLL_INTERVAL_MS') or "3000000")
if("SSL" == kafka_security_protocol):
    bus = FlaskKafka(INTERRUPT_EVENT
             , bootstrap_servers=",".join([kafka_brokers])
             , group_id=kafka_group
             , security_protocol=kafka_security_protocol
             , ssl_cafile=kafka_ssl_cafile
             , ssl_certfile=kafka_ssl_certfile
             , ssl_keyfile=kafka_ssl_keyfile
             , max_poll_interval_ms=kafka_max_poll_interval_ms
             , max_poll_records=kafka_max_poll_records
             )
else:
    bus = FlaskKafka(INTERRUPT_EVENT
             , bootstrap_servers=",".join([kafka_brokers])
             , group_id=kafka_group
             , security_protocol=kafka_security_protocol
             , max_poll_interval_ms=kafka_max_poll_interval_ms
             , max_poll_records=kafka_max_poll_records
             )

@bus.handle(kafka_topic_sumo_run)
def test_topic_handler(msg):
    try:
        print("received message from %s topic: %s" % (kafka_topic_sumo_run, msg))
        sumoBinary = checkBinary('sumo')
        simulation_report = json.loads(msg.value)
    
        initial_par = simulation_report.get('paramInitialPar', [10., 20., 30., 50., 10., 10., 8., 8., 5., 5.])
        initial_par = [float(s) for s in initial_par]

        lam = simulation_report.get('paramLam', [10., 10., 6., 6.])
        lam = [float(s) for s in lam]

        demand_scale = simulation_report.get('paramDemandScale', [1., 1.])
        step_size = float(simulation_report.get('paramStepSize', 1.))
        run_time = int(simulation_report.get('paramRunTime', 1000))
        total_iter_num = int(simulation_report.get('paramTotalIterNum', 10))
        iters_per_par = int(simulation_report.get('paramItersPerPar', 5))

        if("SSL" == kafka_security_protocol):
            producer = KafkaProducer(
                    bootstrap_servers=kafka_brokers
                    , security_protocol=kafka_security_protocol
                    , ssl_cafile=kafka_ssl_cafile
                    , ssl_certfile=kafka_ssl_certfile
                    , ssl_keyfile=kafka_ssl_keyfile
                    )
        else:
            producer = KafkaProducer(
                    bootstrap_servers=kafka_brokers
                    , security_protocol=kafka_security_protocol
                    , sasl_mechanism="PLAIN"
                    )

        result = { "pk": simulation_report.get("pk"), "setUpdatedParameters": [], "setUpdatedPerformance": [], "setReportStatus": "Running" }
        producer.send(kafka_topic_sumo_run_report, json.dumps(result).encode('utf-8'))
    
        updated_parameters, updated_performance = main_pedestrian.ipa_gradient_method_pedestrian(
                producer
                , simulation_report
                , initial_par=initial_par
                , lam=lam
                , demand_scale=demand_scale
                , step_size=step_size
                , run_time=run_time
                , total_iter_num=total_iter_num
                , iters_per_par=iters_per_par
                , print_mode=False)

        result = { "pk": simulation_report.get("pk"), "setReportStatus": "Completed" }
        producer.send(kafka_topic_sumo_run_report, json.dumps(result).encode('utf-8'))
    except Exception as e:
        ex_type, ex_value, ex_traceback = sys.exc_info()
        # Extract unformatter stack traces as tuples
        trace_back = traceback.extract_tb(ex_traceback)
    
        # Format stacktrace
        stack_trace = list()
    
        for trace in trace_back:
            stack_trace.append("File : %s , Line : %d, Func.Name : %s, Message : %s" % (trace[0], trace[1], trace[2], trace[3]))
    
        print("%s occured processing a message on %s topic: %s\n%s" % (ex_type.__name__, kafka_topic_sumo_run, ex_value, '\n'.join(stack_trace)))

def listen_kill_server():
    signal.signal(signal.SIGTERM, bus.interrupted_process)
    signal.signal(signal.SIGINT, bus.interrupted_process)
    signal.signal(signal.SIGQUIT, bus.interrupted_process)
    signal.signal(signal.SIGHUP, bus.interrupted_process)

@app.route('/')
def hello():
    return "Hello World!"

def start():
    flask_port = os.environ.get('FLASK_PORT') or 8081
    flask_port = int(flask_port)

    bus.run()
    listen_kill_server()
    app.run(port=flask_port, host='0.0.0.0')

if __name__ == "__main__":
    start()

