from flask import Flask
from threading import Event
from flask_kafka import FlaskKafka
from kafka import  KafkaProducer
from sumolib import checkBinary  # noqa
import signal
import os
import main_pedestrian

app = Flask(__name__)

INTERRUPT_EVENT = Event()

#kafka_brokers = os.environ.get('KAFKA_BROKERS') or "smartvillage-kafka-kafka-brokers.smartabyar-smartvillage.svc.cluster.local:9092"
#kafka_group = os.environ.get('KAFKA_GROUP') or "smartvillage-kafka-group"
#kafka_topic = os.environ.get('KAFKA_TOPIC') or "smartvillage-sumo-run"
#bus = FlaskKafka(INTERRUPT_EVENT,
#         bootstrap_servers=",".join([kafka_brokers]),
#         group_id=kafka_group
#         )
#
#@bus.handle(kafka_topic)
#def test_topic_handler(msg):
#    print("received message from %s topic: %s" % (kafka_topic, msg))

def listen_kill_server():
    signal.signal(signal.SIGTERM, bus.interrupted_process)
    signal.signal(signal.SIGINT, bus.interrupted_process)
    signal.signal(signal.SIGQUIT, bus.interrupted_process)
    signal.signal(signal.SIGHUP, bus.interrupted_process)

@app.route('/')
def hello():
#    producer = KafkaProducer(bootstrap_servers=kafka_brokers)
#    producer.send(kafka_topic, b"test")
    sumoBinary = checkBinary('sumo')
    main_pedestrian.ipa_gradient_method_pedestrian(initial_par=[10, 20, 30, 50, 10, 10, 8, 8, 5, 5], lam=[10, 10, 6, 3],
            demand_scale=1, step_size=1, par_update_step_size=30, run_time=1000,
            total_iter_num=3, iters_per_par=3, print_mode=False)
    return "Hello World!"

def start():
    flask_port = os.environ.get('FLASK_PORT') or 8080
    flask_port = int(flask_port)

#    bus.run()
#    listen_kill_server()
    app.run(port=flask_port, host='0.0.0.0')

if __name__ == "__main__":
    start()
