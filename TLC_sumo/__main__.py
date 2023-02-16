from flask import Flask
from threading import Event
from flask_kafka import FlaskKafka
from kafka import  KafkaProducer
import signal
import os

app = Flask(__name__)

INTERRUPT_EVENT = Event()

kafka_brokers = os.environ.get('KAFKA_BROKERS') or "smartvillage-kafka-kafka-brokers.smartabyar-smartvillage.svc.cluster.local:9092"
kafka_group = os.environ.get('KAFKA_GROUP') or "smartvillage-kafka-group"
kafka_topic = os.environ.get('KAFKA_TOPIC') or "smartvillage-sumo-run"
bus = FlaskKafka(INTERRUPT_EVENT,
         bootstrap_servers=",".join([kafka_brokers]),
         group_id=kafka_group
         )

def listen_kill_server():
    signal.signal(signal.SIGTERM, bus.interrupted_process)
    signal.signal(signal.SIGINT, bus.interrupted_process)
    signal.signal(signal.SIGQUIT, bus.interrupted_process)
    signal.signal(signal.SIGHUP, bus.interrupted_process)

@bus.handle(kafka_topic)
def test_topic_handler(msg):
    print("received message from %s topic: %s" % (kafka_topic, msg))

@app.route('/')
def hello():
    producer = KafkaProducer(bootstrap_servers=kafka_brokers)
    producer.send(kafka_topic, b"test")
    return "Hello World!"

def main():
    flask_port = os.environ.get('FLASK_PORT') or 8080
    flask_port = int(flask_port)

    bus.run()
    listen_kill_server()
    app.run(port=flask_port, host='0.0.0.0')

if __name__ == "__main__":
    main()
