export FLASK_PORT=8081
export KAFKA_BROKERS=smartvillage-kafka-kafka-bootstrap:9093
export KAFKA_GROUP=smartvillage-kafka-group
export KAFKA_TOPIC_SUMO_RUN=smartvillage-sumo-run
export KAFKA_TOPIC_SUMO_RUN_REPORT=smartvillage-sumo-run-report
export KAFKA_SECURITY_PROTOCOL=SSL
export KAFKA_SSL_CAFILE=/opt/app-root/src/certs/ca.crt
export KAFKA_SSL_CERTFILE='/opt/app-root/src/certs/user.crt'
export KAFKA_SSL_KEYFILE='/opt/app-root/src/certs/user.key'
export KAFKA_MAX_POLL_RECORDS='1'
export KAFKA_MAX_POLL_INTERVAL_MS='3000000'
export ZOOKEEPER_HOST_NAME='zookeeper'
export ZOOKEEPER_PORT='2181'

(cd $HOME/TLC && python app.py)
