FROM quay.io/computateorg/smartabyar-data-science-notebook:latest

MAINTAINER Christopher Tate <computate@computate.org>

# By default, listen on port 8081
EXPOSE 8081/tcp

ENV FLASK_PORT=8081 \
  KAFKA_BROKERS=kafka0.apps-crc.testing:32000 \
  KAFKA_GROUP=smartvillage-kafka-group \
  KAFKA_TOPIC_SUMO_RUN=smartvillage-sumo-run \
  KAFKA_TOPIC_SUMO_RUN_REPORT=smartvillage-sumo-run-report \
  KAFKA_USERNAME='smartvillage' \
  KAFKA_PASSWORD='' \
  KAFKA_SECURITY_PROTOCOL=SSL \
  KAFKA_SSL_CAFILE=/opt/kafka/truststore/ca.crt \
  KAFKA_SSL_CERTFILE='/opt/kafka/keystore/user.crt' \
  KAFKA_SSL_KEYFILE='/opt/kafka/keystore/user.key' \
  KAFKA_MAX_POLL_RECORDS='1' \
  KAFKA_MAX_POLL_INTERVAL_MS='3000000' \
  ZOOKEEPER_HOST_NAME='zookeeper.apps-crc.testing' \
  ZOOKEEPER_PORT='30081'

# Set the working directory in the container
WORKDIR /usr/local/src/TLC

# Copy the dependencies file to the working directory
COPY requirements.txt /usr/local/src/TLC/

# Install any dependencies
RUN pip install -r requirements.txt

# Copy the content of the local src directory to the working directory
COPY . .

# Add write privileges to input and output directory
RUN chmod -R a+rw /usr/local/src/TLC/input /usr/local/src/TLC/output

# Specify the command to run on container start
CMD [ "python", "app.py" ]

