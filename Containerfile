FROM quay.io/computateorg/smartabyar-data-science-notebook:latest

MAINTAINER Christopher Tate <computate@computate.org>

# By default, listen on port 8081
EXPOSE 8081/tcp

ENV FLASK_PORT=8081 \
  KAFKA_BROKERS=smartvillage-kafka-kafka-brokers.smartabyar-smartvillage.svc.cluster.local:9092 \
  KAFKA_GROUP=smartvillage-kafka-group \
  KAFKA_TOPIC=smartvillage-sumo-run 

# Set the working directory in the container
WORKDIR /usr/local/src/TLC

# Copy the dependencies file to the working directory
COPY requirements.txt /usr/local/src/TLC/

# Install any dependencies
RUN pip install -r requirements.txt

# Copy the content of the local src directory to the working directory
COPY . .

# Specify the command to run on container start
CMD [ "python", "app.py" ]

