
## How to run the application as a Podman container

### Install the prerequiste packages for buildah and podman

```bash
pkcon install -y buildah
pkcon install -y podman
```

### Build the container with podman

```bash
cd ~/.local/src/TLC
podman build -t computateorg/smartvillage-traffic-light-controller-sumo:computate-api .
```

### Push the container up to quay.io
```bash
podman login quay.io
podman push computateorg/smartvillage-traffic-light-controller-sumo:computate-api quay.io/computateorg/smartvillage-traffic-light-controller-sumo:computate-api
```

### Pull down Kafka secrets from OpenShift Local for development

```bash
sudo install -d -o $USER -g $USER -m 771 /opt/kafka/truststore
sudo install -d -o $USER -g $USER -m 771 /opt/kafka/keystore
oc extract -n smartvillage secret/smartvillage-kafka --to=/opt/kafka/keystore/ --keys=user.crt --confirm
oc extract -n smartvillage secret/smartvillage-kafka --to=/opt/kafka/keystore/ --keys=user.key --confirm
oc extract -n smartvillage secret/smartvillage-kafka-cluster-ca-cert --to=/opt/kafka/truststore/ --keys=ca.crt --confirm
```

### Run the container for local development

```bash
podman run -v /opt/kafka:/opt/kafka computateorg/smartvillage-traffic-light-controller-sumo:computate-api
```
