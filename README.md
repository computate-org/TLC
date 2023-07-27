
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
