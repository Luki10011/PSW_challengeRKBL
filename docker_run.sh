#!/bin/bash

XSOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth
SRC_LOCAL=${PWD}
SRC_DOCKER=/root/sim_ws/src/psw_challengeRKBL
touch $XAUTH
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -

echo "Running Docker Container"
CONTAINER_NAME=psw_challengeRKBL

# Get distro of the built image
run_args=""

for (( i=1; i<=$#; i++));
do
  param="${!i}"

  if [ "$param" == "--run-args" ]; then
    j=$((i+1))
    run_args="${!j}"
  fi

done

# Check if there is an already running container with the same name
running_container="$(docker container ls -al | grep ${CONTAINER_NAME})"
if [ -z "$running_container" ]; then
  echo "Running ${CONTAINER_NAME} for the first time!"
else
  echo "Found an open ${CONTAINER_NAME} container. Starting and attaching!"
  eval "docker start ${CONTAINER_NAME}"
  eval "docker attach ${CONTAINER_NAME}"
  exit 0
fi

# Check if using GPU
gpu_enabled="--gpus all"
run_args="$gpu_enabled $run_args"

docker run \
  $run_args \
  -it \
  --network host \
  --privileged \
  --volume=$XSOCK:$XSOCK:rw \
  --volume=$XAUTH:$XAUTH:rw \
  --volume=$SRC_LOCAL:$SRC_DOCKER:Z \
  --env="XAUTHORITY=${XAUTH}" \
  --env DISPLAY=$DISPLAY \
  --env NVIDIA_VISIBLE_DEVICES=all \
  --env NVIDIA_DRIVER_CAPABILITIES=compute,utility,display \
  --env QT_X11_NO_MITSHM=1 \
  --env TERM=xterm-256color \
  --name ${CONTAINER_NAME} \
  psw_challenge \
  /bin/bash
