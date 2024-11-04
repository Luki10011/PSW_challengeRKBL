#!/bin/bash

build_args=""
for (( i=1; i<=$#; i++));
do
  param="${!i}"
  echo $param

  if [ "$param" == "--build-args" ]; then
    j=$((i+1))
    build_args="${!j}"
  fi

done

echo "Building psw_challenge with additional docker arguments $build_args."

docker build \
    $build_args \
    -f .devcontainer/Dockerfile \
    -t psw_challenge_rklb .
