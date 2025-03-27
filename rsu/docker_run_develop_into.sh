#!/bin/bash
#coding=utf-8
SELF_RELATIVE_DIR=`dirname $0`                       # 获取 脚本文件所在的相对路径
SELF_ABSOLUTE_DIR=`readlink -f "$SELF_RELATIVE_DIR"` # 当前 脚本文件，所在的绝对路径

# 判断系统架构
if [ "$(uname -m)" = "aarch64" ]; then
  PLATFORM="arm64"
fi
if [ "$(uname -m)" = "x86_64" ]; then
  PLATFORM="amd64"
fi


# --volume=$HOME/yolov7deepsort:/home/erlang/yolov7deepsort:rw
XSOCK=/tmp/.X11-unix
FONTS=/usr/share/fonts # 字体问题
SHARED_DOCKER_DIR=${SELF_ABSOLUTE_DIR}
SHARED_HOST_DIR=${SELF_ABSOLUTE_DIR}
VOLUMES="--volume=$XSOCK:$XSOCK:rw
         --volume=$HOME/rsu:/home/promote/rsu:rw
         --volume=$SHARED_HOST_DIR:$SHARED_DOCKER_DIR:rw
         --volume=/dev:/dev:rw
         --volume=$FONTS:$FONTS:rw" 

if [ $PLATFORM == "amd64" ]; then
  RUNTIME="--gpus all"
else
  RUNTIME="--runtime=nvidia"
fi
IMAGE_NAME=camera_pereception
TAG_PREFIX=tag1
IMAGE="${IMAGE_NAME}:${TAG_PREFIX}"

docker run \
    -it --rm \
    --name="rsu_docker" \
    --workdir="$SHARED_HOST_DIR" \
    $VOLUMES \
    --env="DISPLAY=${DISPLAY}" \
    --privileged \
    --device=/dev/ttyUSB0 \
    --privileged \
    --net=host \
    --ipc=host \
    -u $(id -u):$(id -g) \
    $RUNTIME \
    ${IMAGE} \
     /bin/bash
