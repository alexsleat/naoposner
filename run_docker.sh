. docker_config

xhost +

docker run \
    -v $(pwd)/code:/home/user/code \
    --rm \
    -it \
    --network host \
    --privileged \
    --device /dev/dri \
    --device /dev/snd \
    -v /run/user/1000/pulse:/run/user/1000/pulse \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -e HOST_UID=$(id -u) -e HOST_GID=$(id -g) \
    -e DISPLAY \
    --name $CONTAINER_NAME \
    $DOCKER_NAME bash                           
