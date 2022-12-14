# naoposner

This repository holds the Code for the ORU-KTH Collaboration.

## Running the docker:

### terminal 1:

    ./run_docker.sh
    roscore &

### terminal 2
    . attach_docker.sh
    python3 psychopy_ros.py

### terminal 3
    . attach_docker.sh
    python2 nao_cueing.py


## Installing NVIDIA-Docker toolkit:

Instructions from here: https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker

    # Set up repo and GPG keys
    distribution=$(. /etc/os-release;echo $ID$VERSION_ID) \
      && curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
      && curl -s -L https://nvidia.github.io/libnvidia-container/$distribution/libnvidia-container.list | \
            sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
            sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

    # Install it:
    sudo apt-get update
    sudo apt-get install -y nvidia-docker2
    # Requires restart
    sudo systemctl restart docker