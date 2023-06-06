ARG ROS_DISTRO=melodic

ARG APP_NAME=robomaker_app

ARG LOCAL_WS_DIR=workspace

ARG USERNAME=robomaker

ARG GAZEBO_VERSION=gazebo-9.1

ARG IMAGE_WS_DIR=/home/$USERNAME/workspace

FROM public.ecr.aws/docker/library/ros:${ROS_DISTRO}-ros-base AS ros-robomaker-base
ARG USERNAME
ARG IMAGE_WS_DIR


RUN apt-get clean
RUN apt-get update && apt-get install -y \
    lsb  \
    unzip \
    wget \
    curl \
    xterm \
    python3-colcon-common-extensions \
    devilspie \
    xfce4-terminal \
    python-pip

RUN pip install boto3

RUN groupadd $USERNAME && \
    useradd -ms /bin/bash -g $USERNAME $USERNAME && \
    sh -c 'echo "$USERNAME ALL=(root) NOPASSWD:ALL" >> /etc/sudoers'
    
USER $USERNAME
WORKDIR /home/$USERNAME

RUN mkdir -p $IMAGE_WS_DIR


FROM ros-robomaker-base as ros-robomaker-application-base
ARG LOCAL_WS_DIR
ARG IMAGE_WS_DIR
ARG ROS_DISTRO
ARG USERNAME

WORKDIR $IMAGE_WS_DIR
COPY --chown=$USERNAME:$USERNAME $LOCAL_WS_DIR/src $IMAGE_WS_DIR/src

RUN sudo apt update && \
    rosdep update && \
    rosdep fix-permissions


RUN rosdep install --from-paths src --ignore-src -r -y


FROM ros-robomaker-application-base AS ros-robomaker-build-stage
LABEL build_step="${APP_NAME}Workspace_Build"
ARG APP_NAME
ARG LOCAL_WS_DIR
ARG IMAGE_WS_DIR

RUN  . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build \
     --install-base $IMAGE_WS_DIR/$APP_NAME


FROM ros-robomaker-application-base AS ros-robomaker-app-runtime-image
ARG APP_NAME
ARG USERNAME
ARG GAZEBO_VERSION

ENV USERNAME=$USERNAME
ENV APP_NAME=$APP_NAME
ENV GAZEBO_VERSION=$GAZEBO_VERSION

RUN rm -rf $IMAGE_WS_DIR/src

COPY --from=ros-robomaker-build-stage $IMAGE_WS_DIR/$APP_NAME $IMAGE_WS_DIR/$APP_NAME

WORKDIR /
COPY entrypoint.sh /entrypoint.sh
RUN sudo chmod +x /entrypoint.sh && \
    sudo chown -R $USERNAME /entrypoint.sh && \
    sudo chown -R $USERNAME $IMAGE_WS_DIR/$APP_NAME

ENTRYPOINT ["/entrypoint.sh"]
