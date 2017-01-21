FROM ubuntu:xenial
MAINTAINER Chao Zhang

ARG USER=docker
ARG PROJECT_NAME=holospace
ARG PROJECT_DIR=/opt/$PROJECT_NAME
ARG TMP_DIR=/tmp

RUN apt-get update                  \
    && apt-get install -y           \
        build-essential \
        cmake           \
        g++             \
        git             \
        pkg-config      \
    && useradd -ms /bin/bash $USER  \
    && adduser docker root

RUN apt-get install -y  \
        libpcl-dev

RUN apt-get install -y          \
        beignet-dev         \
        libglfw3-dev        \
        libjpeg-turbo8-dev  \
        libopenni2-dev      \
        libturbojpeg        \
        libusb-1.0-0-dev    \
    && cd $TMP_DIR              \
    && git clone                \ 
        https://github.com/OpenKinect/libfreenect2.git  \
    && cd libfreenect2          \
    && git checkout v0.2.0      \
    && cmake                    \
        -DCMAKE_INSTALL_PREFIX=/usr/local   \
        .                                   \
    && make -j4                 \
    && make install             \
    && ldconfig -v

COPY . $PROJECT_DIR
WORKDIR $PROJECT_DIR

RUN cmake .                         \
    && make install                 \
    && chmod -R ug+rwx $PROJECT_DIR
