FROM ubuntu:xenial

RUN apt-get update
RUN apt-get -y install gcc make libboost-all-dev
RUN apt-get -y install g++

COPY . cobra
WORKDIR cobra/COBRA
RUN make all
