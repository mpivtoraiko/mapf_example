FROM ubuntu:20.04

RUN apt install build-essential cmake libboost-all-dev
RUN mkdir /root/build
WORKDIR /root/build
RUN cmake /root/src