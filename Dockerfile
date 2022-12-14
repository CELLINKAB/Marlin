FROM ubuntu:22.04
USER root
RUN apt-get update 
RUN apt-get install git -y
RUN apt-get install python3 python3-pip python3.10-venv -y 
RUN useradd -ms /bin/bash builder



USER builder
WORKDIR /home/builder
RUN pip3 install PyYaml
RUN pip3 install platformio

RUN git config --global user.email 'service-account-github@cellink.com'
RUN git config --global user.name 'Cellink Github Service Account - Yocto Builder'