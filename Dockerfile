FROM ubuntu:22.04
USER root
RUN ln -fs /usr/share/zoneinfo/UTC  /etc/localtime
RUN apt-get update && apt-get install git wget mono-complete libcurl4 -y
RUN wget https://github.com/GitTools/GitVersion/releases/download/5.12.0/gitversion-linux-x64-5.12.0.tar.gz
RUN tar -xvf gitversion-linux-x64-5.12.0.tar.gz
RUN chmod a+rx gitversion
RUN mv gitversion /usr/local/bin/gitversion
RUN apt-get install python3 python3-pip python3.10-venv -y 
RUN useradd -ms /bin/bash builder



USER builder
WORKDIR /home/builder
RUN pip3 install PyYaml
RUN pip3 install platformio

RUN git config --global user.email 'service-account-github@cellink.com'
RUN git config --global user.name 'Cellink Github Service Account - Yocto Builder'