FROM ubuntu:22.04
USER root
RUN apt-get update && apt-get install git wget mono-complete libcurl3 -y
RUN wget http://github.com/GitTools/GitVersion/releases/download/5.11.1/gitversion-linux-x64-5.11.1.tar.gz 
RUN tar -xvf gitversion-linux-x64-5.11.1.tar.gz
RUN chmod +x gitversion
RUN mv gitversion /usr/local/bin/gitversion
RUN apt-get install python3 python3-pip python3.10-venv -y 
RUN useradd -ms /bin/bash builder



USER builder
WORKDIR /home/builder
RUN pip3 install PyYaml
RUN pip3 install platformio
RUN pip3 install gitversion

RUN git config --global user.email 'service-account-github@cellink.com'
RUN git config --global user.name 'Cellink Github Service Account - Yocto Builder'