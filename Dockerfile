FROM cmu-mars/cp1_base

ADD src src/cp1_controllers/src
COPY CMakeLists.txt src/cp1_controllers/
COPY package.xml src/cp1_controllers/
COPY requirements.txt src/cp1_controllers/

RUN git clone https://github.com/cmu-mars/model-learner.git src/model-learner

RUN sudo chown -R $(whoami):$(whoami) .

ENV PYTHON_VERSION=3.6.4
RUN sudo apt-get install -y python3
RUN sudo apt-get install -y python3-pip

RUN sudo pip3 install catkin_pkg rospkg numpy psutil defusedxml flask-script

CMD ["/bin/bash"]
