FROM cmu-mars/cp1_base

ADD src src/cp1_controllers/src
ADD cp1 ../cp1/
COPY CMakeLists.txt src/cp1_controllers/
COPY package.xml src/cp1_controllers/
COPY requirements.txt src/cp1_controllers/

RUN . /opt/ros/kinetic/setup.sh
RUN sudo chown -R $(whoami):$(whoami) .
RUN sudo chown -R $(whoami):$(whoami) ../cp1/

RUN . /opt/ros/kinetic/setup.sh && \
    catkin_make

RUN sudo apt-get install -y python-pip

RUN sudo pip install catkin_pkg rospkg numpy psutil defusedxml flask-script

CMD ["/bin/bash"]
