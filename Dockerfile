FROM cmu-mars/cp1_base

ADD cp1controllers src/cp1_controllers/cp1controllers
ADD cp1 ../cp1/
COPY CMakeLists.txt src/cp1_controllers/
COPY package.xml src/cp1_controllers/
COPY requirements.txt src/cp1_controllers/
COPY setup.py src/cp1_controllers/

RUN . /opt/ros/kinetic/setup.sh
RUN sudo chown -R $(whoami):$(whoami) .
RUN sudo chown -R $(whoami):$(whoami) ../cp1/

#RUN . /opt/ros/kinetic/setup.sh && \
#    catkin_make

RUN sudo pip install catkin_pkg rospkg numpy psutil defusedxml flask-script

RUN sudo apt-get install -y python3-pip
RUN python3 -m pip install --upgrade src/cp1_controllers

CMD ["/bin/bash"]
