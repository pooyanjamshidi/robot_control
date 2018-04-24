FROM cmu-mars/cp1_base

ADD src src/cp1_controllers/src
ADD cp1 ../cp1/
COPY CMakeLists.txt src/cp1_controllers/
COPY package.xml src/cp1_controllers/
COPY requirements.txt src/cp1_controllers/

RUN git clone https://github.com/cmu-mars/model-learner.git src/model-learner

RUN sudo chown -R $(whoami):$(whoami) .
RUN sudo chown -R $(whoami):$(whoami) ../cp1/

RUN sudo apt-get install -y python-pip

RUN sudo pip install catkin_pkg rospkg numpy psutil defusedxml flask-script

CMD ["/bin/bash"]
