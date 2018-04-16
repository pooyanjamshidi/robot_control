FROM cmu-mars/cp1_base

ADD ../cp1_controllers src/cp1_controllers

RUN git clone https://github.com/cmu-mars/model-learner.git src/model-learner

ENV PYTHON_VERSION=3.6.4
RUN sudo apt-get install -y python3
RUN sudo apt-get install -y python3-pip

CMD ["/bin/bash"]
