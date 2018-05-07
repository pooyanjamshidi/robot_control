from threading import Lock
import subprocess
import time
import os
import rospy
import threading

RAINBOW_PATH = os.path.expanduser('~/das/rainbow-brass')

class Command(object):
    def __init__(self, cmd):
        self.cmd = cmd
        self.process = None

    def run(self, timeout):
        def target():
            print(self.cmd)
            self.process = subprocess.Popen(self.cmd)
            self.process.communicate()

        thread = threading.Thread(target=target)
        thread.start()

        thread.join(timeout)
        if thread.is_alive():
            self.process.terminate()
            thread.join()
        return self.process.returncode


class RainbowInterface:
	# Class to manage interaction with Rainbow

    def __init__(self):
        self.processStarted = False
        self.lock = Lock()
        self.target = None

    def getTarget (self, challenge_problem):
        target=None
        if challenge_problem == "cp1":
            target="brass-p2-cp1"
        elif challenge_problem == "cp3":
            target="brass-p2-cp3"
        return target

    def startRainbow(self):
        with self.lock:
            if self.processStarted:
                rospy.logerr("Trying to start Rainbow when Rainbow is already running.")
            self.processStarted = True

        if (self.target is None):
            return True
        rospy.loginfo("Starting Rainbow (DAS)...")
        command = Command([RAINBOW_PATH+"/brass.sh", "-w", RAINBOW_PATH, "-s", self.target, os.path.expanduser("~/rainbow-start.log")])
        ret = command.run(timeout=60)
        rospy.loginfo("Rainbow started, exit=%s"%str(ret))
        return ret == 0

    def launchRainbow(self, challenge_problem, log):
        """
        Starts Rainbow process. Needs to (a) start in background, (b) wait some time until it is up
        """

        print ("Configuring rainbow for %s"%challenge_problem)
        os.environ["LD_LIBRARY_PATH"] = os.environ["LD_LIBRARY_PATH"] + ":" + os.path.expanduser("~/das/prism-4.3.1-linux64/lib")
        print ("LD_LIBRARY_PATH=%s" %os.environ["LD_LIBRARY_PATH"])
        self.target = self.getTarget(challenge_problem)
        if (self.target is not None):
            time.sleep(10)
            print("Starting %s/run-oracle.sh %s"%(RAINBOW_PATH,self.target))
            subprocess.Popen([RAINBOW_PATH+"/run-oracle.sh", "-h", "-w", RAINBOW_PATH, self.target], stdout=log)
            time.sleep(40)


    def stopRainbow(self):
        rospy.loginfo("Stopping Rainbow (DAS)...")
        ret = subprocess.call ([RAINBOW_PATH+"/brass.sh","-q", "-w", RAINBOW_PATH, self.target])
        with self.lock:
            self.processStarted = False