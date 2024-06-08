import subprocess
from datetime import datetime
from gpiozero import Button
import os

triggerPin = 17
trigger = Button(triggerPin)
# get epoch time
unixTime = int((datetime.now() - datetime(1970, 1, 1)).total_seconds())
logFile = open(os.path.expanduser("~/video/log_" + str(unixTime) + ".txt"), "w")
videoWidth = "832"
videoHeight = "640"

# handy logging statements
def logPrint(str):
    print(str, end='')
    logFile.writelines(str)
    logFile.flush()

def logPrintln(str):
    print(str)
    logFile.write(str + "\n")
    logFile.flush()

def startVideo():
    cam = subprocess.Popen(["libcamera-vid", "-t", "300000", "--no-raw", "--codec", "yuv420", "--width", videoWidth, "--height", videoHeight, "--framerate", "30", "-o", "-" ], stdout=subprocess.PIPE, stderr=logFile)
    enc = subprocess.Popen(["aomenc", "-w", videoWidth, "-h", videoHeight, "--fps=30/1" "--profile=0", "--kf-max-dist=240", "--end-usage=cbr", "--min-q=1", "--max-q=63", "--undershoot-pct=100", "--overshoot-pct=50", "--buf-sz=1000", "--buf-initial-sz=500", "--buf-optimal-sz=600", "--max-intra-rate=600", "--passes=1", "--lag-in-frames=0", "--error-resilient=0", "--tile-columns=1", "--tile-rows=3", "--aq-mode=2", "--enable-obmc=0", "--enable-global-motion=1", "--enable-warped-motion=0", "--deltaq-mode=0", "--enable-tpl-model=0", "--mode-cost-upd-freq=2", "--coef-cost-upd-freq=2", "--enable-ref-frame-mvs=0", "--mv-cost-upd-freq=3", "--enable-order-hint=0", "--cpu-used=9", "--rt", "--usage=1", "--threads=3", "--obu", "--target-bitrate=250", "-", "-o" , "-"], stdout=subprocess.PIPE, stdin=cam.stdout, stderr=logFile)
    radio = subprocess.Popen(["~/lv-github/teensy-interface/build/main"], stdout=logFile, stdin=enc.stdout)
    cam.wait()

def main():
    logPrintln("Live video handler started")

    logPrintln("Waiting for signal on pin " + str(triggerPin))
    trigger.wait_for_press()
    logPrintln("Starting video")
    startVideo()

main()
logFile.close()
