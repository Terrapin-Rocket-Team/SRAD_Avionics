import subprocess
from datetime import datetime
from gpiozero import Button
import os

triggerPin = 17
trigger = Button(triggerPin)
# get epoch time
unixTime = int((datetime.now() - datetime(1970, 1, 1)).total_seconds())
logFile = open(os.path.expanduser("~/video/log_" + str(unixTime) + ".txt"), "w")
videoFilePath = os.path.expanduser("~/video/video_" + str(unixTime) + ".mp4")

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
    cam = subprocess.Popen(["libcamera-vid", "-t", "300000", "--no-raw", "--codec", "yuv420", "--width", "1920", "--height", "1080", "--framerate", "30", "-o", "-" ], stdout=subprocess.PIPE, stderr=logFile)
    enc = subprocess.Popen(["ffmpeg", "-s", "1920x1080", "-f", "rawvideo", "-pix_fmt", "yuv420p", "-i", "-", "-vcodec", "h264_v4l2m2m", "-r", "30", "-b:v", "10M", "-an", "-sn", "-dn", "-y", videoFilePath], stdin=cam.stdout, stderr=logFile)
    cam.wait()

def main():
    logPrintln("Local video handler started")

    logPrintln("Waiting for signal on pin " + str(triggerPin))
    trigger.wait_for_press()
    logPrintln("Starting video")
    startVideo()

main()
logFile.close()
