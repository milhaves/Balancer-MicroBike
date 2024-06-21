import socket
import sys, select
import time
from numpy import *
from scipy.signal import *
from matplotlib.pyplot import *
import sys,traceback
import serial
from threading import Thread
import copy

import tkinter as tk

# Implement the default Matplotlib key bindings.
from matplotlib.backend_bases import key_press_handler
from matplotlib.backends.backend_tkagg import (FigureCanvasTkAgg,
                                               NavigationToolbar2Tk)
from matplotlib.figure import Figure

#create our figure for viewing data
fig = Figure(figsize=(5, 4), dpi=100)
#initialize variables to hold our data
buffer_len = 200 #how many points should we plot? adjust this as needed.
tvec = []
commandvec = []
feedbackvec = []
#create axis object on the figure
ax = fig.add_subplot()
#create objects for the two datasets, commanded and feedback
commandline, = ax.plot(tvec,commandvec)
feedbackline, = ax.plot(tvec,feedbackvec)
#set labels
ax.set_xlabel("time [s]")
ax.set_ylabel("Servo Position")

#create a global variable to hold the servo command
servo_cmd = 0
servo_enable_cmd = 0
#the serial communication will run in a separate "thread"
#this will allow the GUI and the communication with the arduino to
#run at different update rates
endSerialThread = False
endPlotThread = False


############### CALLBACKS FOR GUI SLIDERS ###################

#every "slider" we define in the GUI will run a "callback" function
def servocallback(v):
    global servo_cmd
    servo_cmd=servoslider.get()


############## CALLBACK FOR RUN BUTTON ##################
def startServoThread():
    global servothread,endSerialThread,endPlotThread,plotthread
    endSerialThread = False
    endPlotThread = False
    statemsg["text"] = "Connected"
    servothread = Thread(target=doServo)
    servothread.start()
    plotthread = Thread(target=doPlot)
    plotthread.start()

def cleanupServoThread():
    global endSerialThread
    endSerialThread=True
    endPlotThread = True




def doPlot():
    global commandline,feedbackline,tvec,commandvec,feedbackvec,canvas,endPlotThread
    #this function will run as a timed loop thread that updates the plot
    #set update rate
    plotDelay = 0.2
    while not endSerialThread:
        commandline.set_data(tvec, commandvec)
        feedbackline.set_data(tvec,feedbackvec)
        if(len(tvec)>3):
            ax.set_xlim([tvec[0],tvec[-1]])
            ax.set_ylim([-1.5,1.5])
            ax.legend(['command','feedback'])

        # required to update canvas and attached toolbar!
        canvas.draw()
        time.sleep(plotDelay)



############## Function to communicate with Arduino ###########
def doServo():
    global servo_cmd,ser,endSerialThread,servo_enable_cmd,tvec,feedbackvec,commandvec
    #initialize old time
    arduino_delay = 0.01

    #connect to the serial port.
    #the portentry.get() call gets the port name
    #from the textbox.
    ser = serial.Serial(
    port=portentry.get(), #ACM100',   #USB0',
    baudrate=115200,
    timeout = arduino_delay)
    print("initializing")
    ser.close()
    time.sleep(2.0)
    ser.open()
    time.sleep(2.0)
    print("done")

    starttime = time.time()
    lastsendtime = time.time()-starttime
    #this is an infinite loop  .
    while not endSerialThread:
        if not servoBool.get():
            servo_cmd_local = copy.deepcopy(servo_cmd)
            enable_cmd=0
        else:

            servo_cmd_local = copy.deepcopy(servo_cmd)
            enable_cmd=1
        #get current time
        tnow = time.time()-starttime
        # print(tnow-lastsendtime)
        #print("length of t vector is: "+str(len(tvec)))
        if tnow-lastsendtime>arduino_delay:     ### also happens super fast
            #print 'sent'
            lastsendtime = tnow
            ser.write('!'.encode())

            ser.write(format(servo_enable_cmd,'0.4f').encode())
            ser.write(','.encode())
            ser.write(format(servo_cmd_local,'0.4f').encode())
            ser.write('\n'.encode())
            # time.sleep(0.01)
            line = ser.readline().decode("utf-8")
            #print ("at time = "+format(tnow,'0.2f')+ " received: "+line)
            #line is now a string. Strip the newline characters
            linestrip = line.strip()
            #split the line at commas
            linesplit = line.split(",")
            #based on the Arduino code we wrote, we expect 2 numbers: millis(),feedback
            #convert first number to float:
            arduinotime = float(linesplit[0])
            #convert second number to float:
            feedbackval = float(linesplit[1])
            #add the data to our vectors for plotting:
            #if we haven't filled buffer yet, fill it.
            if(len(tvec)<buffer_len):
                tvec.append(tnow)
                commandvec.append(servo_cmd_local)
                feedbackvec.append(feedbackval)
            else: #if we're here, it means we need to drop oldest in the buffer and add new
                tvec.append(tnow)
                tvec = tvec[1:]
                commandvec.append(servo_cmd_local)
                commandvec = commandvec[1:]
                feedbackvec.append(feedbackval)
                feedbackvec = feedbackvec[1:]


            #print(line)
            #echomsg["text"] = "at t = "+format(tnow,'0.2f')+" received: "+line
        else:
          time.sleep(arduino_delay)
        # print(cmd)
        # time.sleep(0.1)
    ser.close()
    statemsg["text"] = "Not Connected"




################## GUI SETUP #########################

#create GUI window
window = tk.Tk()
#title the window
window.title('Servo Control')
#set window size
window.geometry("750x750+100+50")

#create a frame to hold the serial port configuration
portframe = tk.Frame(window,relief=tk.GROOVE,borderwidth=3)
portframe.pack()
#create a label for port config:
portmsg = tk.Label(portframe,text="port: ")
portmsg.pack(side=tk.LEFT)
#create a textbox for the port name
portentry = tk.Entry(portframe)
#insert a default port
portentry.insert(0,"/dev/cu.usbmodem101")
portentry.pack(side=tk.LEFT)
# create a button to connect to platform
servobut = tk.Button(
    portframe,
    text="Connect",
    command=startServoThread)
servobut.pack(side=tk.RIGHT)
killbut = tk.Button(
    portframe,
    text="Disconnect",
    command=cleanupServoThread)
killbut.pack(side=tk.RIGHT)
#create a status message in this frame to show serial port status
statemsg = tk.Label(window,text="Not Connected")
statemsg.pack()


#create a slider for roll
servoframe = tk.Frame(window,relief=tk.GROOVE,borderwidth=3)
servoframe.pack()
servolabel = tk.Label(servoframe,text="Servo Command (rad)")
servolabel.pack(side=tk.LEFT)
servoslider = tk.Scale(servoframe,orient=tk.HORIZONTAL,from_=-1,to=1,resolution=.01,command=servocallback,length=400)
servoslider.pack(side=tk.RIGHT)

#create a status message in this frame to show serial port status
echomsg = tk.Label(window,text="No Data Received")
echomsg.pack()

servoBool = tk.IntVar()
servocheck = tk.Checkbutton(window, text='Enable Servo',variable=servoBool, onvalue=1, offvalue=0)
servocheck.pack()

#the plot, from Tk's perspective, belongs in this 'canvas'
canvas = FigureCanvasTkAgg(fig, master=window)  # A tk.DrawingArea.
canvas.draw()

toolbar = NavigationToolbar2Tk(canvas, window, pack_toolbar=False)
toolbar.update()
#I don't know what this does.
# canvas.mpl_connect(
#     "key_press_event", lambda event: print(f"you pressed {event.key}"))
# canvas.mpl_connect("key_press_event", key_press_handler)

#add the canvas to our window.
canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)



#run the TK mainloop to keep the window up and open.
window.mainloop()
