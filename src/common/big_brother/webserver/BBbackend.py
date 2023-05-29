#!/usr/bin/env python3

from logging import log
from pytz import timezone
import socketio as socketIO
import time
import socket
import pickle
import yaml
from threading import Thread
import sqlite3
import datetime
from slack_sdk.webhook import WebhookClient
import pytz
import numpy as np
import cv2 as cv
import base64


###################### General Variables #######################

BUFFER_SIZE = 50000

# config file info
CAR_IP = ""
SERVER_IP = ""
nodeList = []
topicList = []
rosbagTopics = []
frequencyLimits = []

trackName = "unknown"

# timer to detect car connection lost
timer = None

# slack variables
slackActive = False
webhook = None

# events
freqEventActive = []
prevNodes = []
prevASInfo = []
currentEvents = []
runEvents = []
eventCounters = [0, 0, 0, False, 0, False, False]

# run counters, one for each mission
runCounters = [0, 0, 0, 0, 0, 0, 0]
resGoCounter = 0
onARun = False
runs = {
    0: 'acceleration',
    1: 'skidpad',
    2: 'autocross',
    3: 'trackdrive',
    4: 'EBS_test',
    5: 'inspection',
    6: 'manual_driving'
}

# current date
timezoneLisbon = pytz.timezone('Europe/Lisbon')
date = datetime.datetime.now(timezoneLisbon)
dateString = str(date.year) + "_" + str(date.month) + "_" + str(date.day)

####################### Data structures ########################

infoData = {
    "carConnected": False,
    "rosInfo": [],
    "asInfo": [],
    "image": None,
    "slamMap": None
}

rosbagData = {
    "record": False,
    "name": "",
    "topics": []
}

slamMap = None

##################### Database connection ######################

db_connection = sqlite3.connect('log.db', check_same_thread=False)
db = db_connection.cursor()

logTable = ""
eventTable = ""

# DB contents
logDB = []
eventDB = []


########################### SocketIO ###########################

socketio = socketIO.Client()

@socketio.event
def connect():
    print('[INFO] Successfully connected to server.')


@socketio.event
def disconnect():
    print('[INFO] Disconnected from server.')


###################### Auxiliar functions ######################

def sendData(sock, port, data):
    # send data to given address, breaking into multiple packets if needed

    global BUFFER_SIZE

    data = pickle.dumps(data)
    size = len(data)
    beg = 0
    end = BUFFER_SIZE if size > BUFFER_SIZE else size

    while size > 0:
        sock.sendto(data[beg:end], (CAR_IP, port))
        size -= BUFFER_SIZE if size > BUFFER_SIZE else size
        beg += BUFFER_SIZE
        end += BUFFER_SIZE if size > BUFFER_SIZE else size


def receiveData(sock):
    # receive multiple packets of the same data

    global BUFFER_SIZE
    
    data = []
    while True:
        part = sock.recv(BUFFER_SIZE)
        data.append(part)
        if len(part) < BUFFER_SIZE:
            break
    return b''.join(data)


def resetTimer():
    # reset car connection timer

    global timer
    timer = time.time()


def checkCarConnection():
    # check if car is still connected

    global timer, infoData, slamMap
    
    if time.time() - timer > 1:

        # print only once, dont spam
        if infoData["carConnected"]:
            print("> Car disconnected")

        infoData = {
            "carConnected": False,
            "rosInfo": [],
            "asInfo": [],
            "image": None,
            "slamMap": None
        }

        slamMap = None
    
    else:
        # print only once, dont spam
        if not infoData["carConnected"]:
            print("> Car connected")

        infoData["carConnected"] = True


def checkRun():
    # checks if car is on a run, increments run counters and add run logs

    global onARun, runCounters, infoData, runs, runEvents, eventCounters, frequencyLimits, freqEventActive, resGoCounter, slamMap

    # check if we have info
    if not infoData["carConnected"] or len(infoData["asInfo"]) == 0:
        return

    mission = infoData["asInfo"][9]
    missionFinished = infoData["asInfo"][10]
    resEmergency = infoData["asInfo"][12]
    resGo = infoData["asInfo"][13]

    if missionFinished or resEmergency: resGoCounter = 0
    elif resGo: resGoCounter += 1

    # run started
    if mission != 6 and resGoCounter >= 5 and not missionFinished and not resEmergency and not onARun:

        onARun = True
        incrementRun(mission)
        runEvents = []
        eventCounters = [0, 0, 0, False, 0, False, False]
        for i in range(len(frequencyLimits)): freqEventActive[i] = False

        # clear SLAM map
        slamMap = None
        
        print(f'> Started run {runCounters[mission]} of mission {mission}.')

        # run time
        t = datetime.datetime.now(timezoneLisbon).time()
        time = f'{t.hour}:'
        if len(str(t.minute)) == 1: time += '0'
        time += f'{t.minute}'

        # add log
        # entry = f'Started {runs[mission]} {runCounters[mission]}.'
        # socketio.emit('new_backend_log', {"entry_num": 0, "time": time, "tag": 'Run', "data": entry, "slack": slackActive}, namespace='/backend')

        # event time
        time += ':'
        if len(str(t.second)) == 1: time += '0'
        time += str(t.second)

        # add event
        currentEvents.append((time, f'Started {runs[mission]} {runCounters[mission]}.'))

    # run ended
    elif onARun and (missionFinished or resEmergency):

        onARun = False
        print(f'> Finished run {runCounters[mission]} of mission {mission}.')

        # run time
        t = datetime.datetime.now(timezoneLisbon).time()
        time = f'{t.hour}:' 
        if len(str(t.minute)) == 1: time += '0'
        time += f'{t.minute}'

        # add log
        # entry = f'Finished {runs[mission]} {runCounters[mission]}.' 

        # # add event summary for all misions except inspection
        # if mission != 5:
        #     entry += f' Summary: {eventCounters[4]} laps; No blue cones detected - {eventCounters[0]} times; No yellow cones detected - {eventCounters[1]} times; Orange cones detected - {eventCounters[2]} times'
        #     if eventCounters[3] == True: entry += "; Mission Finished"
        #     if eventCounters[5] == True: entry += "; RES Emergency"
        #     if eventCounters[6] == True: entry += "; Loop Closure detected"
        #     else: entry += "; No Loop Closure detected"
        #     if runs[mission] + '_' + str(runCounters[mission]) in rosbagData["name"]: entry += "; Rosbag recorded."
        #     else: entry += "; Rosbag not recorded."

        # socketio.emit('new_backend_log', {"entry_num": 0, "time": time, "tag": 'Run', "data": entry, "slack": slackActive}, namespace='/backend')


def incrementRun(mission: int):
    # increment run counter and save to file

    global runCounters, runs

    runCounters[mission] += 1

    # write to run file
    try:
        with open("../config/runs.yaml", 'r') as file:
            doc = yaml.safe_load(file)

        doc[runs[mission]] = runCounters[mission]
        print(doc)

        with open("../config/runs.yaml", 'w') as file:
            yaml.dump(doc, file)
    except yaml.YAMLError as exc:
        print(exc)
    finally:
        file.close()

def generateMap():
    # generate SLAM map image from map cones

    global infoData, slamMap

    if infoData["asInfo"] == [] or infoData["asInfo"][23] == []:
        return
       
    print("Building Slam map image")

    # map size
    x_max = 300
    y_max = 300

    # creating the slam map
    slamMap = np.zeros((x_max,y_max,3), np.uint8)
    # filling up backgorund color
    cv.rectangle(slamMap,(0,0),(x_max,y_max),(160,160,160),-1)
    
    map = infoData["asInfo"][23]
    for cone in map:
        
        y = int((y_max/2) + (cone[0][1]*(y_max/2))/50)
        x = int((x_max/2) - (cone[0][0]*(x_max/2))/50)
        
        # yellow cone
        if cone[1] == 1:
            cv.circle(slamMap,(x,y), 1, (0,255,255), -1)
        # blue cone
        elif cone[1] == 0:
            cv.circle(slamMap,(x,y), 1, (255,0,0), -1)
        # orange cone    
        elif cone[1] == 3 or cone[1] == 4:
            cv.circle(slamMap,(x,y), 1, (0,140,255), -1)
        else:
            cv.circle(slamMap,(x,y), 1, (255,255,255), -1)
    
    infoData["slamMap"] = image2jpeg(slamMap)
    
    # infoData["slamMap"] = None 
    # cv.imshow('Slam Map',slamMap)
    # cv.waitKey(0)

def image2jpeg(image):

    # encode frame as jpeg
    frame = cv.imencode('.jpg', image)[1].tobytes()

    # encode frame in base64 representation and remove utf-8 encoding
    frame = base64.b64encode(frame).decode('utf-8')
    return "data:image/jpeg;base64,{}".format(frame)


####################### Config functions #######################

def setupConnection(addr, port):

    print('[INFO] Connecting to server http://{}:{}...'.format(addr, port))

    # try connecting to webserver until it is successful
    connected = False 
    while not connected:
        try:
            socketio.connect('http://{}:{}'.format(addr, port), transports=['websocket'], namespaces=['/backend'])
        except socketIO.exceptions.ConnectionError:
            print('[INFO] Waiting for server to become available.')
        else:
            connected = True
        
        time.sleep(1)


def configDatabase():

    global db_connection, db, logTable, eventTable, dateString, runCounters

    logTable = "test_log_" + dateString
    eventTable = "test_events_" + dateString
    
    # create tables if not already created
    db.execute("CREATE TABLE IF NOT EXISTS " + logTable + " (entryNum integer PRIMARY KEY AUTOINCREMENT, time text, tag text, entry text)")
    db.execute("CREATE TABLE IF NOT EXISTS " + eventTable + " (num integer PRIMARY KEY AUTOINCREMENT, time text, event text)")
    db_connection.commit()

    # read run file
    try:
        file = open("../config/runs.yaml", 'r')
        doc = yaml.safe_load(file)
    except yaml.YAMLError as exc:
        print(exc)
    finally:
        file.close()
    
    runCounters = [
        doc['acceleration'],
        doc['skidpad'],
        doc['autocross'],
        doc['trackdrive'],
        doc['EBS_test'],
        doc['inspection'],
        doc['manual_driving']
    ]


def configInfo():

    global CAR_IP, SERVER_IP, db, logTable, eventTable, rosbagData, slackActive, webhook, nodeList,\
        trackName, topicList, frequencyLimits, freqEventActive, rosbagTopics, logDB, eventDB

    # load log from database
    db.execute(f'SELECT time, tag, entry FROM {logTable}')
    logDB = db.fetchall()

    # load only last 20 events from database
    db.execute(f'SELECT time, event FROM {eventTable}')
    eventDB = db.fetchall()

    # read config file
    try:
        file = open("../config/config.yaml", 'r')
        doc = yaml.safe_load(file)
    except yaml.YAMLError as exc:
        print(exc)
    finally:
        file.close()

    # save info
    CAR_IP = doc["car_ip"]
    SERVER_IP = doc["server_ip"]
    slackActive = doc["slack"]
    webhook = WebhookClient(doc["slack_channel"])
    nodeList = doc["nodes"]
    topicList = doc["topics"]
    rosbagTopics = doc["record"]
    trackName = doc["track_name"]
    frequencyLimits = doc["frequency_limits"]

    for i in range(len(topicList)): freqEventActive.append(False)


@socketio.on('config_request', namespace='/backend')
def sendConfig():

    global nodeList, topicList, rosbagTopics, logDB, eventDB, slamMap
    
    # send config to server
    msg = {"nodes": nodeList, "topics": topicList, "record": rosbagTopics, "log": logDB, "events": eventDB[-30:]}
    socketio.emit('config', msg, namespace='/backend')

    # regenerate map
    slamMap = None


######################### Log function #########################

@socketio.on('new_log', namespace='/backend')
def receiveLog(message):

    global db_connection, db, logTable, logDB
    
    print(f'> New log: {message}')

    # save locally
    logDB.append((message["time"], message["tag"], message["data"]))

    # insert into database
    try:
        db.execute(f'INSERT INTO {logTable} (time, tag, entry) VALUES ("{message["time"]}", "{message["tag"]}", "{message["data"]}")')
        db_connection.commit()
    except Exception:
        print("> Failed to add log to DB")

    if not slackActive: return

    if message["tag"] == "Slack" or message["slack"]:
        response = webhook.send(text=message["data"])
        if response.status_code == 200 and response.body == 'ok':
            print("> Log posted to slack")
        else:
            print("> Posting log to slack failed")


####################### Rosbag functions #######################

@socketio.on('rosbag_record', namespace='/backend')
def recordRosbag(message):
    
    global rosbagData, onARun, runCounters, infoData, dateString, trackName

    if infoData["asInfo"] != []: mission = infoData["asInfo"][9]
    else: mission = -1

    rosbagData["record"] = True
    rosbagData["topics"] = message["topics"]
    run = runCounters[mission]
    if not onARun: run += 1      
    rosbagData["name"] = dateString + '_' + trackName + '_' + str(runs[mission]) + '_' + str(run)

    # only increment a run in manual_driving if a rosbag is recorded
    if mission == 6: incrementRun(mission) 


@socketio.on('rosbag_stop', namespace='/backend')
def stopRosbag():
    
    global rosbagData

    rosbagData["record"] = False


######################## Event functions #######################

def checkEvents():

    global infoData, currentEvents, prevNodes, prevASInfo, eventCounters

    # check if we have info
    if not infoData["carConnected"] or len(infoData["asInfo"]) == 0:
        return
    
    mission = infoData["asInfo"][9]


    # check if we are on a run and not in inspection mission
    if mission == 5 or not onARun:
        return


    # event time
    t = datetime.datetime.now(timezoneLisbon).time()
    time = f'{t.hour}:'
    if len(str(t.minute)) == 1: time += '0'
    time += f'{t.minute}:'
    if len(str(t.second)) == 1: time += '0'
    time += str(t.second)

    # check nodes
    checkNodes(time)

    # check topic frequencies
    checkFrequencies(time)

    # check AS info
    checkASInfo(time)

    # save events on DB
    saveEvents()

    # send events to server
    if currentEvents != []:
        socketio.emit('events', {'events': currentEvents}, namespace='/backend')

    # clear current event list
    currentEvents.clear()


def checkNodes(time):
    # compare current node values against previous

    global infoData, prevNodes, nodeList, currentEvents

    if len(infoData["rosInfo"]) == 0: return

    # get nodes
    nodes = infoData["rosInfo"][0]

    # save previous nodes on first time
    if prevNodes == []: prevNodes = nodes

    # return if no change
    if nodes == prevNodes:
        return
    
    # check for change
    for i in range(len(nodes)):
        if nodes[i] != prevNodes[i] and nodes[i] == False:
            print("> Event: " + nodeList[i] + " died")
            currentEvents.append((time, f'Node {nodeList[i]} died.'))
    
    # save cache
    prevNodes = nodes


def checkFrequencies(time):

    global infoData, frequencyLimits, topicList, freqEventActive, currentEvents

    if len(infoData["rosInfo"]) == 0: return

    # get topic frequencies
    frequencies = infoData["rosInfo"][1]

    # check if frequencies are under the limit
    for i in range(len(topicList)):
        if topicList[i] in frequencyLimits and frequencies[i] < frequencyLimits[topicList[i]] and freqEventActive[i] == False:
            if topicList[i] == "/estimation/gps_position":
                print(f'> Event: GPS frequency dropped under {frequencyLimits[topicList[i]]} Hz (f = {frequencies[i]} Hz)')
                currentEvents.append((time, f'GPS frequency dropped to {frequencies[i]} Hz'))
            if topicList[i] == "/velodyne_points":
                print(f'> Event: LIDAR frequency dropped under {frequencyLimits[topicList[i]]} Hz (f = {frequencies[i]} Hz)')
                currentEvents.append((time, f'LIDAR frequency dropped to {frequencies[i]} Hz'))
            if topicList[i] == "/estimation/odom3D":
                print(f'> Event: AHRS frequency dropped under {frequencyLimits[topicList[i]]} Hz (f = {frequencies[i]} Hz)')
                currentEvents.append((time, f'AHRS frequency dropped to {frequencies[i]} Hz'))                
            freqEventActive[i] = True
        elif topicList[i] in frequencyLimits and frequencies[i] >= frequencyLimits[topicList[i]] and freqEventActive[i] == True:
            if topicList[i] == "/estimation/gps_position":
                print(f'> Event: Topic {topicList[i]} frequency settled above {frequencyLimits[topicList[i]]} Hz (f = {frequencies[i]} Hz)')
                currentEvents.append((time, f'GPS frequency rose to {frequencies[i]} Hz'))
            if topicList[i] == "/velodyne_points":
                print(f'> Event: LIDAR frequency dropped under {frequencyLimits[topicList[i]]} Hz (f = {frequencies[i]} Hz)')
                currentEvents.append((time, f'LIDAR frequency rose to {frequencies[i]} Hz'))
            if topicList[i] == "/estimation/odom3D":
                print(f'> Event: AHRS frequency dropped under {frequencyLimits[topicList[i]]} Hz (f = {frequencies[i]} Hz)')
                currentEvents.append((time, f'AHRS frequency rose to {frequencies[i]} Hz'))        
            freqEventActive[i] = False


def checkASInfo(time):
    
    global infoData, currentEvents, prevASInfo, eventCounters

    mission = infoData["asInfo"][9]

    # Lap count
    if prevASInfo != [] and infoData["asInfo"][11] > prevASInfo[4]:
        print(f'> Event: Lap {infoData["asInfo"][11]}')
        currentEvents.append((time, f'Lap {infoData["asInfo"][11]}'))
        eventCounters[4] = infoData["asInfo"][11]
    # Mission Finished
    if infoData["asInfo"][10] and (prevASInfo == [] or not prevASInfo[3]):
        print("> Event: Mission Finished")
        currentEvents.append((time, "Mission Finished."))
        eventCounters[3] = True
        currentEvents.append((time, f'Finished {runs[mission]} {runCounters[mission]}.'))
    # RES emergency
    elif infoData["asInfo"][12] and (prevASInfo == [] or not prevASInfo[5]):
        print("> Event: RES emergency")
        currentEvents.append((time, "RES emergency."))
        eventCounters[5] = True
        currentEvents.append((time, f'Finished {runs[mission]} {runCounters[mission]}.'))
    # loop closure
    if infoData["asInfo"][18] and (prevASInfo == [] or not prevASInfo[6]):
        print("> Event: loop closure")
        currentEvents.append((time, "Loop closure detected."))
        eventCounters[6] = True
    
    #Cones
    n_cones =  infoData["asInfo"][0] + infoData["asInfo"][1] + infoData["asInfo"][2] + infoData["asInfo"][3] + infoData["asInfo"][4]
    if n_cones == 0 and (prevASInfo == [] or not ((prevASInfo[0] + prevASInfo[1] + prevASInfo[2]) == 0)):
        print("> Event: No cones detected.")
        currentEvents.append((time, "No cones detected."))
        
          
    # save previous info on cache but only if on a run    
    prevASInfo = [infoData["asInfo"][0], infoData["asInfo"][1], infoData["asInfo"][2] + infoData["asInfo"][3] + infoData["asInfo"][4], infoData["asInfo"][10], infoData["asInfo"][11], infoData["asInfo"][12], infoData["asInfo"][18]]


def saveEvents():
    # save events on database and locally

    global db_connection, db, eventTable, currentEvents

    if len(currentEvents) == 0: return

    # insert into event table
    try:
        request = f'INSERT INTO {eventTable} (time, event) VALUES '
        for event in currentEvents:
            request += f'("{event[0]}", "{event[1]}"),'
        db.execute(request[:-1])
        db_connection.commit()

    except Exception:
        print("> Failed to add events to DB")
    
    # save locally
    for event in currentEvents:
        eventDB.append((event[0], event[1]))


################## Car info receiver function ##################

def receiveFromCar(port, var_num):
    # continuously receive info from car (var_num = which info)

    global infoData

    # create socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server_address = (SERVER_IP, port)
    sock.bind(server_address)

    while True:
        try:
            # receive data
            data = receiveData(sock)
            # save data on corresponding slot
            infoData[var_num] = pickle.loads(data)
            resetTimer()

        # except KeyboardInterrupt:
        #     sock.close()
        #     break
        except Exception:
            print("packet drop")


######################## Main function #########################

def run():

    global rosbagData, timer, slamMap

    # connect to webserver
    setupConnection("localhost", 3000)

    # config database
    configDatabase()

    # read config, load database and send info to server
    configInfo()

    # each thread will be receiving info from the car on a designated socket
    threadRosInfo = Thread(target=receiveFromCar, args=(3001, "rosInfo"))
    threadASInfo = Thread(target=receiveFromCar, args=(3002, "asInfo"))
    # threadImage = Thread(target=receiveFromCar, args=(3003, "image"))

    threadRosInfo.start()
    threadASInfo.start()
    # threadImage.start()

    # socket to send rosbag data to car
    # sockRosbag = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # timer to detect lost connection to car
    timer = time.time()

    while True:
        try:
            checkCarConnection()

            checkEvents()
            
            checkRun()

            # generate SLAM map
            if slamMap is None:
                generateMap()
            else:   # clear map if already sent
                infoData["slamMap"] = None

            # copy info msg and send to server
            msg = infoData.copy()
            socketio.emit('info', msg, namespace='/backend')
            
            # send rosbag data to car
            # if infoData["carConnected"]:
            #     sendData(sockRosbag, 3004, rosbagData)

            time.sleep(1/15)    # 15Hz
    
        except KeyboardInterrupt:
            # sockRosbag.close()
            break



if __name__ == '__main__':

    try:
        while True:
            try:
                run()
            except socketIO.exceptions.BadNamespaceError:
                print("> Bad Namespace Error! Restarting...")
    except KeyboardInterrupt:
        print("Exiting...")

