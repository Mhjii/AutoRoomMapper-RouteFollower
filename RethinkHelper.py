import rethinkdb as r
import json
import numpy as np

#r.connect("192.168.1.10",28015,"Eratosthenes").repl()
r.connect("localhost",28015,"Eratosthenes").repl()

def putWaypoints(Data):
    print("adding new waypoint")
    print(Data)
    r.table("Waypoints").get(1).update(
        {"Point":{"x": Data[0], "y": Data[1]}}
    ).run()

def getWaypoints():
    Point = r.table("Waypoints").get(1)["Point"].run()
    Point = [Point['x'],Point['y']]
    return Point

def watchWaypoints():
    cursor = r.table("Waypoints").changes().run()
    while 1:
        for document in cursor:
            print(document)
            if document['new_val']['Point'] == 'None':
                return

def watchPoints():
    cursor = r.table("Points").changes().run()
    for document in cursor:
        if document['new_val'] == None:
            return

def clearPoints():
    r.table("Points").delete().run()

def putPoints(Data):
    write = []
    for index, point in enumerate(Data):
            write.append({"id": index, "Point": {"R": point[0], "Theta": point[1]}})
    r.table("Points").insert(write).run()


def getPoints():
    cursor = r.table("Points").order_by("id")["Point"].run()
    R = []
    Theta = []
    for document in cursor:
        R.append(document["R"])
        Theta.append(document["Theta"])
    return R,Theta

def setStop():
   r.table("EStop").get(1).update({'State': "True"}).run()

def disStop():
    r.table("EStop").get(1).update({'State': "False"}).run()

def setStart():
    r.table("EStop").get(2).update({'State': "True"}).run()

def disStart():
    r.table("EStop").get(2).update({'State': "False"}).run()

def setSimpleScan():
    r.table("EStop").get(3).update({'State': "True"}).run()

def disSimpleScan():
    r.table("EStop").get(3).update({'State': "False"}).run()

def setScanComplete():
    r.table("EStop").get(4).update({'State': "True"}).run()

def disScanComplete():
    r.table("EStop").get(4).update({'State': "False"}).run()

def getScanComplete():
    cursor = r.table("EStop").get(4).changes().run()
    while 1:
        for State in cursor:
            if State['new_val']['State'] == 'True':
                return


def setScanRequest():
    r.table("EStop").get(5).update({'State': "True"}).run()

def getScanRequest():
    cursor = r.table("EStop").get(5).changes().run()
    while 1:
        for State in cursor:
            print(State)
            if State['new_val']['State'] == 'True':
                return

def disScanRequest():
    r.table("EStop").get(5).update({'State': "False"}).run()

def getRange():
    Buffer = r.table("Settings").get(1)["Settings"]['Range'].run()
    return Buffer



def setSettings(Buffer,CompletionPerc,Consentors,Range,Resolution,Start):
    r.table("Settings").get(1).update({"Settings":{
        "Buffer": Buffer,
        "CompletionPerc": CompletionPerc,
        "Consentors": Consentors,
        "Range": Range,
        "Resolution": Resolution,
        "Start": {"X": Start[0],
                  "Y": Start[1]
                  }
    }}).run()

