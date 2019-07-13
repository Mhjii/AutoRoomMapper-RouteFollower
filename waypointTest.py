import RethinkHelper as db

while 1:
    x = input("Enter X coordinate desired: ")
    y = input("Enter Y coordinate desired: ")
    db.putWaypoints([x,y])
    db.watchWaypoints()