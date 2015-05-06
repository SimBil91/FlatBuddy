#!/usr/bin/env python
# author: Simon Bilgeri, Munich, Germany
# date: March 2015
# Build Database for Flatbuddy V1.0

import sqlite3

db=sqlite3.connect('budspeech.db')
cursor = db.cursor()
cursor.execute('''CREATE TABLE IF NOT EXISTS objects(id INTEGER PRIMARY KEY, type TEXT,room TEXT, location TEXT, how TEXT)''')
cursor.execute('''CREATE TABLE IF NOT EXISTS sockets(id INTEGER PRIMARY KEY, objectID INTEGER, IP TEXT, state INTEGER)''')
cursor.execute('''CREATE TABLE IF NOT EXISTS mods(id INTEGER PRIMARY KEY, objectID INTEGER, mod TEXT)''')

"""# objects
cursor.execute('''INSERT INTO objects(type,location,how) VALUES('light','bed','small')''')
cursor.execute('''INSERT INTO objects(type,location,how) VALUES('light','desk','big')''')
cursor.execute('''INSERT INTO objects(type,location,how) VALUES('light','sofa','tall')''')
cursor.execute('''INSERT INTO objects(type,location,how) VALUES('grill','balcony','black')''')

# sockets
cursor.execute('''INSERT INTO sockets(objectID,IP,state) VALUES(1,'192.168.2.115','off')''')
cursor.execute('''INSERT INTO sockets(objectID,IP,state) VALUES(4,'192.168.2.116','off')''')
cursor.execute('''INSERT INTO sockets(objectID,IP,state) VALUES(3,'192.168.2.1','off')''')
cursor.execute('''INSERT INTO sockets(objectID,IP,state) VALUES(2,'192.168.2.1','off')''')

# actions
cursor.execute('''INSERT INTO mods(objectID,mod) VALUES(1,'turn on')''')
cursor.execute('''INSERT INTO mods(objectID,mod) VALUES(1,'turn off')''')
cursor.execute('''INSERT INTO mods(objectID,mod) VALUES(2,'turn on')''')
cursor.execute('''INSERT INTO mods(objectID,mod) VALUES(2,'turn off')''')
cursor.execute('''INSERT INTO mods(objectID,mod) VALUES(3,'turn on')''')
cursor.execute('''INSERT INTO mods(objectID,mod) VALUES(3,'turn off')''')
cursor.execute('''INSERT INTO mods(objectID,mod) VALUES(4,'turn on')''')
cursor.execute('''INSERT INTO mods(objectID,mod) VALUES(4,'turn off')''')"""

db.commit()

print('Objects in DB:')
cursor.execute('''SELECT ID,type, location, how FROM objects''')

for row in cursor:
    # row[0] returns the first column in the query (name), row[1] returns email column.
    print('{0} : {1}, {2}, {3}'.format(row[0], row[1], row[2],row[3]))

print('Sockets in DB:')
cursor.execute('''SELECT objectID, IP, state FROM sockets''')
for row in cursor:
    # row[0] returns the first column in the query (name), row[1] returns email column.
    print('{0} : {1}, {2}'.format(row[0], row[1], row[2]))

print('Modifications in DB:')
cursor.execute('''SELECT objectID, mod FROM mods''')
for row in cursor:
    # row[0] returns the first column in the query (name), row[1] returns email column.
    print('{0} : {1}'.format(row[0], row[1]))

db.close()
