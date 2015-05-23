#!/usr/bin/env python
# author: Simon Bilgeri, Munich, Germany
# date: March 2015
# Build Database for Flatbuddy V1.0

import MySQLdb

#Open database connection
db = MySQLdb.connect('192.168.178.28','Buddy','Penner12','FB')
cursor = db.cursor()
cursor.execute('''CREATE TABLE IF NOT EXISTS objects(id INT PRIMARY KEY AUTO_INCREMENT, type TEXT,room TEXT, location TEXT, how TEXT)''')
cursor.execute('''CREATE TABLE IF NOT EXISTS sockets(id INT PRIMARY KEY AUTO_INCREMENT, objectID INT, IP TEXT, state INT)''')
cursor.execute('''CREATE TABLE IF NOT EXISTS mods(id INT PRIMARY KEY AUTO_INCREMENT, objectID INT, modification TEXT)''')
db.commit();
db.close();


##print('Objects in DB:')
##cursor.execute('''SELECT ID,type, location, how FROM objects''')
##
##for row in cursor:
##    # row[0] returns the first column in the query (name), row[1] returns email column.
##    print('{0} : {1}, {2}, {3}'.format(row[0], row[1], row[2],row[3]))
##
##print('Sockets in DB:')
##cursor.execute('''SELECT objectID, IP, state FROM sockets''')
##for row in cursor:
##    # row[0] returns the first column in the query (name), row[1] returns email column.
##    print('{0} : {1}, {2}'.format(row[0], row[1], row[2]))
##
##print('Modifications in DB:')
##cursor.execute('''SELECT objectID, mod FROM mods''')
##for row in cursor:
##    # row[0] returns the first column in the query (name), row[1] returns email column.
##    print('{0} : {1}'.format(row[0], row[1]))
##
##db.close()
