###########################################################################
 #   Copyright (C) 2007-2014 by Guy Rutenberg                              #
 #   guyrutenberg@gmail.com                                                #
 #                                                                         #
 #   This program is free software; you can redistribute it and/or modify  #
 #   it under the terms of the GNU General Public License as published by  #
 #   the Free Software Foundation; either version 2 of the License, or     #
 #   (at your option) any later version.                                   #
 #                                                                         #
 #   This program is distributed in the hope that it will be useful,       #
 #   but WITHOUT ANY WARRANTY; without even the implied warranty of        #
 #   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
 #   GNU General Public License for more details.                          #
 #                                                                         #
 #   You should have received a copy of the GNU General Public License     #
 #   along with this program; if not, write to the                         #
 #   Free Software Foundation, Inc.,                                       #
 #   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             #
############################################################################

import sys
import os
import subprocess
import time
import ConfigParser
import tempfile
import random
import threading
from stations_local import StationsLocal
from stations_tunein import StationsTunein
class StationResolver():
    handlers = [StationsLocal(), StationsTunein()]

    @staticmethod
    def get_station(name):
        for h in StationResolver.handlers:
            station = h.get_station(name)
            if station is not None:
                return station




class Player:
    """
    The radiopy player
    """
    def __init__(self, options={}):
        self.station_list = StationsLocal()

    def play(self, station_name, wake=0, sleep=0, cache=320, record=""):
        """
        station_name - The name of the station to play.
        wake - Number of minutes to wait before starting to play.
        sleep - Number of minutes to play radio before killing it.
        cache - The cache size ink kbits.
        record - A file name to record stream to.
        """
        execargs =['mplayer','-nolirc']
        station = StationResolver.get_station(station_name)
        if not station:
            print "radiopy couldn't find the requested station \"%s\"" % station_name
            print "Try --list to see the available stations list"
            return

        if wake > 0:
            print "radiopy will wake up in {} minutes".format(wake)
            time.sleep(60*wake)

       # if cache < 32: #mplayer requires cache>=32
       #     cache = 64
        execargs += []

       # if station.has_key("stream_id"):
            #execargs += ['-aid', station["stream_id"]]

        #if station.get("playlist", False) == "yes":
            #execargs.append('-playlist')
        print "Playing {}".format(station_name)

        execargs.append(station['stream'])
        pid = None
        """ if record:
            record_args = ['-dumpstream', '-dumpfile', record]
            execargs += record_args
        if sleep:
            print "radiopy will go to sleep in %d minutes" % sleep
            if not pid:
                pid = subprocess.Popen(execargs).pid
            threading.Timer(60*sleep, self.kill_mplayer).start()
        else:"""
        FNULL = open(os.devnull, 'w')
        return subprocess.Popen(execargs, stdout=FNULL, stdin=FNULL, stderr=FNULL).pid
        
    def kill_mplayer(self,pid):
        os.kill(pid, 15) # 15 = SIGTERM
    def play_random(self, *args, **kwds):
        """
        Plays a random station.
        """
        station_name = random.choice(self.station_list._stations.keys())
        #print "Playing {}".format(station_name)
        sys.stdout.flush() # before out output gets mangled in mplayer's output
        return self.play(station_name, *args, **kwds)
        

    def print_list(self):
        """
        Prints the station list.
        """
        maxname = max(self.station_list, key=lambda x: len(x[0]))
        maxlen = len(maxname)
        for name,station in self.station_list:
            print name.ljust(maxlen+1), station.get("home","")

        print "Total:", len(self.station_list), "recognized stations"

# vim: ai ts=4 sts=4 et sw=4
