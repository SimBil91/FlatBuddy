###########################################################################
 #   Copyright (C) 2007-2013 by Guy Rutenberg                              #
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

import os.path
import ConfigParser
from pkg_resources import resource_stream

class StationsLocal():
    """
    Initialize the station list by parsing files
    """
    def __init__(self):
        config = ConfigParser.RawConfigParser()
        default_config = resource_stream(__name__, 'data/radiopy.default')
        config.readfp(default_config, 'default config')

        files = ['/etc/radio.py']
        files.append(os.path.expanduser(("~/.radiopy"))) # user settings

        config.read(files)

        self._stations = {}
        for section in config.sections():
            station = {}
            for name, value in config.items(section):
                station[name] = value
            self._stations[section] = station
            self._stations[section]["name"] = section


    def get_station(self, name, fuzzy_search = True, distance = 5):
        """Returns a dict representing the station given by name. If no
        station is found returns None.
        """
        if name in self._stations:
            return self._stations[name]
        elif not fuzzy_search:
            return None

    def __iter__(self):
        for i in sorted(self._stations.items()):
            yield i # (name, dict) pair

    def __len__(self):
        return len(self._stations)

