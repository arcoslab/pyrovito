#!/usr/bin/python
# Copyright (c) 2012 Technische Universitaet Muenchen, Informatik Lehrstuhl IX.
# Author: Federico Ruiz-Ugalde <memeruiz@gmail.com>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>.

import yarp
cstyle=yarp.ContactStyle()
cstyle.persistent=True

class Roboviewer_objects():
    def __init__(self,portbasename,roboviewerportbasename,counter=-1):
        self.out_port=yarp.BufferedPortBottle()
        self.out_port.open(portbasename+"/roboviewer_out")
        yarp.Network.connect(portbasename+"/roboviewer_out", roboviewerportbasename+"/objects:i", cstyle)
        self.counter=counter
    
    def create_object(self,object_type):
        self.counter+=1
        bottle=self.out_port.prepare()
        bottle.clear()
        bottle_list=bottle.addList()
        bottle_list.addInt(self.counter)
        bottle_list.addString("type")
        bottle_list.addString(object_type)
        self.out_port.write(True)
        return(self.counter)
    
    def send_prop(self,object_id,prop_name, prop_data):
        bottle=self.out_port.prepare()
        bottle.clear()
        bottle_list=bottle.addList()
        bottle_list.addInt(object_id)
        bottle_list.addString(prop_name)
        for i in prop_data:
            bottle_list.addDouble(i)
        self.out_port.write(True)

    def send_prop_multi(self,object_ids,prop_names, prop_datas):
        bottle=self.out_port.prepare()
        bottle.clear()
        for object_id, prop_name, prop_data in zip(object_ids,prop_names,prop_datas):
            bottle_list=bottle.addList()
            bottle_list.clear()
            bottle_list.addInt(object_id)
            bottle_list.addString(prop_name)
            for i in prop_data:
                bottle_list.addDouble(i)
        self.out_port.write(True)

def main():
    return(False)



if __name__ == "__main__":
    main()


