//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

package node;
//
// This file is part of an OMNeT++/OMNEST simulation example.
//
// 2012 L.Jacob Mariscal Fernández based on Copyright (C) 1992-2008 Andras Varga
//
// This file is distributed WITHOUT ANY WARRANTY. See the file
// `license' for details on this and other legal matters.
//

// Un Punto de acceso ("PuntoAcceso") representa a la entidad física(router) con capacidad para generar/recibir tráfico
// a través del modelo de redes Lan802.11  


//
// An Acces Point ("PuntoAcceso") consists of the simple entity who represents a router capable of generate/receive traffic
// through the network model 802.11
//
module PuntoAcceso
{
    parameters:
        int address;
        @node();
        @labels(node,ethernet-node,wireless-node);
        @display("i=device/accesspoint");
        int xpos @unit(m) = default(20m); // postion in metres
        int ypos @unit(m) = default(20m); // in XY matrix
    gates:
        inout port[];
    submodules:

        red: AntNetAP {
            parameters:
                @display("p=140,174");
            gates:
                in[sizeof(port)];
                out[sizeof(port)];
        }
        enlace[sizeof(port)]: Wlan80211 {
            parameters:
                @display("p=140,240,row");
        }
    connections:

        for i=0..sizeof(port)-1 {
            red.out[i] --> enlace[i].in;
            red.in[i] <-- enlace[i].out;
            enlace[i].line <--> port[i];
        }
}


