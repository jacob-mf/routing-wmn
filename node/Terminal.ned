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

// Un "Terminal" representa a la entidad física (ej: computador) con capacidad para generar/recibir tráfico
// a través del modelo de redes Lan802.11  


//
// A "Terminal" consists of the simple entity who represents a computer capable of generate/receive traffic
// through the network model 802.11
//
module Terminal
{
    parameters:
        int address;
        string aplicacionTipo;
        string transporteTipo;
        @node();
        @labels(node,ethernet-node,wireless-node);
        @display("i=device/wifilaptop");
        int xpos @unit(m) = default(20m); // postion in metres
        int ypos @unit(m) = default(20m); // in XY matrix
    gates:
        inout port[];
    submodules:
        aplicacion: <aplicacionTipo> like IApp {
            parameters:
                address = address;
                @display("i=block/app;p=140,42");
                @display("p=140,0");
        }
        transporte: <transporteTipo> like ITransport {
            parameters:
                address = address;
           		@display("i=block/transport;p=140,108");
            	@display("p=140,125");


        }
        red: AntNet {
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
        aplicacion.in <-- transporte.appOut;
        aplicacion.out --> transporte.appIn;
        red.localOut --> transporte.ipIn;
        red.localIn <-- transporte.ipOut;
        for i=0..sizeof(port)-1 {
            red.out[i] --> enlace[i].in;
            red.in[i] <-- enlace[i].out;
            enlace[i].line <--> port[i];
        }
}


