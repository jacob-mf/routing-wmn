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
// L. Jacob Mariscal Fernández 2014
// This file is distributed WITHOUT ANY WARRANTY. See the file
// 'license' for details on this and other legal matters.

package networks;
import node.Terminal;
import node.**;

network KliKle
{
    
    parameters:
        int numHosts = default(2);
        int numAPs = default(0);

    types:
        channel C extends ned.DatarateChannel
        {
            parameters:
                volatile double jitter @ unit(ms)  =  default(uniform(0ms,7ms ))  ;
                delay = 2ms + convertUnit(jitter, "ms") ;
                datarate = default(18Mbps);
                volatile double noise @ unit ("dB") = default (1dB); // if it's 4dB are considered not understandable signal
                @display("ls= grey,0");
        }
    submodules:
////        host[numHosts]: AdhocHost {
//            parameters:
//                @display("r=,,#707070");//;p=250,58

        Kli: Terminal {
            @display("p=266,78;r=60,,#707070");
            address=2;
            gates:
                port[4];
        }
        Kle: Terminal {
            @display("p=177,78;r=60,,#707070");
            address=3;
            gates:
                port[4];
        }
        connections allowunconnected:
        	Kle.port[2] <--> C <--> Kli.port[3];

//        ap[numAPs] : AccessPoint {
//            @display("r=,,#707070"); // p=213,174;
//           
//        }  

}

