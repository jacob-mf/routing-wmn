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
// 2013 L.Jacob Mariscal Fernández based on Copyright (C) 1992-2008 Andras Varga
//
// This file is distributed WITHOUT ANY WARRANTY. See the file
// `license' for details on this and other legal matters.
//

package node;

//
// Application model to generate traffic for wireless mesh netowrks
//
simple AppUDP like IApp
{
    parameters:
        int address;  // local node address
        string destAddresses;  // destination addresses
        string srcAddresses;   // source addresses
        bool flooding;         // select flooding way
        int maxArray= default(49); // define max length of path stored in packet 
        int sort = default(2); // (could be defined by sort in omnet.ini) specifies sort of routing as 1 : Static , 2 : AntWmNet , 3 : CPANT , 4 : AODV
        volatile double sendIaTime @unit(s)  = default(exponential(1s)); // time between generating packets
        volatile double sendFaTime @unit(s)  = default(exponential(1s)); // time between forward ant packets 
        volatile double sendFirstFA @unit(s)  = default(0.03s); // time to send first forward ant packet 
        volatile double stopFA @unit(s)  = default(1s); // time to stop launching forward ants packets 
        volatile int packetLength @unit(byte);  // length of one message (fixed! no "volatile" modifier)
        volatile int setupTime @unit(s) = default(3s); // set-up time
        @display("i=block/browser");
        @signal[endToEndDelay](type="simtime_t");
        @signal[hopCount](type="int");
        @signal[sourceAddress](type="int");
        @signal[pkCount](type="int");
        @signal[sentPk](type=cPacket);
        @signal[rcvdPk](type=cPacket);
        @statistic[rcvdPk](title="packets received"; source=rcvdPk; record=count,"sum(packetBytes)","vector(packetBytes)"; interpolationmode=none);
        @statistic[sentPk](title="packets sent"; source=sentPk; record=count,"sum(packetBytes)","vector(packetBytes)"; interpolationmode=none);
        //@statistic[rcvdPkLifetime](title="received packet lifetime"; source="messageAge(rcvdPk)"; unit=s; record=stats,vector; interpolationmode=none);
        @statistic[endToEndDelay](title="end-to-end delay of arrived packets";unit=s;record=vector,mean,max;interpolationmode=none);
        @statistic[hopCount](title="hop count of arrived packets";interpolationmode=none;record=vector?,mean,max);
        @statistic[sourceAddress](title="source address of arrived packets";interpolationmode=none;record=vector?);
        @statistic[pkCount] (title="packet count";interpolationmode=none;record=vector,mean,max,sum);
    gates:
        input in;
        output out;
}
