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
simple UDPtrans like ITransport
{
    parameters:
        int address;
        @display("i=block/transport");
        @signal[sentPk](type=cMessage);
        @signal[rcvdPk](type=cMessage);
        @signal[passedUpPk](type=cMessage);
        @signal[droppedPkWrongPort](type=cMessage);
        @signal[droppedPkBadChecksum](type=cMessage);
        @statistic[sentPk](title="packets sent"; source=sentPk; record=count,"sum(packetBytes)","vector(packetBytes)"; interpolationmode=none);
        @statistic[rcvdPk](title="packets received"; source=rcvdPk; record=count,"sum(packetBytes)","vector(packetBytes)"; interpolationmode=none);
        @statistic[passedUpPk](title="packets passed up"; record=count,"vector(count)"; interpolationmode=none);
        @statistic[droppedPkWrongPort](title="packets dropped (wrong port)"; record=count,"vector(count)"; interpolationmode=none);
        @statistic[droppedPkBadChecksum](title="packets dropped (checksum error)"; record=count,"vector(count)"; interpolationmode=none);

    gates:
        input appIn @labels(UDPControlInfo/down);
        input ipIn @labels(UDPPacket,IPv4ControlInfo/up);
        //input ipv6In @labels(UDPPacket,IPv6ControlInfo/up);
        output appOut @labels(UDPControlInfo/up);
        output ipOut @labels(UDPPacket,IPv4ControlInfo/down);
        //output ipv6Out @labels(UDPPacket,IPv6ControlInfo/down);
}
