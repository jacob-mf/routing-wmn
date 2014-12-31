//
// This file is part of an OMNeT++/OMNEST simulation example.
//
// 2012-4 L.Jacob Mariscal Fernández based on Copyright (C) 1992-2008 Andras Varga
//
// This file is distributed WITHOUT ANY WARRANTY. See the file
// `license' for details on this and other legal matters.
//


//
// Represents a packet in the network.
//
packet Packet 
// kind = 0 Data, 1 Hello_Msg, Pheromone diffusion, 2 Forward_Ant or RREQ, 3 Backward_Ant or RREP
// kind = 4 Proactive Forward Ant or RERR, 5 RREQ or FA (local repair), 6 stored waiting local repair solution(self message)
// kind = 8 Proactive Backward ant, kind=9 dataReply start message
{
    string displayString = "default,kind";
    int srcAddr;
    int destAddr;
    int hopCount;
    simtime_t travelTime;
    int transientNodes[]; //std::vector<int> transientNodes; size gives hops in path for BA
}

