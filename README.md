routing-wmn
===========

OMNeT++ project about routing protocols under wireless mesh networks. Presents main brand new ACO based routing protocol AntWMNet, and  AODV and static protocols versions, also 3 WMNs samples

---------------------
﻿This is a brief description of the WMN-routing project for OMNeT++ 4.2.1
You can import the project to your workspace and run it as usual by git or download zip and unzip in directory to import to OMNet workspace.
Be aware to adjust your package settings that depends on which directory the project is placed in.

WMN-routing (Wireless Mesh Networks routing) is project about differents routing protocols to apply on wireless mesh networks, from KliKle (2 nodes) and  Wmn-base (9-415 nodes possible) capable to selected from init file 'omnetpp.ini' included in configurations 'Base' and 'KliKle'.

Most important files are 'AntNet.cc' y 'AntNetAP.cc' very  similar, describe routing protocols for normal terminal or access point (mesh node)respectively.
Also in 'omnetpp.ini' file routing protocol and other important parameters are selected, 'sort' parameter specifies routing protocol as: 1: static, 2: AntWMNet , 3: Cpant and 4: AODV

1. Demonstrates static shortest-path routing. Routing tables are set up at the
beginning of the simulation using the cTopology class. The model is
intentionally kept simple to facilitate understanding.

The network topology is the one widely known as the "NTT backbone", and it was
contributed by Mudassar Farooq.

Every node queries the topology of the network independently, using a cTopology
object. Then it computes shortest paths to every other node, and stores the
first nodes of the paths in a next-hop table. (Actually the table contains
the port number to the next-hop node not the node address itself -- the table
thus provides dest-address -> next-hop-address mapping). All the above takes
place once, at the beginning of this simulation. The topology is static during
the simulation, and so there's no need for the nodes to do anything to keep
the tables up-to-date. There's no routing protocol in the model.

Once the routing tables are set up, nodes start sending packets at random
intervals. Every node gets a list of destination addresses in a parameter,
and for every packet it randomly chooses a destination from the list.

Not a WMN good protocol, this is the same as in /samples/routing directory in OMNeT++, developed by Andras Varga. 
Test on network 'WmnStatic', in other dynamic networks, results are poor. 

2. Routing protocol AntWMNet, core of the project, based on ACO (Ant Colony Optimization). Begin from first beacon (hello) packets exchanged by neighbours nodes, updating neighbourhood tables and also routing tables (pheromone, hops estimation and last travel time). Then start a reactive set-up process from source nodes, launching FA (forward ants) searching by flooding destination node. If succeed BA (backward ant) is generated to go back and updae routing info (depends on choosen metric). At the same ttime, data transmission is starting (by  configurable paramter) from source to destination nodes. Also proactive proccess are active, checking known routes and discovering new ones , by proactive , FA,BA and pheromone diffusion (by hello packets) if link failures or topology changes on the network. The protocol start locla repair process in the moment a data packet has no positive pheromon choice to route.  

3. Cpant is not available at the moment
CPANT (Colored Pheromones Ants) is an algorithm based on Ant Colony Optimization by M. Dorigo and his AntHocNet routing protocol for Ad hoc networks.
The algorithm looks for a way of QoS (quality of service) of different kinds of packets, traffic , making paths, graded by feautures are good for every kind of data packet (they think the following kinds 1: P2P or FTP packet 2: TCP 80 browser packet 3: Streaming (Flash, MPEG...) packet 4: Concversational, VoIP packet).
Designed by Martina Umlauft and Wilfried Elmenreich.

4. AODV (Adhoc On-Demand Distance-Vector) big rival to compare results, very famous reactive protocol. Very similar at the beginning to our proposal,  launching RREQ (route request packets) in source nodes when no available route, and by flooding path is searched. Then RREP (route reply packet) go back, updating crossed nodes through successful route. If fails, new RREQ will be launched either optionally, a local repair proccess will start.
Right now need fix, no good results as at past, looks like corrupted by AntWMNet protocol, in the near future, good idea to separate routing protocol in different files.

---------------------------

There are three apps provided: App generates packets with exponential interarrival times, while BurstyApp alternates between active and idle periods. BurstyApp's implementation demonstrates the use of the FSMs (Finite State Machine). These ones are only interested with static protocol and not designed for WMN. Third option, 'AppUDP' is the most suitable option, act as UDP, and strat transmission of data packet from sources selected by vector 'srcAddresses' and destination by 'destAddresses'. 
You can choose them in the 'omnet.ini' file 
as **.appType= 'App' or 'BurstyApp' or 'AppUDP' (most used for AntWMNet and AODV)

Project is only tested in 'WMNBase' and 'WmnStatic' network in dir /networks
Check files 'License' and 'gpl' for license details

-------------------------
Added in github on folder /images/msg new icons for ant packets FA, BA (forward/backward ant)
These files should be in dir /images/msg from main OMNeT++ directory, e.g. /omnetpp-4.21  ,depends on version.

-------------------------
Output files, statistics, for calc sheets applications (xCel, Calc or similar);
'aodv-control.csv' AODV results
'app-packets.csv' Application UDP statistics
'control-packet.csv' AntWMNet results
   
2012-4 L.Jacob Mariscal Fernández based on Routing sample  Copyright (C) 1992-2008 Andras Varga
