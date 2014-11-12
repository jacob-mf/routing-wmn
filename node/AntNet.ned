//
// This file is part of an OMNeT++/OMNEST simulation example.
//
// Septiembre 2013 L.Jacob Mariscal Fernández based on Andras Varga's file
//
// This file is distributed WITHOUT ANY WARRANTY. 
//
// This file is free software: you can redistribute it and/or modify
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
// Models a routing protocol for wireless mesh networks, normal node
//
simple AntNet
{
    parameters:
        int sort=default(2); // Sort of routing 1: Static, 2: AntHocNet, 3: CPANT (Colored Pheromones Ant)
        double coefPh=default(0.7); // Update Pheromone Coefficient 
        int metrics=default(1);  //  # 1: time 2: hops 3: both
        double maxHopsRate=default(1.75); // Maximum Hops Rate
        double dataCoef=default(16); // Data packets spreading paths coeficient 
        bool flooding=default(true); // selects flooding or not
        int numNodes = default(9); // 9 in Wmn-base netowrk
        int maxArray = default(49); // define max array size in packets for Transient nodes field
        bool rTableInit= default(true); // define if Routing table initialization is performed
        bool mobile=default(false); // define if node X moves on the Wmn_base scenario
        bool stochastic=default(true); // Define if follow stochastic formula
        bool evaporation=default(true); //define evaporation in routing table values
        bool antsToApp=default(true); // choose if ants packets go to Application level
        bool goodDiffusion = default(true); // Define if use good diffusion (of best estimation hops) , default true
        volatile double sendFirstRREQ @unit(s)  = default(0.03s); // time to send first  RREQ packet in AODV 
        volatile double replyTime @unit(s)  = default(0.33s); // time to send reply timer to check 
        													  // RREP packet received in AODV 
        bool take1stPath=default(true); // take 1st path(neighbour on sight)  to destination
        bool localRepair=default(false); // select local repair on error node for AODV option 
        bool proBa = default (true); // active Proactive Backward ants
        bool alternateCheck = default(true); // active alternate launching of proactive BA from destination and source
        volatile double sendIaTime @unit(s) = default(exponential(1s)); // time between generating packets,
        																// here to perform good hit ratio
        string destAddresses; // contains all the destination addresses
        string srcAddresses; // contains all the source addresses
        double helloTime @unit(s) = default(1s); // time interval to send hello packets , kind = 1 
        double updateVTime @unit(ms) = default(0ms); // time until update Visiting Table 
        											// (default average 1 hop travel time)
        double hopTime @unit (s) = default(0.020s); // hop time estimation, default 20ms
        double checkTime @unit (s) = default(21s); // route check time (launch proactive BA) , default 21s
        int maxLocalRepairFlooding = default(3) ; // max flooding forwarding in local repair
        int cutRange = default (1); // specify cut conditions for FA, 0 : always, 1 : if worst or equal, 2: only if worst 
                    				// relative to hops estimation
        int cutRangePro = default (1); // specify cut conditions for local repair or Proactive FA, 0 : always, 
        							// 1 : if worst or equal, 2: only if worst ; relative to hops estimation
                    				                    				
        //int headerCost = default (1); // 1: minimal cost, 2: add pheromone diffusion cost 
        
        @display("i=block/switch");
        @signal[drop](type="long");
        @signal[outputIf](type="int");
        @signal[controlHops] (type="long");
        @signal[overheadCost] (type="long");
		@signal[hitRatio] (type="double");

		@statistic[controlHops](title="hop count of control packets, except pheromone diffusion packets";interpolationmode=none;record=vector?,mean,max);
		@statistic[overheadCost](title="hop count of control packets, define a new way of overhead cost";interpolationmode=none;record=vector?,mean,max);
		@statistic[hitRatio](title="Hit ratio of successful arrived data packets";unit=s;record=vector,mean,max;interpolationmode=none); 
        @statistic[drop](title="dropped packet byte length";unit=bytes;record=vector?,count,sum;interpolationmode=none);
        @statistic[outputIf](title="output interface for each routed packet";record=histogram;interpolationmode=none);
    gates:
        input in[];
        output out[];
        input localIn;
        output localOut;
}


