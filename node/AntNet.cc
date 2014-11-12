//
// This file is part of an OMNeT++/OMNEST simulation example.
//
// 2012-4 L.Jacob Mariscal FernÃ¡ndez based on Copyright (C) 1992-2008 Andras Varga
//
// This file is distributed WITHOUT ANY WARRANTY. See the file
// `license' for details on this and other legal matters.
//

#ifdef _MSC_VER
#pragma warning(disable:4786)
#endif

#include <map>
#include <omnetpp.h>
#include  <string.h>
#include "Packet_m.h"
#include <iostream>
#include <fstream>
#include <math.h>
//#include "INETDefs.h"

#define SimTimeLimit 120 // define simulation time ending for check (BA Proactive launch)
/**
 * Demonstrates static routing, utilising the cTopology class.
 * Also introduces AntWMNet and CPANT routing protocols
 */
class AntNet : public cSimpleModule
{
  private:
    // Parameters
    cPar *mySort; // MySort(defined by sort in omnet.ini) specifies sort of routing as follows:
                  // 1 : Static (by Andras Varga) , 2 : AntWmNet , 3 : CPANT  , 4 : AODV
    cPar *coefPh; // Pheromone learning coefficient (defined in omnet.ini) should be on  0 < coefPh  < 1
    cPar *cutRange; // specify cut conditions for FA, 0 : always, 1 : if worst or equal, 2: if worst
                     // relative to hops estimation
    cPar *cutRangePro; // specify cut conditions for local repair, proactive FA, 0 : always, 1 : if worst or equal
                        // 2: if worst ; relative to hops estimation
    cPar *flooding; // specify if use flooding technique (boolean defined in omnet.ini)
    cPar *metrics;  // select either 1 :TimeToEndDelay 2: Hops 3: Linear combination of previous metrics
                    // (defined in omnet.ini)
    cPar *numNodes; // number of nodes in the network
    cPar *maxHopsRate; // Maximum hops Rate coefficient
    cPar *rTableInit;  // routing table initialisation flag
    cPar *updateVTime; // delay time when updating Visiting table (seeking more new paths) //not good idea
    cPar *replyTime;  // reply time until check if route request was successful
    cPar *helloTime; // time interval to send hello packets , packets with kind = 1
    cPar *evaporation; // flag if evaporation occurs
    cPar *antsToApp; // select if Ants packets reach Application level (default true for Application statistics)
    cPar *take1stPath; // go to destination at the first sight trail
    cPar *maxArray; // max array size limit (prevent memory problems)
    cPar *sendFirstRREQ; // time to send first RREQ packet in AODV
    cPar *localRepair; // (boolean) select local repair on error node for AODV option
    cPar *mobile;// select mobile scenario, node X travel
    cPar *sendIaTime; // interval of data packets
    cPar *dataCoef;  // Data packets routing spreading paths coefficient
    cPar *stochastic; // Define if follow stochastic formula
    cPar *goodDiffusion; // Define if use good diffusion (of best estimation hops) , default true
    cPar *hopTime; // For local repairing wait time formula
    cPar *maxLocalRepairFlooding; // Max flooding steps in Local Repair process
    cPar *checkTime; // time interval to send proactive BA to recalculate known paths
    cPar * proBa; // active proactive BA
    cPar *alternateCheck; // active alternate check self messages from destination and source to produce
                          // less control hops (overhead cost) ; default true
    //cPar *headerCost; // 1: minimal cost, 2: add pheromone diffusion cost, bad idea, better show  both cost

    // vectors
    std::vector<int> destAddresses; // list of destination addresses (usually mesh nodes)
    std::vector<int> srcAddresses; // list of destination addresses (usually mesh and request data transmission nodes)

    // Local variables
    int myAddress; // define node address
    //int numNb;        // Neighbours number, not useful
    int numData;      // Data packets total
    int maxHops;     // Maximum hops estimation
    int controlHops; // counter of Control packets hops (overhead cost, neighbour packets not included)
    int controlHops2; // counter of control packets (pheromone diffusion) for other overhead cost
    int redundant;   // Counter of redundant packets generated
    int baCounter,rrepCounter; // counter of BA packets, RREP (AODV)
    int faCounter,rreqCounter; // counter of FA packets, RREQ (AODV)
    int nbCounter,rerrCounter; // counter of Hello packets (detect neighbours issue), RERR (AODV)
    int proBaCounter; // counter of Proactive BA
    double costSrc, costDest; // store last calculated cost in node to source/destination (debugging info)
    int phDiffCounter; // counter of Pheromone diffusion packets (hello packets)
    int dataCounter;// data packets counter
    int dropCounter; // data drop packets counter
    int hitCounter; // arrived right destination data packets counter
    int repairCounter; // local repair packets
    bool waitingReply; // flag to indicate waiting Reply timer state (avoid send multiple) on AODV
    double avgTravel,avgTime1hop,avgHops; // Hit data packets travel time average, hops average
    double checkingTime; // store checkTime (alternate or not)
    int totalCounter,proactiveCounter,minHops;// total packets counter, proactive FA counter, minimum hops possible
        // (for standard numNodes in Base & Kli Kle configurations)
    int visitor; // FA packets visitor counter to handle Visiting table (terrible error when forgetting to initialise)

    // Local Tables
    typedef std::map<int,int> RoutingTable; // destination address -> gate index (gate index -1, means unknown path)
                                      //or gate index  -> number of hops to destination (as part of RHtable for AntWMNet)
    RoutingTable rtable;  // Routing table for static and AODV protocols
    RoutingTable ntable; // Neighbours table; gate index -> neighbour address
    typedef std::map<int,double> ProbTable; // gate index -> probability (or time cost in RTtable)
    typedef std::map<int,ProbTable> RPtable; // destination address -> probability table of gateOutIndex
    typedef std::map<int,RoutingTable> RHtable; // destination address -> number hops from gate index
    typedef std::map<int,long int> VisitingTable; // position visitor -> FA id visitor
    typedef std::map<int,ProbTable> AuxTable; // gate -> hops & last cost time stored
    typedef std::map<int,AuxTable> RTtable; // destination address -> gate -> hops -> last cost time stored

    RTtable ttable; //routing time cost table for AntWMNet
    RPtable ptable; // routing probability table for AntWMNet
    VisitingTable vtable; // visitors table for AntWMNet
    RHtable htable; // number of hops to destination routing table for AntWMNet protocol

    // Messages: define movement, generations and start procedures
    cMessage * move1,* move2,*move3,*move4,*updateVT,*generateRREQ; // *replyTimer;
            // move messages 1-4, message to start update of Visiting table and start of RREQ process
    cMessage *move5,*move6,*move7,*move8,*move9,*move10,*move11,*move12,*move13; // movement messages 5-13
    cMessage * goodDiff; // start good Diffusion procedure

    // Signals
    simsignal_t dropSignal;
    simsignal_t outputIfSignal;
    simsignal_t controlHopsSignal;
    simsignal_t overheadCostSignal;
    simsignal_t hitRatioSignal;

    // Local functions
    int bestHopsEstimation(int dest); // return best hops estimation to destination node
    int bestHopsEstimationChoices(RoutingTable & choices ,int dest,cMessage *msg);
                             // return number of gates with best hops estimation to destination and these gates in a vector
    int bestHopsEstimationChoicesLR(RoutingTable & choices ,int dest,cMessage *msg); // for local repair 1st step
                              // return number of gates with best hops estimation to destination and these gates in a vector
    void calcAvg1hop(double d); // debugging function to calculate time average per one hop
    int changeChannelParamsByGate(int gate,double dr, double delay, double jit, double noise);// Pre: bidirectional links
    // return 1 if can change channel parameters, 0 if unconnected gate
    bool checkFloodingLimit(cMessage *msg); // Pre: local repair FA ; check also if local repair is on
    int checkLimits(int hops);
                // checking max hops and max array limits of packet, return 0 if correct, 1 if max hops, 2 if max array
    bool checkPreviousVisit(int source, int hops,unsigned int origin, long treeid);
        // check if already visited and if better estimation hops (need source address, hops, tree id (long)
    void dataReport(cMessage *msg); // specific used for Wmn-Base network, data packets debug
    void forwardDataPacketByRPTable(cMessage *msg); // AntWMNet data forwarding process
                                // Pre: Data packet parameter, Positive pheromones values confirmed for destination packet
    double getLastTime(int dest, int gate, int hops);// get last cost time saved by destination, gate, hops
    void goodPhDiffusion(int dest, int bestHops, int origin); // good pheromone spread over neighbours except origin gate
    void initReactiveFA(int dest); // Pre: we know no positive pheromones values on actual node
                                    // start reactive FA process from actual node to destination
    bool isBetterHopsEstimation(int dest, int gate, int hops); // return if it is better hops estimation than actual value
    bool isConnected(int g); // check if gate is connected
    bool isDestAddress(int addr); // check if it is a destination address
    bool isEqualPath(int dest, int hops); // with hops metrics save packet with shortest path
                                          // (shorter or same than previous)
    bool isInPath(cMessage *msg, int g); // inspect if node (int) was crossed before by packet
    bool isShortestPath(int dest, int hops); // with hops metrics save packet with shortest path
                                                // (shorter or same than previous)
    bool isSrcAddress(int addr); // check if it is a source address
    void linkError(int dest, int gate); // link error to dest from gate after update routing info and neighbour table
    bool locateNeighbor(int dest , int & outgate);
    // check if destination is on Neighbour table, and collect the gateId to the destination on variable outgate
    void moveNode(int pos); // node X movement process
    int neighbourGates(RoutingTable & gates); // store neighbour gates from neighbour table and return number of neighbours
    int neighbourGates(RoutingTable & gates, int origin); // skip origin gate
    int nextGateProBA(cMessage * msg); // return next gate to forward Proactive BA, -1 means error, no route in actual node
    int nextTravelChoices(RoutingTable & choices, int origin, cMessage *msg); // check travel choices but no origin gate
    bool noisyChannel(cPacket *pk); // detect noisy channels
    double normCost(double d); // normalise to non zero neither negative value (for bad symmetric estimations)
    void pheromoneDiffusion(int dest, int bestHops); // spread routing info to neighbours about
                                                    // new best estimation hops distance to destination
    void pheromoneDiffusion(int dest, int bestHops, int origin); // spread over neighbours except origin gate
    void pheromoneDiffusion(int dest, int bestHops, int origin, cPacket *pk); // spread packet, to keep same TreeId
                                                                        // to allow detect redundant packets
    int positiveGoodPhChoices(int dest,unsigned int origin,cMessage * msg); // number of good positive pheromone choices
                                 // skipping origin and gates to previous visited nodes
    int positivePheromonChoices(int dest); // return number of positive pheromone choices
    int positivePheromonChoices(RoutingTable & choices, int dest, cMessage * msg);
    // return number of positive pheromone choices, and store it (gates) in choices array
    void routeRequest(int dest); // AODV start route request process to dest
    bool routeRequestRERR(int dest, int origin, int next); // route request launched by RERR, generates local repair RREQ
                                                // true if RREQ local repair were sent
    int selectOneDestination(); // from destination addresses vector // Pre : not empty destAddresses vector
    int selectOneSource(); // from source addresses vector // Pre : not empty srcAddresses vector
    void sendNbPacket(int k); // send neighbour packet from gate k
    void sendNbPacketTo(int gate, int dest); // send neighbour packet from gate k to dest
    void sendProactiveBA(); // Pre: launched from source or destination address
                            // Generate Proactive BA and launching to destination to recalculate path
    void sendProactiveFA(int dest, double cost,int hops,int origin); // send proactive FA from uncompleted route
    void sendProactiveFAtoSrc(int source, double cost,int hops, int origin); // Pre: detected new path to destination
                                            // send Proactive FA to source addresses
    void sendRERR(int gate, int dest); // AODV, send RERR packet to dest by gate
    void sendRERR(int gate, int source, int dest); // AODV, send RERR packet to dest by gate from source
    void showCost(double cost, int dest); // show old cost info and new cost evaluation to dest address (debug)
    void showCostTable(); // show time cost tables (debug) // (not working perfect still)
    void showChannelParams(); // display request channel (recently used) parameters
    bool showChannelParamsByPort(int port); // display channel parameters requested by port
    void showRoutingInfo(); // show RPTable of pheromone, estimated hops and last time cost values to
                            //  singular destinations(src, dest addresses) and actual neighbours
    void showRoutingInfo(int dest);  // show RPTable of pheromone, estimated hops and last time cost values
                                    // about specified destination
    void showRTable(); // display interesting routing table info in AODV or static protocols
    int skipOriginChoices(RoutingTable & choices, int origin); // Pre: In local repair process
                            // delete choices across origin port
    void saveIntValue(int v); // save to file integer value (debug)
    int symmetricGate(int g); // return symmetric (opposite) gateID  // Pre : special network BaseWmn specifications
    const char * toDoubleCalc(double d); // return numeric value formatted for Math applications (swap '.' to ',' notation)
    int travelChoices(RoutingTable & choices, cMessage *msg); // return number and possible destinations (in vector choices)
    void tryLocalRepair(cMessage * msg, bool & comeback); // Pre: Data packet found no positive pheromone value to dest
                                        // begin local repair procedure with local repair FA
    void updateHTable(int dest, unsigned int outgate,int hops); // update number of hops routing table for AntWMNet
    void updateNb(int gate, int value,int hops); // update hops (if value not zero), neighbour table with value and
                                            // sent to new neighbour a hello packet by gate
    void updateRoutingInfoOnGateError(int gate); // reset tables on position across selected gate (recently unconnected)
                                                // Pre: Gate error, node out of sight from output gate
    void updateRPTable (int dest, unsigned int outgate, double cost,int hops ); // update routing probability table and
                                            // last time cost for AntWMNet protocol
    bool wasHereBefore(long int id);// check if same occurrence of FA packet passed by before

  public:
    AntNet();
    virtual ~AntNet();
  protected:
    virtual void initialize();
    virtual void handleMessage(cMessage *msg);
    virtual void finish();
};

Define_Module(AntNet);

AntNet::AntNet() // comment on static scenario the move initialisation and cancel&delete
{
    generateRREQ = NULL;move1=NULL;move2=NULL;move3=NULL;move4=NULL;move5=NULL;move6=NULL;
    //if (goodDiffusion->boolValue())  goodDiff = NULL;
    move7=NULL;move8=NULL;move9=NULL;move10=NULL;move11=NULL;move12=NULL;move13=NULL;//replyTimer=NULL;
//    updateVT=NULL;
}

AntNet::~AntNet()
{
    cancelAndDelete(generateRREQ);cancelAndDelete(move1);cancelAndDelete(move2);cancelAndDelete(move3);
    cancelAndDelete(move4);
    cancelAndDelete(move5);cancelAndDelete(move6);cancelAndDelete(move7);cancelAndDelete(move8);
    cancelAndDelete(move11);cancelAndDelete(move12);cancelAndDelete(move10);cancelAndDelete(move9);
    cancelAndDelete(move13);//if (goodDiffusion->boolValue()) cancelAndDelete(goodDiff);
//    //cancelAndDelete(replyTimer);
//    cancelAndDelete(updateVT);
}

void AntNet::initialize()
{
    // parameters
    myAddress = getParentModule()->par("address");
    dataCoef= &par("dataCoef");
    stochastic=&par("stochastic");
    mySort =  &par("sort") ;
    mobile = &par("mobile");
    flooding = &par("flooding");
    metrics = &par("metrics");
    numNodes = &par("numNodes");
    rTableInit = &par("rTableInit");
    evaporation = &par("evaporation");
    antsToApp=&par("antsToApp");
    take1stPath=&par("take1stPath");
    maxHopsRate=&par("maxHopsRate");
    sendFirstRREQ = &par("sendFirstRREQ");
    replyTime = &par("replyTime");
    helloTime = &par("helloTime");
    maxArray=&par("maxArray");
    sendIaTime=&par("sendIaTime");
    localRepair=&par("localRepair");
    goodDiffusion = &par("goodDiffusion");
    maxLocalRepairFlooding = &par("maxLocalRepairFlooding");
    hopTime= &par("hopTime");
    coefPh= &par("coefPh"); // Pheromone update coefficient
    updateVTime=&par("updateVTime");
    checkTime = & par("checkTime");
    //headerCost=&par("headerCost");
    proBa = & par("proBa");
    alternateCheck= & par("alternateCheck");
    cutRange = & par ("cutRange");
    cutRangePro = & par ("cutRangePro");

    // signals
    dropSignal = registerSignal("drop");
    outputIfSignal = registerSignal("outputIf");
    controlHopsSignal= registerSignal("controlHops");
    overheadCostSignal = registerSignal("overheadCost");
    hitRatioSignal = registerSignal("hitRatio");

    // Local variables
    proactiveCounter=0;phDiffCounter =0; repairCounter=0;//numNb=0;
    baCounter=0;faCounter=0;avgTime1hop=0;avgHops=0;minHops=0;controlHops=0;rerrCounter=0;rrepCounter=0;
    nbCounter=0;totalCounter=0; dropCounter=0;dataCounter=0;hitCounter=0;avgTravel=0;rreqCounter=0;
    waitingReply= false;controlHops2=0;proBaCounter= 0;costSrc=0;costDest=0;
    visitor=0;checkingTime = 0;
    if (numNodes->longValue()==2) minHops=1;
    if (numNodes->operator int()==9) minHops=4;
    if (numNodes->longValue()==99) minHops=19; // 9 (Width-1) + 6 (height-3) +4 (base)
    if (numNodes->operator int()==205) minHops=28; // h,w (14,14)
    if (numNodes->longValue()==315) minHops=35; // h,w(18,17)
    if (numNodes->longValue()==409) minHops=40; // h,w (20,20)
    // these numbers are only valid for 2 minutes of simulation as its default on omnet.ini
    if (sendIaTime->doubleValue() == 0.1) numData=1200;
    else if (sendIaTime->doubleValue() == 0.25) numData=480;
    else if (sendIaTime->doubleValue() == 0.5) numData=240;
    else if (sendIaTime->doubleValue() == 0.1666666) numData=720;
    else if (sendIaTime->doubleValue() == 0.083333) numData=1440;
    else if (sendIaTime->doubleValue() == 0.125) numData=960;
    maxHops = (maxHopsRate->doubleValue()) * numNodes->doubleValue(); // other chance: topo->getNumNodes() - 1;
                                                                // other option:  maxHopsRate(~1.75) * NumNodes

    // Watch variables for simulation environment
    WATCH(baCounter);WATCH(faCounter);WATCH(dropCounter);WATCH(dataCounter);
    WATCH(hitCounter);WATCH(totalCounter);WATCH(baCounter);WATCH(nbCounter);
    WATCH(redundant);WATCH(hitCounter);WATCH(avgTravel);

    if (  mySort->longValue() == 1 ) // static routing
    {
       //
       // Brute force approach -- every node does topology discovery on its own,
       // and finds routes to all other nodes independently, at the beginning
       // of the simulation. This could be improved: (1) central routing database,
       // (2) on-demand route calculation
       //
      cTopology *topo = new cTopology("topo");
      std::vector<std::string> nedTypes;
      nedTypes.push_back(getParentModule()->getNedTypeName());
      topo->extractByNedTypeName(nedTypes);
      EV << "cTopology found " << topo->getNumNodes() << " nodes\n";
      cTopology::Node *thisNode = topo->getNodeFor(getParentModule());
      maxHops =  topo->getNumNodes() - 1;
      // find and store next hops
      for (int i=0; i<topo->getNumNodes(); i++)
        {
            if (topo->getNode(i)==thisNode) continue; // skip ourselves
            topo->calculateUnweightedSingleShortestPathsTo(topo->getNode(i));

            if (thisNode->getNumPaths()==0)
                {
                EV << "Not connected , node: " << i << endl;
                continue; // not connected
                }

      cGate *parentModuleGate = thisNode->getPath(0)->getLocalGate();
      int gateIndex = parentModuleGate->getIndex();
      int address = topo->getNode(i)->getModule()->par("address");
      rtable[address] = gateIndex;
      EV << "  towards address " << address << " gateIndex is " << gateIndex << endl;
       }
      if ((myAddress == 7 ) && mobile->boolValue())  // node cell phone X, start moving on if mobile scenario
      {
           move1 = new cMessage("Move 1"); // toggle comment on static studies
           scheduleAt(14.63,move1);
      }
      delete topo;
    }
    else if (mySort->longValue() >= 2) // AntHocNet, AODV routing
    {
        if (mySort->longValue() == 2)
        {}// EV << "AntWMNet routing chosen, but is still in progress" << endl;
        else if (mySort->longValue() == 3)
                    EV << "CPANT routing chosen, but is still in progress" << endl;
        else if (mySort->longValue() == 4)
            EV << "AODV routing chosen" << endl;
        // Locate Neighbors

        //for (int j=0; j<thisNode->getNumOutLinks();j++)
        //{
          //  cTopology::LinkOut *nlink = thisNode->getLinkOut(j);
            //if (nlink->isEnabled()) {
                //enable = true;
              //  int neighbourIndex = nlink->getLocalGate()->getIndex();
                //EV << "Node: " << i << " OutGate :" << gateIndex <<  << endl;
                //char pkname[40];
                //sprintf(pkname,"pk-locate-neihgbor-node %d-from gate %d-#%d", myAddress,neighbourIndex,nbCounter++ );
                //EV << "generating packet " << pkname << endl;
                //Packet *pk = new Packet(pkname);
                //cPar *packetLengthBytes = &par("packetLength");
                //pk->setByteLength(packetLengthBytes->longValue());
                //pk->setSrcAddr(myAddress);
                //pk->setKind(1); // 1: hello msg
                //pk->setDestAddr(myAddress);
                //pk->setTransientNodesArraySize(1); // initialize to safe value
                //send(pk,"out",neighbourIndex);
                  //                  }
         //}

    // initialize Probability Routing Table to 1/NumNodes -> not needed, better a option

    EV << "Node : " << myAddress << " Name:" << getParentModule()->getName() << endl; // <<" Size : " << size() << endl;
            //bool enable, enable_dest = false;
            //for (int i=0; i<topo->getNumNodes(); i++)
            //{
              //  if (topo->getNode(i)==thisNode) continue; // skip ourselves
                //topo->calculateUnweightedSingleShortestPathsTo(topo->getNode(i));
                //if  (thisNode->isEnabled() ) { //(thisNode->getNumPaths()==0) {
                  //  EV << "Not connected , node: " << i << endl;
                   // continue; // not connected
                    //}
               //for (int j=0; j<topo->getNode(i)->getNumOutLinks();j++) {
                 //  cTopology::LinkOut *link_dest = topo->getNode(i)->getLinkOut(j);
                   //if (link_dest->isEnabled()) {
                     //  enable_dest = true;
                   //}
               //}
               //if (!enable_dest) {
                 // EV << "Not connected to destination node: " << i << endl;
                   //                   continue; // not connected
               //}
                //for (int j=0; j<thisNode->getNumOutLinks();j++)
                //{
                    //int address = topo->getNode(i)->getModule()->par("address");
                    //probtable[address][j] = 1.0 / (topo->getNumNodes());
                  //  cTopology::LinkOut *link = thisNode->getLinkOut(j);
                    //int r = link->getRemoteGateId();
                    //if (link->isEnabled()) {
                      //  enable = true;
                        //int gateIndex = link->getLocalGate()->getIndex();
                        //int address = topo->getNode(i)->getModule()->par("address");
                        //ptable[address] [gateIndex]= 1.0 / (thisNode->getNumOutLinks());
                        //EV << "Node: " << i << " name: " << getParentModule()->getName() << " OutGate :" << gateIndex << " Prob: " << ptable[address][gateIndex]  << " OutLinks: " << thisNode->getNumOutLinks() << endl;
                          //                  }
                //}
                //int k = getParentModule()->gateCount();
                //int gateId = getParentModule()->gateBaseId("port$i");
                //k--;
               // while (k > 0)            {
    maxHops = numNodes->doubleValue() * maxHopsRate->doubleValue(); // other chance numNodes -1

    const char *srcAddressesPar = par("srcAddresses"); // get source addresses
    cStringTokenizer tokenizer2(srcAddressesPar);
    const char *token2;
    while ((token2 = tokenizer2.nextToken())!=NULL)
           srcAddresses.push_back(atoi(token2));

    const char *destAddressesPar = par("destAddresses"); // get destination addresses
    cStringTokenizer tokenizer(destAddressesPar);
    const char *token;
    while ((token = tokenizer.nextToken())!=NULL)
          destAddresses.push_back(atoi(token));

    int j = getParentModule()->gateSize("port"); // network neighbours searching
    int numDest =0;
    int gateDest[7]; // Initialise array
    for (int k=0; k<7;k++)
        gateDest[k]=0;
    for (int k=0; k<j; k++) {
        if ( getParentModule()->gate("port$o",k)->isConnected()) {
            //bool enable1 = getParentModule()->gate(gateId+k)->isConnected();
            ntable[k] = 0; // initialise neighbour table to 0
            gateDest[numDest] = k;numDest++;
            //EV << "Connection detected between nodes in " << getParentModule()->getName() << " by gate out: " << k << endl;
            sendNbPacket(k);// send neighbour packets to gate k
            }
    }

    //numNb=numDest;  // neighbours number
    if ((numDest !=0 ) && (mySort->longValue() == 2)) { // AntWMnet branch, initialise routing tables
       double temp = 1.0 / (numDest);
       for (int k=0; k<numDest;k++ ) {
           for (unsigned int n=0; n< destAddresses.size(); n++ ) {
               if (rTableInit->boolValue())   //
               {   ptable[ destAddresses[n]] [gateDest[k]] = 1.0 / (numDest) ; // update destination addresses
                   htable[destAddresses[n]] [gateDest[k]]=0; //initialise to 0 the routing hops table
               }
               else {ptable[ destAddresses[n]] [gateDest[k]] = 0  ; // initialise to 0 routing table addresses
                     htable[destAddresses[n]] [gateDest[k]]=0; //initialise to 0 the routing hops table
                     temp=0;
                     // for testing process initReactiveFA to 10 (node F)
//                   ptable[10][gateDest[k]] = 0;
//                   htable[10][gateDest[k]] = 0;
                    }
               //EV << "Probability: " <<  temp << " in node " << getParentModule()->getName() << " gate: " << gateDest[k] << " dest: "<< destAddresses[n] << endl;
            }
       }
    }

    if (numDest == 0) {
       EV << "Not connected to outside, source node: " << getParentModule()->getName() << endl;
       //continue; // not connected
    }

    if (proBa->boolValue()) { // assign correct check Time
        if (alternateCheck->boolValue() ) checkingTime = (2 * checkTime->doubleValue());
        else checkingTime = checkTime->doubleValue();
    }


    if ((myAddress == 2) ||  (myAddress == 8) || (myAddress ==11)) { //if node Kli or other source nodes
       if  (mySort->longValue() == 4) { // AODV branch
           if ((numNodes->longValue() > 10 ) && (myAddress == 11)) {
              generateRREQ = new cMessage("firstRREQ");
              scheduleAt(simTime()+sendFirstRREQ->doubleValue(),generateRREQ);
              EV << "Generate packet First RREQ from address: "<< myAddress << endl;
            }
            if ((numNodes->longValue() == 9 ) && (myAddress == 8)) {
               generateRREQ = new cMessage("firstRREQ");
               scheduleAt(simTime()+sendFirstRREQ->doubleValue(),generateRREQ);
               EV << "Generate packet First RREQ from address: "<< myAddress << endl;
             }
      }

      if ((mySort->longValue() == 2) && (proBa->boolValue())) { //AntWMNet & proactive BA active

           if ((numNodes->longValue() > 10 ) && (myAddress == 11)) { // scenario with more than 10 nodes
               cMessage * check2 = new cMessage("checking");
               check2->setKind(8); // flag as checking self message (launching proactive BA)
               scheduleAt(simTime() + checkingTime, check2); // in checkTime ,or 2* checkTime (control overhead cost)
           }
           if ((numNodes->longValue() == 9 ) && (myAddress == 8)) { // basic, default scenario
               cMessage * check3 = new cMessage("checking");
               check3->setKind(8);
               scheduleAt(simTime() + checkingTime, check3); // in checkTime or double to control overhead cost
           }
       }
    }
    if ((myAddress == 7 ) && mobile->boolValue())  // node cell phone X, start moving on if mobile scenario
    {
        move1 = new cMessage("Move 1"); // toggle comment on static studies
        scheduleAt(14.63,move1);
        // initial BA proactive test at checkTime (default 21s interval)
        if (proBa->boolValue()) {
            cMessage * check = new cMessage("checking");
            check->setKind(8);
            scheduleAt(simTime() + checkTime->doubleValue(),check);
            // in checkTime, 2*checkTime (different interval from source node)
        }
    }
//                if ((myAddress == 4 )) { // just testing initReactiveFA process from A to route to node F (10)
//                    cMessage * test = new cMessage("Testing initReactiveFA");
//                    test->setKind(7); // to distinguish it
//                    scheduleAt(33,test);
//                }
            //}
            //int temp=3;
            //double test = ptable[myAddress-1] [temp] +  (double) 1/temp;
            //EV << "Double test: " <<  test << " tempâ�»1: " << (double) 1/ temp << " Valor a anterior nodo en tercer puerto: "<< ptable[myAddress-1] [temp]<< endl;
    }
    else // Cpant routing
    {
        EV << "CPANT routing chosen, but still is not available" << endl;
        //EV << "Node : " << thisNode->getModuleId() << " Address : " << myAddress <<  " Size : " << size() << endl;

    }
    //delete topo;
}

int AntNet::bestHopsEstimation(int dest) // return best hops estimation to destination node
{
 int hops=0;
 int maxGates = getParentModule()->gateSize("port");
 for (int i=0; i< maxGates;i++){
     if (htable[dest][i] > 0) {
         if (hops ==0) hops= htable[dest][i];
         else{
             if (htable[dest][i] < hops) hops= htable[dest][i];
         }
     }
 }
 return hops;
}

int AntNet::bestHopsEstimationChoices(RoutingTable & choices ,int dest, cMessage * msg)
{ // return number of gates with best hops estimation to destination and these gates in a vector
    int num=0;
    int bestHops = bestHopsEstimation(dest);
    unsigned int maxChoices = getParentModule()->gateSize("port");
    for (unsigned int i=0; i < maxChoices; i++) { // use port out size, previous error use ntable size
        if (isConnected(i) && (htable[dest][i] == bestHops) && (!(isInPath(msg,ntable[i]))))
        { // check if could be a loop way)
           //EV << "Adding choice gate " << i  <<" in neighbour table with size " << ntable.size() << " is neighbour "<< ntable[i] << " for destination:"<< dest << " with best hops estimation:" << bestHops << endl;
           choices[num]=i;
           num++;
        }
    }
    return num;
}

int AntNet::bestHopsEstimationChoicesLR(RoutingTable & choices ,int dest,cMessage *msg) // for local repair 1st step
{  // return number of gates with best hops estimation to destination and these gates in a vector
    int num=0;
    int bestHops = bestHopsEstimation(dest);
    unsigned int maxChoices = getParentModule()->gateSize("port");
    for (unsigned int i=0; i < maxChoices; i++) { // use port out size, previous error use neighbour table size
         if (isConnected(i) && (htable[dest][i] == bestHops))  // && (!(isInPath(msg,ntable[i]))))
         { // no checking if could be a loop way)
            //EV << "Adding choice gate " << i  <<" in neighbour table with size " << ntable.size() << " is neighbour "<< ntable[i] << " for destination:"<< dest << " with best hops estimation:" << bestHops << endl;
            choices[num]=i;
            num++;

            }
        }
        return num;
}

void AntNet::calcAvg1hop(double d)
{ // debugging procedure to calculate time average per one hop
    std::ofstream myfile("avg1hop.csv", myfile.app );
            if (myfile.is_open())
              {
                myfile << toDoubleCalc(d) << " ";

                myfile.close();
              }
              else { EV << "Unable to open file" << endl; }
}

int AntNet::changeChannelParamsByGate(int gate,double dr, double delay, double jit, double noise)
// Pre: bidirectional links
{ // return 1 if can change channel parameters, 0 if unconnected gate
    if ( getParentModule()->gate("port$i",gate)->isConnected()) { // check if selected gate is connected
                    getParentModule()->gate("port$o",gate)->getTransmissionChannel()->par("jitter").setDoubleValue(jit);
                    getParentModule()->gate("port$i",gate)->getIncomingTransmissionChannel()->par("jitter").setDoubleValue(jit);
                    getParentModule()->gate("port$i",gate)->getIncomingTransmissionChannel()->par("datarate").setDoubleValue(dr);
                    getParentModule()->gate("port$o",gate)->getTransmissionChannel()->par("datarate").setDoubleValue(dr);
                    getParentModule()->gate("port$o",gate)->getTransmissionChannel()->par("delay").setDoubleValue(delay);
                    getParentModule()->gate("port$i",gate)->getIncomingTransmissionChannel()->par("delay").setDoubleValue(delay);
                    getParentModule()->gate("port$i",gate)->getIncomingTransmissionChannel()->par("noise").setDoubleValue(noise);
                    getParentModule()->gate("port$o",gate)->getTransmissionChannel()->par("noise").setDoubleValue(noise);
                } else return 0; // wrong, gate unconnected

    return 1; // successful channel changed
}

bool AntNet::checkFloodingLimit(cMessage *msg) // Pre:  local repair FA
{
    Packet *pk = check_and_cast<Packet *>(msg);
    //if (pk->getKind() == 5) { // local repair FA
    int floods = pk->par("floods").longValue();
    EV << "Previous flooding times done: " << floods << endl;
    if (floods == maxLocalRepairFlooding->longValue())
       return true;
    floods++;
    pk->par("floods").setLongValue(floods); // increase floods
    //}
    return false;
}

int AntNet::checkLimits(int hops)
// checking max hops and max array limits of packet, return 0 if ok, 1 if max hops, 2 if max array
{
    if (hops > maxHops) return 1;
    if (hops > maxArray->longValue()) return 2;
    return 0;
}

bool AntNet::checkPreviousVisit(int source, int hops,unsigned int origin, long treeid)
//check if already visited and if better estimation hops (need source address, hops, treeId (long)
{ // for Local repair and proactive FA
    if (flooding->boolValue()){ // needs active flooding
       if ((visitor == 0) || (! wasHereBefore(treeid))) // beware, so only with one visit destroy FA packets
       {
          vtable[visitor] = treeid;
          //EV << "Updated  visiting table, position: "<< visitor << " FA id: " << vtable[visitor] << endl;
          visitor++;
          if (isBetterHopsEstimation(source,origin,hops))
              // good idea save here time to use then by BA as cost time to source (need TravelTime in process)
              updateHTable(source,origin,hops); // update hops table, as its first packet to enter node
       } else {
           if (cutRangePro->longValue() == 0){ // cut always, even if shortest path
               if (isBetterHopsEstimation(source,origin,hops))
                   updateHTable(source,origin,hops);
               return true; // no need to check sth more
           }
            if (metrics->longValue() > 1 ) { // if hops or hops & time metrics, update hops table if less hops
               //EV <<"Hops table value is:"  << htable[source][origin] << " dest:"<< source << " gate:"<< origin << endl;
               if (isShortestPath(source,hops)) {
               // process to check if pk take the shortest route by the moment (isShortestPath(dest,g hops) )
                  if ((cutRangePro->longValue() == 1) && isEqualPath(source,hops) ) {
                      EV << "FA id: " <<  treeid << " already checked here by other packet, but by equal path (in hops), so discarding Proactive FA packet"  << endl;
                      if (isBetterHopsEstimation(source,origin,hops)) // looks no necessary
                          updateHTable(source,origin,hops);
                      return true;
                  }
                   updateHTable(source,origin,hops);
                  EV << "FA id: " <<  treeid << " already checked here by other packet, but by longest or equal path (in hops), so continue with Proactive FA packet"  << endl;
               } else {
                     return true; }
            } else {  // no need to check hops count
                return true;
                                }
                    }
       }
    return false;
}

void AntNet::dataReport(cMessage *msg)
{ // specific used for Wmn-Base network, data packets debug
    std::ofstream myfile("data-report.txt", myfile.app );
        Packet *pk = check_and_cast<Packet *>(msg);
        if (myfile.is_open())
          {

            myfile << "Node: " << getParentModule()->getName() << ", packet:" << pk->getName() << ",hops:" << pk->getHopCount() << ", max hops:" << maxHops <<  ", simulation time:" <<  simTime().dbl() << ", error packets:" <<  dropCounter << " avg travel time: " << avgTravel <<", total packets:" << totalCounter <<  "\n";
            if (pk->getDestAddr()==myAddress) { // successful packet arrived
                myfile << "Transient nodes: ";
                for (int i=0; i< pk->getHopCount();i++)
                        myfile << pk->getTransientNodes(i) << ",";
                myfile << " ; travel time: " << pk->getTravelTime() << endl;
                if (pk->getHopCount() == 4) {
                    if (pk->getTransientNodes(2) == 3) myfile << "; Best solution selected!!"<< endl; // cross ap1,ap2
                    else myfile << "; Good solution taken!" << endl; // good solution in 4hops
                }
            } else myfile << "Packet error" << "\n" << endl;
            myfile.close();
          }
          else { EV << "Unable to open file" << endl; }
}

void AntNet::forwardDataPacketByRPTable(cMessage * msg) // AntWMNet data forwarding process
{ // Pre: Data packet parameter, Positive pheromones values confirmed for destination packet
    Packet *pk = check_and_cast<Packet *>(msg);
    int dest = pk->getDestAddr();

    RoutingTable aux;
    int choices = positivePheromonChoices(aux,dest,pk);
    pk->setTransientNodes(pk->getHopCount() ,myAddress);
    pk->setHopCount(pk->getHopCount()+1);
    int outGateIndex;
    double win = 0;
    int estimatedHops = 0;
    //showRoutingInfo(dest);
    if (choices == 1) { // only one option
       EV << "forwarding Data packet " << pk->getName() << " on gate index " << aux[0] << " with prob: " << ptable[dest][aux[0]] << " estimated hops: "<< htable[dest][aux[0]] << endl;
       emit(outputIfSignal, aux[0]);
       send(pk, "out", aux[0]);
       return;
    } else {
        //stochastic , data packets only care about positive pheromones values
        if (stochastic->boolValue()) {
           double totalNbPheromon = 0.0;
           //int estimatedHops=0;
           double prob[4];

           // vector for routing probabilities
           for (int i=0; i < 4; i++)
               prob[i] = 0;
           //EV << "Probability for data routing Pheromone values (1st step) : " << prob[0] <<"," << prob[1] << ","<< prob[2] << ","<< prob[3] << " Total neighbours probabilities: " << totalNbPheromon  <<endl;
           for (int i=0; i < choices; i++) {
               prob[i] = pow(ptable[dest][aux[i]],dataCoef->doubleValue());
               totalNbPheromon = totalNbPheromon + ptable[dest][aux[i]];
                                    }
           //EV << "Probability for data routing Pheromone values (2nd step) : " << prob[0] <<"," << prob[1] << ","<< prob[2] << ","<< prob[3] << " Total neighbours probabilities: " << totalNbPheromon  <<endl;
           totalNbPheromon = pow(totalNbPheromon, dataCoef->doubleValue());
           for (int i=0; i < choices; i++)
               if (prob[i] != 0.0)
                   prob[i] = prob[i] / totalNbPheromon;
           double d= uniform(0,prob[0] + prob[1] + prob[2] + prob[3],0);
           //EV << "Choices : " << choices  << " Choices in vector(possible gates) : " << aux[0] << " " << aux[1] << " "<<aux[2] << " "<< aux[3] << endl;
           //EV << "Double random generated : " << d << " Pheromones values : " << ptable[dest][aux[0]] <<"," << ptable[dest][aux[1]] << ","<< ptable[dest][aux[2]] << ","<<ptable[dest][aux[3]] << endl;
           //EV << "Probability for data routing Pheromones values : " << prob[0] <<"," << prob[1] << ","<< prob[2] << ","<< prob[3] << " Total neighbours probabilities: " << totalNbPheromon << " total prob: "<< prob[0] + prob[1] + prob[2] + prob[3] <<endl;
           if (d <= prob[0]) {
               outGateIndex = aux[0];
               win = ptable[dest][aux[0]];
               estimatedHops=htable[dest][aux[0]];
           } else {d = d - prob[0]; //EV << "Skip branch A" << endl;
                 if (d <= prob[1]) {
                    outGateIndex = aux[1];
                    win = ptable[dest][aux[1]];
                    //                branchB= branchB + 1;EV << "Enter branch B" << endl;
                    estimatedHops=htable[dest][aux[1]];
                    } else { d = d - prob[1];//EV << "Skip branch B" << endl;
                       if(d <= prob[2]) {
                         outGateIndex = aux[2];
                         win = ptable[dest][aux[2]];
                         //              branchC++;
                         estimatedHops=htable[dest][aux[2]];
                    } else if (ptable[dest][aux[3]] > 0) { // if positive pheromon value, take last chance
                            outGateIndex = aux[3];
                            win = ptable[dest][aux[3]];
                            //            branchD++;
                           estimatedHops=htable[dest][aux[3]];
                        }   }  }
                       // EV << "Branch 0: " << branchA << " branch 1: " << branchB << " branch 2: "<< branchC << " branch 3: " <<  branchD << endl;
                       // EV << "Selected gate: "<< outGateIndex << " with pheromone value:"<< win << " and estimated hops:"<< estimatedHops << endl;
                    } else {
                    choices--;
                    int b = intuniform(0,choices); // in case of draw, select randomly the out gate, don't care about pheromones value
                    //EV << "Random chosen : " << b  << " Aux : " << aux[0] << aux[1] << aux[2] << aux[3] << endl;
                    outGateIndex = aux[b];
                    win = ptable[dest][aux[b]];
                    }
    }
    EV << "forwarding Data packet " << pk->getName() << " on gate index " << outGateIndex << " with prob: " << win << " estimated hops: "<< estimatedHops << endl;
    emit(outputIfSignal, outGateIndex);
    send(pk, "out", outGateIndex);
}

double AntNet::getLastTime(int dest, int gate, int hops) // get last cost time by dest, gate and hops
{
    double lastCost = 0;
    lastCost = ttable [dest] [gate] [hops]; // easy way...works? great
//    RTtable::iterator it = ttable.find(dest);
//    if (it == ttable.end()) return 0; // no value (it==ptable.end())
//    AuxTable::iterator it2 = (*it).second.find(gate);
//    if (it2 == (*it).second.end()) return 0; // no value
//    //ProbTable aux = (*it2).second;
//    ProbTable::iterator it3 = (*it2).second.find(hops);
//    if (it3 == (*it2).second.end()) return 0; // no value
//    lastCost = (*it2).second.at(hops);
    return lastCost;
}

void AntNet::goodPhDiffusion(int dest, int bestHops, int origin)
{ // spread routing info about new best estimation hops distance to dest , to neighbours (but no origin) by hello packets
    bestHops++; // after going to neighbour node, best hops estimation will be increased by 1 (so 1 means 0 hops (unknown way))
    RoutingTable aux;
    int choices; // = travelChoices(aux, NULL);
    //EV << "Travel choices: " << choices << endl;
    //aux.clear();
    choices = neighbourGates(aux,origin); // easy use here neighbourGates No Origin selection
    //EV << "Neighbours connected: " << choices << endl;
    if (choices == 0) {
            EV << "Abort pheromone diffusion, actual node " << myAddress << " is connected only to previous checked node"<< endl;
            return;
        }

    char pkname[40];
    sprintf(pkname,"PhDiffusion-%d-about-%d-bestHops-%d#%d", myAddress, dest,bestHops, phDiffCounter++); // not consider
    Packet *pk = new Packet(pkname); // no necessary an extra good Diffusion  counter
    pk->setSrcAddr(myAddress);
    pk->setKind(1); // 1: hello msg
    pk->setDestAddr(dest);
    pk->setHopCount(bestHops); // store best hops estimation to dest
    pk->setTransientNodesArraySize(2); // initialise to safe value, specify is a pheromone diffusion packet
    if (flooding->boolValue()) { // we know is not possible to route to destination (origin gate)
       EV << "Forwarding Pheromone diffusion packet " << pkname << " on gate "<< aux[0] << endl;
       //if (headerCost->longValue() == 2)
       controlHops2++;
       //int j=1;
       //if (aux[0] != origin)
           send(pk,"out",aux[0]);
       //else {
        //   send(pk,"out",aux[1]);
          // j=2;
       //}
       for (int i=1; i < choices; i++) {
           //if (j == 2) {;}
           //else {
           //j++;
           EV << "Flooding...Pheromone diffusion packet " << pkname << " on gate "<< aux[i] << endl;
           //if (headerCost->longValue() == 2)
            controlHops2++;
            send(pk->dup(),"out",aux[i]);
           //}
              }
    } else {
         int r = intrand(choices);
         EV <<"Choosing random choice " << r << " from " << choices << " choices"<<endl;
         EV << "Forwarding Pheromone diffusion " << pkname << " on gate "<< aux[r] << endl;
         //if (headerCost->longValue() == 2)
         controlHops2++;
         send(pk,"out",aux[r]);
    }
    aux.clear();
}

void AntNet::initReactiveFA(int dest) // Pre: we know no positive pheromones values on actual node
{
    char pkname[40];
    sprintf(pkname,"FA-%d-to-%d-#%d", myAddress, dest, faCounter++);
    Packet *pk = new Packet(pkname);
    pk->setKind(2);  // Forward Ant packet
    pk->setDisplayString("i=msg/ant3_s,green"); // FA displays as green ant
    //cPar *packetLengthBytes = &par("packetLength");
    //pk->setByteLength(packetLengthBytes->longValue());
    pk->setSrcAddr(myAddress);
    pk->setDestAddr(dest);
    pk->setTransientNodesArraySize(maxArray->longValue()); // initialise to safe value
    EV << "generating Panic! Forward Ant " << pkname << endl;
    RoutingTable aux;
    int choices= bestHopsEstimationChoices(aux, dest, NULL); // try process with no message
    EV << "Best hops estimation choices: " << choices << endl;
    if (choices ==0) choices = travelChoices(aux, NULL);
    EV << "Travel choices: " << choices << endl;
    if (choices == 0) {
        EV << "Abort reactive Forward ant process, destination unconnected from source node... deleting packet." << endl;
        delete pk; return;
    }
    else
    {
        // set hops to 1 and initialise transient nodes with actual node
        pk->setHopCount(1);
        pk->setTransientNodes(0,myAddress);
        if (flooding->boolValue()) {
            EV << "Forwarding Forward Ant " << pkname << " on gate "<< aux[0] << endl;
            controlHops++;
            send(pk,"out",aux[0]);
            for (int i=1; i < choices; i++) {
                EV << "Flooding...Forwarding Forward Ant " << pkname << " on gate "<< aux[i] << endl;
                controlHops++;
                send(pk->dup(),"out",aux[i]);
            }
        } else {
            int r = intrand(choices);
            EV <<"Choosing random choice " << r << " from " << choices << " choices"<<endl;
            EV << "Forwarding Forward Ant " << pkname << " on gate "<< aux[r] << endl;
            controlHops++;
            send(pk,"out",aux[r]);
        }
    }
    aux.clear();
}

bool AntNet::isBetterHopsEstimation(int dest, int gate, int hops)
{ // return if it is better hops estimation than actual value
    if ((htable[dest][gate] == 0) || (htable[dest][gate] > hops)) return true;
    return false;
}

bool AntNet::isConnected(int g)
{ // check if gate is connected
    int maxGates = getParentModule()->gateSize("port");
    //EV << "Checking isConnected in gate: " << g << " max. gates: "<< maxGates << endl;
    if ((g < maxGates) && (getParentModule()->gate("port$i",g)->isConnected())) return true;
    else return false;
}

bool AntNet::isDestAddress(int addr)
{ // check if it is a destination address

    for (unsigned int n=0; n< destAddresses.size(); n++ ) {
        if (addr == destAddresses[n]) return true;
    }
    return false;
}

bool AntNet::isEqualPath(int dest, int hops) // Pre: equal as shortest when is greater than zero
{ // with hops (or hops + time)  metrics
    int minHops= maxArray->longValue();
    int j = getParentModule()->gateSize("port");
      for (int k=0; k<j; k++) {
          if ((htable[dest][k] > 0) && (htable[dest][k] <= minHops)) minHops= htable[dest][k];
      }
      //EV << "Minimum hops to " << dest << " are " << minHops << " ; new path uses " << hops << " hops." << endl;
      if (minHops == hops) // can't use >= so it could create two path with same cost (or not bad idea... )
          return true;
      else
          return false;
}

bool AntNet::isInPath(cMessage *msg, int g)
{ // inspect if node (int) was crossed before by packet
    if (msg == NULL) return false;
    bool enc =false;
    Packet *pk = check_and_cast<Packet *>(msg);
    for (int i=0;i< pk->getHopCount();i++) {
                                if (pk->getTransientNodes(i) == g) {
                                    enc = true;  // loop detected
                                    EV << "Curl skipped, detected on node :  " << g <<endl;
                                    return enc;
                                }
                    }
    return enc;
}

bool AntNet::isShortestPath(int dest, int hops) // with hops metrics save packet with shortest path
// (shorter or same than previous)
{
    int minHops= maxArray->longValue();
    int j = getParentModule()->gateSize("port");
    for (int k=0; k<j; k++) {
        if ((htable[dest][k] > 0) && (htable[dest][k] <= minHops)) minHops= htable[dest][k];
    }
    //EV << "Minimum hops to " << dest << " are " << minHops << " ; new path uses " << hops << " hops." << endl;
    if (minHops >= hops) // can't use >= so it could create two path with same cost (or not bad idea... )
        return true;
    else
        return false;
}

bool AntNet::isSrcAddress(int addr)
{ // check if it is a source address

    for (unsigned int n=0; n< srcAddresses.size(); n++ ) {
        if (addr == srcAddresses[n]) return true;
    }
    return false;
}

void AntNet::linkError(int dest, int gate) // link error on gate to dest, after update local routing info
{
    // good to think about panic process if node is source or destination node and no available needed paths(panic)
    int bestHops = bestHopsEstimation(dest); // actual estimation to destination injured by new link error
    //EV << "Now best hops estimation to destination " << dest << " is around " << bestHops << " hops" << endl;
    //panic if actual node is srcAddr and dest is important destAddr or vice versa
    //(actual is important destAddr and dest is srcAddr) and no path (with positive pheromon) between us
    if (bestHops == 0) { // no path to Destination
        if (((isSrcAddress(myAddress) && (isDestAddress(dest))) ||
            ((isDestAddress(myAddress)) && (isSrcAddress(dest)))) && (positivePheromonChoices(dest) == 0)) {
            initReactiveFA(dest);
            return;
        }
    }
    // pheromone diffusion to neighbours by helllo packets with new best estimation across actual node
    // (spread until good value encounter)
    pheromoneDiffusion(dest,bestHops);
}

bool AntNet::locateNeighbor (int dest, int & outgate)
{ // check if destination is on Neighbour table, and collect the gateId to the destination on variable outgate
    unsigned int maxChoices = getParentModule()->gateSize("port");
    for (unsigned int i=0; i < maxChoices; i++) { // use port out size, previous error use ntable size
         if ((dest == ntable[i]) && (isConnected(i))) { //(ptable[dest][i] > 0))
             //EV << "Checking if destination address " << dest << " in table with size " << ntable.size() << " is neighbour "<< ntable[i] << " on gate : " << i << " prob: " << ptable[dest][i] << endl;
             //outgate = gate(i)->getIndex();
             outgate = i;
             return true;
          }
          //EV << "Checking if destination address " << dest << " in table with size " << ntable.size() << " is neighbour "<< ntable[i] << " on gate : " << i <<  " prob: " << ptable[dest][i] << endl;
    }
    return false;
}

void AntNet::moveNode(int pos)
{ // simulate movement on node X cell phone by different phases
    //int j = getParentModule()->gateSize("port");
    double dr= 8000000;double ruido =2;
    double jit=uniform(0,7) ;double retraso= (2+jit)/1000; int ypos=58;
    switch (pos) {
    case 1: break;
    case 2: move3 = new cMessage("Move 3"); //29s
            scheduleAt(44,move3);
            getParentModule()->setDisplayString("i=device/cellphone;p=469,160;r=60,,#707070"); // draw node X
            //getParentModule()->setDisplayString("i=device/wifilaptop;p=266,58;r=60,,#707070");
            // draw Kli below (further from Kle)
            break;
    case 3: move4 = new cMessage("Move 4");  // 44s
            scheduleAt(49,move4);jit = uniform(0,9);retraso=(3+jit)/1000;dr=2000000;ypos=38 ;ruido=3.7;
            changeChannelParamsByGate(3,dr,retraso,jit,ruido); // now AP2 is visible (neighbour of node X, around 49m)
            jit= uniform(0,7);
            changeChannelParamsByGate(0,12000000,(jit+2)/1000,jit,3); // node C is further from nodo X (25m)
            getParentModule()->setDisplayString("i=device/cellphone;p=469,184;r=60,,#707070"); // draw node X move South
//            if ( getParentModule()->gate("port$i",3)->isConnected()) { // change channel 2 conditions (to AP2) via port 3 to zone around 49metres
//                getParentModule()->gate("port$o",3)->getTransmissionChannel()->par("jitter").setDoubleValue(jit);
//                getParentModule()->gate("port$i",3)->getIncomingTransmissionChannel()->par("datarate").setDoubleValue(dr);
//                getParentModule()->gate("port$o",3)->getTransmissionChannel()->par("delay").setDoubleValue(retraso);
//                getParentModule()->gate("port$i",3)->getIncomingTransmissionChannel()->par("noise").setDoubleValue(ruido);
//            }
            //getParentModule()->setDisplayString("i=device/wifilaptop;p=266,38;r=60,,#707070"); // draw Kli below (further from Kle)
            break;
    case 4: jit=uniform(0,8);retraso=(3+jit)/1000;dr=6000000;ypos=28;ruido=3.4; // on 49 s simulation
            changeChannelParamsByGate(3,dr,retraso,jit,ruido);   // node X is closer to AP2 (44m)
            jit=uniform(0,7);
            move5 = new cMessage("Move 5");
            scheduleAt(54,move5);// send next movement5 message at 54s
            changeChannelParamsByGate(0,8000000,(jit+2)/1000,jit,3.3); // node C is further from node X (31m)
            getParentModule()->setDisplayString("i=device/cellphone;p=469,199;r=60,,#707070"); // draw node X move South
            if (mySort->longValue() ==2 ) // AntWMNet, detects neighbour AP2 for node X
                updateNb(3,3,1); // gate 3, node AP2 (address 3) ,hops1
            //showRoutingInfo();
            break;
    case 5: // 39m from AP2 to node X (port 3) 54 s simulation
        dr=8000000;jit=uniform(0,7); retraso=(2+jit)/1000;ruido=3.3;
        changeChannelParamsByGate(3,dr,retraso,jit,ruido);
        getParentModule()->setDisplayString("i=device/cellphone;p=469,206;r=60,,#707070");
        move6 = new cMessage("Move 6");
        scheduleAt(59,move6);// send next movement6 message at 59s
        break;
    case 6: // 59 s simulation, node C (port 0) 40m from node X
        dr=6000000;jit=uniform(0,8); retraso=(3+jit)/1000;ruido=3.6;
        changeChannelParamsByGate(0,dr,retraso,jit,ruido);
        getParentModule()->setDisplayString("i=device/cellphone;p=469,214;r=60,,#707070");
        move7 = new cMessage("Move 7");
        scheduleAt(64,move7);// send next movement7 message at 64s
        break;
    case 7: // 64s simulation, node C 45m and AP2 30m from node X
        dr=2000000;jit=uniform(0,9); retraso=(3+jit)/1000;ruido=3.8;
        changeChannelParamsByGate(0,dr,retraso,jit,ruido); // node C from 45m
        jit=uniform(0,7);
        changeChannelParamsByGate(3,12000000,(2+jit)/1000,jit,3); // AP2 from 30m
        getParentModule()->setDisplayString("i=device/cellphone;p=469,226;r=60,,#707070");
        move8 = new cMessage("Move 8");
        scheduleAt(69,move8);// send next movement8 message at 69s
        break;
    case 8: // 69s simulation, node C further than 50m (no visible) , AP2 24m from node X
        dr=1000000;jit=uniform(0,9); retraso=(3+jit)/1000;ruido=4.2;
        changeChannelParamsByGate(0,dr,retraso,jit,ruido); // node C from 50m
        // update  neighbour table, gate 0, to 0(unknown), 0hops
        updateNb(0,0,0); // active link error process to node C
        //update routing probability table
        if (mySort->longValue() == 2) { //AntWMNet this better in handle message process after link error notification
            updateRoutingInfoOnGateError(0); // proceed with reset tables on position across selected gate
            //updateHTable(6,0,0); // update hops table to node C (6) on gate 0 to 0
            //updateRPTable(6,0,0,0); //update routing probability table to node C (6) on gate 0 to 0
            linkError(6,0);// Link error process on node X, gate 0, destination 6 (node C)
        }
        //EV << "Update neighbour table, now gate 0 unconnected (noisy channel) cause node C is further than 49m " << endl;
        //update neighbour table
        jit=uniform(0,7);
        changeChannelParamsByGate(3,18000000,(2+jit)/1000,jit,2.7); // AP2 from 30m
        getParentModule()->setDisplayString("i=device/cellphone;p=469,238;r=60,,#707070");
        move9 = new cMessage("Move 9");
        scheduleAt(80,move9);// send next movement9 message at 80s
        //showRoutingInfo();
        break;
    case 9: // 80s now node F (F port 5 <-> 4 X) is visible from node X
        dr=2000000;jit=uniform(0,9); retraso=(3+jit)/1000;ruido=3.9;
        changeChannelParamsByGate(4,dr,retraso,jit,ruido); // node F from 49m
        getParentModule()->setDisplayString("i=device/cellphone;p=469,274;r=60,,#707070");
        move10 = new cMessage("Move 10");
        scheduleAt(85,move10);// send next movement10 message at 85s
        break;
    case 10: // 85s node F from 44m of node X
        dr=6000000;jit=uniform(0,8); retraso=(3+jit)/1000;ruido=3.6;
        changeChannelParamsByGate(4,dr,retraso,jit,ruido);
        getParentModule()->setDisplayString("i=device/cellphone;p=469,286;r=60,,#707070");
        move11 = new cMessage("Move 11");
        scheduleAt(90,move11);// send next movement11 message at 90s
        if (mySort->longValue() ==2 ) // AntWMNet, detects neighbour AP2 for node X
                        updateNb(4,10,1); // node X  in gate 4 ,to node F (address 10) ,hops1
        //showRoutingInfo();
        break;
    case 11: // 90s node F and AP2 31-9m from node X
        dr=8000000;jit=uniform(0,7); retraso=(2+jit)/1000;ruido=3.3;
        changeChannelParamsByGate(3,dr,retraso,jit,ruido);
        changeChannelParamsByGate(4,dr,retraso,jit,ruido);
        getParentModule()->setDisplayString("i=device/cellphone;p=469,298;r=60,,#707070");
        move12 = new cMessage("Move 12");
        scheduleAt(101,move12);// send next movement12 message at 101s
        break;
    case 12: // 101s    node F from 30m, AP2 from 44m
        dr=12000000;jit=uniform(0,7); retraso=(2+jit)/1000;ruido=3;
        changeChannelParamsByGate(4,dr,retraso,jit,ruido);ruido=3.7;jit=uniform(0,8);
        changeChannelParamsByGate(3,6000000,(3+jit)/1000,jit,ruido);
        getParentModule()->setDisplayString("i=device/cellphone;p=469,308;r=60,,#707070");
        move13 = new cMessage("Move 13");
        scheduleAt(106,move13);// send next movement13 message at 106s
        break;
    case 13: // 106s AP2 not visible 50m, node F from 24-20m
        dr=18000000;jit=uniform(0,7); retraso=(2+jit)/1000;ruido=2.4;
        changeChannelParamsByGate(4,dr,retraso,jit,ruido);ruido=4;jit=uniform(0,9);
        changeChannelParamsByGate(3,1000000,(3+jit)/1000,jit,ruido);
        updateNb(3,0,0); // change neighbour table in port 3 (now unconnected) to 0 (unknown), 0 hops
        //EV << "Update neighbour table, now gate 3 unconnected (noisy channel) cause node AP2 is further than 49m " << endl;//update neighbour table
        getParentModule()->setDisplayString("i=device/cellphone;p=469,322;r=60,,#707070");
        if (mySort->longValue() == 2) { //AntWMNet ;this is better in handle message process after link error notification
            updateRoutingInfoOnGateError(3); // proceed with reset tables on position across selected gate
            //updateHTable(3,3,0); // update hops table to node AP2 (3) on gate 3 to 0
            //updateRPTable(3,3,0,0); //update routing probability table to node AP2 (3) on gate 3 to 0
            linkError(3,3);// Link error process on node X, gate 3, destination 3 (node AP2)
          //  showRoutingInfo();
               }
        // no more movements
        break;
    }


//    for (int i=0; i<j; i++) {
//             if ( getParentModule()->gate("port$i",i)->isConnected()) {
//                  getParentModule()->gate("port$i",i)->getIncomingTransmissionChannel()->par("datarate").setDoubleValue(dr);
//                  getParentModule()->gate("port$o",i)->getTransmissionChannel()->par("datarate").setDoubleValue(dr); // change also output gate, so port is inout
//                  getParentModule()->gate("port$i",i)->getIncomingTransmissionChannel()->par("jitter").setDoubleValue(jit);
//                  getParentModule()->gate("port$i",i)->getIncomingTransmissionChannel()->par("delay").setDoubleValue(retraso);
//                  getParentModule()->gate("port$i",i)->getIncomingTransmissionChannel()->par("noise").setDoubleValue(ruido);
//                  getParentModule()->gate("port$o",i)->getTransmissionChannel()->par("jitter").setDoubleValue(jit);
//                  getParentModule()->gate("port$o",i)->getTransmissionChannel()->par("delay").setDoubleValue(retraso);
//                  getParentModule()->gate("port$o",i)->getTransmissionChannel()->par("noise").setDoubleValue(ruido);
//                                                                          }
//                        }
}

int AntNet::neighbourGates (RoutingTable & gates)  // check all the gates choices (from neighbour table)
{
    int num=0;
    int maxNeighbours = getParentModule()->gateSize("port");
    for (int i=0; i < maxNeighbours; i++) { // use port out size, previous error use ntable size
        if (isConnected(i) && (ntable[i] > 0)) { // check if neighbour table contains valid value
                 //EV << "Adding choice gate " << i << " in neighbour table with size " << ntable.size() << " is neighbour "<< ntable[i]  << endl;
                 gates[num]=i;
                 num++;
              }
        //EV << "Testing choice gate " << i << " in neighbour table with size? " << ntable.size() << " is neighbour "<< ntable[i]  << endl;
        }
    return num;
}

int AntNet::neighbourGates (RoutingTable & gates, int origin)
// check all the gates choices (from neighbour table) except origin
{
    int num=0;
    int maxNeighbours = getParentModule()->gateSize("port");
    for (int i=0; i < maxNeighbours; i++) { // use port out size, previous error use ntable size
        if (isConnected(i) && (ntable[i] > 0) && (i != origin)) {
            // check if neighbour table contains valid value and is not the origin gate
                 //EV << "Adding choice gate " << i << " in neighbour table with size " << ntable.size() << " is neighbour "<< ntable[i]  << endl;
                 gates[num]=i;
                 num++;
              }
        //EV << "Testing choice gate " << i << " in neighbour table with size " << ntable.size() << " is neighbour "<< ntable[i]  << endl;
        }
    return num;
}

int AntNet::nextGateProBA(cMessage * msg) // return next gate to forward Proactive BA
{ // -1 means error, no route in actual node
    int outGate;
    Packet *pk = check_and_cast<Packet *>(msg);
    int dest = pk->getDestAddr();
    int origin = gate(pk->getArrivalGateId())->getIndex();
    RoutingTable aux;
    int choices = positivePheromonChoices(aux,dest,pk);
    if (choices == 1)
        outGate = aux[0];
    else {
        if (choices == 0) { // error
            return -1;
        } // multiple choices, select random one
        bool sent = false;
        while (!sent){
              int r = intrand(choices);
              //EV << "Random selection: "<< r << " from choices: " << choices << endl;
              if (aux[r] != origin) {
                  outGate = aux[r];
                  sent = true;
              }
        }
    }
    aux.clear();
    return outGate;
}

int AntNet::nextTravelChoices(RoutingTable & choices, int origin, cMessage *msg) // check travel choices but no origin gate
{
    int num=0;
    unsigned int temp = origin;
    unsigned int maxChoices = getParentModule()->gateSize("port");
        for (unsigned int i=0; i < maxChoices; i++) { // use port out size, previous error use ntable size
            if (isConnected(i) && (temp != i) && (!isInPath(msg,ntable[i]))) { // check also if loops
                     //EV << "Adding choice gate " << i << " in neighbour table with size " << ntable.size() << " is neighbour "<< ntable[i]  << endl;
                     choices[num]=i;
                     num++;
                  }

            }
        return num;
}

bool AntNet::noisyChannel(cPacket *pk)
{  // detect noisy channels
    bool b = false;

   // EV << "Checking if it is a noisy Channel last message path, sender gateId: " << pk->getSenderGateId() << " gate index: "<< pk->getArrivalGate()->getIndex() << endl;
    int arrivalGateId = pk->getArrivalGate()->getIndex();
    if (getParentModule()->gate("port$i",arrivalGateId)->getIncomingTransmissionChannel()->isTransmissionChannel()) {
       // EV << "Transmission channel detected, channel name: " << getParentModule()->gate("port$i",arrivalGateId)->getIncomingTransmissionChannel()->getNedTypeName() << endl;
    }
    double noise = getParentModule()->gate("port$i",arrivalGateId)->getIncomingTransmissionChannel()->par("noise").doubleValue();
    if (noise >= 4) {
        b = true;EV <<"Wrong message received, channel signal noise: " << noise << " is too high" << endl;
    }
    return b;
}

double AntNet::normCost(double d) // normalise to non zero neither negative value
{
    if (d <= 0) d = 0.001;
    return d;
}

void AntNet::pheromoneDiffusion(int dest, int bestHops)
{ // spread routing info about new best estimation hops distance towards dest , to neighbours by hello packets
    if (bestHops != 0) // keep no path defined by 0
        bestHops++; // after going to neighbour node, best hops estimation will be increased by 1
    RoutingTable aux;
    //int choices = travelChoices(aux, NULL);
    //EV << "Travel choices: " << choices << endl;
    aux.clear();
    int choices = neighbourGates(aux);
    //EV << "Neighbours connected: " << choices << endl;
    if (choices == 0) {
            EV << "Abort pheromone diffusion, actual node " << myAddress << " is isolated, unconnected right now"<< endl;
            return;
        }
    char pkname[40];
    sprintf(pkname,"PhDiffusion-%d-about-%d-bestHops-%d#%d", myAddress, dest,bestHops, phDiffCounter++);
    Packet *pk = new Packet(pkname);
    pk->setSrcAddr(myAddress);
    pk->setKind(1); // 1: hello msg
    pk->setDestAddr(dest);
    pk->setHopCount(bestHops); // store best hops estimation to dest, 0 means no way
    pk->setTransientNodesArraySize(2); // initialise to safe value, specify is a pheromone diffusion packet
    if (flooding->boolValue()) { // 1st step no possible to reach dest address
       EV << "Forwarding Pheromone diffusion packet " << pkname << " on gate "<< aux[0] << endl;
       //if (headerCost->longValue() == 2)
       controlHops2++;
       send(pk,"out",aux[0]);
       for (int i=1; i < choices; i++) {
            EV << "Flooding...Pheromone diffusion packet " << pkname << " on gate "<< aux[i] << endl;
            //if (headerCost->longValue() == 2)
            controlHops2++;
            send(pk->dup(),"out",aux[i]);
              }
    } else {
         int r = intrand(choices);
         EV <<"Choosing random choice " << r << " from " << choices << " choices"<<endl;
         EV << "Forwarding Pheromone diffusion " << pkname << " on gate "<< aux[r] << endl;
         //if (headerCost->longValue() == 2)
         controlHops2++;
         send(pk,"out",aux[r]);
    }
    aux.clear();
}


void AntNet::pheromoneDiffusion(int dest, int bestHops, int origin)
{  // spread routing info about new best estimation hops distance to dest , to neighbours (but no origin) by hello packets
    if (bestHops != 0)
        bestHops++; // after going to neighbour node, best hops estimation will be increased by 1
    RoutingTable aux; // (so 0 means 0 hops (unknown way))
    //int choices = travelChoices(aux, NULL); // not necesary, just testing
    //EV << "Travel choices: " << choices << endl;
    aux.clear();
    int choices = neighbourGates(aux,origin); // easy use here neighbourGates No Origin selection
    EV << "Neighbours connected: " << choices << endl;
    if (choices == 0) {
       EV << "Abort pheromone diffusion, actual node " << myAddress << " is connected only to previous checked node"<< endl;
       return;
        }

    char pkname[40];
    sprintf(pkname,"PhDiffusion-%d-about-%d-bestHops-%d#%d", myAddress, dest,bestHops, phDiffCounter++);
    Packet *pk = new Packet(pkname);
    pk->setSrcAddr(myAddress);
    pk->setKind(1); // 1: hello msg
    pk->setDestAddr(dest);
    pk->setHopCount(bestHops); // store best hops estimation to dest
    pk->setTransientNodesArraySize(2); // initialise to safe value, specify is a pheromone diffusion packet
    if (flooding->boolValue()) {
       if (ntable[aux[0]] != dest) { // skip forwarding to dest
           EV << "Forwarding Pheromone diffusion packet " << pkname << " on gate "<< aux[0] << endl;
           //if (headerCost->longValue() == 2)
           controlHops2++;
       //int j=1;
       //if (aux[0] != origin)

           send(pk,"out",aux[0]);
       } else {
           EV << "Skipping diffusion by gate : " << aux[0] << " so is going to destination node related to diffusion routing info"<< endl;
       }
       //else {
        //   send(pk,"out",aux[1]);
          // j=2;
       //}
       for (int i=1; i < choices; i++) {
           //if (j == 2) {;}
           //else {
           //j++;
           if (ntable[aux[i]] != dest) { // skip forwarding to dest
               EV << "Flooding...Pheromone diffusion packet " << pkname << " on gate "<< aux[i] << endl;
               //if (headerCost->longValue() == 2)
               controlHops2++;
               send(pk->dup(),"out",aux[i]);
           } else {
               EV << "Skipping diffusion by gate : " << aux[i] << " so is going to destination node related to diffusion routing info"<< endl;
           }
           //}
              }
    } else { // also good to check not going to dest
         int r = intrand(choices);
         EV <<"Choosing random choice " << r << " from " << choices << " choices"<<endl;
         EV << "Forwarding Pheromone diffusion " << pkname << " on gate "<< aux[r] << endl;
         //if (headerCost->longValue() == 2)
         controlHops2++;
         send(pk,"out",aux[r]);
    }
    aux.clear();
}

void AntNet::pheromoneDiffusion(int dest, int bestHops, int origin,cPacket * pk) // spread routing info about new best estimation hops distance to dest , to neighbours (but no origin) by hello packets
{
    if (bestHops != 0)
        bestHops++; // after going to neighbour node, best hops estimation will be increased by 1
    // (so 0 means 0 hops (unknown way))
    RoutingTable aux;
    //int choices = travelChoices(aux, NULL);
    //EV << "Travel choices: " << choices << endl;
    aux.clear();
    int choices = neighbourGates(aux,origin); // easy use here neighbourGates No Origin selection
    EV << "Neighbours connected: " << choices << endl;
    if (choices == 0) {
            EV << "Abort pheromone diffusion, actual node " << myAddress << " is connected only to previous checked node"<< endl;
            delete pk;
            return;
        }

    char pkname[40];
    sprintf(pkname,"PhDiffusion-%d-about-%d-bestHops-%d#%d", myAddress, dest,bestHops, phDiffCounter++);
    Packet *pk2 = check_and_cast<Packet *>(pk);
    pk2->setSrcAddr(myAddress);
    pk2->setKind(1); // 1: hello msg
    pk2->setDestAddr(dest);
    pk2->setHopCount(bestHops); // store best hops estimation to dest, 0 means no known way
    pk2->setTransientNodesArraySize(2); // initialise to safe value, specify is a pheromone diffusion packet
    if (flooding->boolValue()) {
        if (ntable[aux[0]] != dest) { // skip forwarding to dest
            EV << "Forwarding Pheromone diffusion packet " << pkname << " on gate "<< aux[0] << endl;
            //if (headerCost->longValue() == 2)
            controlHops2++;
       //int j=1;
       //if (aux[0] != origin)
            send(pk2,"out",aux[0]);
        } else {
            EV << "Skipping diffusion by gate : " << aux[0] << " so is going to destination node related to diffusion routing info"<< endl;
        }
       //else {
        //   send(pk,"out",aux[1]);
          // j=2;
       //}
       for (int i=1; i < choices; i++) {
           //if (j == 2) {;}
           //else {
           //j++;
           if (ntable[aux[i]] != dest) { // skip forwarding to dest
               EV << "Flooding...Pheromone diffusion packet " << pkname << " on gate "<< aux[i] << endl;
               //if (headerCost->longValue() == 2)
               controlHops2++;
               send(pk2->dup(),"out",aux[i]);
           }else {
               EV << "Skipping diffusion by gate : " << aux[i] << " so is going to destination node related to diffusion routing info"<< endl;
               }
           //}
              }
    } else {
         int r = intrand(choices);
         EV <<"Choosing random choice " << r << " from " << choices << " choices"<<endl;
         EV << "Forwarding Pheromone diffusion " << pkname << " on gate "<< aux[r] << endl;
         //if (headerCost->longValue() == 2)
         controlHops2++;
         send(pk2,"out",aux[r]);
    }
    aux.clear();
}

int AntNet::positiveGoodPhChoices(int dest,unsigned int origin,cMessage * msg) // number of good positive pheromone choices
{                             // skipping origin and gates to previous visited nodes
    int num=0;
    unsigned int maxChoices = getParentModule()->gateSize("port");
    for (unsigned int i=0; i < maxChoices; i++) { // use port out size, previous error use ntable size
        if (isConnected(i) && (i != origin) && (ptable[dest][i] > 0) && (!(isInPath(msg,ntable[i])))) {
            // check if could be a loop way)
           //EV << "Adding choice gate " << i  <<" in neighbour table with size " << ntable.size() << " is neighbour "<< ntable[i] << " for destination:"<< dest << " with probability:" << ptable[dest][i] << endl;
           //choices[num]=i;
           num++;
                      }

                }
   return num;
}

int AntNet::positivePheromonChoices(int dest)
{ // return number of positive pheromone choices
    int num=0;
    unsigned int maxChoices = getParentModule()->gateSize("port");
        for (unsigned int i=0; i < maxChoices; i++) { // use port out size, previous error use ntable size
            if (isConnected(i) && (ptable[dest][i] > 0))  {
                     //EV << "Adding choice gate " << i  <<" in neighbour table with size " << ntable.size() << " is neighbour "<< ntable[i] << " for destination:"<< dest << " with probability:" << ptable[dest][i] << endl;
                     num++;
                  }

            }
    return num;
}

int AntNet::positivePheromonChoices(RoutingTable & choices, int dest, cMessage * msg)
{  // return number of positive pheromone choices, and store it (gates) in choices array
    int num=0;
    unsigned int maxChoices = getParentModule()->gateSize("port");
        for (unsigned int i=0; i < maxChoices; i++) { // use port out size, previous error use ntable size
            if (isConnected(i) && (ptable[dest][i] > 0) && (!(isInPath(msg,ntable[i])))) { // check if could be a loop way)
                     //EV << "Adding choice gate " << i  <<" in neighbour table with size " << ntable.size() << " is neighbour "<< ntable[i] << " for destination:"<< dest << " with probability:" << ptable[dest][i] << endl;
                     choices[num]=i;
                     num++;
                  }

            }
    return num;
}

void AntNet::routeRequest(int dest)
{ // AODV start route request process to dest
   int outgate = 0;
   if (locateNeighbor(dest,outgate)) { // destination is a neighbour
      rtable[dest] = outgate;
      EV << "Destination: " << dest << " detected on gate " << rtable[dest] << endl;
      return;
   }
   char pkname[40];
   sprintf(pkname,"RREQ-%d-to-%d-#%d", myAddress, dest, rreqCounter++);

   EV << "generating RREQ packet " << pkname << endl;
   Packet *pk = new Packet(pkname);
   pk->setKind(2);  // RREQ packet
   pk->setDisplayString("i=msg/req_s"); // RREQ displays as green? packet
   pk->setByteLength(1024);
   pk->setSrcAddr(myAddress);
   pk->setDestAddr(dest);
   pk->setTransientNodesArraySize(maxArray->longValue());
   pk->setHopCount(1);
   int j = getParentModule()->gateSize("port");
   int localSent=0;
   for (int k=0; k<j; k++) {
        if ( getParentModule()->gate("port$o",k)->isConnected() && (ntable[k] > 0)) {
            // also good idea to check neighbour table (maybe better just checking if noise channel between
           EV << "Connection detected between  nodes in " << getParentModule()->getName() << " gateOut: " << k << endl;
           pk->setTransientNodes(0,myAddress); // store myAddress on step 0
           EV << "Flooding! forwarding packet " << pk->getName() << " on gate index " << k << ", copy: "<< localSent << endl;
           emit(outputIfSignal, k);
           controlHops++; // increase control Hops counter
           if (localSent == 0)  send(pk,"out",k);
           else send(pk->dup(), "out", k);
           localSent++;
                                                                                     }
                                           }
   EV << "Control hops: " << controlHops << endl;
}

bool AntNet::routeRequestRERR(int dest, int origin, int next) // proceed for local repair, true if RREQ were sent
{
    int outgate = 0;
    if (locateNeighbor(dest,outgate)) { // destination is a neighbour
        rtable[dest] = outgate;
        EV << "Destination: " << dest << " detected on gate " << rtable[dest] << endl;
        return false; // should be another case, so can forward  pk now, not use bool , use int
      }
      char pkname[40];
      sprintf(pkname,"RREQ-Local-repair-%d-to-%d-#%d", myAddress, dest, rreqCounter++);
      //special RREQ local repairing packet

      EV << "generating RREQ local repair packet " << pkname << endl;
      Packet *pk = new Packet(pkname);
      pk->setKind(5);  // RREQ local repair packet
      pk->setDisplayString("i=msg/req_s"); // RREQ displays as green ? packet
      pk->setByteLength(1024);
      pk->setSrcAddr(myAddress);
      pk->setDestAddr(dest);
      pk->setTransientNodesArraySize(maxArray->longValue());
      pk->setHopCount(1);
      pk->setTransientNodes(0,myAddress); // store my address on step 0
      int j = getParentModule()->gateSize("port");
      int localSent=0;
      EV << "Port size: " << j << endl;
      //return;
      for (int k=0; k<j; k++) {
           //if ( getParentModule()->gate("port$o",k)->isConnected() && (k =! origin) && (k != next)) {
           if ((isConnected(k)) && (k != origin) && (k != next)) {
              EV << "Connection detected between  nodes in " << getParentModule()->getName() << " gateOut: " << k << endl;
              EV << "iteration: " << k << " local Sent: " << localSent << endl;
              EV << "Flooding! forwarding packet " << pk->getName() << " on gate index " << k << ", copy: "<< localSent << endl;
              //return;
              emit(outputIfSignal, k);
              controlHops++; // increase control Hops counter
              if (localSent == 0)  send(pk,"out",k);
              else send(pk->dup(), "out", k);
              localSent++; //return;
                                                                                      }
                                              }
      //return;
      EV << "Control hops: " << controlHops << endl;
      if (localSent == 0) {

          EV << "No connections available for local repairing, sending RERR to following point" << endl;
          pk->setKind(4); // set packet as RERR
          sprintf(pkname,"RERR-%d-to-%d-#%d", myAddress, ntable[next], rerrCounter++); // rename packet
          rreqCounter--; //undo previous RREQ counting
          pk->setName(pkname);
          pk->setDisplayString("i=msg/resp_s,red"); // RERR displays as red resp ! packet
          emit(outputIfSignal, next);
          controlHops++; // increase control Hops counter
          send(pk,"out",next);
          return false;
      }
      return true;
}

void AntNet::saveIntValue(int v) // save to file int value, debugging issues
{
   std::ofstream myfile("random.txt", myfile.app );
   if (myfile.is_open())
   {
      myfile << v << "\n";
      myfile.close();
   }
   else { EV << "Unable to open file" << endl; }

}

int AntNet::selectOneDestination() // select one destination address from vector DestAddresses
{
   int dest;
   int choices = destAddresses.size();
   //EV << "Choosing one destination address from vector with size: " << choices << " and capacity: "<< destAddresses.capacity() << endl;
   if (choices == 1)
       dest = destAddresses[0];
   else { // chooses randomly
       int r = intrand(choices);
       dest = destAddresses[r];
   }
   return dest;
}

int AntNet::selectOneSource() // select one source address from vector SrcAddresses
{
   int dest;
   int choices = srcAddresses.size();
   //EV << "Choosing one source address from vector with size: " << choices << " and capacity: "<< srcAddresses.capacity() << endl;
   if (choices == 1)
       dest = srcAddresses[0];
   else { // chooses randomly
       int r = intrand(choices);
       dest = srcAddresses[r];
   }
   return dest;
}

void AntNet::sendNbPacket(int k) // send neighbour packet from gate k
{
    char pkname[40];
    sprintf(pkname,"pk-locate-neighbour-node %d-gate %d-#%d", myAddress,k,nbCounter++ );
    EV << "generating neighbour packet " << pkname << endl;
    Packet *pk = new Packet(pkname);
    //cPar *packetLengthBytes = &par("packetLength");
    //pk->setByteLength(packetLengthBytes->longValue());
    pk->setSrcAddr(myAddress);
    pk->setKind(1); // 1: hello msg
    pk->setDestAddr(myAddress);
    pk->setTransientNodesArraySize(1); // initialise to safe value
    EV << "Forwarding " << pkname << " on gate "<< k << endl;
    send(pk,"out",k);
}

void AntNet::sendNbPacketTo(int k, int dest)
{ // send neighbour packet from gate k to dest
    char pkname[40];
    sprintf(pkname,"pk-locate-neighbour-node %d-gate- %d-#%d", myAddress,k,nbCounter++ );
    EV << "generating neighbour packet " << pkname << endl;
    Packet *pk = new Packet(pkname);
    //cPar *packetLengthBytes = &par("packetLength");
    //pk->setByteLength(packetLengthBytes->longValue());
    pk->setSrcAddr(myAddress);
    pk->setKind(1); // 1: hello
    pk->setDestAddr(dest);
    pk->setTransientNodesArraySize(1); // initialise to safe value
    EV << "Forwarding " << pkname << " on gate "<< k << endl;
    send(pk,"out",k);
}

void AntNet::sendProactiveBA()// Pre: launched from source or destination address
{ // Generate single Proactive BA and launching to destination to recalculate path
    int dest;
    if (isDestAddress(myAddress)) { // select source address as destination
        dest = selectOneSource();
    } else { // select destination address as destination of Proactive BA packet
        dest = selectOneDestination();
    }
    int outGate;
    int choices = positivePheromonChoices(dest);
    int bestHops = bestHopsEstimation(dest);
    RoutingTable aux;
    if (choices == 0) {
        if (bestHops == 0) {
            EV << "Neither positive pheromone choices or hops estimation to destination: " << dest << " ; alert! launching reactive FA" << endl;
            aux.clear();
            initReactiveFA(dest);
            return;
        } else { // a possible route with hops estimation upper than zero
             choices = bestHopsEstimationChoices(aux, dest, NULL);
             if (choices == 1) outGate = aux[0];
             else { // select random gate with best hops estimation
                 int r = intrand(choices);
                 //EV << "Random selection: "<< r << " from choices: " << choices << endl;
                 outGate = aux[r];
             }
        }
    } else { // positive pheromone choices
        int choices2 = positivePheromonChoices(aux,dest, NULL);
        if (choices == 1) outGate = aux[0];
        else { // select random gate with positive pheromone
            int r = intrand(choices2);
            //EV << "Random selection: "<< r << " from choices: " << choices2 << endl;
            //saveIntValue(r);// store random values to check if works (and yes, it is works)
            outGate = aux[r];
        }
    }
    aux.clear();
    char pkname[40];
    sprintf(pkname,"ProactiveBA-%d-to-%d-#%d", myAddress, dest, proBaCounter++); // name packet
    EV << "generating Proactive BA packet " << pkname << endl;
    Packet *pk = new Packet(pkname);
    pk->setDisplayString("i=msg/ant3_s,violetred"); // Proactive BA displays as violet-red ant packet
    pk->setByteLength(1024);
    pk->setSrcAddr(myAddress);
    pk->setDestAddr(dest);
    pk->setTransientNodesArraySize(maxArray->longValue());
    pk->setHopCount(1);
    pk->setTransientNodes(0,myAddress);
    pk->setKind(8); // set packet as Proactive BA
    controlHops++;
    emit(outputIfSignal,outGate);
    EV << "Forwarding packet " << pkname << " on gate index " << outGate << " with estimated hops travel: "<< bestHops << endl;
    send(pk,"out",outGate);
}

void AntNet::sendProactiveFA(int dest, double cost,int hops,int origin)
{ // send proactive FA from broken route
   char pkname[40];
   sprintf(pkname,"ProactiveFA-%d-to-%d-#%d", myAddress, dest, proactiveCounter++); // rename packet
   EV << "generating Proactive FA packet " << pkname << endl;
   Packet *pk = new Packet(pkname);
   pk->setDisplayString("i=msg/ant3_s,orange"); // Proactive FA displays as orange ant packet
   pk->setByteLength(1024);
   pk->setSrcAddr(myAddress);
   pk->setDestAddr(dest);
   pk->setTransientNodesArraySize(maxArray->longValue());
   pk->setHopCount(hops);
   pk->setTravelTime(cost);
   pk->setTransientNodes(0,myAddress);
   //pk->setTransientNodes(1,myAddress);
   pk->setKind(4); // set packet as Proactive FA
   int maxChoices = getParentModule()->gateSize("port");
   int cont=0;
   if (bestHopsEstimation(dest) == 0) { // no idea about dest location
       for (int j=0; j < maxChoices; j++) {
            if (j != origin) { // spread over network but origin gate
               emit(outputIfSignal, j);
               if (cont == 0){
                   EV << "Forwarding packet " << pkname << " on gate index " << j << endl;
                   send(pk,"out",j);
                }else{
                    EV << "Flooding! forwarding packet " << pkname << " on gate index " << j << endl;
                    send(pk->dup(),"out",j);
                }
                cont++;
                controlHops++; // increase control Hops counter
            }
        }
   } else { // normal branch, so from source nodes reactive setup were performed
        for (int j=0; j < maxChoices; j++) {
             if (j != origin) { // spread over network with positive pheromone or best hops estimation but origin gate
                 if (( ptable [dest][j] > 0) || (bestHopsEstimation(dest) == htable[dest][j])) {
                     emit(outputIfSignal, j);
                      if (cont == 0){
                          EV << "Forwarding packet " << pkname << " on gate index " << j << endl;
                          send(pk,"out",j);
                      }else{
                          EV << "Flooding! forwarding packet " << pkname << " on gate index " << j << endl;
                          send(pk->dup(),"out",j);
                      }
                      cont++;
                      controlHops++; // increase control Hops counter
                }
            }
       }

    }

}

void AntNet::sendProactiveFAtoSrc(int source, double cost,int hops,int origin)
{ // Pre: detected new path to destination addresss // send Proactive FA to source addresses
    //EV << "Inside sendProactiveFAtoSrc , srcAddresses size:" << srcAddresses.size() << endl;
    for (unsigned int i=0; i< srcAddresses.size();i++) {
        char pkname[40];
        sprintf(pkname,"ProactiveFA-%d-to-%d-#%d", source, srcAddresses[i], proactiveCounter++); // rename packet
        EV << "generating Proactive FA packet " << pkname << endl;
        Packet *pk = new Packet(pkname);
        pk->setDisplayString("i=msg/ant3_s,orange"); // Proactive FA displays as orange ant packet
        pk->setByteLength(1024);
        pk->setSrcAddr(source);
        pk->setDestAddr(srcAddresses[i]);
        pk->setTransientNodesArraySize(maxArray->longValue());
        pk->setHopCount(hops);
        pk->setTravelTime(cost);
        pk->setTransientNodes(0,source);
        pk->setTransientNodes(1,myAddress);
        pk->setKind(4); // set packet as Proactive FA
        int maxChoices = getParentModule()->gateSize("port");
        int cont=0;
        if (bestHopsEstimation(srcAddresses[i]) == 0) { // no idea about source location
            for (int j=0; j < maxChoices; j++) {
                if (j != origin) { // spread over network but origin gate
                    emit(outputIfSignal, j);
                    if (cont == 0){
                        EV << "Forwarding packet " << pkname << " on gate index " << j << endl;
                        send(pk,"out",j);
                    }else{
                        EV << "Flooding! forwarding packet " << pkname << " on gate index " << j << endl;
                        send(pk->dup(),"out",j);
                }
                cont++;
                controlHops++; // increase control Hops counter
                }
            }
        } else { // normal branch, so from source nodes reactive setup were performed
            for (int j=0; j < maxChoices; j++) {
                if (j != origin) { // spread over network with positive pheromone or best hops estimation but origin gate
                    if (( ptable[srcAddresses[i]][j] > 0) ||
                        (bestHopsEstimation(srcAddresses[i]) == htable[srcAddresses[i]][j])) {
                        emit(outputIfSignal, j);
                        if (cont == 0){
                           EV << "Forwarding packet " << pkname << " on gate index " << j << endl;
                           send(pk,"out",j);
                        }else{
                           EV << "Flooding! forwarding packet " << pkname << " on gate index " << j << endl;
                           send(pk->dup(),"out",j);
                        }
                        cont++;
                        controlHops++; // increase control Hops counter
                    }
                }
            }

        }
    }
    srcAddresses.begin();
}

void AntNet::sendRERR(int gate, int dest)
{ // AODV, send RERR packet to dest by gate
   char pkname[40];
   sprintf(pkname,"RERR-%d-to-%d-#%d", myAddress, dest, rerrCounter++); // rename packet
   EV << "generating RERR packet " << pkname << endl;
   Packet *pk = new Packet(pkname);
   pk->setDisplayString("i=msg/resp_s,red"); // RERR displays as red packet
   pk->setByteLength(1024);
   pk->setSrcAddr(myAddress);
   pk->setDestAddr(dest);
   pk->setTransientNodesArraySize(maxArray->longValue());
   pk->setHopCount(1);
   pk->setKind(4); // set packet as RERR
   emit(outputIfSignal, gate);
   controlHops++; // increase control Hops counter
   send(pk,"out",gate);
}

void AntNet::sendRERR(int gate,int source, int dest)
{ // AODV, send RERR packet to dest by gate from source
   char pkname[40];
   sprintf(pkname,"RERR-%d-to-%d-#%d", source, dest, rerrCounter++); // rename packet
   EV << "generating RERR packet " << pkname << endl;
   Packet *pk = new Packet(pkname);
   pk->setDisplayString("i=msg/resp_s,red"); // RERR displays as red packet
   pk->setByteLength(1024);
   pk->setSrcAddr(source);
   pk->setDestAddr(dest);
   pk->setTransientNodesArraySize(maxArray->longValue());
   pk->setHopCount(1);
   pk->setKind(4); // set packet as RERR
   emit(outputIfSignal, gate);
   controlHops++; // increase control Hops counter
   send(pk,"out",gate);
}

void AntNet::showCost(double cost, int dest) // show old cost info and new cost evaluation to dest address
{ // We consider in our sample source/destination addresses only can be one of the possibilities, not both
    EV << "Old cost values, to destination: " << costDest << " to source: "<< costSrc << endl;
    if (isDestAddress(dest))
        EV << "New cost value: "<< cost << " to destination address: "<< dest << endl;
    else
        EV << "New cost value: "<< cost << " to source address: "<< dest << endl;
}

void AntNet::showCostTable() // show cost time table info
{
  // source addresses and neighbours, so try to show all relevant cost time routing info in actual node
  unsigned int maxChoices = getParentModule()->gateSize("port");
  EV << "Checking Time cost table, size: " << ttable.size() << endl;
  RTtable::iterator it;
  AuxTable::iterator it2;
  ProbTable aux2;
  AuxTable aux;
  // check destination addresses
  for (unsigned int n=0; n< destAddresses.size(); n++ ) {
      int dest = destAddresses[n];
      it = ttable.find(dest);
      if (it == ttable.end()) EV << "No time cost table info in selected destination" << endl;
      aux = (*it).second;
      EV << "Checking Time cost table, dest: " << dest << " size: " << aux.size() << endl;
      //ProbTable aux2 = (*it2).second;
      //aux2.
      EV << "Time cost table info for destination: " << dest <<" on node " << myAddress << " id: " << getParentModule()->getName() <<endl;
      EV << "Gate----Pheromone value------Estimated hops----Hops-----Last Time cost" << endl;
      for (unsigned int i=0; i < maxChoices; i++) {
                if ((ptable[dest][i] > 0) ) {// if interesting routing info upper than zero
                    it2 = aux.find(i); // locate gate
                    if (it2 == aux.end()) EV << "No time cost table info in selected gate" << endl;
                    aux2 = (*it2).second;EV << "Checking Time cost table, dest: " << dest << " gate: " << i <<" size: " << aux2.size() << endl;
                    //aux2 = (*it2)
                    EV <<"  " << i << "          " << ptable[dest][i] << "                "<< htable[dest][i] << "           "<< aux2[htable[dest][i]] <<  endl;
                    aux2.clear(); it2 = aux.begin();
                }
        }
      it = ttable.begin();aux.clear();
  }
        EV << "---------------------------------------------"<< endl;
  // check source addresses
  //it = ttable.begin(); it2 = aux.begin();// aux2.clear(); // initialise iterators
  for (unsigned int n=0; n< srcAddresses.size(); n++ ) {
       int src = srcAddresses[n];
       it = ttable.find(src);
       if (it==ttable.end()) EV << "No time cost table info in selected source address" << endl;
       aux = (*it).second;
       EV << "Time cost table info for source: " << src <<" on node " << myAddress << " id: " << getParentModule()->getName() <<endl;
       EV << "Gate----Pheromone value------Estimated hops----Hops-----Last Time cost" << endl;
       for (unsigned int i=0; i < maxChoices; i++) {
                if ((ptable[src][i] > 0) ) { // if interesting routing info upper than zero
                    it2 = aux.find(i); // locate gate
                    if (it2==aux.end()) EV << "No time cost table info in selected gate" << endl;
                    aux2 = (*it2).second;EV << "Checking Time cost table, source: " << src << " gate: " << i <<" size: " << aux2.size() << endl;
                    EV <<"  " << i << "          " << ptable[src][i] << "                "<< htable[src][i] <<  "           "<< aux2[htable[src][i]] <<endl;
                    aux2.clear();it2 = aux.begin();
                }
       }
       it = ttable.begin();aux.clear();
  }
  EV << "---------------------------------------------"<< endl;
  //it = ttable.begin(); it2 = aux.begin();// aux2.clear(); // initialise iterators
  // check neighbours
  RoutingTable aux3;
  unsigned int maxNeighbours = neighbourGates(aux3);
  for (unsigned int n=0; n < maxNeighbours; n++ ) {
      int actualNeighbour = ntable [aux3[n]];
      it = ttable.find(actualNeighbour);
      if (it==ttable.end()) EV << "No time cost table info in selected neighbour address" << endl;
      aux = (*it).second;
      it2 = aux.find(aux3[n]); // locate gate
      if (it2==aux.end()) EV << "No time cost table info in selected gate" << endl;
      aux2 = (*it2).second;EV << "Checking Time cost table, neighbour: " << actualNeighbour << " gate: " << aux3[n] <<" size: " << aux2.size() << endl;
      EV << "Time cost table info for neighbour: " << actualNeighbour <<" on node " << myAddress << " id: " << getParentModule()->getName() <<endl;
      EV << "Gate----Pheromone value------Estimated hops--------Last Time cost" << endl;
      EV <<"  " << aux3[n] << "          " << ptable[actualNeighbour][aux3[n]] << "                "<< htable[actualNeighbour][aux3[n]]<<  "           "<< aux2[1] << endl;
      it = ttable.begin();aux.clear();aux2.clear(); it2 = aux.begin();// aux2.clear(); // initialise iterators
                }
   aux.clear(); aux2.clear();aux3.clear();
}

void AntNet::showChannelParams()
{ // display request channel (recently used) parameters
    int j = getParentModule()->gateSize("port");
    //int gateId = getParentModule()->gateBaseId("port$i");
        double delay,datarate =0; double jit =0;
       // EV << getParentModule()->getClassName() << getParentModule()->gate(gateId)->info() << endl;
       for (int i=0; i<j; i++) {
         if ( getParentModule()->gate("port$i",i)->isConnected()) {
             datarate =  getParentModule()->gate("port$i",i)->getIncomingTransmissionChannel()->par("datarate");
             jit = getParentModule()->gate("port$i",i)->getIncomingTransmissionChannel()->par("jitter");
             delay =  getParentModule()->gate("port$i",i)->getIncomingTransmissionChannel()->par("delay");
                     }
       }

       //double jit = getParentModule()->gate(gateId+2)->getChannel()->par("jitter");
       EV << "Simulation time: " << simTime()<< " Datarate: "<< datarate/1000000 << " Mbps ; Delay: "<< delay << "s ; jitter: " << jit << "ms"<< endl;
}

bool AntNet::showChannelParamsByPort(int port)
{ // display request channel (recently used) parameters selected by port
    int j = getParentModule()->gateSize("port");
    //int gateId = getParentModule()->gateBaseId("port$i");
    if (port >= j) {
        EV << "Error. Wrong port specified: "<< port << " Max: "<< j << endl;
        return false;
    }
        double delay,datarate =0; double jit,noise =0;
       // EV << getParentModule()->getClassName() << getParentModule()->gate(gateId)->info() << endl;

     if ( getParentModule()->gate("port$i",port)->isConnected()) {
             datarate =  getParentModule()->gate("port$i",port)->getIncomingTransmissionChannel()->par("datarate");
             jit = getParentModule()->gate("port$i",port)->getIncomingTransmissionChannel()->par("jitter");
             delay =  getParentModule()->gate("port$i",port)->getIncomingTransmissionChannel()->par("delay");
             noise =  getParentModule()->gate("port$i",port)->getIncomingTransmissionChannel()->par("noise");
             EV << "Simulation time: " << simTime()<< " Port:"<< port << " Datarate: "<< datarate/1000000 << " Mbps ; Delay: "<< delay << "s ; jitter: " << jit << "ms ; "<<"Noise: " << noise << "dB"<< endl;
             return true;
                     }

     return false;
     //double jit = getParentModule()->gate(gateId+2)->getChannel()->par("jitter");

}

void AntNet::showRoutingInfo() // show the routing info in actual node abut interesting destinations, like destAddresses
{ // source addresses and neighbours, so try to show all relevant routing info in actual node
    unsigned int maxChoices = getParentModule()->gateSize("port");
    // check destination addresses
    for (unsigned int n=0; n< destAddresses.size(); n++ ) {
        int dest = destAddresses[n];
        EV << "Routing table info for destination: " << dest <<" on node " << myAddress << " id: " << getParentModule()->getName() <<endl;
        EV << "Gate----Pheromone value------Estimated hops------Last time cost---" << endl;
        for (unsigned int i=0; i < maxChoices; i++) {
            if ((ptable[dest][i] > 0) || (htable[dest] [i] > 0)) // if interesting routing info upper than zero
                EV <<"  " << i << "          " << ptable[dest][i] << "                "<< htable[dest][i] << "          "<< ttable [dest] [i] [htable [dest] [i]] << endl;
            }
    }
    EV << "--------------------------------------------------------------------"<< endl;
    // check source addresses
    for (unsigned int n=0; n< srcAddresses.size(); n++ ) {
        int src = srcAddresses[n];
        EV << "Routing table info for source: " << src <<" on node " << myAddress << " id: " << getParentModule()->getName() <<endl;
        EV << "Gate----Pheromone value------Estimated hops------Last time cost---" << endl;
        for (unsigned int i=0; i < maxChoices; i++) {
            if ((ptable[src][i] > 0) || (htable[src] [i] > 0)) // if interesting routing info upper than zero
               EV <<"  " << i << "          " << ptable[src][i] << "                "<< htable[src][i] << "          "<< ttable [src] [i] [htable [src] [i]] << endl;
        }
    }
    EV << "--------------------------------------------------------------------"<< endl;
    //EV << "---------------------------------------------"<< endl;
    // check neighbours
    RoutingTable aux;
    unsigned int maxNeighbours = neighbourGates(aux);
    for (unsigned int n=0; n < maxNeighbours; n++ ) {
            int actualNeighbour = ntable [aux[n]];
            EV << "Routing table info for neighbour: " << actualNeighbour <<" on node " << myAddress << " id: " << getParentModule()->getName() <<endl;
            EV << "Gate----Pheromone value------Estimated hops------Last time cost---" << endl;
            EV <<"  " << aux[n] << "          " << ptable[actualNeighbour][aux[n]] << "                "<< htable[actualNeighbour][aux[n]] << "          "<< ttable [actualNeighbour] [aux[n]] [htable [actualNeighbour] [aux[n]]] << endl;
    }
    aux.clear();
}

void AntNet::showRoutingInfo(int dest) // show RPTable of pheromone, estimated hops and last time cost values
{ // about specified destination
    unsigned int maxChoices = getParentModule()->gateSize("port");
        EV << "Routing table info for destination: " << dest  <<" on node " << myAddress << " id: " << getParentModule()->getName() <<endl;
        EV << "Gate----Pheromone value------Estimated hops------ Last time Cost---" << endl;
        for (unsigned int i=0; i < maxChoices; i++) {
            EV <<"  " << i << "          " << ptable[dest][i] << "                "<< htable[dest][i]<< "          "<< ttable [dest] [i] [htable [dest] [i]]  << endl;
        }
}

void AntNet::showRTable()
{ // display interesting routing table info in AODV or static protocols
    //unsigned int maxPorts = getParentModule()->gateSize("port");

    for (unsigned int i=0; i < destAddresses.size(); i++)
        EV << "Routing table for destination address: "<< destAddresses.at(i) << " ; port value: "<< rtable[destAddresses.at(i)] <<endl;
    EV << "Routing table for destination address: 11 (possible source)" << " ; port value: "<< rtable[11] <<endl;
    EV << "Routing table for destination address: 8 (possible source)" << " ; port value: "<< rtable[8] <<endl;
}

int AntNet:: symmetricGate(int g ) // return symmetric (opposite) gateID
{ // Pre: special network BaseWmn specifications
    int sym= g+1;
    if ((g== 0) || (g==2) ) {
        EV << "sym: " << sym << endl;

    }else {
        if (g==4) sym =1; // special network BaseWmn  specifications
        else sym = g-1;
        EV << "sym: " << sym << endl;
    }
    return sym;
}

const char * AntNet::toDoubleCalc(double d)
{ // return numeric value formatted for Math applications (swap '.' to ',' notation)

    char s[22] = "hola como estan uds";
    //char * s= new char*();// = getParentModule()->getName();
   sprintf(s,"%f",d);
    for (unsigned int i=0;i<strlen(s);i++)
    {
        if (s[i] == '.') {
              s[i]=',';
              return strdup(s);
        }
    }
    return strdup(s);
}

int AntNet::travelChoices (RoutingTable & choices,cMessage *msg)  // check all the travel choices (possible destinations)
{
    int num=0;
    unsigned int maxChoices = getParentModule()->gateSize("port");
    for (unsigned int i=0; i < maxChoices; i++) { // use port out size, previous error use ntable size
        if (isConnected(i) && (!(isInPath(msg,ntable[i])))) { // check if could be a loop way) { //(ptable[dest][i] > 0))
                 //EV << "Adding choice gate " << i << " in neighbour table with size " << ntable.size() << " is neighbour "<< ntable[i]  << endl;
                 choices[num]=i;
                 num++;
              }

        }
    return num;
}

void AntNet::tryLocalRepair(cMessage * msg, bool & comeback) // Pre: Data packet, no positive pheromone value to dest
{
    comeback = false;
    Packet *pk = check_and_cast<Packet *>(msg);
    pk->setKind(5); // FA local repair
    pk->setDisplayString("i=msg/ant3_s,yellow"); // Local repair FA displays as yellow packet
    pk->setByteLength(1024);
    int src = pk->getSrcAddr();
    int dest = pk->getDestAddr();
    char pkname[40];
    sprintf(pkname,"LocalRepair-FA-%d-to-%d-#%d",src,dest, repairCounter++);
    pk->setName(pkname);
    pk->setTransientNodes(pk->getHopCount() ,myAddress);
    pk->setHopCount(pk->getHopCount()+1);
    EV << "generating Local repair FA packet " << pkname << endl;
    // add new field, if possible , store flooding numbers
    cMsgPar*  floods = new cMsgPar();
    floods->setLongValue(0);
    floods->setName("floods");
    pk->addPar(floods);
    // check chances, we know no positive pheromones ones
    RoutingTable aux;
    int bestHops = bestHopsEstimation(dest);
    if (bestHops > 0) { // some interesting routing info
        int choices = bestHopsEstimationChoicesLR(aux,dest,msg); // should take also loop come back choice
        if (choices > 1) pk->par("floods").setLongValue(1); // increase flooding counter
            if (isInPath(msg,ntable[aux[0]])) { // packet coming back
                pk->setTransientNodes(pk->getHopCount()-1, 0); // maybe not needed, just fix with update hops count
                pk->setHopCount(pk->getHopCount()-2);
                comeback = true;
                EV << "Forwarding local repair FA packet " << pkname << " on gate "<< aux[0] << " (coming back)" << endl;
            }
            else EV << "Forwarding local repair FA packet " << pkname << " on gate "<< aux[0] << endl;
        controlHops++;
        send(pk,"out",aux[0]);
        for (int i=1; i < choices; i++) {
            if (isInPath(msg,ntable[aux[i]])) { // packet coming back
                pk->setTransientNodes(pk->getHopCount()-1, 0); // maybe not needed, just fix with update hops count
                pk->setHopCount(pk->getHopCount()-2);
                comeback = true;
                EV << "Forwarding local repair FA packet " << pkname << " on gate "<< aux[i] << " (coming back)" << endl;
             }
             else  EV << "Flooding...Local repair FA packet " << pkname << " on gate "<< aux[i] << endl;
            controlHops++;
            send(pk->dup(),"out",aux[i]);
                      }
    } else { // no routing info at all
        int choices = travelChoices(aux,msg);
        if (choices == 0) { // no routing options, discard, left local repair process
            EV << "No routing choices to look for local repair solution, giving up the process..."<< endl;
            delete pk;return;
        }
        if (choices > 1) pk->par("floods").setLongValue(1); // increase flooding counter

        EV << "Forwarding local repair FA packet " << pkname << " on gate "<< aux[0] << endl;
           controlHops++;
           send(pk,"out",aux[0]);
           for (int i=1; i < choices; i++) {
                EV << "Flooding...Local repair FA packet " << pkname << " on gate "<< aux[i] << endl;
                controlHops++;
                send(pk->dup(),"out",aux[i]);
                    }
    }
}

void AntNet::updateHTable(int dest,unsigned int outgate, int hops)
{ // update hops estimation routing table to dest, from outgate with hops value
    if (htable[dest] [outgate] != hops) {
        htable[dest] [outgate] = hops;
       // EV << "Hops table was updated to value:" << hops << " Dest: " << dest << " Gate: " << outgate << endl;
    } //else
       // EV << "Hops table is already with correct value:" << hops << " Dest: " << dest << " Gate: " << outgate << endl;
}

void AntNet::updateNb(int gate, int value,int hops) // gate, neighbour address
{ // update neighbour table with value and hops estimation
    ntable[gate] = value; //update neighbour table in local address
    if (value > 0) // check if dest is not unknown (represented by 0)
        updateHTable(value,gate,hops); // update hops estimation table
    if (mySort->longValue() == 1) // static
        return;
    sendNbPacketTo(gate,value); // send neighbour packet to destination that must be changed
}

void AntNet::updateRoutingInfoOnGateError(int gate) // Pre: Gate error, node out of sight from output gate
{ // reset tables on position across selected gate (recently unconnected)
    // check destination addresses
    for (unsigned int n=0; n < destAddresses.size(); n++ ) {
        int dest = destAddresses[n];
        if  (ptable[dest][gate] > 0) {
            updateRPTable(dest,gate,0,0);
            updateHTable(dest,gate,0);

        } else if (htable[dest] [gate] > 0) {// if interesting routing info upper than zero
               updateHTable(dest,gate,0);//EV <<"  " << i << "          " << ptable[dest][i] << "                "<< htable[dest][i] << endl;
                }
    }

    // check source addresses
    for (unsigned int n=0; n< srcAddresses.size(); n++ ) {
         int src = srcAddresses[n];
         if ((ptable[src][gate] > 0)) {
             updateRPTable(src,gate,0,0);
             updateHTable(src,gate,0);

         } else if (htable[src] [gate] > 0) {// if interesting routing info upper than zero
                updateHTable(src,gate,0);
                    }
            }

    // check possible neighbour
    if (ntable[gate] > 0) {// if neighbour detected in unconnected gate
        updateRPTable(ntable[gate], gate,0,0);
        updateHTable(ntable[gate],gate,0);
        ntable[gate] = 0; // reset neighbour table

    }
}

void AntNet::updateRPTable (int dest,unsigned int outgate, double cost,int hops )
{ // Update Routing Pheromone Table in [dest, outgate] position
   if ( (cost == 0) && (hops==0)) { // signal to reset value, to avoid div zero  errors
       ptable [dest] [outgate] = 0;
       ttable [dest] [outgate] [hops] = 0;
       EV << "Pheromone Table resets to 0 values: "<< ptable [dest] [outgate] << " Dest: " << dest << " Gate "<< outgate << " Hops "<< hops << " Cost: "<< cost << endl;
       return;
   }
    //double c = cost.dbl(); // use with simtime_t parameter
    unsigned int maxChoices = getParentModule()->gateSize("port");
    // evaporation in other values
  if (evaporation->boolValue() ) {
      //EV << "evaporation is the problem?, ntable size: " << ntable.size()  << " outgate: "<< outgate << " dest:"<< dest << endl;
    for (unsigned int j=0; j< maxChoices;j++){ // use port out size, previous error use ntable size
        if ((j != outgate)&& (ptable [dest][j]!= 0 ) ) {
            ptable [dest] [j] = ptable [dest] [j] * coefPh->doubleValue();
            EV << "Pheromone Table evaporates to value: "<< ptable [dest] [j] << " Dest: " << dest << " Gate "<< j << " Hops "<< hops << " Cost: "<< cost << endl;
        }
    }
  }
    if (metrics->longValue() ==1) // Travel time
    ptable [dest] [outgate] = ptable [dest] [outgate] * coefPh->doubleValue() + ((1/(cost * 1000) )* (1 - coefPh->doubleValue())); // use cost as Travel time; added 10â�»^3 to normalize (unitary)
    else if (metrics->longValue() == 2) // Hops
        if (hops > 1 )  ptable [dest] [outgate] = ptable [dest] [outgate] * coefPh->doubleValue() + (double) 1/hops; // use cost as Hops, makes hops more accuracy for paths
        else ptable [dest] [outgate] = ptable [dest] [outgate] * coefPh->doubleValue() + ((1/(hops) )* (1 - coefPh->doubleValue())); // use cost as Hops;
    else {  // Linear combination of Travel time and hops
        ptable [dest] [outgate] = ptable [dest] [outgate] * coefPh->doubleValue() + 0.5*( (((1/(hops) )* (1 - coefPh->doubleValue())) + ((1/(cost * 1000) )* (1 - coefPh->doubleValue()))));
        // use lineal combination ; added 10â�»^3 to normalise (unitary)
        // if value greater than 1 (typical for 1 hops re-update) just recalculate value
        if (ptable [dest] [outgate] > 1)  ptable [dest] [outgate] =  0.5*( (((1/(hops) )* (1 - coefPh->doubleValue())) + ((1/(cost * 1000) )* (1 - coefPh->doubleValue()))));
    }
    ttable [dest] [outgate] [hops] = cost;
    EV << "Pheromone Table Updated to value: "<< ptable [dest] [outgate] << " Dest: " << dest << " Gate "<< outgate << " Hops "<< hops << " Cost: " << cost << " Ttable:" << ttable [dest] [outgate] [hops]<< endl;

}

bool AntNet::wasHereBefore(long int id)
{
    for (int i=0; i< visitor; i++){
        if (vtable[i] == id) return true;
    }
    return false;
}


void AntNet::handleMessage(cMessage *msg)
{
    if (msg->isSelfMessage()) {
        //if (msg->getKind() ==7) { // test message in node A
            //EV << "Testing new process initReactiveFA from node A(4) to node F (10)" << endl;
            //initReactiveFA(10);
          //  delete msg;
           // return;
        //}
        if (msg->getKind() == 6) { // data packet stored waiting local repairing
           EV << "Checking if local repairing find out a path " << endl;
           // cast msg
           Packet *pk = check_and_cast<Packet *>(msg);
           pk->setKind(0); // change to data packet again
           if (positivePheromonChoices(pk->getDestAddr())) { // normal
               EV << "Local repair mission successful" << endl;
               forwardDataPacketByRPTable(pk); // route packet by RPTable pheromone value
               return;
           } else {
                EV << "Local repair mission failed, discarding data packet" << endl;
                dropCounter++; dataCounter++;
                delete msg;
                return;
                    }
                }
        if (msg == goodDiff) { // Good diffusion request
            if (goodDiffusion->boolValue()) { // kind store the origin gate needed to start the process
                EV << "Launching good diffusion process, with origin gate: "<< msg->getKind() << endl;
                goodPhDiffusion(ntable[msg->getKind()],1,msg->getKind());
// need to create new goodPhDiffusion proc, because of strange 1 meaning 0... better to fix such misunderstanding in future
                delete msg;
                return;
            }
        }
        if (msg->getKind() ==4) { // AODV RERR message (timer) local repair option
            EV << "here we are" << endl;
            //return;//
            Packet *pk = check_and_cast<Packet *>(msg);
            if ( rtable [pk->getDestAddr() ] != -1 ) { // check if a new route was discovered
                EV << "RERR packet,  end mission, new route available" << endl;
                waitingReply= false;
                delete pk; // maybe not needed, later delete msg ?
                return;
            }else { // still no route encounter
                int reGateIndex =0;
                pk->setHopCount(pk->getHopCount()+1);
                int dest = pk->getTransientNodes(pk->getTransientNodesArraySize()-1 -pk->getHopCount());
                showRTable();
                if (dest ==0){ // from data packet, no TransientNodes
                    EV << "Looking come back route, by port "<< rtable[pk->getSrcAddr()] << " to address "<< pk->getSrcAddr() << " for packet going before to "<< pk->getDestAddr() << endl;
                    dest = pk->getSrcAddr();
                    int source = pk->getDestAddr();
                    reGateIndex = rtable[dest];
                    delete pk; // delete timer
                    EV << "Sending RERR packet to following node by port " << ntable[dest] << " destination:"<< dest << endl;
                    sendRERR(reGateIndex,source,dest); // new send RERR gate,source,dest
                    return;
                }
                if (locateNeighbor(dest, reGateIndex)) {
                    //check if next node destination is a neighbour node and possible way
                    EV << "Destination detected in neighbour table, gate: " << reGateIndex << " destination: " << dest << endl;
                    EV << "Sending RERR packet to following node"<< endl;
                    emit(outputIfSignal, reGateIndex);
                    controlHops++; // increase control Hops counter
                    send(pk,"out",reGateIndex);
                    return;
                    } else {
                       EV << "RERR packet fail, when coming back to next node, deleting packet..."<< endl;
                       delete pk; // no idea if needed ??
                       return;
                            }
            }

        }
//        if (msg == replyTimer) { // AODV reply timer reached
//            //msg
//        }
        if (msg == generateRREQ) { // AODV, starting route  request process
            for (unsigned int n=0; n< destAddresses.size(); n++ ) {
                routeRequest(destAddresses[n]);
            }
        }
        if (msg == move1) { // Node X moves in line of Node C
            //int j = getParentModule()->gateSize("port");
            EV << "14,6s; Node X start moving... " << endl; //ports: " << j  << endl;
            move2 = new cMessage("Move 2");
            scheduleAt(29,move2); // use signal to register moment ?
            getParentModule()->setDisplayString("i=device/cellphone;p=424,111;r=60,,#707070");
            // draw node X (going to South east)
        }
        if (msg == move2) { moveNode(2);
                                     EV << "29s; Node X moves in line with node C"   << endl;

                                    }
          if (msg == move3) {
                              //if (showChannelParamsByPort(3) && showChannelParamsByPort(0))  {;}
                              EV << "Node X moves South, will be visible by AP2, and further from Node C " << endl;
                              moveNode(3);
                              //if (showChannelParamsByPort(3)&& showChannelParamsByPort(0)) {
                                  EV << "44s; Now node X and AP2 are visible each other" << endl;
                              //}
                              }
          if (msg == move4) { //if (showChannelParamsByPort(3)&& showChannelParamsByPort(0)) {;}
                              moveNode(4);
                              //if (showChannelParamsByPort(3)&& showChannelParamsByPort(0)) {
                              EV << "49s; Node X more reachable  by AP2, node C is around 31metres "   << endl;
                               //   }
                              }
          if (msg == move5) { //if (showChannelParamsByPort(3)&& showChannelParamsByPort(0)) {;}
                                 moveNode(5);
                                // if (showChannelParamsByPort(3)&& showChannelParamsByPort(0)) {
                                    EV << "54s; Node X more reachable  by AP2 (39metres) "   << endl;
                                  //         }
                                        }
          if (msg == move6) { //if (showChannelParamsByPort(3)&& showChannelParamsByPort(0)) {;}
                                  moveNode(6);
                                //  if (showChannelParamsByPort(3)&& showChannelParamsByPort(0)) {
                                      EV << "59s; node C is around 40metres "   << endl;
                                  //          }
                                        }
          if (msg == move7) { //if (showChannelParamsByPort(3)&& showChannelParamsByPort(0)) {;}
                                        moveNode(7);
                                //        if (showChannelParamsByPort(3)&& showChannelParamsByPort(0)) {
                                        EV << "64s; Node X more reachable  by AP2 (30m), node C is around 45metres "   << endl;
                                  //          }
                                        }
          if (msg == move8) { //if (showChannelParamsByPort(3)&& showChannelParamsByPort(0)) {;}
                                        moveNode(8);
                                //        if (showChannelParamsByPort(3)&& showChannelParamsByPort(0)) {
                                        EV << "69s; Node X more reachable  by AP2(24m), node C out of sight "   << endl;
                                  //          }
                                        }
          if (msg == move9) { //if (showChannelParamsByPort(3)&& showChannelParamsByPort(4)) {;}
                                        moveNode(9);
                                //        if (showChannelParamsByPort(3)&& showChannelParamsByPort(4)) {
                                        EV << "80s; Node X now visible by node F, around 49metres "   << endl;
                                  //          }
                                        }
          if (msg == move10) { //if (showChannelParamsByPort(3)&& showChannelParamsByPort(4)) {;}
                                        moveNode(10);
                                 //       if (showChannelParamsByPort(3)&& showChannelParamsByPort(4)) {
                                        EV << "85s; Node X more reachable  by node F, around 44metres "   << endl;
                                   //         }
                                        }
          if (msg == move11) { //if (showChannelParamsByPort(3)&& showChannelParamsByPort(4)) {;}
                                        moveNode(11);
                                 //       if (showChannelParamsByPort(3)&& showChannelParamsByPort(4)) {
                                        EV << "90s; Node X same distance from AP2 and node F (around 31-39metres) "   << endl;
                                   //         }
                                        }
          if (msg == move12) { //if (showChannelParamsByPort(3)&& showChannelParamsByPort(4)) {;}
                                        moveNode(12);
                                 //       if (showChannelParamsByPort(3)&& showChannelParamsByPort(4)) {
                                        EV << "101s; Node X more reachable  by  node F (30m), AP2 is around 44metres "   << endl;
                                   //         }
                                        }
          if (msg == move13) { //if (showChannelParamsByPort(3)&& showChannelParamsByPort(4)) {;}
                                        moveNode(13); //last move
                                 //       if (showChannelParamsByPort(3)&& showChannelParamsByPort(4)) {
                                        EV << "106s; Node X more reachable  by node F around 22metres, AP2 is out of range now "   << endl;
                                   //         }
                                        }
          if (msg == updateVT){
                  vtable[visitor] = msg->getKind(); // possible better msg->par("treeId") ?
                  EV << "Updated visiting table, position: "<< visitor << " FA id(kind): " << vtable[visitor] << " par treeId: "<< msg->par("treeId").longValue() << endl;
                  visitor++;
                  //delete msg;
                              }
          if (msg->getKind() == 8) { // checking message to launch BA proactive in destination/source address
              EV << "Sent Proactive BA from "<< getParentModule()->getName()<< endl;
              sendProactiveBA();
              //showCostTable(); // debugging problem procedure
              // check if resent the checking message (in checkingTime or twice checking Time)
              if ((simTime() + checkingTime) > SimTimeLimit) {
                  delete msg; // no time to resent check message
                  return;
              } else {
                  EV << "Next check will be in "<<  checkingTime << " seconds" << endl;
                  scheduleAt(simTime() + checkingTime, msg);
                  return;
              }
          }

    //delete msg; //needed just one time, or not if cancel, delete process ?
    return;}

    Packet *pk = check_and_cast<Packet *>(msg);
    if (pk->getSrcAddr() != myAddress) { // Packet coming from outside, to skip packets from App layer
        if (noisyChannel(pk)) {  // ERROR packet branch
            EV << "Received wrong packet (indecipherable), detected as noise, cannot process it, from gate " << msg->getSenderGateId() << endl;
            if (mySort->longValue() == 1) { // Static , Dijkstra
                dropCounter++;
                EV << "Data packet loss, total: "<<  dropCounter <<endl;dataCounter++;
                delete pk;
                return;
            }
            if (mySort->longValue() == 4) { // AODV
                switch (msg->getKind()) {
                                case 0: dropCounter++;
                                        EV << "Data packet loss, total: "<<  dropCounter <<endl;dataCounter++;
                                        // update routing table (symmetric)
                                        rtable[pk->getSrcAddr()] =-1 ;
                                        // send RERR to origin to change routing table there
                                        sendRERR(gate(pk->getArrivalGateId())->getIndex() ,pk->getSrcAddr());
                                break;
                                case 1: EV << "Neighbour packet loss, total: "<<  nbCounter++ <<endl;
                                        ntable[gate(msg->getArrivalGateId())->getIndex()]=0;
                                        EV << "Update neighbour table, now is out of reach the address: " << pk->getSrcAddr() << " from: " << myAddress << " by gate:" << gate(msg->getArrivalGateId())->getIndex() << endl;
                                        //update neighbour table
                                break;
                                case 2: EV << "RREQ packet loss, total: "<<  (faCounter++)+1 <<endl;
                                // just to check if now good value shown
                                break;
                                case 3: EV << "RREP packet loss, total: "<<  baCounter++ <<endl;
                                break;
                                case 4: EV << "RERR packet loss, total: "<<  baCounter++ <<endl;
                                       // update routing table
                                        rtable[pk->getSrcAddr()] =-1;
                                        //rtable[pk->getDestAddr()] =-1; // if not same as myAddress (never be noisy, it supposed)
                                break;
                                case 5: EV << "RREQ Local repair packet loss, total: "<<  (faCounter++)+1 <<endl;
                                // just to check if now good value shown
                                                                break;
                                                }
                            //delete pk;
                            delete msg; // only once
                            return;
            } else { // CPANT, AntWMNet
            switch (msg->getKind()) {
                case 0: dropCounter++;EV << "Data packet loss, total: "<<  dropCounter <<endl;dataCounter++;
                break;
                case 1: EV << "Neighbour packet loss, total: "<<  nbCounter++ <<endl;
                        // check first if it was visible before
                        if (ntable[gate(msg->getArrivalGateId())->getIndex()] == 0) { // not visible before
                            EV << "Link unconnected right now"<< endl;
                        } else { // change, link broken
                        EV << "Detected broken link on gate " <<  gate(msg->getArrivalGateId())->getIndex() << endl;
                        updateRoutingInfoOnGateError(gate(msg->getArrivalGateId())->getIndex());
                        // previous sentence perform this and 2 following lines and more update info
                        //ntable[gate(msg->getArrivalGateId())->getIndex()]=0;
                        //htable[pk->getSrcAddr()][gate(msg->getArrivalGateId())->getIndex()]= 0; // reset hops table
                        //ptable[pk->getSrcAddr()][gate(msg->getArrivalGateId())->getIndex()]= 0; // reset prob routing table
                        EV << "Update routing info tables, now it is unreachable the address: " << pk->getSrcAddr() << " from: " << myAddress << " by gate:" << gate(msg->getArrivalGateId())->getIndex() << endl;
                        //update neighbour table
                        //showRoutingInfo();
                        linkError(pk->getSrcAddr(),gate(msg->getArrivalGateId())->getIndex());// repairing procedure to neighbours about dest (pk->getSrcAddr() ) reached from gate on intermediate node (actual) (myAddress)
                        }
                break;
                case 2: EV << "Forward Ant packet loss, total: "<<  faCounter++ <<endl;
                break;
                case 3: EV << "Backward Ant packet loss, total: "<<  baCounter++ <<endl;
                break;
                                }
            delete pk;
            return;
            }
                             }
    }
    if (pk->getKind() == 1) { // hello msg -> neighbour purpose
        if (pk->getTransientNodesArraySize() == 2) { // Pheromone diffusion packets
           EV << "Pheromone diffusion packet: "<< pk->getName()<< " received" << endl;
           // got new best hops estimation to dest on the origin gate
           int originGateId = pk->getArrivalGateId();
           int origin = gate(originGateId)->getIndex();
           int bestHops = pk->getHopCount(); int dest= pk->getDestAddr();
           if ((visitor == 0) || (! wasHereBefore(pk->getTreeId())))
           {
              vtable[visitor] = pk->getTreeId();
              //EV << "Updated  visiting table, position: "<< visitor << " PhDiff id: " << vtable[visitor] << endl;
              visitor++;
              //updateHTable(pk->getSrcAddr(),origin,hops); // update hops table, as its first packet to enter node
           } else { // Packet already visited here
           // check if could add info
             if (htable[dest] [origin] == 0) { // no info in gate
                updateHTable(dest,origin,bestHops);
                EV << "Added info so no previous estimation hops in gate "<< origin << endl;
             } else if (htable [dest] [origin] != bestHops) {
                       if (htable [dest] [origin] < bestHops) {
                           updateRPTable(dest,origin,0,0); // reset routing table, so a bad change happened
                           EV << "Reset probability routing table so now worse estimation hops by gate "<< origin << endl;
                       }
                     updateHTable(dest,origin,bestHops); // fixed,, big error
                     EV << "Added info so now different estimation hops in gate "<< origin << endl;
                     }
                 // possible check if coming from reseting pheromone and only way node, to diffusion the broken way
             redundant++;
             EV << "PhDiff id: " <<  pk->getTreeId() << " already checked here by same packet, discarding... " << pk->getName() << endl;
             emit(dropSignal, (long)pk->getByteLength());
             //showRoutingInfo(dest);
             delete pk; return;
             }
             if (bestHops == 0) { // check 1st if bestHops ==0 (means is unknown) so reset hops estimation value (unknown)
             // check if already have good value
                 if (htable[dest] [origin] == 0) { // already updated
                     EV << "Hops estimation table already with good value: " <<  "0 hops, ending pheromone diffusion now"<< endl;
                     emit(dropSignal, (long)pk->getByteLength());
                     delete pk;
                     //showRoutingInfo(dest);
                     return;
                            }
             updateHTable(dest,origin,0);
             updateRPTable(dest,origin,0,0);
             } else  {
                  if (bestHops == htable[dest] [origin]) { // hops estimation table already with good value
                     EV << "Hops estimation table already with good value: " << bestHops << " hops, ending pheromone diffusion now"<< endl;
                     emit(dropSignal, (long)pk->getByteLength());
                     delete pk;
                     //showRoutingInfo(dest);
                     return;
                  } else { // different value, need to update to new estimation
                  //bool retest = false;
                  // check 1st if bestHops ==1 (means is unknown) so reset hops estimation value (unknown)
                  //if (bestHops == 1) { // already updated
                  // updateHTable(dest,origin,0);
                  //  updateRPTable(dest,origin,0,0);
                  //}
                  //else
                  //check if worser than previous
                      if (bestHops > htable [dest] [origin]) {
                         if (ptable[dest][origin] != 0) {
                             updateRPTable(dest,origin,0,0); // reset pheromone value, path changed to worser
                             updateHTable(dest,origin,bestHops);
                             EV << "Reset pheromone table value so old path have changed, need to test again and diffusion to neighbours" << endl;
                             // spread to next destinations with best estimation in actual node
                             pheromoneDiffusion(dest,bestHopsEstimation(dest),origin,pk->dup());
                             //pheromoneDiffusion(dest,bestHops++,origin);
                             emit(dropSignal, (long)pk->getByteLength());
                             //showRoutingInfo(dest);
                             //temp.clear();
                             delete pk;
                             return;
                             //retest = true;
                          }
                      }
                      updateHTable(dest,origin,bestHops);
                  }
             }
             // check if continue diffusion
             RoutingTable temp;
             int bestHopsEstimated = bestHopsEstimation(dest);
             int bestHopsEstimatedChoices = bestHopsEstimationChoices(temp, dest, NULL);
             if (((bestHopsEstimated == bestHops) && (bestHopsEstimatedChoices == 1)&& (temp[0] == origin)) ||
                ((bestHops == 0) && (bestHopsEstimated == 0)) ) {
                 //|| retest   ) { // if only best estimation choice is the new one
                 // or best estimation is like new one and is 0 (1 for new)
                 EV << "Continue pheromone diffusion, because best hops (from diffusion):" << bestHops << " and best hop estimation in actual node:" << bestHopsEstimated <<" and best hops choices:"<< bestHopsEstimatedChoices << " and origin gate:"<< origin<< " same as best hop gate:" << temp[0]<< endl;
                 //if (bestHops == 0)
                 pheromoneDiffusion(dest,bestHopsEstimated,origin,pk->dup());
                 //pheromoneDiffusion(dest,bestHops,origin); // new diffusion to neighbours but no origin gate
                 //else
                 //   pheromoneDiffusion(dest,bestHopsEstimated,origin,pk->dup()); // no sense if
                 //pheromoneDiffusion(dest,bestHops++,origin);
                 emit(dropSignal, (long)pk->getByteLength());
                 //showRoutingInfo(dest);
                 temp.clear();
                 delete pk;
                 return;
             }
           //}
       //showRoutingInfo(dest);
       delete pk;
       return;
       }
//        if (pk->getTransientNodesArraySize() == 2) { // Pheromone diffusion packets , old branch
//            EV << "Pheromone diffusion packet: "<< pk->getName()<< " received" << endl;
//            // got new best hops estimation to dest on the origin gate
//            int originGateId = pk->getArrivalGateId();
//            int origin = gate(originGateId)->getIndex();
//            int bestHops = pk->getHopCount(); int dest= pk->getDestAddr();
//            if (bestHops == htable[dest] [origin]) { // hops estimation table already with good value
//                EV << "Hops estimation table already with good value: " << bestHops << " hops, ending pheromone diffusion now"<< endl;
//                emit(dropSignal, (long)pk->getByteLength());
//                delete pk;
//                return;
//            } else { // different value, need to update to new estimation
//                // check 1st if bestHops ==1 (means is unknown) so reset hops estimation value (unknown)
//                if (bestHops == 1){
//                    updateHTable(dest,origin,0);
//                    updateRPTable(dest,origin,0,0); // restet pheromone value
//                }
//                else updateHTable(dest,origin,bestHops);
//                // check if continue diffusion
//                RoutingTable temp;
//                if (((bestHopsEstimation(dest) == bestHops) && (bestHopsEstimationChoices(temp,dest,NULL) == 1)) ||
//                     ((bestHops == 1) && (bestHopsEstimation(dest)==0))   ) { // if only best estimation choice is the new one
//                                                                       // or best estimation is like new one and is 0 (1 for new)
//                    if (bestHops == 1) pheromoneDiffusion(dest,bestHops,origin); // new diffusion to neighbours but no origin gate
//                    else pheromoneDiffusion(dest,bestHops++,origin);
//                    emit(dropSignal, (long)pk->getByteLength());
//                    temp.clear();
//                    delete pk;
//                    return;
//                }
//
//            }
//            showRoutingInfo(dest);
//            return;
//        }
        if (pk->getDestAddr() == pk->getSrcAddr() ) {

            // hello msg, locate neighbour
            pk->setSrcAddr(myAddress);
            pk->setHopCount(pk->getHopCount()+1);
            pk->setTravelTime(pk->getArrivalTime() - pk->getCreationTime());
            int originGateId = pk->getArrivalGateId();
            int origin = gate(originGateId)->getIndex();
            //EV << "In (time calculated in Routing) : " << pk->getTravelTime() << endl;
            //avgTime1hop = pk->getTravelTime().dbl();
            //calcAvg1hop(avgTime1hop);
            ntable[origin] = pk->getDestAddr(); // update with the origin neighbour acknowledge?
            EV <<"Detected neighbour " << ntable[origin] << " on node "<< myAddress << " on gate " << origin << " #" << originGateId << endl;
            if (mySort->operator int() == 2) { //AntWMNet
                updateRPTable(ntable[origin],origin,pk->getTravelTime().dbl(),1); // update Routing table to neighbour
                updateHTable(ntable[origin],origin,1); // update Routing hops table
            } else if (mySort->longValue() == 4) // AODV
                rtable [pk->getDestAddr()] = origin;
            EV << "forwarding seeking-neighbour packet " << pk->getName() << " on gate index " << origin << " (coming home) " << endl;
            emit(outputIfSignal, origin);
            send(pk, "out", origin);
            return;
        } else {
            // update neighbour table
            int originGateId = pk->getArrivalGateId();
            int origin = gate(originGateId)->getIndex();
            //int symGateId = pk->getSenderGateId();
            //EV << "In (time calculated in Routing) : " << pk->getTravelTime() << endl;
            //avgTime1hop = pk->getTravelTime().dbl();
            //calcAvg1hop(avgTime1hop);
            if (ntable[origin] ==  pk->getSrcAddr()) { // neighbour table already updated
                EV << "Neighbour table is already up to date, discarding packet:  "<< pk->getName() << endl;
                redundant++;
                //EV << "Redundant packets : "<< redundant << endl;
                emit(dropSignal, (long)pk->getByteLength());
                delete pk;
                return;
            }
            ntable [origin] = pk->getSrcAddr();
            if (mySort->longValue() == 4) // AODV
                rtable [pk->getSrcAddr()] = origin;
            if (pk->getTravelTime() > 0)
                pk->setTravelTime(simTime() - pk->getTravelTime()); // now reflect one hop travel time
            else
                pk->setTravelTime(pk->getArrivalTime() - pk->getCreationTime());
            EV <<"Detected neighbour " << ntable[origin] << " on node "<< myAddress << " on gate " << origin << " #" << originGateId << endl;
            if (mySort->longValue() == 2) { // AntWMNet
                    if ((simTime() > 2) && isDestAddress(ntable[origin]) && ((bestHopsEstimation(ntable[origin]) > 1) ||
                            (bestHopsEstimation(ntable[origin]) == 0))) {
                        // when old longer route active or unknown // cuando habia camino a destino mas largo o no se conocia
                        EV << "Detected new path to destination address, time to send proactive forward ant to sourceAddresses" << endl;
                        sendProactiveFAtoSrc(ntable[origin],pk->getTravelTime().dbl(), 2, origin);
                        // perfect place for good diffusion process after helloTime
                        if (goodDiffusion->boolValue()) { // message need origin parameter
                            goodDiff = new cMessage("Begin good diffusion");
//                            cMsgPar*  gd = new cMsgPar(); // not necessary if save value in kind field
//                            gd->setLongValue(origin);    // also easy to identify self message by kind field
//                            gd->setName("origin");
//                            goodDiff->addPar(gd);
                            goodDiff->setKind(origin); // store in kind field the origin gate
                            EV << "Schedule good diffusion process in (helloTime): "<< helloTime->doubleValue() << " seconds"<< endl;
                            scheduleAt(simTime()+(helloTime->doubleValue()),goodDiff);
                            //goodPhDiffusion(ntable[origin],1,origin);
// need to create new goodPhDiffusion proc, because of strange 1 meaning 0... better to fix such misunderstanding in future
                        }
                    }

                updateRPTable(ntable[origin],origin,pk->getTravelTime().dbl(),1); // update routing probability table
                updateHTable(ntable[origin],origin,1); // update Routing hops table
                //showRoutingInfo(ntable[origin]);
            }
            //EV << "local delivery of packet " << pk->getName() << endl; // not needed send to App neighbour msg
            //pk->setTravelTime(pk->getArrivalTime() - pk->getCreationTime()); // not needed
            //send(pk, "localOut"); // not needed
            //emit(outputIfSignal, -1); // -1: local, not needed
            delete pk;
            return;
        }
    }
    int destAddr = pk->getDestAddr();
    if (destAddr == myAddress)   // packet arrived desired destination
    {
        EV << "local delivery of packet " << pk->getName() << endl;
        pk->setTravelTime(pk->getArrivalTime() - pk->getCreationTime());
        EV << "In (time calculated in Routing) : " << pk->getTravelTime() << " ,hops: "<< pk->getHopCount() << endl;
        //showChannelParams(); // debug Channel parameters changes
        //EV << "Inside arrival branch, before send LocalOut" << endl;
        unsigned int originGateId = pk->getArrivalGateId();
        //unsigned int senderGateId = pk->getSenderGateId();
        //EV << "originGateIds : " << originGateId << " senderGateId: " << senderGateId << endl;
        //if (showChannelParamsByPort(gate(originGateId)->getIndex())) {;}  // show incoming channel parameters
        // show Routing Table values to interesting destinations
        //showRoutingInfo();
        if (pk->getKind() == 0) { // Data packet arrives
            dataCounter++;
            hitCounter++;
            avgTravel = avgTravel +  pk->getTravelTime().dbl();
            avgHops = avgHops + pk->getHopCount();
            send(pk, "localOut");
            return;
            //dataReport(pk); // checking results process
        }
        if (mySort->longValue() ==4) // AODV
        {
            if ((pk->getKind() == 2) || (pk->getKind() ==5)) // RREQ arrives destination
                    {
                //int originGateId = pk->getArrivalGateId();
                EV << "originGateId : " << originGateId << " Id pk: " << pk->getTreeId() << endl;
                unsigned int origin = gate(originGateId)->getIndex();
                EV << "origin : " << origin << endl;
                if ((visitor == 0) || (! wasHereBefore(pk->getTreeId())))
                    // beware, so only with one visit destroy FA packets
                    {
                   if (updateVTime->doubleValue() == 0) { // no need to send message if there is no delay
                       vtable[visitor] = pk->getTreeId();
                       EV << "Updated  visiting table, position: "<< visitor << " RREQ id: " << vtable[visitor] << endl;
                       visitor++;
                   } else {
                      updateVT = new cMessage("Update Visiting Table");
                      cMsgPar * vt = new cMsgPar("treeId");
                      vt->setLongValue(pk->getTreeId());
                      vt->setName("treeId");
                      updateVT->addPar(vt);
                      updateVT->setKind(pk->getTreeId()); // store in kind field the pk id
                      scheduleAt(simTime()+(updateVTime->doubleValue()/1000),updateVT);
                      // vtable[visitor] = pk->getTreeId();
                      //EV << "Updated  visiting table, position: "<< visitor << " FA id: " << vtable[visitor] << endl;
                      //                visitor++;
                           }
                     } else {
                        EV << "RREQ id: " <<  pk->getTreeId() << " already checked here by best route, discarding packet " << pk->getName() << endl;
                        emit(dropSignal, (long)pk->getByteLength());
                        delete pk;
                        return;
                               }
                     EV << "Route request mission completed, generating route reply packet" << endl;
                     //int origin = gate(originGateId)->getIndex();
                     char pkname[40];
                     sprintf(pkname,"RREP-%d-to-%d-#%d", myAddress, pk->getSrcAddr(), rrepCounter++);
                     Packet *rrep = new Packet(pkname);
                     rrep->setSrcAddr(myAddress);
                     rrep->setKind(3); // RREP packet
                     rrep->setDestAddr(pk->getSrcAddr()); // fatal error, its src address!
                     int k = pk->getHopCount();
                     rrep->setTransientNodesArraySize(k); // initialise to safe value, store hops count
                     rrep->setHopCount(0);
                     //->setTravelTime(pk->getTravelTime());
                     rrep->setDisplayString("i=msg/resp_s,blue"); // RREP in blue
                     for (int i=0;i<k;i++) {
                         rrep->setTransientNodes(i,pk->getTransientNodes(i));
                               }
                     for (unsigned int i =0; i< rrep->getTransientNodesArraySize(); i++) {
                         EV << "Transient node in position" << i << " value: " << rrep->getTransientNodes(i) << endl;
                         // print transientNodesArray could help
                                           }
                     EV << "forwarding RREP packet " << rrep->getName() << " on gate index " << origin << " (coming back) " << endl;
                     //updateRPTable(ba->getDestAddr(),origin,pk->getTravelTime().dbl(),k);
                     rtable[pk->getSrcAddr()]= origin; // update symmetric way
                     EV << "Update routing table, pos: "<< pk->getSrcAddr() << " value: " << origin << endl;
                     controlHops++; // increase control Hops counter
                     emit(outputIfSignal, origin);
                     send(rrep,"out",origin);
                    } else
             if (pk->getKind() ==3) { // RREP arrives destination
                 EV << "Route reply mission completed"<< endl;
                 int origin = gate(originGateId)->getIndex();
                 rtable[pk->getSrcAddr()]= origin; // update routing table
                 EV << "Update routing table, pos: "<< pk->getSrcAddr() << " value: " << origin << endl;
             } else
             if (pk->getKind() == 4) { // RERR arrives destination
                 EV << "Route error mission completed"<< endl;
                 rtable[pk->getSrcAddr()]= -1; // update routing table to -1 (no path known)
                 routeRequest(pk->getSrcAddr()); // initiate new route request process
             }
            delete pk; // needed?
            return; // if delete pk, need to abort now
        }

        if (((pk->getKind() == 2) || (pk->getKind() == 3) || pk->getKind() ==4 || pk->getKind() == 5) && antsToApp->boolValue() && (mySort->longValue() == 2) )
            // here can select if control packets (FA,BA) go to lower levels or not
        {   emit(outputIfSignal, -1); // -1: local
            send(pk, "localOut");

        }
        if (pk->getSrcAddr() == destAddr) return; // same destination and source, no need to check extra features
        else{
        if (mySort->longValue() == 2) // AntHocNet
        {

            if (pk->getKind() == 2 || pk->getKind() ==4 || pk->getKind() == 5) { // FA arrived -> BA created & launched back
                //EV << "Inside AntHocNet-FA arrived branch" << endl;
                //unsigned int originGateId = pk->getArrivalGateId();
                //unsigned int senderGateId = pk->getSenderGateId();
                //EV << "originGateIds : " << originGateId << senderGateId << endl;
                //unsigned int sender = gate(senderGateId)->getIndex();
                //EV << "sender : " << sender << endl;
                //cGate * originGate = pk->getArrivalGate();
                //int origin =0;
                //if (originGate->isConnectedOutside() ) {
                int origin = gate(originGateId)->getIndex();
                //int symmetric = symmetricGate(origin);

                //int symmetric = gate(senderGateId);
                 //EV << "origin (1st): " << origin << " symmetric: " << symmetric << " Id. pk: "<< pk->getId() << endl;
                //}else {
                //        origin = pk->getTransientNodes(pk->getHopCount()-1);
                //        EV << "origin (good branch): " << origin << endl;
                //        int dest = origin;
                //        if  (locateNeighbor(dest,origin)) EV << "origin(gate) : " << origin << endl;
                //       }

                char pkname[40];
                int src = pk->getSrcAddr();
                sprintf(pkname,"BA-%d-to-%d-#%d", myAddress, src, baCounter++);
                EV << "generating Backward Ant " << pkname << endl;
                Packet *ba = new Packet(pkname);
                //ba->setByteLength(packetLengthBytes->longValue());
                ba->setSrcAddr(myAddress);
                ba->setKind(3); // BA packet
                ba->setDestAddr(src); // fatal error, its src address!

                int k = pk->getHopCount();
                ba->setTransientNodesArraySize(k); // initialise to safe value, store hops count
                ba->setHopCount(0);
                ba->setTravelTime(pk->getTravelTime());
                ba->setDisplayString("i=msg/ant3_s,blue"); // BA in blue

                for (int i=0;i<k;i++) {
                    ba->setTransientNodes(i,pk->getTransientNodes(i));
                }
                EV << "forwarding Backward Ant packet " << ba->getName() << " on gate index " << origin << " (coming back) "<< " ; hops estimation: " << k  << endl;

                //updateRPTable(destAddr,origin,pk->getTravelTime().dbl(),ba->getTransientNodesArraySize());
                // same node, dest, not necessary
                updateRPTable(src,origin,pk->getTravelTime().dbl(),k);
                if (htable[src][origin] > k || // update only if better or
                    (htable [src] [origin] == 0)) // or no previous estimation
                    // update routing hops table to source (here we are in dest, not need update other
                    // (could perform transitive property to transient nodes)
                    updateHTable(src,origin,k);
                costSrc = pk->getTravelTime().dbl(); // debugging data
                //delete pk;
                controlHops++; // increase control Hops counter
                //EV << "Control packets, neighbours: " << nbCounter << " FA: " << faCounter++ << " BA: "<< baCounter << endl; // increase FA when received
                emit(outputIfSignal, origin);
                //EV << "Control hops: " << controlHops << endl;
                showRoutingInfo(src);
                send(ba,"out",origin);
            } else if (pk->getKind() == 3) { // BA arrived
                //EV << "Inside AntWMNet-BA arrived branch" << endl;
                //int originGateId = pk->getArrivalGateId();
                //EV << "originGateId : " << originGateId << endl;
                unsigned int origin = gate(originGateId)->getIndex();
                //int symmetric = symmetricGate(origin);
                int hops = pk->getTransientNodesArraySize();
                int src = pk->getSrcAddr();
                costDest= (simTime() - pk->getCreationTime()).dbl(); // debugging info
                //EV << "origin : " << origin << " symmetric: " << symmetric << endl;
                // TOCheck update routing table ,symmetric
                //updateRPTable(destAddr,origin,simTime().dbl() - pk->getCreationTime().dbl(),pk->getTransientNodesArraySize()); // same dest and actual node, not neccessary
                updateRPTable(src,origin,costDest,hops);
                if (hops < htable [src] [origin] || (htable [src] [origin] == 0)) // only if better or no previous estimation
                    // update routing hops table to source (here we are in dest, not need update other
                    // (could perform transitive property to transient nodes)
                    updateHTable(src,origin,hops);
                pk->setHopCount(pk->getHopCount()+1);
                showRoutingInfo(src);
                emit(dropSignal, (long)pk->getByteLength());
                EV << "BA mission completed"<< endl; // no need to update baCounter++;
                // delete pk;
            } else if (pk->getKind() == 8) { // Proactive BA arrived
                //EV << "Old values in node " << getParentModule()->getName() << " costDest:" << costDest << " costSrc:"<< costSrc << endl;
                //showRoutingInfo();
                //showCostTable();
                int hops = pk->getHopCount();
                int src = pk->getSrcAddr();
                unsigned int origin = gate(originGateId)->getIndex();
                double old = getLastTime(src, origin, hops); // better getLastTime function
                //EV << "Old values in node " << getParentModule()->getName() << " costDest:" << costDest << " costSrc:"<< costSrc << " ttable:"<< old << endl;
                bool better, same; double newCost = (simTime() - pk->getCreationTime()).dbl();
                double old2;
                if (isSrcAddress(myAddress)) {
                    old2  = costDest;
                    costDest = newCost; same = (old == old2);
                    better = (costDest < old2);
                    //EV << "New cost value to destination: " << costDest << " , is it better than old value? " << better <<  " , is it same old variable value and last time table one? "<< same  << endl;
                } else { // in destination address
                    old2 = costSrc;
                    costSrc = newCost; same = (old == old2);
                    better = (costSrc < old2);
                    //EV << "New cost value to source address: " << costSrc << " , is it better than old value? " << better << " , is it same old variable value and last time table one? "<< same  << endl;
                }
                // check if new cost time is better than previous
                better = (newCost < old);
                if (better) {}
                else { // worse value, update to less value than previous
                    // check if same hops estimation and number of hops to source
                    if (hops == htable [src] [origin])
                        updateRPTable(src,origin,0,0); // first reset value, then update to worse value
                }
                updateRPTable(src,origin,newCost,hops);
                if (hops < htable [src] [origin] ||
                   (htable [src] [origin] == 0)) // only if better hops estimation or no previous info
                    updateHTable(src,origin,hops);
                // update routing hops table to source ;here we are in dest, not need update other
                    // (could perform transitive property to transient nodes)
                emit(dropSignal, (long)pk->getByteLength());
                EV << "Proactive BA mission completed"<< endl;
                showRoutingInfo(src);
                //showCostTable();
                delete pk; return;
           }
        }
        return;
     } return;
    }
    if (mySort->longValue() == 2) // AntHocNet, in intermediate node
    {
        if (pk->getKind() == 4 || pk->getKind() == 5) { // Proactive FA or local repair FA
            int hops = pk->getHopCount();
            int limit = checkLimits(hops);// check array and hops limits, need hops count
            if (limit == 1) { // max hops reached
                EV << "Max hops = " << maxHops << " reached, discarding route & packet " << pk->getName() << endl;
                emit(dropSignal, (long)pk->getByteLength());
                delete pk; return;
            } else if (limit == 2) { //max array reached
                EV << "Max array limit = " << maxArray->longValue() << " reached, when hops: "<< hops << " ,discarding route & packet " << pk->getName() << endl;
                emit(dropSignal, (long)pk->getByteLength());
                delete pk; return;
            }
            int originGateId = pk->getArrivalGateId();
            unsigned int origin = gate(originGateId)->getIndex();
            if (checkPreviousVisit(pk->getSrcAddr(),hops,origin,pk->getTreeId())) {
     // check if already visited and if better estimation hops (need source address, hops,origin (unsigned), treeid (long)
                EV << "Proactive FA id: " <<  pk->getTreeId() << " already checked here by other route, discarding packet " << pk->getName() << endl;
                emit(dropSignal, (long)pk->getByteLength());
                //EV << "Control packets, neighbours: " << nbCounter << " FA: " << faCounter << " BA: "<< baCounter << " ProFA: "<< proactiveCounter++ << endl; // increase FA before deleted
                delete pk;
                return;
            }

            RoutingTable aux;
            int dest = pk->getDestAddr();
            int outGateIndex; double win =0;
            int choices = positivePheromonChoices(aux,dest,pk);
            int originInt = origin; // avoid warning
            showRoutingInfo(dest);
            // choices = skipOriginChoices(aux, origin)    // skip origin choices, loops for Local repair or
            if ((pk->getKind() == 5) && (choices ==1) && (aux[0] == originInt)) { // if come back in local repair
                choices = 0; // try other way, we know this way is not good
                EV <<"Avoid origin gate and reseting known problem route from origin gate: " << origin << endl;
                updateRPTable(dest,origin,0,0); // reset origin choice (we know a problem this way)
                EV << "Now local repair is working under this node, so data packet will be launched from here" << endl;
                // mark data packet to easy identification as self message
                cPacket * pk2 = pk->dup(); pk2->setKind(6);
                double timeLocalR = 2 * bestHopsEstimation(destAddr) * hopTime->doubleValue(); // apply formula
                EV << "Store packet until checking in " << timeLocalR << " if local repair could fix routing and build new path"<< endl;
                scheduleAt(simTime() + timeLocalR, pk2);
                // send data pk after a formula, hoping local repair works
            }
            pk->setTransientNodes(pk->getHopCount() ,myAddress);
            pk->setHopCount(pk->getHopCount()+1);
            if (choices > 0) {
                //pk->setTransientNodes(pk->getHopCount() ,myAddress);
                //pk->setHopCount(pk->getHopCount()+1);
                if (choices > 1) { // flooding Proactive FA
                    // check local repair FA flooding limit
                    if (pk->getKind() == 5) // local repair FA branch
                        if (checkFloodingLimit(pk)) { // checking also increase value if pass
                            EV << "Max flooding limit = " << maxLocalRepairFlooding->operator short int() << " reached, discarding route & packet " << pk->getName() << endl;
                            emit(dropSignal, (long)pk->getByteLength());
                            delete pk; return;
                        }
                  for (int i=0; i < choices; i++) {
                    outGateIndex= aux[i];
                    win = ptable[dest][aux[i]];
                    EV << "Flooding! forwarding packet " << pk->getName() << " on gate index " << outGateIndex << " with prob: " << win << " and hops estimation " << htable [dest][outGateIndex] << endl;
                    emit(outputIfSignal, outGateIndex);
                    controlHops++; // increase control Hops counter
                    if (i == 0)  send(pk,"out",outGateIndex);
                    else send(pk->dup(), "out", outGateIndex);
                                           }
                    //EV << "Control hops: " << controlHops << endl;
                    aux.clear();
                    return;
                } else { // 1 choice
                  outGateIndex= aux[0];
                  win = ptable[dest][aux[0]];
                  EV << "Forwarding packet " << pk->getName() << " on gate index " << outGateIndex << " with prob: " << win<< " and hops estimation " << htable [dest][outGateIndex]  << endl;
                  emit(outputIfSignal, outGateIndex);
                  controlHops++; // increase control Hops counter
                  send(pk,"out",outGateIndex);
                  aux.clear(); return;
                           }
            } else {  // no positive pheromone choices
                aux.clear();
                //showRoutingInfo(dest);
                int choices2 = bestHopsEstimationChoices(aux,pk->getDestAddr(),pk);
                //EV << "Choices: " << choices << " choices2: " << choices2 << endl;
                if (choices2 > 0) {
                   if (choices2 > 1) { // flooding Proactive FA
                       if (pk->getKind() == 5) // local repair FA branch
                           if (checkFloodingLimit(pk)) { // checking also increase value if pass
                               EV << "Max flooding limit = " << maxLocalRepairFlooding->longValue() << " reached, discarding route & packet " << pk->getName() << endl;
                               emit(dropSignal, (long)pk->getByteLength());
                               delete pk; return;
                           }
                     for (int i=0; i < choices2; i++) { // big error, previous use choices instead choices2
                        outGateIndex= aux[i];
                        win = ptable[dest][aux[i]];
                        EV << "Flooding! forwarding packet " << pk->getName() << " on gate index " << outGateIndex << " with prob: " << win << " and hops estimation " << htable [dest][outGateIndex]  << endl;
                        emit(outputIfSignal, outGateIndex);
                        controlHops++; // increase control Hops counter
                        if (i == 0)  send(pk,"out",outGateIndex);
                        else send(pk->dup(), "out", outGateIndex);
                                         }
                        //EV << "Control hops: " << controlHops << endl;
                        aux.clear();
                        return;
                 } else { // 1 choice
                     outGateIndex= aux[0];
                     win = ptable[dest][aux[0]];
                     EV << "Forwarding packet " << pk->getName() << " on gate index " << outGateIndex << " with prob: " << win << " and hops estimation " << htable [dest][outGateIndex] << endl;
                     emit(outputIfSignal, outGateIndex);
                     controlHops++; // increase control Hops counter
                     send(pk,"out",outGateIndex);
                     aux.clear(); return;
                                }
              } else { // error, no choices
                 EV << "No choices to route, so discarding route & packet " << pk->getName() << endl;
                 emit(dropSignal, (long)pk->getByteLength());
                 //proactiveCounter++; // no sense to increase here
                 delete pk;
                 return;
                       }
                    }
        }
        if (pk->getKind() == 3) { // BA arrives
            int baGateIndex =0;
            pk->setHopCount(pk->getHopCount()+1);
            destAddr = pk->getTransientNodes(pk->getTransientNodesArraySize()-1 -pk->getHopCount());
            if (locateNeighbor(destAddr, baGateIndex)) { // &&(ptable2[outGateIndex] > 0)) {
                // check if destination is a neighbour node and possible way
            //    EV << "Destination detected in neighbour table, gate: " << baGateIndex << " destination: " << destAddr << endl;
                // TOCHECK update routing table
                int originGateId = pk->getArrivalGateId();
                //int senderGateId = pk->getSenderGateId();
                int src = pk->getSrcAddr();
                int dest = pk->getDestAddr();
                int hops = pk->getHopCount();
                int hops2 = pk->getTransientNodesArraySize() - hops;
                costDest = (simTime() - pk->getCreationTime()).dbl();
                unsigned int origin = gate(originGateId)->getIndex();
                //unsigned int symmetric = symmetricGate(origin);
                double symCost = normCost(pk->getTravelTime().dbl() - costDest);
                // normalise cost symmetric value estimation
                //EV << "Origin gate id: "<< originGateId << " origin: "<< origin << " symmetric: " << symmetric << endl;
                //updateRPTable(destAddr,baGateIndex,pk->getTravelTime().dbl(),1);
                // not necesary if neighbour packets perform routing table update
                updateRPTable(src, origin, costDest, hops); // update path to BA source
                updateRPTable(dest, baGateIndex, symCost, hops2); // update path to BA destination (source node) symmetric (but use destination port no symmetric)
                if (hops < htable [src] [origin] || (htable [src] [origin] == 0)) // better or not info at all
                    updateHTable(src, origin, hops); // symmetric
                if (hops2 < htable [dest] [baGateIndex] || // better
                   (htable [dest] [baGateIndex] == 0))  // no previous info
                    updateHTable(dest, baGateIndex, hops2); // destination
                costSrc = symCost; // debugging data
                //showRoutingInfo();
                EV << "forwarding BA packet " << pk->getName() << " on gate index " << baGateIndex << " steps left: " << pk->getTransientNodesArraySize()-pk->getHopCount() << endl;
                //pk->setHopCount(pk->getHopCount()-1);
                emit(outputIfSignal, baGateIndex);
                controlHops++; // increase control Hops counter
                //EV << "Control hops: " << controlHops << endl;
                send(pk, "out", baGateIndex);
                return;
            } else {
                EV << "Error detected when routing BA: " << pk->getName() << " when routing to " << destAddr << endl;
                //baCounter++;
                emit(dropSignal, (long)pk->getByteLength());
                delete pk;
                return;
            }
        }
        if (pk->getKind() == 8) { // Proactive BA, can only calculate one way, to the source of the packet
            int baGateIndex = nextGateProBA(pk);
            int src = pk->getSrcAddr();
            int hops = pk->getHopCount();
            unsigned int origin = gate(pk->getArrivalGateId())->getIndex();
            double newCost = (simTime()-pk->getCreationTime()).dbl(); // to packet source
            //showCost(newCost, src); // debugging info
            //showCostTable();
            if (baGateIndex == -1) {// error, no positive pheromone choices
                EV << "No positive pheromone choices to route Proactive BA" << endl;
                EV << "Discarding Proactive BA & Launching Proactive FA to destination: "<< destAddr << endl;
                sendProactiveFA(destAddr,simTime().dbl(),1,origin);
                emit(dropSignal, (long)pk->getByteLength());
                delete pk;
                return;
             }
            // check if improve the route (better cost time)
            bool worse; // check if worse time collected
            double old2;
            //showCostTable();
            bool same; double old = ttable [src] [origin] [hops]; // better findLastTimeCost function
            if (htable [src] [origin] > hops || (htable [src] [origin] == 0)) // better or not info at all)
                updateHTable(src,origin,hops); // check if same cost inside process
            if (isDestAddress(src)) {
                same = (costDest == old);
                worse = newCost > costDest;
                old2 = costDest;
                costDest = newCost;
            }
            else {
                old2 = costSrc;
                same = (costSrc == old);
                worse = costSrc < newCost;
                costSrc = newCost;
            }
            if (!same ) {
                worse = (old < newCost); //EV << "Different value in cost time table and variable, ttable: "<< old<< " ,variable:"<< old2<< endl;
            } else {} //EV << "Same value in cost time table and variable "<< endl;
            if (worse && (htable [src] [origin] == hops)) { // only if same hops
                // check if same hops performed from last time, and actual travel
                //EV << "Worse time and same hops, so first reset pheromone value to place worse value on it" << endl;
                updateRPTable(src,origin,0,0); // if worse time, first reset
            }
            updateRPTable(src,origin,newCost,hops);
            EV << "forwarding Proactive BA packet " << pk->getName() << " on gate index " << baGateIndex << " estimated steps left: " << htable[destAddr][baGateIndex] << endl;
            pk->setTransientNodes(hops,myAddress);
            pk->setHopCount(hops+1);
            emit(outputIfSignal, baGateIndex);
            controlHops++; // increase control Hops counter
            //EV << "Control hops: " << controlHops << endl;
            //showRoutingInfo(src);
            send(pk, "out", baGateIndex);
            return;
        }   // data packets and reactive FA
        RPtable::iterator it = ptable.find(destAddr);
            if (it==ptable.end())
            {
                EV << "address " << destAddr << " unreachable, discarding packet " << pk->getName() << endl;
                if (pk->getKind() == 0) { // Data packet lost
                            dataCounter++;
                            dropCounter++;
                        //    dataReport(pk);
                        } else faCounter++; //only could be FA packet
                emit(dropSignal, (long)pk->getByteLength());
                delete pk;
                return;
            }
        if (pk->getHopCount() == maxHops)
        {
            EV << "Max hops = " << maxHops << " reached, discarding route & packet " << pk->getName() << endl;
            emit(dropSignal, (long)pk->getByteLength());
            //if (pk->getKind() == 2) { // FA branch, counting
            //EV << "Control packets, neighbours: " << nbCounter << " FA: " << faCounter++ << " BA: "<< baCounter << endl;
            // increase FA
            //}
            if (pk->getKind() == 0) { // Data packet lost
                        dataCounter++;dropCounter++;
                      //  dataReport(pk);
                    }
            delete pk;
            return;
        }
        int k = pk->getHopCount();

        if ((k+1) >= maxArray->longValue()) { // reach array memory limit
            EV << "Max array limit = " << maxArray->longValue() << " reached, when hops: "<< k << " ,discarding route & packet " << pk->getName() << endl;
            emit(dropSignal, (long)pk->getByteLength());
//            if (pk->getKind() == 2) { // FA branch, counting
//                EV << "Control packets, neighbors: " << nbCounter << " FA: " << faCounter++ << " BA: "<< baCounter << endl; // increase FA
//                     }
            if (pk->getKind() == 0) { // Data packet lost
                dataCounter++;dropCounter++;
               // dataReport(pk);
                               }
           delete pk;
           return;
        }
//        if (k > 2){
//            EV << "Last 3 transient nodes: " << pk->getTransientNodes(k-3)  << pk->getTransientNodes(k-2) << pk->getTransientNodes(k-1) << " Total : " << k << endl;
//        }
        int originGateId = pk->getArrivalGateId();
  //      EV << "originGateId : " << originGateId << " Id pk: " << pk->getTreeId() << endl;
        unsigned int origin = gate(originGateId)->getIndex();
    //    EV << "origin : " << origin << endl;
        if ((pk->getKind() == 2) && (flooding->boolValue())){ // FA arrives and active flooding
            //VisitingTable::iterator itv = vtable.find(pk->getTreeId());
                if ((visitor == 0) || (! wasHereBefore(pk->getTreeId())))
                    // beware, so only with one visit destroy FA packets
                {
                    if (updateVTime->doubleValue() == 0) { // no need to send message if there is no delay
                        vtable[visitor] = pk->getTreeId();
                        //EV << "Updated  visiting table, position: "<< visitor << " FA id: " << vtable[visitor] << endl;
                        visitor++;
                        updateHTable(pk->getSrcAddr(),origin,k); // update hops table, as its first packet to enter node
                        // could be good to store cost time here, to better calculation of symmetric pheromone
                    } else {
                        updateVT = new cMessage("Update Visiting Table");
                        cMsgPar * vt = new cMsgPar("treeId");
                        vt->setLongValue(pk->getTreeId());
                        vt->setName("treeId");
                        updateVT->addPar(vt);
                        updateVT->setKind(pk->getTreeId()); // store in kind field the pk id
                        updateHTable(pk->getSrcAddr(),origin,k);// update hops table, as its first packet to enter node
                        // could be good to store cost time here, to better calculation of symmetric pheromone
                        scheduleAt(simTime()+(updateVTime->doubleValue()/1000),updateVT);
                        // vtable[visitor] = pk->getTreeId();
                        //EV << "Updated  visiting table, position: "<< visitor << " FA id: " << vtable[visitor] << endl;
                        //                visitor++;
                    }
                } else {
                    showRoutingInfo(pk->getSrcAddr());
                    if  (cutRange->longValue() == 0) { // always cut
                        if (isBetterHopsEstimation(pk->getSrcAddr(),origin,k))
                            updateHTable(pk->getSrcAddr(),origin,k); // only if better
                        EV << "FA id: " <<  pk->getTreeId() << " already checked here by other route, discarding packet " << pk->getName() << endl;
                        emit(dropSignal, (long)pk->getByteLength());
                        delete pk;
                        return;
                    }
                    if (metrics->longValue() > 1 ) { // if hops or hops & time metrics, update hops table if less hops
                        EV <<"Hops table actual  value:"  << htable[pk->getSrcAddr()][origin] << " for  dest:"<< pk->getSrcAddr() << " and gate:"<< origin << " actual path hops value: "<< k << endl;
                        if (isShortestPath(pk->getSrcAddr(),k)) { // also if equal to shortest
                        // process to check if pk take the shortest route by the moment(isShortestPath(dest,g hops))
                            //updateHTable(pk->getSrcAddr(),origin,k);
                            // could be good to store cost time here, to better calculation of symmetric pheromone
                              if  ((cutRange->longValue() == 1) && (isEqualPath(pk->getSrcAddr(), k))) {
                                  //  cut if worst or equal
                                        updateHTable(pk->getSrcAddr(),origin,k);
                                        EV << "FA id: " <<  pk->getTreeId() << " already checked here by equal route (in hops), discarding packet " << pk->getName() << endl;
                                        emit(dropSignal, (long)pk->getByteLength());
                                        delete pk;
                                        return;
                            }

                            if (bestHopsEstimation(pk->getSrcAddr()) == k)
                                EV << "FA id: " <<  pk->getTreeId() << " already checked here by other packet, but by same long path (in hops), so continue with " << pk->getName() << endl;
                            else EV << "FA id: " <<  pk->getTreeId() << " already checked here by other packet, but by longest path (in hops), so continue with " << pk->getName() << endl;
                            updateHTable(pk->getSrcAddr(),origin,k);
                        }
                        else {
                            if (htable[pk->getSrcAddr()][origin] == 0) {// no previous info
                               updateHTable(pk->getSrcAddr(),origin,k);
                               EV << "No previous info, so update hops estimation before deleting..." << endl;
                             }
                            EV << "FA id: " <<  pk->getTreeId() << " already checked here by best route, discarding packet " << pk->getName() << endl;
                            emit(dropSignal, (long)pk->getByteLength());
                            //EV << "Control packets, neighbours: " << nbCounter << " FA: " << faCounter++ << " BA: "<< baCounter << endl; // increase FA before deleted
                            delete pk;
                            return;
                        }
                    } else {  // no need to check hops count
                    EV << "FA id: " <<  pk->getTreeId() << " already checked here by best route, discarding packet " << pk->getName() << endl;
                    emit(dropSignal, (long)pk->getByteLength());
                    //EV << "Control packets, neighbours: " << nbCounter << " FA: " << faCounter++ << " BA: "<< baCounter << endl; // increase FA before deleted
                    delete pk;
                    return;
                            }
                }
                                 }

        for (int i=0;i<k;i++) {
            if (pk->getTransientNodes(i) == myAddress) {
                EV << "Loop detected, in node: " << myAddress << endl;
                pk->setHopCount(k-2);
                // pk->setTransientNodesArraySize(k-2); // delete last transient
                EV << "forwarding packet " << pk->getName() << " on gate index " << origin << " (coming back) " << endl;
                emit(outputIfSignal, origin);
                if (pk->getKind() == 2) controlHops++; // increase control Hops counter
                //EV << "Control hops: " << controlHops << endl;
                send(pk, "out", origin);
                return;
            }
        }
        ProbTable ptable2 = (*it).second;
        // stochastic selection win = rand(0,1); for ants r
        // double win = uniform(0,1); // probability of the winner gate
        //EV << "Random : " << win << endl;

        // check if data packet have positive pheromone option (neither origin or loop choices)
        if (!positiveGoodPhChoices(destAddr,origin,pk) && // check also if first hop, and positive ph from zero origin
            !((ptable[destAddr][origin] > 0) && (origin == 0) && (pk->getHopCount() == 0))) {
            EV << "No Good Pheromone value on the actual node to destination" << endl;
            if (pk->getKind() == 0) { // data packets
                // local repair, need all data pk duplication
                EV << "Panic! Data packet with no route, trying local repairing..." << endl;
                showRoutingInfo(destAddr);
                bool comeback; // flag to know if local repair FA takes origin gate
                tryLocalRepair(pk->dup(), comeback); // warning, will try localRepair without adding actual node to visited ones
                if (comeback) { // packet takes come back route
                   EV << "Data packet will try local repair from previous node, so deleting packet here..." << endl;
                   delete pk;
                   return;
                }
                pk->setKind(6); // mark packet to easy identification
                double timeLocalR = bestHopsEstimation(destAddr) * 2 * hopTime->doubleValue();
                if (timeLocalR == 0) // when no hops estimation, use default reply time
                    timeLocalR = replyTime->doubleValue();
                EV << "Store packet until checking in " << timeLocalR << " if local repair could fix routing and build new path"<< endl;
                scheduleAt(simTime() + timeLocalR, pk); // send data pk after a formula, hoping local repair works
                return;
            }
        }
        double win = 0;
        int  outGateIndex = 0;
        int choices=0,choices2 = 0;
        bool curl = false;
        //showChannelParams();
        //if (isSrcAddress(myAddress)) showRoutingInfo(); // show source address routing info
        if (take1stPath->boolValue() && (locateNeighbor(destAddr, outGateIndex)) && (ptable2[outGateIndex] > 0)) {
            // check if destination is a neighbour node and possible way
            EV << "Destination detected in neighbour table, gate: " << outGateIndex << endl;
            win=ptable2[outGateIndex]; // chance to choose different from neighbour... new variable?
        } else {
        RoutingTable aux; // choice -> gateIndex
        unsigned int maxChoices =  getParentModule()->gateSize("port");
        if (!rTableInit->boolValue()) {
                    for (unsigned int j=0; j < maxChoices;j++){ // use port out size, previous error use ntable size
                        if (((pk->getHopCount() == 0)) || (j != origin)) { // skipping origin gate
                            //EV << "Debugging choices, 1st jump, for possible gate: " << j << " Neighbour table size: "<< ntable.size() << " maxChoices: "<< maxChoices << endl;
                            if (win < ptable2[j] ) { // routing table value detected
                                if (!isInPath(pk,ntable[j])) { // check if could be a loop way
                                    win = ptable2[j];
                                    outGateIndex = j;
                                    aux[choices]= j;
                                    if (pk->getKind() == 2) { // FA , not only seek on best route way
                                         choices++;
                                    } else if (stochastic->boolValue()) choices++;
                                    // stochastic include all positive pheromone options
                                    else
                                        choices=1; // Data packets takes only the best route
                                }
                            } else if ((ptable2[j] == win) && (ptable2[j]>0)) {
                                if (!isInPath(pk,ntable[j])) { // check if could be a loop way
                                                            win = ptable2[j];
                                                            outGateIndex = j;
                                                            aux[choices]= j;
                                                            choices++;
                                }
                            } else if ((ptable2[j] > 0) && ((pk->getKind() == 2) || ((pk->getKind()==0) && stochastic->boolValue()))) {
                                // FA  and stochastic include all no null possibilities
                                if (!isInPath(pk,ntable[j])) { // check if could be a loop way
                                                            outGateIndex = j;
                                                            aux[choices]= j;
                                                            choices++;
                                    }
                            } else if ((win ==0) && isConnected(j)&& (ntable[j]>0)) { // possible way, unexplored
                              //  EV << "Debugging choices, 1st jump, pk should enter here 2 times, for possible gate: " << j << endl;
                                if (!isInPath(pk,ntable[j])) { // check if could be a loop way
                                    if (pk->getKind()==0) { // data packets branch
                                         outGateIndex = j;
                                         aux[choices2]= j;
                                         choices2++;
                                    } else { // FA branch, not data packets
                                           outGateIndex = j;
                                           aux[choices]= j;
                                           choices++;
                                           }
                                                            }
                            } else if (isConnected(j)&& (ntable[j]>0) && pk->getKind()==2) { // FA exploring branch
                                if (!isInPath(pk,ntable[j])) { // check if could be a loop way
                                    aux[choices]=j;
                                    choices++;
                                }
                            }

                        }
                    }
                    if ((choices ==0) && (choices2>0)) choices = choices2; // use choice with no value in routing table
                }
                else { // Routing table is initialised
        for (unsigned int j=0; j< maxChoices;j++) // use port out size, previous error use ntable size
        {
          if (((pk->getHopCount() == 0)) || (j != origin)) { // skipping origin gate
            if (win < ptable2[j] )
            {
                for (int i=0;i<k;i++) {
                            if (pk->getTransientNodes(i) == ntable[j]) {
                                curl = true; i = k; // loop detected
                                EV << "Curl skipped, node :  " << ntable[j] <<endl;
                            }
                }
                if (curl) {
                } else {
                win = ptable2[j];
                outGateIndex = j;
                aux[choices]= j;
                if (pk->getKind() == 2) { // FA , not only seek on best route way
                    choices++;
                            }
                else if (stochastic->boolValue()) { // if stochastic, also data packets are interested on positive routes
                    choices++;
                } else
                    choices=1; // Data packets takes only the best route if not stochastic selected
                }
            }else if ((ptable2[j] > 0) && (win == ptable2[j])){
                for (int i=0;i<k;i++) {
                    if (pk->getTransientNodes(i) == ntable[j]) {
                    curl = true; i = k; // loop detected
                    EV << "Curl skipped, node :  " << ntable[j] <<endl;
                    }
                }
                if (curl) { ;
                } else {
                aux[choices]=j;
                choices++;      }
            }else if ((ptable2[j] > 0) && (((pk->getKind() == 2)) || ((pk->getKind()==0)&&(stochastic->boolValue()))))
            { // FA include all no null possibilities
                for (int i=0;i<k;i++) {
                                    if (pk->getTransientNodes(i) == ntable[j]) {
                                    curl = true; i = k; // loop detected
                                    EV << "Curl skipped, node :  " << ntable[j] <<endl;
                                    }
                                }
                                if (curl) {
                                } else {
                                aux[choices]=j;
                                choices++;      }
            }
          }
          curl = false;
           // EV << "Node: "<< myAddress << " Destination :" << destAddr  << " Origin : " << origin <<  " Choices: " << choices << " Gate: " << j << " Prob:" << ptable2[j] << " win: "<< win << " neighbour:"<< ntable[j] << endl;
        }
                }
        //EV << "Choices found : " << choices << endl;

        if (choices > 1) {
            if ((pk->getKind() == 2) && (flooding->boolValue())) { // FA , implement flooding option
                pk->setTransientNodes(pk->getHopCount() ,myAddress);
                pk->setHopCount(pk->getHopCount()+1);
                for (int i=0; i < choices; i++) {
                    outGateIndex= aux[i];
                    win = ptable2[aux[i]];
                    EV << "Flooding! forwarding packet " << pk->getName() << " on gate index " << outGateIndex << " with prob: " << win << " and hops estimation " << htable [destAddr][outGateIndex] << endl;
                    emit(outputIfSignal, outGateIndex);
                    controlHops++; // increase control Hops counter
                    if (i == 0)  send(pk,"out",outGateIndex);
                    else send(pk->dup(), "out", outGateIndex);
                }
                aux.clear();
         //       EV << "Control hops: " << controlHops << endl;
                return;
            }
            // stochastic branch
            //stochastic , data packets only care about positive pheromonr values
            if (stochastic->boolValue()) {
                double totalNbPheromon = 0.0;
                    //EV << totalNbPheromon << " should be 0 " << endl;
                    int estimatedHops=0;
                    double prob[4];
                   // vector for routing probabilities
                   for (int i=0; i < 4; i++)
                       prob[i] = 0;
                   //EV << "Probability for data routing Pheromone values (1st step) : " << prob[0] <<"," << prob[1] << ","<< prob[2] << ","<< prob[3] << " Total neighbors probabilities: " << totalNbPheromon  <<endl;
                    for (int i=0; i < choices; i++) {

                        if (ptable2[aux[i]] > 0.0) {
                            prob[i] = pow(ptable2[aux[i]],dataCoef->doubleValue());
                            totalNbPheromon = totalNbPheromon + ptable2[aux[i]];
                        } else {
                               //EV << prob[i] << " deberia ser 0 " << endl; // no changes in totalNbPheromon
                        }

                    }
                    //EV << "Probability for data routing Pheromone values (2nd step) : " << prob[0] <<"," << prob[1] << ","<< prob[2] << ","<< prob[3] << " Total neighbors probabilities: " << totalNbPheromon  <<endl;

                    totalNbPheromon = pow(totalNbPheromon, dataCoef->doubleValue());
                    for (int i=0; i < choices; i++)
                        if (prob[i] != 0.0)
                            prob[i] = prob[i] / totalNbPheromon;
                    double d= uniform(0,prob[0] + prob[1] + prob[2] + prob[3],0);
//                    EV << "Choices : " << choices  << " Choices in vector(possible gates) : " << aux[0] << " " << aux[1] << " "<<aux[2] << " "<< aux[3] << endl;
//                    EV << "Double random generated : " << d << " Pheromone values : " << ptable2[aux[0]] <<"," << ptable2[aux[1]] << ","<< ptable2[aux[2]] << ","<<ptable2[aux[3]] << endl;
//                    EV << "Probability for data routing Pheromone values : " << prob[0] <<"," << prob[1] << ","<< prob[2] << ","<< prob[3] << " Total neighbours probabilities: " << totalNbPheromon << " total prob: "<< prob[0] + prob[1] + prob[2] + prob[3] <<endl;
                    if (d <= prob[0]) {
                        outGateIndex = aux[0];
                        win = ptable2[aux[0]];
                        //EV << "Enter branch A" << endl;
                        estimatedHops=htable[destAddr][aux[0]];
                    } else {d = d - prob[0]; //EV << "Skip branch A" << endl;
                            if (d <= prob[1]) {
                                outGateIndex = aux[1];
                                win = ptable2[aux[1]];
                                //EV << "Enter branch B" << endl;
                                estimatedHops=htable[destAddr][aux[1]];
                            } else { d = d - prob[1];//EV << "Skip branch B" << endl;
                                if(d <= prob[2]) {
                                    outGateIndex = aux[2];
                                    win = ptable2[aux[2]];
                                    estimatedHops=htable[destAddr][aux[2]];
                                } else if (ptable2[aux[3]] > 0) { // if positive pheromone value, take last chance
                                    outGateIndex = aux[3];
                                    win = ptable2[aux[3]];
                                    estimatedHops=htable[destAddr][aux[3]];
                    }   }  }
                    //EV << "Branch 0: " << branchA << " branch 1: " << branchB << " branch 2: "<< branchC << " branch 3: " <<  branchD << endl;
                   // EV << "Selected gate: "<< outGateIndex << " with pheromone value:"<< win << " and estimated hops:"<< estimatedHops << endl;

            }else{
            choices--;
            int b = intuniform(0,choices); // in case of draw, select randomly the outgate
            //EV << "Random chosen : " << b  << " Aux : " << aux[0] << aux[1] << aux[2] << aux[3] << endl;
            outGateIndex = aux[b];
            win = ptable2[aux[b]];
                        }
        } else if (choices == 0) {
            EV << "No choices to route, so discarding route & packet " << pk->getName() << endl;
            emit(dropSignal, (long)pk->getByteLength());
            if (pk->getKind() == 2) {
            //EV << "Control packets, neighbours: " << nbCounter << " FA: " << faCounter++ << " BA: "<< baCounter << endl; // increase FA before deleting
            }
            if (pk->getKind() == 0) { // Data packet miss
                        dataCounter++; dropCounter++;
                        //dataReport(pk);
                    }
            delete pk;
            return;
        }
        aux.clear();
        }

        pk->setTransientNodes(pk->getHopCount() ,myAddress);
        showRoutingInfo(destAddr);
        EV << "forwarding packet " << pk->getName() << " on gate index " << outGateIndex << " with prob: " << win << " and estimated hops: "<< htable[destAddr][outGateIndex] << endl;
        pk->setHopCount(pk->getHopCount()+1);
        emit(outputIfSignal, outGateIndex);
        if (pk->getKind() == 2 || (pk->getKind() == 4) || (pk->getKind() == 5)) controlHops++; // increase control Hops counter
        //EV << "Control hops: " << controlHops << endl;
        send(pk, "out", outGateIndex);
        return;
    }
    if (mySort->longValue() == 1) { // static routing
        RoutingTable::iterator it = rtable.find(destAddr);
        if (it==rtable.end())
        {
            EV << "address " << destAddr << " unreachable, discarding packet " << pk->getName() << endl;
            emit(dropSignal, (long)pk->getByteLength());
//        if (pk->getKind() == 2) {
//            EV << "Control packets, neighbors: " << nbCounter << " FA: " << faCounter++ << " BA: "<< baCounter << endl; // increase FA before deleting it
//        }
            delete pk;
            return;
        }

    int outGateIndex = (*it).second;

    EV << "forwarding packet " << pk->getName() << " on gate index " << outGateIndex << endl;
    pk->setHopCount(pk->getHopCount()+1);
    emit(outputIfSignal, outGateIndex);

    send(pk, "out", outGateIndex);
    }
    if (mySort->longValue() == 4) { // AODV branch, pk not in destination node
        if (pk->getKind()== 0) { // Data packet
            RoutingTable::iterator it = rtable.find(destAddr);
            if (it==rtable.end())
            {
                EV << "address " << destAddr << " unreachable by routing table right now, discarding packet " << pk->getName() << endl;
                emit(dropSignal, (long)pk->getByteLength());
                if (pk->getSrcAddr() == myAddress ) {
                    EV << "Route request process is proceeding now... waiting new path   "<< endl;
                    dropCounter++;
                    delete pk;
                    return;
                }
                // error proceeding
                int originGateId = pk->getArrivalGateId();
                EV << "originGateId : " << originGateId << " Id pk: " << pk->getTreeId() << endl;
                unsigned int origin = gate(originGateId)->getIndex();
                if (localRepair->boolValue()) { // local repair branch, dunno if will be performing anytime
                    EV << "No routing table value, probing local repairing; trying to generate route request(s) packet to find new path" << endl;
                    routeRequestRERR(destAddr,origin,-1);// route request to all possible choices except origin and next dest (this time not needed, replaced by -1
                    pk->setKind(4); // set packet as RERR
                    scheduleAt(simTime() +replyTime->doubleValue(),pk);// send on replyTime  (on same pk) to check route request success
                    dropCounter++;
                    return; //

                } else { // no local repair
                    EV << "No routing table value, generating route error packet to warn about it" << endl;
                    char pkname[40];
                    sprintf(pkname,"RERR-%d-to-%d-#%d", myAddress, pk->getSrcAddr(), rerrCounter++);
                    Packet *rerr = new Packet(pkname);
                    rerr->setSrcAddr(destAddr); // could be good idea to save here dest Address for updating routing table
                    rerr->setKind(4); // RERR packet
                    rerr->setDestAddr(pk->getSrcAddr()); // fatal error, its src address!
                    //int k = pk->getHopCount();
                    rerr->setTransientNodesArraySize(49); // initialize to safe value, 49
                    rerr->setHopCount(1);
                    //->setTravelTime(pk->getTravelTime());
                    rerr->setDisplayString("i=msg/resp_s,red"); // RREP in red
//                    for (int i=0;i<k;i++) {
//                         rerr->setTransientNodes(i,pk->getTransientNodes(i));
//                                                       }
//                    for (unsigned int i =0; i< rerr->getTransientNodesArraySize(); i++) {
//                        EV << "Transient node in position" << i << " value: " << rerr->getTransientNodes(i) << endl;// print transientNodesArray could help
//                                                                }
                    EV << "forwarding RERR packet " << rerr->getName() << " on gate index " << origin << " (coming back) " << endl;
                    rtable[pk->getSrcAddr()]= origin; // update symmetric way
                    EV << "Update routing table, pos: "<< pk->getSrcAddr() << " value: " << origin << endl;
                    controlHops++; // increase control Hops counter
                    emit(outputIfSignal, origin);
                    send(rerr,"out",origin);
                }
                dropCounter++; // error data pk
                delete pk;
                return;
            }

            int outGateIndex = (*it).second;
            if (outGateIndex == -1) { // Path lost
                EV << "address " << destAddr << " unreachable by routing table right now, path lost; discarding packet " << pk->getName() << endl;
                emit(dropSignal, (long)pk->getByteLength());
                dropCounter++; // data pk error
                if (pk->getSrcAddr() == myAddress ) {   // new data packet transmission before finding new route
                   EV << "Route request process is proceeding now... waiting new path   "<< endl;
                   delete pk;
                   return;
                                }
                // error procedure
                int originGateId = pk->getArrivalGateId();
                EV << "originGateId : " << originGateId << " Id pk: " << pk->getTreeId() << endl;
                unsigned int origin = gate(originGateId)->getIndex();
                if (localRepair->boolValue()) { // local repair branch, dunno if will be performing anytime
                    if (waitingReply) {
                        EV << "Waiting active local repair active procedure... "  << endl;
                        delete pk;
                        EV << "Packet deleted " << endl;
                        return;
                        EV << "This never should be visible" << endl;
                                            }
                    EV << "No routing table value, probing local repairing; trying to generate route request(s) packet to find new path, origin:" << origin << "; destination:" << destAddr << endl;
                    bool success =routeRequestRERR(destAddr,origin,origin);// route request to all possible choices except origin and next dest (this time not needed, replaced by origin)
                    if (success) { // RREQ were sent, waiting results, send reply timer
                    pk->setKind(4); // poner paquete  a tipo RERR
                    char pkname[40];
                    sprintf(pkname,"Reply-timer-from-%d", myAddress);
                    pk->setName(pkname); // renombrar paquete
                    pk->setDisplayString("i=msg/resp_s,red"); // RERR in red
                    showRTable();
                    scheduleAt(simTime() +replyTime->doubleValue(),pk);// send replyTimer (on same pk) to check route request success
                    waitingReply= true;
                    //dropCounter++;
                    return;} else { // already updated rtable (good idea to resent message here aswell instead deleting (future improve) or sent the RERR back
                        delete pk;
                        return;
                        }

                 } else { // no local repair
                       EV << "No routing table value, generating route error packet to warn about it" << endl;
                       char pkname[40];
                       sprintf(pkname,"RERR-%d-to-%d-#%d", myAddress, pk->getSrcAddr(), rerrCounter++);
                       Packet *rerr = new Packet(pkname);
                       rerr->setSrcAddr(destAddr); // could be good idea to save here dest Address for updating routing table
                       rerr->setKind(4); // RERR packet
                       rerr->setDestAddr(pk->getSrcAddr()); // fatal error, its src address!
                       //int k = pk->getHopCount();
                       rerr->setTransientNodesArraySize(49); // initialize to safe value
                       rerr->setHopCount(1);
                       //->setTravelTime(pk->getTravelTime());
                       rerr->setDisplayString("i=msg/resp_s,red"); // RERR in red
//                       for (int i=0;i<k;i++) {
//                            rerr->setTransientNodes(i,pk->getTransientNodes(i));
//                                              }
//                       for (unsigned int i =0; i< rerr->getTransientNodesArraySize(); i++) {
//                            EV << "Transient node in position" << i << " value: " << rerr->getTransientNodes(i) << endl;// print transientNodesArray could help
//                                                                     }
                       EV << "forwarding RERR packet " << rerr->getName() << " on gate index " << origin << " (coming back) " << endl;
                       rtable[pk->getSrcAddr()]= origin; // update symmetric way
                       EV << "Update routing table, pos: "<< pk->getSrcAddr() << " value: " << origin << endl;
                       controlHops++; // increase control Hops counter
                       emit(outputIfSignal, origin);
                       send(rerr,"out",origin);
                                }
                delete pk;
                return;
            }
            // optional neighbour table checking needed to get error (better change routing table to -1 when  change (out of range) detected
            EV << "forwarding packet " << pk->getName() << " on gate index " << outGateIndex << endl;
            pk->setHopCount(pk->getHopCount()+1);
            emit(outputIfSignal, outGateIndex);

            send(pk, "out", outGateIndex);
            }
        if ((pk->getKind() == 2) || (pk->getKind()==5) ) // RREQ packet
                            {
            int k = pk->getHopCount();
            if (k == maxHops)
                    {
                EV << "Max hops = " << maxHops << " reached, discarding route & packet " << pk->getName() << endl;
                emit(dropSignal, (long)pk->getByteLength());
                delete pk;
                return;
                    }
            if ((k+1) >= maxArray->longValue()) { // reach array memory limit
               EV << "Max array limit = " << maxArray->longValue() << " reached, when hops: "<< k << " ,discarding route & packet " << pk->getName() << endl;
               emit(dropSignal, (long)pk->getByteLength());
               delete pk;
               return;
                    }
            if (pk->getKind() ==5) { // RREQ local repair
                if (rtable[pk->getDestAddr()] > 0 ) { // node on the way back (will be explored later if no path around closer
                    EV << "Node already in path, to destination: " << pk->getDestAddr() << " by port: "<< rtable[pk->getDestAddr()]<< endl;
                    EV << "Route could be checked later if needed, so discarding the route & packet " << pk->getName() << endl;
                    emit(dropSignal, (long)pk->getByteLength());
                    delete pk;
                    return;
                }
            }
            if (k > 2){
               EV << "Last 3 transient nodes: " << pk->getTransientNodes(k-3)  << pk->getTransientNodes(k-2) << pk->getTransientNodes(k-1) << " Total : " << k << endl;
                    }
            int originGateId = pk->getArrivalGateId();
            EV << "originGateId : " << originGateId << " Id pk: " << pk->getTreeId() << endl;
            unsigned int origin = gate(originGateId)->getIndex();
            EV << "origin : " << origin << endl;
            if ((visitor == 0) || (! wasHereBefore(pk->getTreeId()))) // beware, so only with one visit destroy FA packets
               {
               if (updateVTime->doubleValue() == 0) { // no need to send message if there is no delay
                   vtable[visitor] = pk->getTreeId();
                   EV << "Updated  visiting table, position: "<< visitor << " RREQ id: " << vtable[visitor] << endl;
                   visitor++;
                   } else {
                   updateVT = new cMessage("Update Visiting Table");
                   cMsgPar * vt = new cMsgPar("treeId");
                   vt->setLongValue(pk->getTreeId());
                   vt->setName("treeId");
                   updateVT->addPar(vt);
                   updateVT->setKind(pk->getTreeId()); // store in kind field the pk id
                   scheduleAt(simTime()+(updateVTime->doubleValue()/1000),updateVT);
                   // vtable[visitor] = pk->getTreeId();
                   //EV << "Updated  visiting table, position: "<< visitor << " FA id: " << vtable[visitor] << endl;
                   //                visitor++;
                   }
                } else {
                EV << "RREQ id: " <<  pk->getTreeId() << " already checked here by best route, discarding packet " << pk->getName() << endl;
                emit(dropSignal, (long)pk->getByteLength());
                delete pk;
                return;
                        }
           int outGateIndex = -1;
           if (take1stPath->boolValue() && (locateNeighbor(destAddr, outGateIndex)) ) { // check if destination is a neighbor node and possible way
                        EV << "Destination detected in neighbor table, gate: " << outGateIndex << endl;
                        pk->setTransientNodes(pk->getHopCount() ,myAddress);
                        pk->setHopCount(pk->getHopCount()+1);
                        EV << "forwarding packet " << pk->getName() << " on gate index " << outGateIndex << endl;
                        emit(outputIfSignal, outGateIndex);
                        controlHops++; // increase control Hops counter
                        send(pk,"out",outGateIndex); // forwarding to neighbour destination
                        return;
            } else {
                // better place here RREQ local repair check, after neighbour checking... deleting safe
                RoutingTable aux4; int numChoices=0;
                if (k ==0) { // first hop, wonder if its a possible chance or not...
                    numChoices = travelChoices(aux4,pk); // check loops
                    //numChoices = firstTravelChoices(aux4,origin); // skip origin gate (comeback)
                } else{
                    numChoices = nextTravelChoices(aux4,origin,pk); // skip origin gate (comeback), check loops
                    // numChoices = travelChoices(aux4,pk); // check loops
                }
                if (numChoices >= 1) {
                   pk->setTransientNodes(pk->getHopCount() ,myAddress);
                   pk->setHopCount(pk->getHopCount()+1);
                   EV << "forwarding packet " << pk->getName() << " on gate index " << aux4[0] << endl;
                   emit(outputIfSignal, outGateIndex);
                   controlHops++; // increase control Hops counter
                   send(pk,"out",aux4[0]); // first forwarding, maybe the only one
                   for (int i=1; i < numChoices; i++) {
                       outGateIndex= aux4[i];
                       EV << "Flooding! forwarding packet " << pk->getName() << " on gate index " << outGateIndex << endl;
                       emit(outputIfSignal, outGateIndex);
                       controlHops++; // increase control Hops counter
                       send(pk->dup(), "out", outGateIndex);
                                }
                   aux4.clear();
                   return;
                            }
                else if (numChoices == 0) {
                        EV << "No choices to route, so discarding route & packet " << pk->getName() << endl;
                        emit(dropSignal, (long)pk->getByteLength());
                        dropCounter++;
                        delete pk;
                        return;
                }
            }

                            } else
        if (pk->getKind() ==3) { // RREP packet
            int baGateIndex =0;
            pk->setHopCount(pk->getHopCount()+1);
            int dest = pk->getTransientNodes(pk->getTransientNodesArraySize()-1 - pk->getHopCount());
            if (locateNeighbor(dest, baGateIndex)) { //check if next node destination is a neighbor node and possible way
                EV << "Destination detected in neighbor table, gate: " << baGateIndex << " destination: " << dest << endl;
                // TOCHECK update routing table
                int originGateId = pk->getArrivalGateId();
                unsigned int origin = gate(originGateId)->getIndex();
                unsigned int symmetric = symmetricGate(origin);
                //double symCost = normCost(pk->getTravelTime().dbl()-( simTime()-pk->getCreationTime()).dbl()); // normalize cost symmetric value estimation
                EV << "Origin gate id: "<< originGateId << " origin: "<< origin << " symmetric: " << symmetric << endl;
                rtable[destAddr] = baGateIndex; // update routing table (symmetric)
                rtable[pk->getSrcAddr()] =origin ; // update routing table
                EV << "Update routing table (symmetric), pos: "<< destAddr << " value: " << baGateIndex << endl;
                EV << "Update routing table, pos: "<< pk->getSrcAddr() << " value: " << origin << endl;
                EV << "forwarding RREP packet " << pk->getName() << " on gate index " << baGateIndex << " steps left: " << pk->getTransientNodesArraySize()-pk->getHopCount() << endl;
                emit(outputIfSignal, baGateIndex);
                controlHops++; // increase control Hops counter
                send(pk, "out", baGateIndex);
                return;
                } else {
                EV << "Error detected when routing RREP: " << pk->getName() << " when looking for node " << dest << " position: " << pk->getTransientNodesArraySize()-1 -pk->getHopCount() << endl;
                for (unsigned int i =0; i< pk->getTransientNodesArraySize(); i++) {
                   EV << "Transient node in position" << i << " value: " << pk->getTransientNodes(i) << endl;// print transientNodesArray could help
                }
                baCounter++;
                emit(dropSignal, (long)pk->getByteLength());
                delete pk;
                return;
                        }
                     } else
        if (pk->getKind() == 4) { // RERR packet
            // should works pretty similar as data packet but beware when route is being updated to -1
            RoutingTable::iterator it = rtable.find(destAddr);
            if (it==rtable.end())
            {
                EV << "address " << destAddr << " unreachable by routing table right now, discarding packet " << pk->getName() << endl;
                emit(dropSignal, (long)pk->getByteLength());
                // possible to add error procedure, but think in our base scenario is useless
                delete pk;
                return;
                    }
             int outGateIndex = (*it).second;
             EV << "Next transition across port:" << outGateIndex << endl;

             pk->setHopCount(pk->getHopCount()+1);
             int originGateId = pk->getArrivalGateId();
             unsigned int origin = gate(originGateId)->getIndex();
             unsigned int symmetric = symmetricGate(origin);
             EV << "Origin gate id: "<< originGateId << " origin: "<< origin << " symmetric: " << symmetric << endl;
             //rtable[destAddr] = -1; // update routing table to -1 (unknown path) also should update symmetric way
             rtable[pk->getSrcAddr()]=-1; // only need to update destination address
             if (outGateIndex == -1) { // no route known
                 EV << "Path is being updated, waiting new path, discarding packet: " << pk->getName() << endl;
                 emit(dropSignal, (long)pk->getByteLength());
                 delete pk;
                 return;
             }
             if (localRepair->boolValue()){ // not active xDD until 15-5-14 try
                 EV << "No routing table value, probing local repairing; trying to generate route request(s) packet to find new path" << endl;
                 bool b=routeRequestRERR(pk->getSrcAddr(),origin,outGateIndex);// route request to all possible choices except origin and next dest
                 if (b) { // RREQ local repair were sent , use RREQ con source address?
                    //pk->setKind(4); // poner paquete  a tipo RERR
                    char pkname[40];
                    sprintf(pkname,"Reply-timer-from-%d", myAddress);
                    pk->setName(pkname); // renombrar paquete
                    //pk->setDisplayString("i=msg/resp_s,red"); // RERR in red
                    showRTable();
                    scheduleAt(simTime() +replyTime->doubleValue(),pk);// send replyTimer (on same pk) to check route request success
                    waitingReply = true; }
                    else delete pk; // ignore so route table already update or already sent RERR back
                    return;
                 //scheduleAt(simTime() +replyTime->doubleValue(),pk);// send replyTimer to check route request success
                 //return;
                    }
             EV << "forwarding RERR packet " << pk->getName() << " on gate index " << outGateIndex << endl;
             emit(outputIfSignal, outGateIndex);
             controlHops++; // increase control Hops counter
             send(pk, "out", outGateIndex);
             return;
        }
    }
}

void AntNet::finish()
{
    //int gateId = getParentModule()->gateBaseId("port$i");
    double datarate =0; double jit =0;
   // EV << getParentModule()->getClassName() << getParentModule()->gate(gateId)->info() << endl;
   for (int i=0; i<4; i++) {
     if ( getParentModule()->gate("port$i",i)->isConnected()) {
         datarate =  getParentModule()->gate("port$i",i)->getIncomingTransmissionChannel()->par("delay");
         jit = getParentModule()->gate("port$i",i)->getIncomingTransmissionChannel()->par("jitter"); }
   }

   //double jit = getParentModule()->gate(gateId+2)->getChannel()->par("jitter");
   //EV << getParentModule()->getName() << " Address: "<< myAddress <<" Simulation time: " << simTime()<< " Delay: "<< datarate <<"s" <<" jitter: " << jit << "ms"<< endl;
    //recordScalar("Simulation time: ", simTime());
    //EV << "Control packets: (nb (redundant), fa, ba) : " << nbCounter  << " (" << redundant << "), " << faCounter << ", " << baCounter << endl;
    struct controlPkcounter{
        int nb;
        int fa, ba, sum;
    };
    controlPkcounter count;
    //cObject * punt = &count;
    count.nb = nbCounter; count.fa = faCounter; count.ba = baCounter; count.sum =  count.ba +count.fa+nbCounter+repairCounter+proactiveCounter+phDiffCounter;
    //snapshot(this,"Control packets ");
//using namespace std;
  totalCounter= dataCounter+count.sum;
  avgHops= avgHops / hitCounter;
  if (proBaCounter > 0) EV << "Proactive BA packets: " << proBaCounter << " on node "<< getParentModule()->getName() << endl;
  if (proactiveCounter > 0) EV << "Proactive FA packets: " << proactiveCounter << " on node "<< getParentModule()->getName() << endl;
  if (repairCounter) EV << "Local repair FA packets: " << repairCounter << " on node "<< getParentModule()->getName() << endl;
  // EV << "Control hops: " << controlHops << endl; // too much in no simulation mode
  //EV << "Data packets: " << dataCounter << " drop packets: " << dropCounter << " Total (Control+Data): "<< totalCounter << endl;
  emit(controlHopsSignal, controlHops);
  emit(overheadCostSignal, controlHops2 + controlHops);
  if ((myAddress ==7)|| (myAddress==2)) { // hostX or Kle dest addresses
      EV << getParentModule()->getName() << " Address: "<< myAddress <<" Simulation time: " << simTime()<< " Delay: "<< datarate <<"s" <<" jitter: " << jit << "ms"<< endl;
      EV << "Control packets: (nb (redundant), fa, ba) : " << nbCounter  << " (" << redundant << "), " << faCounter << ", " << baCounter << endl;
      EV << "Data packets: " << dataCounter << " drop packets: " << dropCounter << " Total (Control+Data): "<< totalCounter << endl;
      EV << "Arrived packets: " << hitCounter <<  " Hit ratio: " << double (hitCounter)/ double(1+dataCounter) << endl;
      EV << "Total data packets:"<< numData <<" ; New hit ratio"<< double (hitCounter)/ double(numData) << endl;
      EV << "Average travel time: " << avgTravel / hitCounter << endl;
      EV << "Average hops: " << avgHops << endl;
      EV << "Minimum hops: "<< minHops << endl;
      EV << "Control hops: " << controlHops << " ; control hops2:"<< controlHops+controlHops2 <<endl;
      emit(hitRatioSignal, double (hitCounter)/ double(numData));
      if (mySort->longValue() == 1) // static routing
          EV << "Static routing Conditions: numNodes:"<< numNodes->longValue() << " max hops rate:" << maxHopsRate->doubleValue() << " maxHops:"<< maxHops << " sort:" << mySort->operator int() << endl;
          else {
          EV << "Conditions: Routing table init: " << rTableInit->boolValue() << " numNodes:"<< numNodes->longValue() << " stochastic:"<< stochastic->boolValue() << " Data coef(beta):" << dataCoef->longValue() <<" max hops rate:" << maxHopsRate->doubleValue() << " maxHops:"<< maxHops <<" metric: "<< metrics->longValue() << " pheromone coef:"<< coefPh->doubleValue() << " take 1st path:"<< take1stPath->boolValue() << " flooding: "<< flooding->boolValue() << " evaporation:"<< evaporation->boolValue() << " sort:" << mySort->operator int() <<" ants to App:" << antsToApp->operator bool() << " updateVTtime:" << updateVTime->doubleValue() << " local repair:"<< localRepair->boolValue() << " reply time:"<< replyTime->doubleValue()<<endl;
          EV << "AntWMNet conditions; data coef: " << dataCoef->longValue() << " good diff:" << goodDiffusion->boolValue() << " hello time:" << helloTime->doubleValue() << " hop time:" << hopTime->doubleValue()<<" maxLocalRepFloods:" << maxLocalRepairFlooding->longValue() <<" baPro:" << proBa->boolValue() << " checkTime:"<< checkTime->doubleValue() << " alternateCheck:" << alternateCheck->boolValue() << " cutRange:"<< cutRange->longValue() << " cutRangePro:" << cutRangePro->longValue() << endl;
          }
      if (mySort->longValue() == 4) { // AODV branch
          std::ofstream myfile("aodv-control.csv", myfile.app );

               if (myfile.is_open())
               {
               totalCounter= dataCounter+nbCounter+rreqCounter+rrepCounter+rerrCounter;
               myfile << nbCounter << " " << rreqCounter << " " << rrepCounter << " " << rerrCounter <<  " " <<  rreqCounter+rrepCounter+rerrCounter << " " << dataCounter << " " <<  dropCounter << " " << totalCounter <<  " "  << controlHops << " " << toDoubleCalc(double (hitCounter)/ double(numData)) << " " << toDoubleCalc(avgTravel / double(hitCounter)) << " "<< toDoubleCalc(avgHops) <<"\n";
               // on our sample one packet have no time to reach destination
               myfile.close();
                }
                else { EV << "Unable to open file" << endl; }
          return;
      }
            std::ofstream myfile("control-packet.csv", myfile.app );

              if (myfile.is_open())
              {

                myfile << count.nb << " " <<  proactiveCounter << " " << repairCounter << " " << count.ba << " " << phDiffCounter << " " << count.sum <<  " " <<  faCounter+baCounter+proactiveCounter+repairCounter << " " << dataCounter << " " <<  dropCounter << " " << totalCounter <<  " "  << controlHops << " " << controlHops2+controlHops << " " <<toDoubleCalc(double (hitCounter)/ double(1+dataCounter)) << " " << toDoubleCalc(avgTravel / double(hitCounter)) << " "<< toDoubleCalc(avgHops) <<"\n";
                // on our sample one packet have no time to reach destination
                myfile.close();
              }
              else { EV << "Unable to open file" << endl; }


  } else // other no destination nodes
  {
      if (mySort->longValue() == 4) { // AODV branch no destination nodes
                                    std::ofstream myfile("aodv-control.csv", myfile.app );

                                         if (myfile.is_open())
                                         {
                                         totalCounter= dataCounter+nbCounter+rreqCounter+rrepCounter+rerrCounter;
                                         myfile << nbCounter << " " << rreqCounter << " " << rrepCounter << " " << rerrCounter <<  " " <<  rreqCounter+rrepCounter+rerrCounter << " " << dataCounter << " " <<  dropCounter << " " << totalCounter <<  " "  << controlHops <<"\n";
                                         // on our sample one packet have no time to reach destination
                                         myfile.close();
                                          }
                                          else { EV << "Unable to open file" << endl; }
                                    return;
                                }

    std::ofstream myfile("control-packet.csv", myfile.app );

    if (myfile.is_open())
    {

        myfile << count.nb << " "<<  proactiveCounter << " " << repairCounter<< " " << count.ba << " " << phDiffCounter << " " << count.sum <<  " " << faCounter+baCounter+proactiveCounter+repairCounter << " " <<  dataCounter << " " <<  dropCounter << " " << totalCounter << " " << controlHops << " " << controlHops2+controlHops <<"\n";
        myfile.close();
    }
    else { EV << "Unable to open file" << endl; }
  }
}
