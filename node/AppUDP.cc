//
// This file is part of an OMNeT++/OMNEST simulation example.
//
// 2013-4 L.Jacob Mariscal Fernández based on Copyright (C) 1992-2008 Andras Varga
//
// This file is distributed WITHOUT ANY WARRANTY. See the file
// `license' for details on this and other legal matters.
//

#ifdef _MSC_VER
#pragma warning(disable:4786)
#endif

#include <vector>
#include <omnetpp.h>
#include "Packet_m.h"
#include <iostream>
#include <fstream>

/**
 * Generates traffic for the network.
 */
class AppUDP : public cSimpleModule
{
  private:
    // configuration
    int myAddress;
    std::vector<int> destAddresses;
    std::vector<int> srcAddresses;
    cPar *sendIATime;
    cPar *sendFATime;
    cPar *sendFirstFA;
    cPar *stopFA;
    cPar *setupTime;
    cPar *packetLengthBytes;
    cPar *flooding;
    cPar *maxArray;
    cPar *mySort; // MySort(defined by sort in omnet.ini) specifies sort of routing as 1 : Static , 2 : AntWmNet , 3 : CPANT , 4 : AODV

    // state
    cMessage *generateFAnt;
    cMessage *dataGenerate;
    long pkCounter,numSent,numReceived,numFA,numData;
    long faR,baR,dataR,helloR;

    // signals
    simsignal_t endToEndDelaySignal;
    simsignal_t hopCountSignal;
    simsignal_t sourceAddressSignal;
    simsignal_t pkCounterSignal;
    simsignal_t rcvdPkSignal;
    simsignal_t sentPkSignal;

  public:
    AppUDP();
    virtual ~AppUDP();

  protected:
    virtual void initialize();
    virtual void handleMessage(cMessage *msg);
    virtual void finish();
};

Define_Module(AppUDP);


AppUDP::AppUDP()
{
    generateFAnt = NULL;
    dataGenerate = NULL;
}

AppUDP::~AppUDP()
{
    cancelAndDelete(generateFAnt);
    cancelAndDelete(dataGenerate);
}

void AppUDP::initialize()
{
    myAddress = par("address");
    packetLengthBytes = &par("packetLength");
    sendIATime = &par("sendIaTime");  // volatile parameter
    sendFATime = &par("sendFaTime");
    sendFirstFA = &par("sendFirstFA");
    setupTime=&par("setupTime");
    stopFA=&par("stopFA");
    maxArray=&par("maxArray");
    flooding = &par("flooding");
    mySort =  &par("sort") ;
    pkCounter = 0;
    numSent = 0;
    numReceived = 0;numFA=0; numData=0;
    dataR =0; faR=0; baR=0; helloR=0;
    WATCH(numSent);
    WATCH(numReceived);
    sentPkSignal = registerSignal("sentPk");
    rcvdPkSignal = registerSignal("rcvdPk");
    endToEndDelaySignal = registerSignal("endToEndDelay");
    hopCountSignal =  registerSignal("hopCount");
    sourceAddressSignal = registerSignal("sourceAddress");
    pkCounterSignal = registerSignal("pkCounter");

    WATCH(pkCounter);
    WATCH(myAddress);

    const char *destAddressesPar = par("destAddresses");
    cStringTokenizer tokenizer(destAddressesPar);
    const char *token;
    while ((token = tokenizer.nextToken())!=NULL)
        destAddresses.push_back(atoi(token));

    if (destAddresses.size() == 0)
        throw cRuntimeError("At least one address must be specified in the destAddresses parameter!");

    const char *srcAddressesPar = par("srcAddresses");
    cStringTokenizer tokenizerSrc(srcAddressesPar);
    const char *tokenSrc;
    while ((tokenSrc = tokenizerSrc.nextToken())!=NULL)
            srcAddresses.push_back(atoi(tokenSrc));

    if (srcAddresses.size() == 0)
          throw cRuntimeError("At least one address must be specified in the srcAddresses parameter!");

    // Searching  myAdress in srcAddresses vector
    bool found = false; unsigned int i = 0;
    while ((! found) && (i < srcAddresses.size())  )  {
        if (srcAddresses[i] == myAddress) {
            found = true;
            EV << "Found valid source address: "<< myAddress << " Step: "<< i << " Val: "<< srcAddresses[i] << endl;
        }
        i++;
    }

    if (found) {
        if (mySort->longValue() == 2) { // AntWMNet selected
                generateFAnt = new cMessage("nextFAnt");
                scheduleAt(simTime()+sendFirstFA->doubleValue(),generateFAnt);
                EV << "Generate packet FA from address: "<< myAddress << endl;
        }
    dataGenerate= new cMessage("nextDataPacket");
    EV << "Generate data packet from address: " << myAddress << endl;
    scheduleAt(sendIATime->doubleValue(),dataGenerate); // also other choice is set-up time...
    }
}
void AppUDP::finish()
{
    recordScalar("packets sent", numSent);
    recordScalar("packets received", numReceived);
    if ((myAddress == srcAddresses[0])  || (myAddress == destAddresses[0]) ) { // || (myAddress== srcAddresses[1]) in source or destination node
        EV <<"App UDP module statistics in source/destination node: " << getParentModule()->getName() << endl;
        EV << "Total packets received: " << numReceived << " data rcvd: " << dataR << " hello rcvd: " << helloR << " fa rcvd: " << faR << " ba rcvd: "<< baR <<" Total packets: " << pkCounter << endl;
        EV << "Total packets sent: "<< numSent << " Data packets sent: "<< numData << " FA packets sent: "<< numFA <<endl;
        std::ofstream myfile("app-packets.csv", myfile.app );
        if (myfile.is_open())
        {
        myfile << getParentModule()->getName() << "," <<numReceived << "," << dataR << "," << faR  << "," << baR <<  "," << pkCounter << "," <<numSent << "," << numData << "," << numFA  << "," <<  numFA - baR <<  endl;

        myfile.close();
        } else { EV << "Unable to open file" << endl; }

    }
}


void AppUDP::handleMessage(cMessage *msg)
{
    // snapshot(this,"1st sample");

    if (msg == generateFAnt) // FA branch only
    {
        // Sending packet
        int destAddress = destAddresses[intuniform(0, destAddresses.size()-1)];
        //bool setup = false;
        char pkname[40];
        //double comp = simTime().dbl();
        //if (comp < (setupTime->doubleValue()) ) { // setup here no sense...
            //sprintf(pkname,"FA-%d-to-%d-#%ld", myAddress, destAddress, pkCounter++);
            // EV << "generating Forward Ant " << pkname << endl;
            //setup = true;

        //} else {
        sprintf(pkname,"FA-%d-to-%d-#%ld", myAddress, destAddress, pkCounter++);
        //EV << "generating packet " << pkname << endl;
        //}
        Packet *pk = new Packet(pkname);
        emit(pkCounterSignal, pkCounter);
        //if (setup) {
        pk->setKind(2);  // Forward Ant packet
        pk->setDisplayString("i=msg/ant3_s,green"); // FA displays as green ant
        //}
        pk->setByteLength(packetLengthBytes->longValue());
        pk->setSrcAddr(myAddress);
        pk->setTransientNodesArraySize(maxArray->longValue()); // initialize to safe value (aware of max409 nodes network) beware memory errors!
        if (flooding->boolValue() ) { // new technique, send FA to all destinations just one time, to avoid overhead cost
                    //setup= false; // skip setup time
                    pk->setDestAddr(destAddresses[0]);
                    sprintf(pkname,"FA-%d-to-%d-#%ld", myAddress, destAddresses[0], pkCounter); // ya se sumo antes pkCounter
                    EV << "generating first Forward Ant (Flooding) " << pkname << endl;
                    pk->setName(pkname);
                    numFA++;
                    emit(sentPkSignal,pk);
                    send(pk,"out");
                    numSent++;
                    for (unsigned int i=1 ; i < destAddresses.size() ; i++) {

                        emit(pkCounterSignal, pkCounter); // new FA (flooding)
                        sprintf(pkname,"FA-%d-to-%d-#%ld",myAddress,destAddresses[i],pkCounter++);
                        Packet *pk2 = new Packet (pkname);
                        pk2->setKind(2);
                        pk2->setDestAddr(destAddresses[i]);
                        pk2->setName(pkname);
                        pk2->setByteLength(packetLengthBytes->longValue());
                        pk2->setSrcAddr(myAddress);
                        pk2->setDisplayString("i=msg/ant3_s,green"); // FA displays as green ant
                        pk2->setTransientNodesArraySize(maxArray->longValue()); // initialize to safe value
                        EV << "generating next Forward Ant (Flooding) " << pkname << endl;
                        numSent++;numFA++;
                        emit(sentPkSignal,pk2);
                        send(pk2 ,"out");
                    }
                    if (ev.isGUI()) getParentModule()->bubble("Generating Forward Ants... (flooding!)");
                    return;
                }
        pk->setDestAddr(destAddress);
        EV << "generating Forward Ant " << pkname << endl;
        numSent++;numFA++;
        emit(sentPkSignal,pk);
        send(pk,"out");
        //if () {
        if (ev.isGUI()) getParentModule()->bubble("Generating Forward Ant...");
        if (simTime().dbl() <= stopFA->doubleValue())
            scheduleAt(simTime() + sendFATime->doubleValue(), generateFAnt); // maybe better with new Ant Interval time variable
        //}else {
        //scheduleAt(simTime() + sendIATime->doubleValue(), generatePacket);
        //if (ev.isGUI()) getParentModule()->bubble("Generating packet...");
        //}
    }
    else
        if (msg == dataGenerate) // data Packets generate
            {
                bool setup = false;
                scheduleAt(simTime()+sendIATime->doubleValue(),dataGenerate); // next packet
                double comp = simTime().dbl();
                if (comp < (setupTime->doubleValue()) ) {
                    setup = true; // skip send data packets
                    return;
                } else {
                    int destAddress = destAddresses[intuniform(0, destAddresses.size()-1)];
                    char pkname[40];
                    sprintf(pkname,"pk-%d-to-%d-#%ld", myAddress, destAddress, pkCounter++);
                    Packet *pk = new Packet(pkname);
                    pk->setByteLength(packetLengthBytes->longValue());
                    pk->setSrcAddr(myAddress);
                    pk->setDestAddr(destAddress);
                    pk->setTransientNodesArraySize(maxArray->longValue()); // initialise to safe value
                                                                // beware high values(100<) cause critical memory errors
                    EV << "generating packet " << pkname << endl;
                    numSent++;numData++;
                    emit(sentPkSignal,pk);
                    send(pk,"out");
                }
                setup = setup;
            } else
    {
        // Handle incoming packet
        if (msg->isSelfMessage()){
            EV << "It supposed auto messages not coming here, error!!" << endl;
        } else {
        Packet *pk = check_and_cast<Packet *>(msg);
        EV << "received packet " << pk->getName() << " after " << pk->getHopCount() << "hops" << endl;
        pk->setTravelTime(simTime()- pk->getCreationTime());
        emit(endToEndDelaySignal, simTime() - pk->getCreationTime());
        EV << "In " << simTime() - pk->getCreationTime() << " seconds."<< endl;
        emit(hopCountSignal, pk->getHopCount());
        emit(sourceAddressSignal, pk->getSrcAddr());
        emit(rcvdPkSignal,pk);
        int temp = pk->getKind();
        numReceived++;
        delete pk;
        switch (temp) { //statistics
            case 0: dataR++;break;
            case 1: helloR++;break;
            case 2: faR++;break;
            case 3: baR++;break;
        }
        //if ((temp == 0) || (temp == 3)) { // if received packet is Data or Backward Ant (neither Hello msg nor FA)
        // generate new packet from SrcAdress only
        //generatePacket = new cMessage("nextPacket");
        //scheduleAt(simTime(), generatePacket); // Immediately scheduled
        //EV << "Generate packet from address: "<< myAddress << endl;
        //endToEndDelaySignal = registerSignal("endToEndDelay");
        //hopCountSignal =  registerSignal("hopCount");
        //sourceAddressSignal = registerSignal("sourceAddress");
        //}
        if (ev.isGUI())
        {
            if (temp == 0) { // only for data packets
            getParentModule()->getDisplayString().setTagArg("i",1,"green");
            }
            getParentModule()->bubble("Arrived!");
            getParentModule()->getDisplayString().setTagArg("i",1,""); // better to update to normal estate after sometime
        }
        }
    }
}

