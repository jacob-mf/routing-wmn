/*
 * UDPtrans.cc
 *
 *  Created on: 10/10/2013
 *      Author: luis-jacob
 */
// L. Jacob Mariscal Fernández 2013-4
// This file is distributed WITHOUT ANY WARRANTY. See the file
// 'license' for details on this and other legal matters.

// UDP Transport layer implementation, CRC check support


#include <string.h>
#include "Packet_m.h"
#include <omnetpp.h>

class UDPtrans : public cSimpleModule
{

    protected:
    // statistics
            int numSent;
            int numPassedUp;
            int numDroppedWrongPort;
            int numDroppedBadChecksum;

            simsignal_t rcvdPkSignal;
            simsignal_t sentPkSignal; // no works with static
            simsignal_t passedUpPkSignal;
            simsignal_t droppedPkWrongPortSignal;
            simsignal_t droppedPkBadChecksumSignal;
            // process UDP packets coming from IP
            virtual void processUDPPacket(cMessage *udpPacket);

            // process packets from application
            virtual void processPacketFromApp(cMessage *appData);

            // process commands from application
            virtual void processCommandFromApp(cMessage *msg);
            // mandatory procedures
            virtual void initialize();
            virtual void handleMessage(cMessage *msg);

    public:
         UDPtrans() {}
         virtual ~UDPtrans();
};

Define_Module(UDPtrans);


UDPtrans::~UDPtrans()
{
    //for (SocketsByIdMap::iterator i=socketsByIdMap.begin(); i!=socketsByIdMap.end(); ++i)
      //  delete i->second;
   // delete UDPtrans(); // works?
}
void UDPtrans::initialize()
{
       numSent = 0;
       numPassedUp = 0;
       numDroppedWrongPort = 0;
       numDroppedBadChecksum = 0;
       WATCH(numSent);
       WATCH(numPassedUp);
       WATCH(numDroppedWrongPort);
       WATCH(numDroppedBadChecksum);
       rcvdPkSignal = registerSignal("rcvdPk");
       sentPkSignal = registerSignal("sentPk");
       passedUpPkSignal = registerSignal("passedUpPk");
       droppedPkWrongPortSignal = registerSignal("droppedPkWrongPort");
       droppedPkBadChecksumSignal = registerSignal("droppedPkBadChecksum");
}

void UDPtrans::handleMessage(cMessage *msg)
{
    // received from IP layer
    if (msg->arrivedOn("ipIn")) // || msg->arrivedOn("ipv6In"))
    {
      //  if  (msg->isPacket()) //(dynamic_cast<UDPPacket *>(msg) != NULL)
            processUDPPacket(msg); // pasar de transformar en paquete UDP ??
          //  processICMPError(PK(msg));  // assume it's an ICMP error
    }
    else // received from application layer
    {
       if (msg->getKind() < 4) // kind = 0 Data, 1 Hello_Msg, 2 Forward_Ant, 3 Backward_Ant in future better FA, BA only from Net level to Net level
           processPacketFromApp(msg);//;
        else
            processCommandFromApp(msg);
    }

  // if (ev.isGUI())
        //updateDisplayString(); // error? need an import
}
void UDPtrans::processCommandFromApp(cMessage *msg) //not implemented
{
    delete msg;
}

void UDPtrans::processPacketFromApp(cMessage *appData)
{
    emit(sentPkSignal, appData);
    send(appData, "ipOut");
    numSent++;
}
void UDPtrans::processUDPPacket(cMessage *udpPacket)
{
    emit(rcvdPkSignal, udpPacket);

    // simulate checksum: discard packet if it has bit error
    EV << "Packet " << udpPacket->getName() << " received from network, from port " << udpPacket->getArrivalGateId() << "\n";

    if (udpPacket->hasPar("CRC Error"))
    {
        EV << "Packet has bit error, discarding\n";
        emit(droppedPkBadChecksumSignal, udpPacket);
        numDroppedBadChecksum++;
        delete udpPacket;

        return;
    }
    emit(passedUpPkSignal, udpPacket);
    send(udpPacket, "appOut");
    numPassedUp++;
}

