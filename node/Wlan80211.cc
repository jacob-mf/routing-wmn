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

// simple wireless link layer implementation
// channel mode with delay, jitter; no noise verification here

#include "Wlan80211.h"
#include <stdio.h>
#include <string.h>
#include <omnetpp.h>

Define_Module(Wlan80211);

Wlan80211::Wlan80211()
{
    endTransmissionEvent = NULL;
}

Wlan80211::~Wlan80211()
{
    cancelAndDelete(endTransmissionEvent);
}

void Wlan80211::initialize()
{
    queue.setName("queue");
        endTransmissionEvent = new cMessage("endTxEvent");

        frameCapacity = par("frameCapacity");

        qlenSignal = registerSignal("qlen");
        busySignal = registerSignal("busy");
        queueingTimeSignal = registerSignal("queueingTime");
        dropSignal = registerSignal("drop");
        txBytesSignal = registerSignal("txBytes");
        rxBytesSignal = registerSignal("rxBytes");

        emit(qlenSignal, queue.length());
        emit(busySignal, 0);
}

void Wlan80211::startTransmitting(cMessage *msg)
{
    if (ev.isGUI()) displayStatus(true);

    EV << "Starting transmission of " << msg << endl;
    int64 numBytes = check_and_cast<cPacket *>(msg)->getByteLength();
    send(msg, "line$o");

    emit(txBytesSignal, (long)numBytes);

    // Schedule an event for the time when last bit will leave the gate.
    simtime_t endTransmission = gate("line$o")->getTransmissionChannel()->getTransmissionFinishTime();
    scheduleAt(endTransmission, endTransmissionEvent);
    updateChannel(); // update Channel parameter delay with new jitter in every new transmission
}

void Wlan80211::handleMessage(cMessage *msg)
{
    if (msg==endTransmissionEvent)
        {
            // Transmission finished, we can start next one.
            EV << "Transmission finished.\n";
            if (ev.isGUI()) displayStatus(false);
            if (queue.empty())
            {
                emit(busySignal, 0);
            }
            else
            {
                msg = (cMessage *) queue.pop();
                emit(queueingTimeSignal, simTime() - msg->getTimestamp());
                emit(qlenSignal, queue.length());
                startTransmitting(msg);
            }
        }
        else if (msg->arrivedOn("line$i"))
        {
            // pass up
            emit(rxBytesSignal, (long)check_and_cast<cPacket *>(msg)->getByteLength());
            updateChannel(); // update Channel parameter delay with new jitter
            send(msg,"out");
        }
        else // arrived on gate "in"
        {
            if (endTransmissionEvent->isScheduled())
            {
                // We are currently busy, so just queue up the packet.
                if (frameCapacity && queue.length()>=frameCapacity)
                {
                    EV << "Received " << msg << " but transmitter busy and queue full: discarding\n";
                    emit(dropSignal, (long)check_and_cast<cPacket *>(msg)->getByteLength());
                    delete msg;
                }
                else
                {
                    EV << "Received " << msg << " but transmitter busy: queueing up\n";
                    msg->setTimestamp();
                    queue.insert(msg);
                    emit(qlenSignal, queue.length());
                }
            }
            else
            {
                // We are idle, so we can start transmitting right away.
                EV << "Received " << msg << endl;
                emit(queueingTimeSignal, 0.0);
                startTransmitting(msg);
                emit(busySignal, 1);
            }
        }
}

void Wlan80211::updateChannel() // update channel with current jitter value
{
    int j = getParentModule()->gateSize("port"); // get port size
    double jitter =0;
    for (int i=0; i<j; i++) {
                 if ( getParentModule()->gate("port$i",i)->isConnected()) {
                      jitter = getParentModule()->gate("port$i",i)->getIncomingTransmissionChannel()->par("jitter").doubleValue();
                      jitter = jitter / 1000; // convert jitter to seconds (defined in ms)
                      getParentModule()->gate("port$i",i)->getIncomingTransmissionChannel()->par("delay").setDoubleValue(0.002+ jitter); // watchout! delay defined in seconds
                      getParentModule()->gate("port$o",i)->getTransmissionChannel()->par("delay").setDoubleValue(0.002+jitter);
                                                                              }
                            }
}

void Wlan80211::displayStatus(bool isBusy)
{
    getDisplayString().setTagArg("t",0, isBusy ? "transmitting" : "idle");
    getDisplayString().setTagArg("i",1, isBusy ? (queue.length()>=3 ? "red" : "yellow") : "");
}

