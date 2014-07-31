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
// 2012 L.Jacob Mariscal Fernández based on Copyright (C) 1992-2008 Andras Varga

// wireless link layer header file

#ifndef __ROUTING_WLAN80211_H_
#define __ROUTING_WLAN80211_H_

#include <omnetpp.h>

/**
 * TODO - Generated class
 */
class Wlan80211 : public cSimpleModule
{
    private:
    long frameCapacity;

    cQueue queue;
    cMessage *endTransmissionEvent;

    simsignal_t qlenSignal;
    simsignal_t busySignal;
    simsignal_t queueingTimeSignal;
    simsignal_t dropSignal;
    simsignal_t txBytesSignal;
    simsignal_t rxBytesSignal;

  public:
    Wlan80211();
    virtual ~Wlan80211();

  protected:
    virtual void initialize();
    virtual void handleMessage(cMessage *msg);

    virtual void startTransmitting(cMessage *msg);

    virtual void displayStatus(bool isBusy);

    virtual void updateChannel();

};

#endif
