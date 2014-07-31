// 2012 L.Jacob Mariscal Fernández based on Copyright (C) 1992-2008 Andras Varga
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

//
// Models a wireless local area network link layer for wireless mesh networks
//
package node;

simple Wlan80211
{
        parameters:
        int maxQueueSize = default(14); // max queue length in frames; only used if queueModule==""
        double bitrate @unit("bps")= default(2400000bps);
        double basicBitrate @unit("bps") = default(2000000bps);
        int rtsThresholdBytes @unit("B") = default(2346B); // longer messages will be sent using RTS/CTS
        int retryLimit = default(-1); // maximum number of retries per message, -1 means default
        int cwMinData = default(-1); // contention window for normal data frames, -1 means default
        int cwMinMulticast = default(-1); // contention window for multicast messages, -1 means default
        int mtu = default(1500);
        int frameCapacity = default(14); // max number of packets; 0 means no limit
        
        string radioModel = "Ieee80211NewRadioModel";  // specify the radio model responsible for modulation, error correction and frame length calculation
        double snirThreshold @unit("dB") = default(4dB); // if signal-noise ratio is below this threshold, frame is considered noise (in dB)
        string berTableFile = default("");
        string phyOpMode @enum("b","g","a","p") = default("g");
        string WifiPreambleMode @enum("LONG","SHORT") = default("LONG"); // Wifi preambre mode Ieee 2007, 19.3.2
        string ErrorModel @enum("YansModel","NistModel") = default("NistModel");
        int    btSize @unit("b") = default(8192b);// test size frame for Airtime Link Metric
        bool   airtimeLinkComputation = default(false);
        
        bool AutoHeaderSize = default(false); // in the receiver the radio model compute the header size in function of timers and bitrate
        
        int channelNumber = default(0); // channel identifier this radio listens. Works only with simlified management module. Otherwise it scans all the channels as specified in ieee 80211
        double carrierFrequency @unit("Hz") = default(2.4GHz);
        //double bitrate @unit("bps");
        double transmitterPower @unit("mW") = default(20mW); // power used for transmission of messages (in mW)
        double thermalNoise @unit("dBm") = default(-110dBm); // base noise level (dBm)
        double sensitivity @unit("dBm") = default(-85dBm); // received signals with power below sensitivity are ignored
        //string radioModel;  // the radio model implementing the IRadioModel interface (C++). e.g. GenericRadioModel, Ieee80211RadioModel
        
        string NoiseGenerator = default("");
        @display("i=block/ifcard;q=queue");
        @signal[qlen](type="int");
        @signal[busy](type="bool");
        @signal[queueingTime](type="simtime_t");
        @signal[drop](type="int");
        @signal[txBytes](type="long");
        @signal[rxBytes](type="long");
        @statistic[qlen](title="queue length";record=vector?,timeavg,max;interpolationmode=sample-hold);
        @statistic[busy](title="server busy state";record=vector?,timeavg;interpolationmode=sample-hold);
        @statistic[queueingTime](title="queueing time at dequeue";unit=s;interpolationmode=none);
        @statistic[drop](title="dropped packet byte length";unit=bytes;record=vector?,count,sum;interpolationmode=none);
        @statistic[txBytes](title="transmitting packet byte length";unit=bytes;record=vector?,count,sum,histogram;interpolationmode=none);
        @statistic[rxBytes](title="received packet byte length";unit=bytes;record=vector?,count,sum,histogram;interpolationmode=none);
    gates:
        input in;
        output out;
        inout line;
        // input radioIn @labels(AirFrame);
}
