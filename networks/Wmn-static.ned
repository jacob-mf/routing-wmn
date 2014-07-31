//
//
// L. Jacob Mariscal Fernández 2014 based on Copyright (C) 1992-2008 Andras Varga
//
// This file is distributed WITHOUT ANY WARRANTY. See the file
// `license' for details on this and other legal matters.
//

package networks;

import node.**;

network WmnStatic
{
    parameters:
        double height @prompt("Number of rows") = default(0);
        double width @prompt("Number of columns") = default(0);
        @display("bgb=640,620");
    types:
        channel C extends ned.DatarateChannel
        {
            parameters:

                volatile double jitter @unit(ms) = default(uniform(0ms,6 ms));
                delay = 2ms + convertUnit(jitter, "ms");
                datarate = default(18Mbps);
                @display("ls= grey,0");
                double noise @unit("dB") = default(1dB); // if it's 4dB are considered not understandable signal
        }
        channel C1 extends ned.DatarateChannel
        {
            parameters:

                volatile double jitter @unit(ms) = default(uniform(0ms,7 ms));
                delay = 2ms + convertUnit(jitter, "ms");
                datarate = default(12Mbps);
                @display("ls= grey,0");
                volatile double noise @unit("dB") = default(1dB); // if it's 4dB are considered not understandable signal
        }
//        channel C2 extends ned.DatarateChannel
//        {
//            parameters:
//
//                volatile double jitter @unit(ms) = default(uniform(0ms,9 ms));
//                delay = 3ms + convertUnit(jitter, "ms");
//                datarate = default(1Mbps);
//                @display("ls= grey,0");
//                double noise @unit("dB") = default(6dB); // if it's 4dB are considered not understandable signal
//        }
        channel Cmax extends ned.DatarateChannel // for mesh nodes, aps
        {
            parameters:

                volatile double jitter @unit(ms) = default(uniform(0ms,4 ms));
                delay = 2ms + convertUnit(jitter, "ms");
                datarate = default(24Mbps);
                @display("ls= grey,0");
                volatile double noise @unit("dB") = default(1dB); // if it's 4dB are considered not understandable signal
        }
        channel Cdoc extends ned.DatarateChannel // for documentation, movement
        {
            @display("ls=blue2,2,da");
        }
    submodules:
        ap1: Terminal {
            @display("p= 260,240;r=60,,#707070");
            address = 1;
            gates:
                port[5];
        }
        ap2: Terminal {
            @display("p= 370,240;r=60,,#707070");
            address = 3;

            gates:
                port[5];
        }
        hostA: Terminal {
            address = 4;
            @display("p=220,160;r=60,,#707070");
            gates:
                port[4];
        }
        hostB: Terminal {
            @display("p=300,160;r=60,,#707070");
            address = 5;
            gates:
                port[4];
        }
        hostC: Terminal {
            @display("p=380,160;r=60,,#707070");
            address = 6;
            gates:
                port[4];
        }
        hostX: Terminal {
            // mobile terminal
            @display("i=/device/cellphone;p=379,62;r=60,,#707070");
            @display("i=device/cellphone");
            address = 7;
            gates:
                port[5];
        }
        hostD: Terminal {
            @display("p=220,333;r=60,,#707070");
            address = 8;
            gates:
                port[4];
        }
        hostE: Terminal {
            @display("p=290,333;r=60,,#707070");
            address = 9;
            gates:
                port[4];
        }
        hostF: Terminal {
            @display("p=370,333;r=60,,#707070");
            address = 10;
            gates:
                port[6];

        }
        node[height*width]: Terminal {
            parameters:
                address = 11+index;
                @display("p=20,40,m,$width,19,40");
            gates:
                port[4];
        }
        //        hostX1: Terminal {
        //            // mobile terminal
        //            @display("i=/device/cellphone;p=462,120;r=60,,#707070");
        //            @display("i=device/cellphone");
        //            address = 7;
        //            gates:
        //                port[4];
        //        }
//        hostX2: Terminal {
//            // mobile terminal
//            @display("i=/device/cellphone;p=469,322;r=60,,#707070");
//            @display("i=device/cellphone");
//            address = 7;
//            gates:
//                port[4];
//        }
    //        hostX3: Terminal {
    //            // mobile terminal
    //            @display("i=/device/cellphone;p=462,329;r=60,,#707070");
    //            @display("i=device/cellphone");
    //            address = 7;
    //            gates:
    //                port[4];
    //        }
    connections allowunconnected:
        hostA.port[2] <--> C <--> hostB.port[3];
        hostB.port[2] <--> C <--> hostC.port[3];
        hostA.port[0] <--> C <--> ap1.port[1];
        hostB.port[0] <--> C <--> ap1.port[3];
        hostB.port[1] <--> C1 <--> ap2.port[4];
        ap1.port[2] <--> Cmax <--> ap2.port[3];
        hostC.port[0] <--> C <--> ap2.port[1];
        hostX.port[0] <--> C <--> hostC.port[1];
        hostD.port[2] <--> C <--> hostE.port[3];
        hostE.port[2] <--> C <--> hostF.port[3];
        hostD.port[1] <--> C1 <--> ap1.port[0];
        hostE.port[1] <--> C1 <--> ap1.port[4];
        hostF.port[1] <--> C1 <--> ap2.port[0];
       // hostX.port[3] <--> C2 <--> ap2.port[2]; // initially out of range
       // hostX.port[4] <--> C2 <--> hostF.port[5]; // initially not connected
        for i=0..height-1, for j=0..width-1 {
            node[i*width+j].port[0] <--> C <--> node[(i+1)*width+j].port[1] if i!=height-1;
            node[i*width+j].port[2] <--> C <--> node[(i*width+j)+1].port[3] if j!=width-1;
        }
        node[(height*width)-1].port[2] <--> C <--> hostD.port[3] if (height*width)>0;
        node[((height-2)*width)-1].port[2] <--> C <--> hostA.port[3] if (height*width)>=3;

}


