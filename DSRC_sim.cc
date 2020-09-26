// Including the required libraries
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/udp-server.h"
#include "ns3/netanim-module.h"
#include "ns3/flow-monitor.h"
#include "ns3/flow-monitor-helper.h"
#include <iostream>
#include "ns3/mobility-module.h"
#include "ns3/mobility-helper.h"
#include "ns3/wifi-module.h"
#include "ns3/stats-module.h"
#include "ns3/wave-module.h"
#include "ns3/netanim-module.h"
#include "ns3/wave-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/core-module.h"
#include "ns3/seq-ts-header.h"
#include "ns3/gpsr-module.h"
#include <ns3/buildings-helper.h>
#include <ns3/buildings-module.h>

// Declare ns3 object
using namespace ns3;

// Declare the required parameters for the simulation
//double totalpkt = 0;
double delay = 0;       // For delay
int64_t uecounts = 10;  // Number of nodes
double throughput;      // For throughput
double pckrcv =0;       // For number of received packets
int counts;             // Variable for counter

//WifiTxVeaMpdu

int seq = 0;    // seq number

 bool
 Receive (Ptr<NetDevice> dev, Ptr<const Packet> pkt, uint16_t mode, const Address &sender)
 {
   ++counts;
   // Computing Throughput efficiency here
   pckrcv += pkt->GetSize();
   throughput = (pckrcv * counts * 8)/ 1e6;

   SeqTsHeader seqTs;
   pkt->PeekHeader (seqTs);
    delay += Now ().GetSeconds () - seqTs.GetTs ().GetSeconds ();
   std::cout << "received a packet: " << std::endl
             << "  sequence = " << seqTs.GetSeq () << ","
             << "  counts = " << counts
             << "  Recv packets = " << pckrcv << "bytes"
             << "  avg delay = "  << delay << "s,"
             << "  Throughput eff = " << throughput << std::endl;
//<< "  protocol = 0x" << std::hex << mode << std::dec  << std::endl;
   return true;
 }


void send(NetDeviceContainer wavenetdev )
{
    uint16_t protocol = 0x88dc;


		  Ptr <WaveNetDevice> wavedev = DynamicCast <WaveNetDevice> (wavenetdev.Get (0));

                  TxInfo tx;
		  tx.channelNumber = CCH;
		  tx.txPowerLevel = 7;
                   tx.dataRate = WifiMode ("OfdmRate6MbpsBW10MHz");
	          // tx.priority = 5; //check the pcap file for the TID value in the Wifi MAC header
                    Ptr <Packet> pck = Create<Packet> (1000);     // Packet size
                   SeqTsHeader seqTs;
  		   seqTs.SetSeq(seq++);
		   pck->AddHeader(seqTs);
                   wavedev->SendX(pck, wavenetdev.Get(uecounts-1)->GetAddress(), protocol, tx);

   Simulator::Schedule ( Seconds (1.2) ,&send, wavenetdev);
}

// Main Function
int
main (int argc, char *argv[])
{


    // Defining variable for simulation time
    double sim_time = 50;

  // Configuration of the PHY layer of the DSRC channel

  YansWifiChannelHelper dsrcchannel;
  dsrcchannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
  dsrcchannel.AddPropagationLoss ("ns3::NakagamiPropagationLossModel");


  YansWifiPhyHelper dsrcPhyHelper = YansWifiPhyHelper::Default();
  dsrcPhyHelper.SetChannel(dsrcchannel.Create());
  dsrcPhyHelper.Set ("TxPowerStart", DoubleValue (33) );    // Set transmitter power for start
  dsrcPhyHelper.Set ("TxPowerEnd", DoubleValue (33) );

  // Configuration of the DSRC MAC layer with QOS
  QosWaveMacHelper dsrcMac = QosWaveMacHelper::Default ();
  WaveHelper waveHelper = WaveHelper::Default ();

  waveHelper.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
  						"DataMode", StringValue ("OfdmRate6MbpsBW10MHz"	),
  						"ControlMode",StringValue ("OfdmRate6MbpsBW10MHz"),
  						"NonUnicastMode", StringValue ("OfdmRate6MbpsBW10MHz"));


    // Declaration of parameters
    NodeContainer uenodes;
    uenodes.Create(uecounts);
    // Modeling obstacles as buildings for the simulation
    Ptr < Building > building;
    building = Create<Building> ();
    building->SetBoundaries (Box (20.0, 40.0,   // Define co-ordinates of the building
                                0.0, 20.0,
                                0.0, 40.0));
    building->SetBuildingType (Building::Office);   // Define building type
    building->SetExtWallsType (Building::ConcreteWithWindows);  // Model the type of wall for adding complexity in obstacles
    building->SetNFloors (3);   // Define number of floors
    building->SetNRoomsX (2);   // Define number of rooms in X direction
    building->SetNRoomsY (3);   // Define number of rooms in Y direction

   // Position allocation for the nodes in the topology
    MobilityHelper uemobility;
    uemobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    uemobility.Install(uenodes);

    BuildingsHelper::Install (uenodes);
   for (uint8_t i =0; i < uecounts; i++)
   {

	uenodes.Get (i)->GetObject<MobilityModel> ()->SetPosition (Vector (35 + 40*i, 0.0, 2.0));    // Position vector for the node
	uenodes.Get(i)->GetObject<ConstantVelocityMobilityModel>()->SetVelocity(Vector(30.0,0.0,0.0)); // Velocity vector for the node
    }

    //Allocation of IP addresses to devices
    InternetStackHelper stack;
    stack.Install (uenodes);
    NetDeviceContainer wavenetdev = waveHelper.Install (dsrcPhyHelper, dsrcMac, uenodes);

    GpsrHelper gpsr;
    Ipv4ListRoutingHelper list;
    list.Add (gpsr,1);
   //InternetStackHelper stack;
    stack.SetRoutingHelper (list);

   // Receive call back
  // for (uint32_t i = 0; i != wavenetdev.GetN (); ++i)
    // {
       Ptr<WaveNetDevice> device = DynamicCast<WaveNetDevice> (wavenetdev.Get (uecounts-1));
       device->SetReceiveCallback (MakeCallback (&Receive));
   //  }

    Ipv4AddressHelper address;
    address.SetBase ("10.1.5.0", "255.255.255.0");
    Ipv4InterfaceContainer waveInterfaces = address.Assign (wavenetdev);

   // Send Wave Messages
   send( wavenetdev);

   //Invoke Rx packet anytime it is called upon

   //Config::Connect("/NodeList/*/DeviceList/*/$ns3::WaveNetDevice/PhyEntities/*/MonitorSnifferRx", MakeCallback (&Rx) );

   // Flow monitor files get generated by this section
   Ptr<FlowMonitor> flowmonitor;
   FlowMonitorHelper flowhelper;
   flowmonitor = flowhelper.InstallAll();
   double pcksent = ((sim_time+1 - 0.05)/0.05) * 1000;

   std::cout << "The total packets sent is: " << pcksent << std::endl;

    Simulator::Stop(Seconds(sim_time+1));
    Simulator::Run();
    flowmonitor->SerializeToXmlFile("mmWaveVanetSims.xml", true, true);
    Simulator::Destroy();

    return 0;

}
