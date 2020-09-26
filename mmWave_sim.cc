/*This file contains the code required to evaluate the Performance of 5G wireless Communication for Non-Safety Applications in VANET with propagation frequency of 28GHz */

#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/mmwave-helper.h"
#include "ns3/buildings-module.h"
#include "ns3/buildings-helper.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/flow-monitor.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/mmwave-propagation-loss-model.h"
#include "ns3/config-store.h"
#include "ns3/mmwave-point-to-point-epc-helper.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/seq-ts-header.h"
#include "ns3/epc-helper.h"
#include <ns3/buildings-helper.h>
#include <ns3/buildings-module.h>
#include "ns3/olsr-helper.h"

double sim_time = 5;
double i = 0.4;
int counts = 1;
using namespace ns3;
using namespace mmwave;


NS_LOG_COMPONENT_DEFINE ("mmWaveVanetSim");

Ptr<PacketSink> sink;
uint64_t lastTotalRx;

// Function to calculate throughput each 200ms
void CalculateThroughput ()
{
  Time now = Simulator::Now ();
  double cur = (sink->GetTotalRx ()) * (double) 8 / 1e4;

  std::cout << now.GetSeconds () << ": \t" << cur << " Mbit/s, \tTotal Rx : " << sink->GetTotalRx() << std::endl;

  //lastTotalRx = sink->GetTotalRx ();
  Simulator::Schedule (MilliSeconds (200), &CalculateThroughput);
}

int main (int argc, char *argv[])
{

    //CommandLine cmd;
    //cmd.Parse(argc, argv);
    LogComponentEnable("mmWaveVanetSim", LOG_LEVEL_INFO);

    //Declaration of parameters
    NodeContainer uenodes;
    NodeContainer enbnodes;
    int64_t uecounts = 10;
    int64_t enbcounts = 2;
    
     Ptr < Building > building;
    building = Create<Building> ();
    building->SetBoundaries (Box (20.0, 40.0,
                                0.0, 20.0,
                                0.0, 40.0));
    building->SetBuildingType (Building::Office);
    building->SetExtWallsType (Building::ConcreteWithWindows);
    building->SetNFloors (3);
    building->SetNRoomsX (2);
    building->SetNRoomsY (3);
  
    double subframe_per = 100;
    int8_t sym_val = 24;
 
   
    bool smallscalefad = true;
    double channelfreq = 28e9;
    std::string channelcond = "n";
    Ipv4Address remoteHostAddr;
   
    /****Configuration of the PHYSICAL & MAC layer Parameters of the mmWave Channel****/

    Config::SetDefault ("ns3::MmWaveAmc::Ber", DoubleValue (0.001));
    Config::SetDefault ("ns3::MmWaveBeamforming::SmallScaleFading", BooleanValue (smallscalefad));
    Config::SetDefault ("ns3::MmWaveBeamforming::FixSpeed", BooleanValue (true));
    Config::SetDefault ("ns3::MmWaveHelper::ChannelModel",StringValue ("ns3::MmWave3gppChannel"));
    Config::SetDefault ("ns3::MmWaveHelper::PathlossModel",StringValue ("ns3::MmWave3gppPropagationLossModel"));
    Config::SetDefault ("ns3::MmWave3gppPropagationLossModel::ChannelCondition", StringValue (channelcond ));
    Config::SetDefault ("ns3::MmWave3gppPropagationLossModel::Scenario", StringValue ("UMa")); //Urban Micro
    Config::SetDefault ("ns3::MmWave3gppPropagationLossModel::Shadowing", BooleanValue (true)); 
    Config::SetDefault ("ns3::MmWave3gppChannel::UpdatePeriod", TimeValue (MilliSeconds (200)));
    Config::SetDefault ("ns3::MmWave3gppChannel::DirectBeam", BooleanValue (true)); //Direct beam of the BS to the receiver
    Config::SetDefault ("ns3::MmWavePhyMacCommon::CenterFreq", DoubleValue(channelfreq));
    Config::SetDefault ("ns3::MmWaveFlexTtiMacScheduler::HarqEnabled", BooleanValue (true));
    Config::SetDefault ("ns3::MmWaveFlexTtiMaxWeightMacScheduler::HarqEnabled", BooleanValue (true));
    Config::SetDefault ("ns3::MmWaveFlexTtiMaxWeightMacScheduler::SymPerSlot", UintegerValue (6));
    Config::SetDefault ("ns3::MmWavePhyMacCommon::ResourceBlockNum", UintegerValue(1));
    Config::SetDefault ("ns3::MmWavePhyMacCommon::ChunkPerRB", UintegerValue(72));
    Config::SetDefault ("ns3::MmWavePhyMacCommon::SymbolsPerSubframe", UintegerValue(sym_val));
    Config::SetDefault ("ns3::MmWavePhyMacCommon::SubframePeriod", DoubleValue (subframe_per));

    //Create an object instance of the mmWave channel
    Ptr<MmWaveHelper> mmWaveObj = CreateObject<MmWaveHelper>();
    mmWaveObj->SetSchedulerType ("ns3::MmWaveFlexTtiMaxWeightMacScheduler");
    mmWaveObj->Initialize();

    //Setting up the Epc Helper (Class instance to facilitate packet exchange with the PDN)
    Ptr<MmWavePointToPointEpcHelper>  epcHelper = CreateObject<MmWavePointToPointEpcHelper> ();
    mmWaveObj->SetEpcHelper (epcHelper);

    ConfigStore inputConfig;
    inputConfig.ConfigureDefaults ();

    //Creation of the Packet Gateway
    Ptr<Node> pgw = epcHelper->GetPgwNode();
    
    //Configuration of remote node
    NodeContainer remoteHostContainer;
    remoteHostContainer.Create (1);
    Ptr<Node> remoteHost = remoteHostContainer.Get (0);
    InternetStackHelper internet;
    internet.Install (remoteHostContainer);

    //Connection of the packetgateway to the remote nodes
    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute ("DataRate", DataRateValue (DataRate ("1Gb/s")));
    p2ph.SetDeviceAttribute ("Mtu", UintegerValue (1200));
    p2ph.SetChannelAttribute ("Delay", TimeValue (MicroSeconds (1)));
    NetDeviceContainer internetDevices = p2ph.Install (pgw, remoteHost);
    Ipv4AddressHelper ipv4h;
    ipv4h.SetBase ("10.0.0.0", "255.0.0.0");
    Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign (internetDevices);
    remoteHostAddr = internetIpIfaces.GetAddress (1);
    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    Ptr<Ipv4StaticRouting> remoteHostStaticRouting = ipv4RoutingHelper.GetStaticRouting (remoteHost->GetObject<Ipv4> ());
    remoteHostStaticRouting->AddNetworkRouteTo (Ipv4Address ("7.0.0.0"), Ipv4Mask ("255.0.0.0"), 1);
    

    //Create the user equipment (nodes) and the base stations (evolved node base)
    uenodes.Create(uecounts);
    enbnodes.Create(enbcounts);
    
    MobilityHelper enbmobility;
    enbmobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    enbmobility.Install(enbnodes);
	
	enbnodes.Get(0)->GetObject<ConstantPositionMobilityModel>()->SetPosition(Vector(0.0,0.0,25.0)); 
	enbnodes.Get(1)->GetObject<ConstantPositionMobilityModel>()->SetPosition(Vector(430.0,0.0,25.0));

    MobilityHelper uemobility;
    uemobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    uemobility.Install(uenodes);
    BuildingsHelper::Install (uenodes);
   for (uint8_t i =0; i < uecounts; i++)
    {
       
	uenodes.Get (i)->GetObject<MobilityModel> ()->SetPosition (Vector (35 + 40*i, 0.0, 2.0));
    }
  
    

    //Create a netdevice for the UE and eNB
    NetDeviceContainer uenetdev;
    NetDeviceContainer eNBnetdev;

    uenetdev = mmWaveObj->InstallUeDevice(uenodes);
    eNBnetdev = mmWaveObj->InstallEnbDevice(enbnodes);
    internet.Install (uenodes);
    Ipv4InterfaceContainer ueInterfaces;

    ueInterfaces = epcHelper->AssignUeIpv4Address (NetDeviceContainer (uenetdev));
    mmWaveObj->AttachToClosestEnb(uenetdev, eNBnetdev);
    mmWaveObj->EnableTraces();
    
    // Set the default gateway for the UE & allocate IP's to the UE
    for (int i =0; i< uecounts; i++)
    {
     Ptr<Node> ueNode = uenodes.Get (i); //set the gateway to be the farthest node
     Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting (ueNode->GetObject<Ipv4> ());
     ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1); 
     ueStaticRouting->AddNetworkRouteTo ("10.0.0.0", "255.0.0.0", 1);
     }


    /*Client host and remote host application initialization*/
    uint16_t sinkPort = 1489;

    ApplicationContainer clientApps;
  ApplicationContainer serverApps;
 
    PacketSinkHelper ulPacketSinkHelper ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), sinkPort));

    serverApps.Add (ulPacketSinkHelper.Install (remoteHost));
    uint32_t pcksize = 1000; 
    // ### sink to collect the total Rx received from uplink
    sink = StaticCast<PacketSink> (serverApps.Get(0));

    UdpClientHelper ulClient (remoteHostAddr, sinkPort);
    ulClient.SetAttribute ("Interval", TimeValue (MilliSeconds(200)));
    ulClient.SetAttribute ("MaxPackets", UintegerValue(1000000));
    ulClient.SetAttribute("PacketSize", UintegerValue(pcksize));

    clientApps.Add (ulClient.Install (uenodes.Get(0)));
    serverApps.Start (Seconds (0.2));
    clientApps.Start (Seconds (0.2));
   
   //Simulation schedule and invocation of a callback function for packet tracing
   Simulator::Schedule (Seconds (0.2), &CalculateThroughput);

   double sendbyte = (((sim_time +1) - 0.2)/0.2) * pcksize;
   Ptr<FlowMonitor> flowmon;
   FlowMonitorHelper flowhelper;
   flowmon = flowhelper.InstallAll();
    

    Simulator::Stop(Seconds(sim_time+1));
    Simulator::Run();
    flowmon->SerializeToXmlFile("mmWaveVanetflow", true, true);
    Simulator::Destroy();	
    
    // ### Print the total Received packets from packet sink
  //double perfeff;
  double averageThroughput = ((sink->GetTotalRx () * 8) / (1e4 * (sim_time+1)));
  //perfeff = ((sendbyte - sink->GetTotalRx())/ sendbyte )* 100;
  std::cout << "\nTotal Rx : " << sink->GetTotalRx () << " bytes" << std::endl;
  std::cout << "The Total Tx: " << sendbyte << std::endl;
  std::cout << "\nAverage throughput: " << averageThroughput << " Mbit/s" << std::endl;

    return 0;	

}  
