/*
 * This script simulates a complex scenario with multiple gateways and end
 * devices. The metric of interest in this script is the throughput of the
 * network.
 */

#include "ns3/gateway-lora-phy.h"
#include "ns3/class-a-end-device-lorawan-mac.h"
#include "ns3/gateway-lorawan-mac.h"
#include "ns3/simulator.h"
#include "ns3/log.h"
#include "ns3/pointer.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/lora-helper.h"
#include "ns3/node-container.h"
#include "ns3/mobility-helper.h"
#include "ns3/position-allocator.h"
#include "ns3/double.h"
#include "ns3/periodic-sender-helper.h"
#include "ns3/command-line.h"
#include "ns3/rng-seed-manager.h"
#include "ns3/lorawan-module.h"
#include "ns3/propagation-module.h"
#include <algorithm>
#include <ctime>

using namespace ns3;
using namespace lorawan;

NS_LOG_COMPONENT_DEFINE ("ComplexLorawanNetworkExample");

// Network settings
int nDevices = 4;
int nGateways = 4;
double sideLength = 1000;
double simulationTime = 600; //600
int appPeriodSeconds = 600; //600
std::vector<int> sfQuantity (6);

int noMoreReceivers = 0;
int interfered = 0;
int received = 0;
int underSensitivity = 0;
std::string file_prefix = "EDs_x_SF";
std::string file_extension = ".dat";
bool okumura = false;
bool realisticChannelModel = false;
Time expDelay = Seconds(0);

// Output control
bool printEDs = true;
bool buildingsEnabled = false;
bool printToA = false;
bool printPercentuals = false;

/**********************
 *  Global Callbacks  *
 **********************/

enum PacketOutcome { _RECEIVED, _INTERFERED, _NO_MORE_RECEIVERS, _UNDER_SENSITIVITY, _UNSET };

struct myPacketStatus
{
  Ptr<Packet const> packet;
  uint32_t senderId;
  Time sentTime;
  Time receivedTime;
  int outcomeNumber;
  std::vector<enum PacketOutcome> outcomes;
};

std::map<Ptr<Packet const>, myPacketStatus> packetTracker;

void
CheckReceptionByAllGWsComplete (std::map<Ptr<Packet const>, myPacketStatus>::iterator it)
{
  // Check whether this packet is received by all gateways
  if ((*it).second.outcomeNumber == nGateways)
    {
      // Update the statistics
      myPacketStatus status = (*it).second;
      for (int j = 0; j < nGateways; j++)
        {
          switch (status.outcomes.at (j))
            {
              case _RECEIVED: {
                received += 1;
                break;
              }
              case _UNDER_SENSITIVITY: {
                underSensitivity += 1;
                break;
              }
              case _NO_MORE_RECEIVERS: {
                noMoreReceivers += 1;
                break;
              }
              case _INTERFERED: {
                interfered += 1;
                break;
              }
              case _UNSET: {
                break;
              }
            }
        }
      // Remove the packet from the tracker
      packetTracker.erase (it);
    }
}

void
TransmissionCallback (Ptr<Packet const> packet, uint32_t systemId)
{
   NS_LOG_INFO ("Transmitted a packet from device " << systemId);
  // Create a packetStatus
  myPacketStatus status;
  status.packet = packet;
  status.senderId = systemId;
  status.sentTime = Simulator::Now();
  status.outcomeNumber = 0;
  status.outcomes = std::vector<enum PacketOutcome> (nGateways, _UNSET);

  packetTracker.insert (std::pair<Ptr<Packet const>, myPacketStatus> (packet, status));
}

void
PacketReceptionCallback (Ptr<Packet const> packet, uint32_t systemId)
{
  // Remove the successfully received packet from the list of sent ones
   NS_LOG_INFO ("A packet was successfully received at gateway " << systemId);

  std::map<Ptr<Packet const>, myPacketStatus>::iterator it = packetTracker.find (packet);
  (*it).second.outcomes.at (systemId - nDevices) = _RECEIVED;
  (*it).second.outcomeNumber += 1;
  (*it).second.receivedTime = Simulator::Now();
  expDelay += (*it).second.receivedTime - (*it).second.sentTime;

  CheckReceptionByAllGWsComplete (it);
}

void
InterferenceCallback (Ptr<Packet const> packet, uint32_t systemId)
{
  // NS_LOG_INFO ("A packet was lost because of interference at gateway " << systemId);

  std::map<Ptr<Packet const>, myPacketStatus>::iterator it = packetTracker.find (packet);
  (*it).second.outcomes.at (systemId - nDevices) = _INTERFERED;
  (*it).second.outcomeNumber += 1;

  CheckReceptionByAllGWsComplete (it);
}

void
NoMoreReceiversCallback (Ptr<Packet const> packet, uint32_t systemId)
{
   NS_LOG_INFO ("A packet was lost because there were no more receivers at gateway " << systemId);

  std::map<Ptr<Packet const>, myPacketStatus>::iterator it = packetTracker.find (packet);
  (*it).second.outcomes.at (systemId - nDevices) = _NO_MORE_RECEIVERS;
  (*it).second.outcomeNumber += 1;

  CheckReceptionByAllGWsComplete (it);
}

void
UnderSensitivityCallback (Ptr<Packet const> packet, uint32_t systemId)
{
   NS_LOG_INFO ("A packet arrived at the gateway under sensitivity at gateway " << systemId);

  std::map<Ptr<Packet const>, myPacketStatus>::iterator it = packetTracker.find (packet);
  (*it).second.outcomes.at (systemId - nDevices) = _UNDER_SENSITIVITY;
  (*it).second.outcomeNumber += 1;

  CheckReceptionByAllGWsComplete (it);
}

time_t oldtime = std::time (0);

// Periodically print simulation time
void
PrintSimulationTime (void)
{
  // NS_LOG_INFO ("Time: " << Simulator::Now().GetHours());
  std::cout << "Simulated time: " << Simulator::Now ().GetHours () << " hours" << std::endl;
  std::cout << "Real time from last call: " << std::time (0) - oldtime << " seconds" << std::endl;
  oldtime = std::time (0);
  Simulator::Schedule (Minutes (30), &PrintSimulationTime);
}

void
PrintToA (std::string file_name)
{
  /***********************
  * Print TimeOnAir Duration  *
  ************************/
  Time duration;
  Ptr<Packet> packet = Create<Packet> (10);
  LoraTxParameters txParams;
  for (int cr = 0; cr < 5; ++cr)
    {
      std::string filename = file_name;
      filename += std::to_string (cr) + ".dat";
      const char *c = filename.c_str ();
      std::ofstream toaFile;
      toaFile.open (c);
      for (int sf = 7; sf < 13; ++sf)
        {
          txParams.sf = sf;
          txParams.headerDisabled = false;
          txParams.codingRate = cr;
          txParams.bandwidthHz = 125000;
          txParams.nPreamble = 8;
          txParams.crcEnabled = 1;
          txParams.lowDataRateOptimizationEnabled = 0;
          duration = LoraPhy::GetOnAirTime (packet, txParams);

          //          toaFile << "ToA (SF "<< sf <<", CR "<< cr <<", BW 125kHz): " << duration.GetSeconds () << std::endl;
          toaFile << sf << " " << cr << " " << duration.GetSeconds () << std::endl;
        }
      toaFile.close ();
    }
}

void
PrintEndDevicesPositions (NodeContainer endDevices, std::string filename)
{
  const char *c = filename.c_str ();
  std::ofstream spreadingFactorFile;
  spreadingFactorFile.open (c);
  for (NodeContainer::Iterator j = endDevices.Begin (); j != endDevices.End (); ++j)
    {
      Ptr<Node> object = *j;
      Ptr<MobilityModel> position = object->GetObject<MobilityModel> ();
      NS_ASSERT (position != nullptr);
      Ptr<NetDevice> netDevice = object->GetDevice (0);
      Ptr<LoraNetDevice> loraNetDevice = netDevice->GetObject<LoraNetDevice> ();
      NS_ASSERT (loraNetDevice != nullptr);
      Ptr<EndDeviceLorawanMac> mac = loraNetDevice->GetMac ()->GetObject<EndDeviceLorawanMac> ();
//      int sf = int (mac->GetDataRate ());
      Vector pos = position->GetPosition ();
      spreadingFactorFile << pos.x << " " << pos.y << " " << pos.z << std::endl;
    }
  spreadingFactorFile.close ();
}

// // Also print the gateways
//for (NodeContainer::Iterator j = gateways.Begin (); j != gateways.End (); ++j)
//  {
//    Ptr<Node> object = *j;
//    Ptr<MobilityModel> position = object->GetObject<MobilityModel> ();
//    Vector pos = position->GetPosition ();
//    spreadingFactorFile << pos.x << " " << pos.y << " GW" << std::endl;
//  }

void
PrintEndDevicesParameters (NodeContainer endDevices, NodeContainer gateways, std::string filename)
{
  const char *c = filename.c_str ();
  std::ofstream spreadingFactorFile;
  spreadingFactorFile.open (c);
  for (NodeContainer::Iterator gw = gateways.Begin (); gw != gateways.End (); ++gw)
    {
      Ptr<Node> oGateway = *gw;
      for (NodeContainer::Iterator ed = endDevices.Begin (); ed != endDevices.End (); ++ed)
        {
          Ptr<Node> oDevice = *ed;
          double distanceFromGW = oDevice->GetObject<MobilityModel> ()->GetDistanceFrom (
              oGateway->GetObject<MobilityModel> ());
          NS_ASSERT (distanceFromGW != 0);
          Ptr<NetDevice> netDevice = oDevice->GetDevice (0);
          Ptr<LoraNetDevice> loraNetDevice = netDevice->GetObject<LoraNetDevice> ();
          NS_ASSERT (loraNetDevice != nullptr);
          Ptr<EndDeviceLorawanMac> mac =
              loraNetDevice->GetMac ()->GetObject<EndDeviceLorawanMac> ();
          int sf = mac->GetSfFromDataRate (mac->GetDataRate ());
          int txPower = mac->GetTransmissionPower ();
          int dBm = mac->GetDbmForTxPower (mac->GetTransmissionPower ());
          std::vector<double> txDbmForTxPower;
          mac->SetTxDbmForTxPower (txDbmForTxPower);
          spreadingFactorFile << oDevice->GetId () << " " << distanceFromGW << " " << sf << " "
                              << txPower << " " << dBm << std::endl;
        }
    }
  spreadingFactorFile.close ();
}

int
main (int argc, char *argv[])
{

  // Create the time value from the period
  Time timeStart = Simulator::Now ();
  /***********
  *  Seed    *
  ***********/

  ns3::RngSeedManager::SetSeed (1);

  /*****************************
  *  Setup via command prompt  *
  ******************************/

  CommandLine cmd;
  cmd.AddValue ("nDevices", "Number of end devices to include in the simulation", nDevices);
  cmd.AddValue ("rings", "Number of gateway rings to include", nGateways);
  cmd.AddValue ("sideLength", "The sideLength of the area to simulate", sideLength);
  cmd.AddValue ("simulationTime", "The time for which to simulate", simulationTime);
  cmd.AddValue ("appPeriod",
                "The period in seconds to be used by periodically transmitting applications",
                appPeriodSeconds);
  cmd.AddValue ("printEDs", "Whether or not to print a file containing the ED's positions",
                printEDs);
  cmd.AddValue ("fName", "File Name", file_prefix);
  cmd.AddValue ("okumura", "The sideLength of the area to simulate", okumura);
  cmd.AddValue ("printToA", "Whether or not generate ToA durations file", printToA);

  cmd.Parse (argc, argv);

  // Set up logging
    LogComponentEnable ("ComplexLorawanNetworkExample", ns3::LOG_LEVEL_ALL);
    LogComponentEnable ("LoraChannel", ns3::LOG_LEVEL_DEBUG);
    LogComponentEnable ("LoraPhy", ns3::LOG_LEVEL_DEBUG);
    LogComponentEnable ("EndDeviceLoraPhy", ns3::LOG_LEVEL_DEBUG);
    LogComponentEnable ("GatewayLoraPhy", ns3::LOG_LEVEL_DEBUG);
    LogComponentEnable ("LoraInterferenceHelper", ns3::LOG_LEVEL_DEBUG);
    //   LogComponentEnable("LoraMac", ns3::LOG_LEVEL_DEBUG);
    //   LogComponentEnable("EndDeviceLoraMac", ns3::LOG_LEVEL_DEBUG);
    //   LogComponentEnable("GatewayLoraMac", ns3::LOG_LEVEL_DEBUG);
    LogComponentEnable ("LogicalLoraChannelHelper", ns3::LOG_LEVEL_DEBUG);
    LogComponentEnable ("LogicalLoraChannel", ns3::LOG_LEVEL_DEBUG);
    LogComponentEnable ("LoraHelper", ns3::LOG_LEVEL_DEBUG);
    LogComponentEnable ("LoraPhyHelper", ns3::LOG_LEVEL_DEBUG);
    //   LogComponentEnable("LoraMacHelper", ns3::LOG_LEVEL_DEBUG);
    LogComponentEnable ("PeriodicSenderHelper", ns3::LOG_LEVEL_DEBUG);
    LogComponentEnable ("PeriodicSender", ns3::LOG_LEVEL_DEBUG);
    //   LogComponentEnable("LoraMacHeader", ns3::LOG_LEVEL_DEBUG);
    LogComponentEnable ("LoraFrameHeader", ns3::LOG_LEVEL_DEBUG);

  /****************
  *  Setup Network*
  *****************/

  // Create the time value from the period
  Time appPeriod = Seconds (appPeriodSeconds);

  // Mobility
  MobilityHelper mobility;
  mobility.SetPositionAllocator ("ns3::RandomRectanglePositionAllocator", "X",
                                 PointerValue (CreateObjectWithAttributes<UniformRandomVariable> (
                                     "Min", DoubleValue (0), "Max", DoubleValue (sideLength))),
                                 "Y",
                                 PointerValue (CreateObjectWithAttributes<UniformRandomVariable> (
                                     "Min", DoubleValue (0), "Max", DoubleValue (sideLength))));
  //  mobility.SetPositionAllocator ("ns3::UniformDiscPositionAllocator", "rho", DoubleValue (sideLength),
  //                                 "X", DoubleValue (0.0), "Y", DoubleValue (0.0));
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");

  /************************
  *  Create the channel  *
  ************************/

  // Create the lora channel object
  // modelo de propagação (okumura ou logdistance)
  Ptr<LoraChannel> channel;
  Ptr<PropagationDelayModel> delay = CreateObject<ConstantSpeedPropagationDelayModel> ();
  if (okumura)
    {
      Ptr<OkumuraHataPropagationLossModel> loss = CreateObject<OkumuraHataPropagationLossModel> ();
      channel = CreateObject<LoraChannel> (loss, delay);
    }
  else
    {
      Ptr<LogDistancePropagationLossModel> loss = CreateObject<LogDistancePropagationLossModel> ();
      loss->SetPathLossExponent (3.76);
      loss->SetReference (1, 7.7);

      if (realisticChannelModel)
        {
          // Create the correlated shadowing component
          Ptr<CorrelatedShadowingPropagationLossModel> shadowing =
              CreateObject<CorrelatedShadowingPropagationLossModel> ();

          // Add the effect to the channel propagation loss
          Ptr<BuildingPenetrationLoss> buildingLoss = CreateObject<BuildingPenetrationLoss> ();

          shadowing->SetNext (buildingLoss);
        }
      channel = CreateObject<LoraChannel> (loss, delay);
    }

  /************************
  *  Create the helpers  *
  ************************/

  // Create the LoraPhyHelper
  LoraPhyHelper phyHelper = LoraPhyHelper ();
  phyHelper.SetChannel (channel);

  // Create the LoraMacHelper
  LorawanMacHelper macHelper = LorawanMacHelper ();

  // Create the LoraHelper
  LoraHelper helper = LoraHelper ();
  helper.EnablePacketTracking ();

  //Create the NetworkServerHelper
  NetworkServerHelper nsHelper = NetworkServerHelper ();

  //Create the ForwarderHelper
  ForwarderHelper forHelper = ForwarderHelper ();

  /************************
  *  Create End Devices  *
  ************************/

  // Create a set of nodes
  NodeContainer endDevices;
  endDevices.Create (nDevices);

  // Assign a mobility model to each node
  mobility.Install (endDevices);

  // Make it so that nodes are at a certain height > 0
  Ptr<UniformRandomVariable> rz = CreateObject<UniformRandomVariable> ();
  rz->SetAttribute ("Min", DoubleValue (1.0));
  rz->SetAttribute ("Max", DoubleValue (3.0));
  for (NodeContainer::Iterator j = endDevices.Begin (); j != endDevices.End (); ++j)
    {
      Ptr<MobilityModel> mobility2 = (*j)->GetObject<MobilityModel> ();
      Vector position = mobility2->GetPosition ();
      position.z = rz->GetValue ();
      mobility2->SetPosition (position);
    }

  // Create the LoraNetDevices of the end devices
  phyHelper.SetDeviceType (LoraPhyHelper::ED);
  macHelper.SetDeviceType (LorawanMacHelper::ED_A);
  helper.Install (phyHelper, macHelper, endDevices);

  // Now end devices are connected to the channel

  // Connect trace sources
  for (NodeContainer::Iterator j = endDevices.Begin (); j != endDevices.End (); ++j)
    {
      Ptr<Node> node = *j;
      Ptr<LoraNetDevice> loraNetDevice = node->GetDevice (0)->GetObject<LoraNetDevice> ();
      Ptr<LoraPhy> phy = loraNetDevice->GetPhy ();
      phy->TraceConnectWithoutContext ("StartSending", MakeCallback (&TransmissionCallback));
    }

  /*********************
  *  Create Gateways  *
  *********************/

  // Create the gateway nodes (allocate them uniformly on the disc)
  NodeContainer gateways;
  gateways.Create (nGateways);

  Ptr<ListPositionAllocator> allocator = CreateObject<ListPositionAllocator> ();
  if (nGateways == 1)
    {
      allocator->Add (Vector (0.0, 0.0, 0.0));
    }
  else if (nGateways == 2)
    {
      allocator->Add (Vector (8000.0, 0.0, 0.0));
      allocator->Add (Vector (-8000.0, 0.0, 0.0));
    }
  else if (nGateways == 4)
    {
      allocator->Add (Vector (-4000.0, 4000.0, 0.0));
      allocator->Add (Vector (4000.0, 4000.0, 0.0));
      allocator->Add (Vector (4000.0, -4000.0, 0.0));
      allocator->Add (Vector (-4000.0, -4000.0, 0.0));
    }
  else if (nGateways == 8)
    {
      allocator->Add (Vector (-2000.0, 2000.0, 0.0));
      allocator->Add (Vector (2000.0, 2000.0, 0.0));
      allocator->Add (Vector (2000.0, -2000.0, 0.0));
      allocator->Add (Vector (-2000.0, -2000.0, 0.0));
      allocator->Add (Vector (-2000.0, 2000.0, 0.0));
      allocator->Add (Vector (2000.0, 2000.0, 0.0));
      allocator->Add (Vector (2000.0, -2000.0, 0.0));
      allocator->Add (Vector (-2000.0, -2000.0, 0.0));
    }

  mobility.SetPositionAllocator (allocator);
  mobility.Install (gateways);

  // Make it so that nodes are at a certain height > 0
  for (NodeContainer::Iterator j = gateways.Begin (); j != gateways.End (); ++j)
    {
      Ptr<MobilityModel> mobility2 = (*j)->GetObject<MobilityModel> ();
      Vector position = mobility2->GetPosition ();
      position.z = 15;
      mobility2->SetPosition (position);
    }

  // Create a netdevice for each gateway
  phyHelper.SetDeviceType (LoraPhyHelper::GW);
  macHelper.SetDeviceType (LorawanMacHelper::GW);
  helper.Install (phyHelper, macHelper, gateways);

  /************************
  *  Configure Gateways  *
  ************************/

  // Install reception paths on gateways
  for (NodeContainer::Iterator j = gateways.Begin (); j != gateways.End (); j++)
    {

      Ptr<Node> object = *j;
      // Get the device
      Ptr<NetDevice> netDevice = object->GetDevice (0);
      Ptr<LoraNetDevice> loraNetDevice = netDevice->GetObject<LoraNetDevice> ();
      NS_ASSERT (loraNetDevice != nullptr);
      Ptr<GatewayLoraPhy> gwPhy = loraNetDevice->GetPhy ()->GetObject<GatewayLoraPhy> ();

      // Set up height of the gateway
      Ptr<MobilityModel> gwMob = (*j)->GetObject<MobilityModel> ();
      Vector position = gwMob->GetPosition ();
      position.z = 15;
      gwMob->SetPosition (position);

      // Global callbacks (every gateway)
      gwPhy->TraceConnectWithoutContext ("ReceivedPacket", MakeCallback (&PacketReceptionCallback));
      gwPhy->TraceConnectWithoutContext ("LostPacketBecauseInterference",
                                         MakeCallback (&InterferenceCallback));
      gwPhy->TraceConnectWithoutContext ("LostPacketBecauseNoMoreReceivers",
                                         MakeCallback (&NoMoreReceiversCallback));
      gwPhy->TraceConnectWithoutContext ("LostPacketBecauseUnderSensitivity",
                                         MakeCallback (&UnderSensitivityCallback));
    }

  /**********************************************
  *  Set up the end device's spreading factor  *
  **********************************************/

  macHelper.SetSpreadingFactorsUp (endDevices, gateways, channel);

  NS_LOG_DEBUG ("Completed configuration");

  /*********************************************
  *  Install applications on the end devices  *
  *********************************************/

  Time appStopTime = Seconds (simulationTime);
  PeriodicSenderHelper appHelper = PeriodicSenderHelper ();

  appHelper.SetPeriod (Seconds (appPeriodSeconds));
  ApplicationContainer appContainer = appHelper.Install (endDevices);
  appContainer.Start (Seconds (0));
  appContainer.Stop (appStopTime);

  /**************************
   *  Create Network Server  *
   ***************************/

  // Create the NS node
  NodeContainer networkServer;
  networkServer.Create (1);

  // Create a NS for the network
  nsHelper.SetEndDevices (endDevices);
  nsHelper.SetGateways (gateways);
  nsHelper.Install (networkServer);

  //Create a forwarder for each gateway
  forHelper.Install (gateways);

  /**********************
   * Print output files *
   *********************/
  if (printEDs)
    {
      std::string ss_filename, par_filename;
      std::string propagation;
      if (okumura)
        propagation = "OHA";
      else
        propagation = "LOG";

      ss_filename = "/home/rogerio/git/sim-res/endDevicesPlacement_" + std::to_string (nDevices) +
                    file_extension;

      //      par_filename = "Param-" + propagation + "-" + file_prefix + "_" + std::to_string (nDevices) +
      //                     "_" + std::to_string (nGateways) + file_extension;
      PrintEndDevicesPositions (endDevices, ss_filename);
      //      PrintEndDevicesParameters (endDevices, gateways, par_filename);
    }

  if (printToA)
    {
      std::string toa_filename;
      toa_filename = "ToA-Durations-per-SF-and-CR";
      PrintToA (toa_filename);
    }

  /****************
  *  Simulation  *
  ****************/

  Simulator::Stop (appStopTime + Hours (2));

  // PrintSimulationTime ();

  Simulator::Run ();

  std::cout << expDelay << std::endl;

  Simulator::Destroy ();

  std::cout << expDelay.GetSeconds() << std::endl;
  /*************
  *  Results  *
  *************/
  if (printPercentuals)
    {
      double receivedProb = double (received) / nDevices;
      double interferedProb = double (interfered) / nDevices;
      double noMoreReceiversProb = double (noMoreReceivers) / nDevices;
      double underSensitivityProb = double (underSensitivity) / nDevices;
      Time timeElapsed = Simulator::Now () - timeStart;
      std::cout << "Time elapsed: " << timeElapsed.GetHours () << ":" << timeElapsed.GetMinutes ()
                << ":" << timeElapsed.GetSeconds () << std::endl;
      double receivedProbGivenAboveSensitivity = double (received) / (nDevices - underSensitivity);
      double interferedProbGivenAboveSensitivity =
          double (interfered) / (nDevices - underSensitivity);
      double noMoreReceiversProbGivenAboveSensitivity =
          double (noMoreReceivers) / (nDevices - underSensitivity);
      std::cout << "Received: " << received << "#DEV: " << nDevices
                << " DEV/TEMP0: " << double (nDevices) / simulationTime
                << " RECEIVED: " << receivedProb << " INTERFERRED: " << interferedProb
                << " NO MORE RECEIVED: " << noMoreReceiversProb
                << " UNDER SENSIT: " << underSensitivityProb
                << " ABOVE SENS: " << receivedProbGivenAboveSensitivity
                << " INTERFERRED ABOVE: " << interferedProbGivenAboveSensitivity
                << " NO MORE ABOVE: " << noMoreReceiversProbGivenAboveSensitivity << std::endl;
    }

  return 0;
}

// Print the packetTracker contents
//     std::cout << "Packet outcomes" << std::endl;
//     std::map<Ptr<Packet const>, myPacketStatus>::iterator i;
//     for (i = packetTracker.begin (); i != packetTracker.end (); i++)
//       {
//         myPacketStatus status = (*i).second;
//         std::cout.width (4);
//         std::cout << status.senderId << "\t";
//         for (int j = 0; j < nGateways; j++)
//           {
//             switch (status.outcomes.at (j))
//               {
//               case _RECEIVED:
//                 {
//                   std::cout << "R ";
//                   break;
//                 }
//               case _UNDER_SENSITIVITY:
//                 {
//                   std::cout << "U ";
//                   break;
//                 }
//               case _NO_MORE_RECEIVERS:
//                 {
//                   std::cout << "N ";
//                   break;
//                 }
//               case _INTERFERED:
//                 {
//                   std::cout << "I ";
//                   break;
//                 }
//               case _UNSET:
//                 {
//                   std::cout << "E ";
//                   break;
//                 }
//               }
//           }
//         std::cout << std::endl;
//       }
