/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2023 UNIVERSIDADE FEDERAL DE GOIÁS
 * Copyright (c) NumbERS - INSTITUTO FEDERAL DE GOIÁS - CAMPUS INHUMAS
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Rogério S. Silva <rogerio.sousa@ifg.edu.br>
 * Created on: 08/11/2023
 */

#include "ns3/antenna-module.h"
#include "ns3/command-line.h"
#include "ns3/core-module.h"
#include "ns3/eps-bearer-tag.h"
#include "ns3/lora-utils.h"
#include "ns3/lorawan-module.h"
#include "ns3/mobility-module.h"
#include "ns3/nr-module.h"

NS_LOG_COMPONENT_DEFINE("E2E_NR_UAV_LORA");

using namespace ns3;
using namespace lorawan;

Time sendPacketTime = Seconds(0.4);
Time appStopTime = Seconds(20);

static void Send3GPPPacket(Ptr<NetDevice> device, Address& bsAddr, Ptr<Packet> pkt);
void SendLoRaPacket(Ptr<Packet> packet, Ptr<NetDevice> loraDevice, Ptr<NetDevice> uavDevice);
void OnRxLoraPacket(Ptr<Packet const> packet, uint32_t systemId);
void OnRx3GPPPacket(Ptr<Packet const> packet, uint32_t systemId);
static void RouteNon3GPPPacket(Ptr<Packet> loraPacket,
                               Ptr<NetDevice> uavDevice,
                               Ptr<NetDevice> bsDevice);
void PrintPacketData(Ptr<Packet> packet, std::string extra);

/**
 * #####################################################################################
 *                              SIMULATION MODEL
 * #####################################################################################
 *                 LoRaWAN communication             5G sub6GHz communication
 *    © © © © ©                              X X                              ((( ^
 *     © © © © )))       NON3GPP         ((( |O| )))         3GPP                /X\
 *   ©© © © ©                                X X                                /XXX\
 * -------------------------------------------------------------------------------------
 * Devices Lorawan                           UAV                              BS 5G/B5G
 * -------------------------------------------------------------------------------------
 *                                            ^
 *                                       LoRa to 3GPP
 *                                      packet routing
 * #####################################################################################
 */

int
main(int argc, char* argv[])
{
    NodeContainer uavNodes;
    NodeContainer nrNodes;
    NodeContainer loraNodes;
    uint32_t n_uavs = 1;
    uint32_t n_bss = 1;
    uint32_t n_loraeds = 1;
    bool verbose = false;

    // Simulation parameters
    CommandLine cmd;
    cmd.AddValue("n_uavs", "number of UAVs", n_uavs);
    cmd.AddValue("n_bss", "number of base stations", n_bss);
    cmd.AddValue("n_loraeds", "number of LoRa end devices", n_loraeds);
    cmd.AddValue("verbose", "Print Log if true [--verbose]", verbose);

    // Parse the command line
    cmd.Parse(argc, argv);

    if (verbose)
    {
        LogComponentEnable("E2E_NR_UAV_LORA", ns3::LOG_LEVEL_ALL);
        //        LogComponentEnable("NetworkServer", LOG_LEVEL_ALL);
        //        LogComponentEnable("GatewayLorawanMac", LOG_LEVEL_ALL);
        //        LogComponentEnable("EndDeviceStatus", LOG_LEVEL_ALL);
        //        LogComponentEnable("EndDeviceLorawanMac", LOG_LEVEL_ALL);
        //        LogComponentEnable("ClassAEndDeviceLorawanMac", LOG_LEVEL_ALL);
        //        LogComponentEnable("ThreeGppSpectrumPropagationLossModel", LOG_LEVEL_ALL);
        //        LogComponentEnable("ThreeGppPropagationLossModel", LOG_LEVEL_ALL);
        //        LogComponentEnable("ThreeGppChannelModel", LOG_LEVEL_ALL);
        //        LogComponentEnable("ChannelConditionModel", LOG_LEVEL_ALL);
        //        LogComponentEnable("UdpClient", LOG_LEVEL_INFO);
        //        LogComponentEnable("UdpServer", LOG_LEVEL_INFO);
        //        LogComponentEnable("LteRlcUm", LOG_LEVEL_LOGIC);
        //        LogComponentEnable("LtePdcp", LOG_LEVEL_INFO);
    }

    if (n_uavs < 1 || n_bss < 1 || n_loraeds < 1)
    {
        NS_LOG_ERROR("Number of nodes must be greater than 0");
        return 1;
    }

    uavNodes.Create(n_uavs);
    nrNodes.Create(n_bss);
    loraNodes.Create(n_loraeds);

    // Mobility Settings
    MobilityHelper mobilityED;
    MobilityHelper mobilityUAV;
    MobilityHelper mobilityBS;
    mobilityED.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobilityUAV.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobilityBS.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    Ptr<ListPositionAllocator> uavsPositions = CreateObject<ListPositionAllocator>();
    Ptr<ListPositionAllocator> bssPositions = CreateObject<ListPositionAllocator>();
    Ptr<ListPositionAllocator> loraedsPositions = CreateObject<ListPositionAllocator>();
    bssPositions->Add(Vector(0, 0, 20));
    uavsPositions->Add(Vector(2500, 2500, 30));
    loraedsPositions->Add(Vector(5000, 5000, 1.2));
    mobilityBS.SetPositionAllocator(bssPositions);
    mobilityUAV.SetPositionAllocator(uavsPositions);
    mobilityED.SetPositionAllocator(loraedsPositions);
    mobilityBS.Install(nrNodes);
    mobilityUAV.Install(uavNodes);
    mobilityED.Install(loraNodes);

    int64_t randomStream = 1;
    // Create the scenario
    //    GridScenarioHelper gridScenario;
    //    gridScenario.SetRows(1);
    //    gridScenario.SetColumns(n_bss);
    //    gridScenario.SetHorizontalBsDistance(5.0);
    //    gridScenario.SetBsHeight(10.0);
    //    gridScenario.SetUtHeight(1.5);
    //    // must be set before BS number
    //    gridScenario.SetSectorization(GridScenarioHelper::SINGLE);
    //    gridScenario.SetBsNumber(n_bss);
    //    gridScenario.SetUtNumber(n_uavs * n_bss);
    //    gridScenario.SetScenarioHeight(10); // Create a 3x3 scenario where the UE will
    //    gridScenario.SetScenarioLength(10); // be distributed.
    //    randomStream += gridScenario.AssignStreams(randomStream);
    //    gridScenario.CreateScenario();

    /** LoRaWAN settings*/
    // LoRa settings
    Ptr<PropagationDelayModel> delay = CreateObject<ConstantSpeedPropagationDelayModel>();
    Ptr<LogDistancePropagationLossModel> loss = CreateObject<LogDistancePropagationLossModel>();
    loss->SetPathLossExponent(3.76);
    loss->SetReference(1, 10.0);
    Ptr<LoraChannel> channel = CreateObject<LoraChannel>(loss, delay);
    LoraPhyHelper loraPhyHelper = LoraPhyHelper();
    loraPhyHelper.SetChannel(channel);
    LorawanMacHelper lorawanMacHelper = LorawanMacHelper();
    // Create the LoraNetDevices of the end devices
    uint8_t nwkId = 54;
    uint32_t nwkAddr = 1864;
    Ptr<LoraDeviceAddressGenerator> addrGen =
        CreateObject<LoraDeviceAddressGenerator>(nwkId, nwkAddr);
    // Create the LoraNetDevices of the end devices
    LoraHelper loraHelper = LoraHelper();
    loraHelper.EnablePacketTracking();
    lorawanMacHelper.SetAddressGenerator(addrGen);
    loraPhyHelper.SetDeviceType(LoraPhyHelper::ED);
    lorawanMacHelper.SetDeviceType(LorawanMacHelper::ED_A);
    lorawanMacHelper.SetRegion(LorawanMacHelper::EU);
    loraHelper.Install(loraPhyHelper, lorawanMacHelper, loraNodes);
    // Create a net device for each gateway
    loraPhyHelper.SetDeviceType(LoraPhyHelper::GW);
    lorawanMacHelper.SetDeviceType(LorawanMacHelper::GW);
    loraHelper.Install(loraPhyHelper, lorawanMacHelper, uavNodes);
    // Create a net device for the network server
    NodeContainer networkServer;
    networkServer.Create(1);
    NetworkServerHelper nsHelper = NetworkServerHelper();
    nsHelper.SetAdr("ns3::AdrComponent");
    nsHelper.EnableAdr(true);
    nsHelper.SetEndDevices(loraNodes);
    nsHelper.SetGateways(uavNodes);
    nsHelper.Install(networkServer);

    // Create a LoRa packet
    Packet::EnablePrinting();
    //    Packet::EnableChecking();

    std::string payload = "LoRa";
    Ptr<Packet> packet = Create<Packet>((uint8_t*)payload.c_str(), payload.size());
    LoraTag loraTag = LoraTag();
    LoraFrameHeader header = LoraFrameHeader();
    LorawanMacHeader macHeader = LorawanMacHeader();
    loraTag.SetSpreadingFactor(7);
    header.SetAdr(uavNodes.Get(0)->GetId());
    packet->AddPacketTag(loraTag);
    packet->AddHeader(header);

    /** NR settings */
    uint16_t numerologyBwp1 = 0;
    //    uint32_t udpPacketSize = 1000;
    double centralFrequencyBand1 = 28e9;
    double bandwidthBand1 = 400e6;
    Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper>();
    Ptr<IdealBeamformingHelper> idealBeamformingHelper = CreateObject<IdealBeamformingHelper>();
    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
    nrHelper->SetBeamformingHelper(idealBeamformingHelper);
    nrHelper->SetEpcHelper(epcHelper);
    // Create one operational band containing one CC with one bandwidth part
    BandwidthPartInfoPtrVector allBwps;
    CcBwpCreator ccBwpCreator;
    const uint8_t numCcPerBand = 1;
    // Create the configuration for the CcBwpHelper
    CcBwpCreator::SimpleOperationBandConf bandConf1(centralFrequencyBand1,
                                                    bandwidthBand1,
                                                    numCcPerBand,
                                                    BandwidthPartInfo::UMa_LoS);
    // By using the configuration created, it is time to make the operation band
    OperationBandInfo band1 = ccBwpCreator.CreateOperationBandContiguousCc(bandConf1);
    Config::SetDefault("ns3::ThreeGppChannelModel::UpdatePeriod", TimeValue(MilliSeconds(0)));
    nrHelper->SetSchedulerAttribute("FixedMcsDl", BooleanValue(true));
    nrHelper->SetSchedulerAttribute("StartingMcsDl", UintegerValue(28));
    nrHelper->SetChannelConditionModelAttribute("UpdatePeriod", TimeValue(MilliSeconds(0)));
    nrHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(false));

    nrHelper->InitializeOperationBand(&band1);
    allBwps = CcBwpCreator::GetAllBwps({band1});

    // Beamforming method
    idealBeamformingHelper->SetAttribute("BeamformingMethod",
                                         TypeIdValue(DirectPathBeamforming::GetTypeId()));

    // Antennas for all the UEs
    nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(2));
    nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(4));
    nrHelper->SetUeAntennaAttribute("AntennaElement",
                                    PointerValue(CreateObject<IsotropicAntennaModel>()));

    // Antennas for all the gNbs
    nrHelper->SetGnbAntennaAttribute("NumRows", UintegerValue(4));
    nrHelper->SetGnbAntennaAttribute("NumColumns", UintegerValue(8));
    nrHelper->SetGnbAntennaAttribute("AntennaElement",
                                     PointerValue(CreateObject<IsotropicAntennaModel>()));

    // Install and get the pointers to the NetDevices
    NetDeviceContainer enbNetDev = nrHelper->InstallGnbDevice(nrNodes, allBwps);
    NetDeviceContainer ueNetDev = nrHelper->InstallUeDevice(uavNodes, allBwps);

    randomStream += nrHelper->AssignStreams(enbNetDev, randomStream);
    randomStream += nrHelper->AssignStreams(ueNetDev, randomStream);

    // Set the attribute of the netdevice (enbNetDev.Get (0)) and bandwidth part (0)
    nrHelper->GetGnbPhy(enbNetDev.Get(0), 0)
        ->SetAttribute("Numerology", UintegerValue(numerologyBwp1));

    for (auto it = enbNetDev.Begin(); it != enbNetDev.End(); ++it)
    {
        DynamicCast<NrGnbNetDevice>(*it)->UpdateConfig();
    }

    for (auto it = ueNetDev.Begin(); it != ueNetDev.End(); ++it)
    {
        DynamicCast<NrUeNetDevice>(*it)->UpdateConfig();
    }

    InternetStackHelper internet;
    internet.Install(uavNodes);
    Ipv4InterfaceContainer ueIpIface;
    ueIpIface = epcHelper->AssignUeIpv4Address(NetDeviceContainer(ueNetDev));

    // attach UEs to the closest eNB
    nrHelper->AttachToClosestEnb(ueNetDev, enbNetDev);

    // ADR LoRaWAN
    ns3::lorawan::LorawanMacHelper::SetSpreadingFactorsUp(loraNodes, uavNodes, channel);

    // Register trace callback for received packets
    for (auto g = uavNodes.Begin(); g != uavNodes.End(); ++g)
    {
        Ptr<Node> object = *g;
        // Get the device
        Ptr<NetDevice> uavNetDevice = object->GetDevice(0);
        Ptr<LoraNetDevice> loraNetDevice = uavNetDevice->GetObject<LoraNetDevice>();
        Ptr<GatewayLoraPhy> gwPhy = loraNetDevice->GetPhy()->GetObject<GatewayLoraPhy>();
        gwPhy->TraceConnectWithoutContext("ReceivedPacket", MakeCallback(&OnRxLoraPacket));
    }

//    nrHelper->TraceConnectWithoutContext("ReceivedPacket", MakeCallback(&OnRx3GPPPacket));

    // Send LoRa to UAV
    Simulator::Schedule(sendPacketTime,
                        SendLoRaPacket,
                        packet,
                        loraNodes.Get(0)->GetDevice(0),
                        uavNodes.Get(0)->GetDevice(0));

    // add IP header to LoRa packet and send to BS gNB
    Simulator::Schedule(sendPacketTime + Seconds(0.2),
                        &RouteNon3GPPPacket,
                        packet,
                        uavNodes.Get(0)->GetDevice(0),
                        nrNodes.Get(0)->GetDevice(0));

    nrHelper->EnableTraces();

    Simulator::Stop(appStopTime);
    Simulator::Run();

    //    loraHelper.DoPrintPhyPerformance(uavNodes, "e2e-nr-uav-loraGW.txt");
    //    loraHelper.DoPrintDeviceStatus(loraNodes, uavNodes, "e2e-nr-uav-loraED.txt");
    //    loraHelper.DoPrintGlobalPerformance("e2e-nr-uav-lora.txt");

    Simulator::Destroy();

    return 0;
}

/**
 * Function creates a single packet and directly calls the function send
 * of a device to send the packet to the destination address.
 * @param device Device that will send the packet to the destination address.
 * @param addr Destination address for a packet.
 * @param packetSize The packet size.
 */
static void
Send3GPPPacket(Ptr<NetDevice> device, Address& bsAddr, Ptr<Packet> pkt)
{
    EpsBearerTag tag(1, 1);
    pkt->AddPacketTag(tag);
    pkt->Print(std::cout);
    device->Send(pkt, bsAddr, Ipv4L3Protocol::PROT_NUMBER);
    NS_LOG_INFO("Packet sent from UAV to BS device at " << Simulator::Now().GetSeconds() << "s");
    //    PrintPacketData(pkt, "UAX Tx");
}

void
OnRxLoraPacket(Ptr<Packet const> packet, uint32_t systemId)
{
    std::cout << "Received packet from LoRaWAN device " << systemId << std::endl
              << "Packet size: " << packet->GetSize() << " bytes" << std::endl
              << "Packet contents: " << packet->ToString()
              << " Now: " << Simulator::Now().GetSeconds() << std::endl;
    NS_LOG_INFO("Received packet from LoRa device at " << Simulator::Now().GetSeconds() << "s");
}

void
OnRx3GPPPacket(Ptr<Packet const> packet, uint32_t systemId)
{
    std::cout << "Received packet from UAV device " << systemId << std::endl
              << "Packet size: " << packet->GetSize() << " bytes" << std::endl
              << "Packet contents: " << packet->ToString()
              << " Now: " << Simulator::Now().GetSeconds() << std::endl;
    NS_LOG_INFO("Received packet from UAV device at " << Simulator::Now().GetSeconds() << "s");
}

static void
RouteNon3GPPPacket(Ptr<Packet> loraPacket, Ptr<NetDevice> uavDevice, Ptr<NetDevice> bsDevice)
{
    // Change packet header LoRaWAN (IP) to NR (IP)
    Ptr<Packet> nrPacket = loraPacket->Copy();
    Ipv4Header ipv4Header;
    ipv4Header.SetProtocol(UdpL4Protocol::PROT_NUMBER);
    nrPacket->AddHeader(ipv4Header);
    // todo: change packet header LoRaWAN (IP) to NR (IP)
    NS_LOG_INFO("Scheduled to send packet from UAV to BS at " << Simulator::Now().GetSeconds()
                                                              << "s");
    Simulator::Schedule(sendPacketTime + Seconds(0.2),
                        Send3GPPPacket,
                        uavDevice,
                        bsDevice->GetAddress(),
                        nrPacket);

}

void
SendLoRaPacket(Ptr<Packet> packet, Ptr<NetDevice> loraDevice, Ptr<NetDevice> uavDevice)
{
    loraDevice->Send(packet, uavDevice->GetAddress(), 0);
    packet->Print(std::cout);
    NS_LOG_INFO("LoRa packet sent to UAV at " << Simulator::Now().GetSeconds() << "s");
}

void
PrintPacketData(Ptr<Packet> packet, std::string extra)
{
    // Recuperação da mensagem do pacote recebido
    uint8_t buffer[packet->GetSize()];
    packet->CopyData(buffer, packet->GetSize());
    std::string receivedMessage(reinterpret_cast<char*>(buffer), packet->GetSize());
    //    NS_LOG_INFO( "Mensagem recebida: " << receivedMessage << " " << extra );
    packet->Print(std::cout);
}