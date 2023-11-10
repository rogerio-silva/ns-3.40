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

static bool g_rxPdcpCallbackCalled = false;
static bool g_rxRxRlcPDUCallbackCalled = false;
static void SendPacket(Ptr<NetDevice> device, Address& addr, uint32_t packetSize);
void RxPdcpPDU(std::string path, uint16_t rnti, uint8_t lcid, uint32_t bytes, uint64_t pdcpDelay);
void RxRlcPDU(std::string path, uint16_t rnti, uint8_t lcid, uint32_t bytes, uint64_t rlcDelay);
void ConnectUlPdcpRlcTraces();
void ConnectPdcpRlcTraces();

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
    Time sendPacketTime = Seconds(0.4);
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
        LogComponentEnable("NetworkServer", LOG_LEVEL_ALL);
        LogComponentEnable("GatewayLorawanMac", LOG_LEVEL_ALL);
        LogComponentEnable("EndDeviceStatus", LOG_LEVEL_ALL);
        LogComponentEnable("EndDeviceLorawanMac", LOG_LEVEL_ALL);
        LogComponentEnable("ClassAEndDeviceLorawanMac", LOG_LEVEL_ALL);
        LogComponentEnable("ThreeGppSpectrumPropagationLossModel", LOG_LEVEL_ALL);
        LogComponentEnable("ThreeGppPropagationLossModel", LOG_LEVEL_ALL);
        LogComponentEnable("ThreeGppChannelModel", LOG_LEVEL_ALL);
        LogComponentEnable("ChannelConditionModel", LOG_LEVEL_ALL);
        LogComponentEnable("UdpClient", LOG_LEVEL_INFO);
        LogComponentEnable("UdpServer", LOG_LEVEL_INFO);
        LogComponentEnable("LteRlcUm", LOG_LEVEL_LOGIC);
        LogComponentEnable("LtePdcp", LOG_LEVEL_INFO);
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

    // Send LoRa packet
    Time appStopTime = Seconds(20);
    OneShotSenderHelper appHelper = OneShotSenderHelper();
    appHelper.SetSendTime(sendPacketTime);
    ApplicationContainer appContainer = appHelper.Install(loraNodes);
    appContainer.Start(Seconds(0));
    appContainer.Stop(appStopTime);
    ForwarderHelper forHelper = ForwarderHelper();
    forHelper.Install(uavNodes);

    /** NR settings */
    uint16_t numerologyBwp1 = 0;
    uint32_t udpPacketSize = 1000;
    double centralFrequencyBand1 = 28e9;
    double bandwidthBand1 = 400e6;
    bool enableUl = true;
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

    if (enableUl)
    {
        Simulator::Schedule(sendPacketTime + Seconds(0.1),
                            &SendPacket,
                            ueNetDev.Get(0),
                            enbNetDev.Get(0)->GetAddress(),
                            udpPacketSize);
    }
    else
    {
        Simulator::Schedule(sendPacketTime,
                            &SendPacket,
                            enbNetDev.Get(0),
                            ueNetDev.Get(0)->GetAddress(),
                            udpPacketSize);
    }

    // attach UEs to the closest eNB
    nrHelper->AttachToClosestEnb(ueNetDev, enbNetDev);

    if (enableUl)
    {
        std::cout << "\n Sending data in uplink." << std::endl;
        Simulator::Schedule(Seconds(0.2), &ConnectUlPdcpRlcTraces);
    }
    else
    {
        std::cout << "\n Sending data in downlink." << std::endl;
        Simulator::Schedule(Seconds(0.2), &ConnectPdcpRlcTraces);
    }

    nrHelper->EnableTraces();

    ns3::lorawan::LorawanMacHelper::SetSpreadingFactorsUp(loraNodes, uavNodes, channel);

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
SendPacket(Ptr<NetDevice> device, Address& addr, uint32_t packetSize)
{
    Ptr<Packet> pkt = Create<Packet>(packetSize);
    Ipv4Header ipv4Header;
    ipv4Header.SetProtocol(UdpL4Protocol::PROT_NUMBER);
    pkt->AddHeader(ipv4Header);
    EpsBearerTag tag(1, 1);
    pkt->AddPacketTag(tag);
    device->Send(pkt, addr, Ipv4L3Protocol::PROT_NUMBER);
}

/**
 * Function that prints out PDCP delay. This function is designed as a callback
 * for PDCP trace source.
 * @param path The path that matches the trace source
 * @param rnti RNTI of UE
 * @param lcid logical channel id
 * @param bytes PDCP PDU size in bytes
 * @param pdcpDelay PDCP delay
 */
void
RxPdcpPDU(std::string path, uint16_t rnti, uint8_t lcid, uint32_t bytes, uint64_t pdcpDelay)
{
    std::cout << "\n Packet PDCP delay:" << pdcpDelay << "\n";
    g_rxPdcpCallbackCalled = true;
}

/**
 * Function that prints out RLC statistics, such as RNTI, lcId, RLC PDU size,
 * delay. This function is designed as a callback
 * for RLC trace source.
 * @param path The path that matches the trace source
 * @param rnti RNTI of UE
 * @param lcid logical channel id
 * @param bytes RLC PDU size in bytes
 * @param rlcDelay RLC PDU delay
 */
void
RxRlcPDU(std::string path, uint16_t rnti, uint8_t lcid, uint32_t bytes, uint64_t rlcDelay)
{
    std::cout << "\n\n Data received at RLC layer at:" << Simulator::Now() << "\n rnti:" << rnti
              << "\n lcid:" << (unsigned)lcid << "\n bytes :" << bytes << "\n delay :" << rlcDelay
              << std::endl;
    g_rxRxRlcPDUCallbackCalled = true;
}

/**
 * Function that connects PDCP and RLC traces to the corresponding trace sources.
 */
void
ConnectPdcpRlcTraces()
{
    Config::Connect("/NodeList/*/DeviceList/*/LteUeRrc/DataRadioBearerMap/1/LtePdcp/RxPDU",
                    MakeCallback(&RxPdcpPDU));

    Config::Connect("/NodeList/*/DeviceList/*/LteUeRrc/DataRadioBearerMap/1/LteRlc/RxPDU",
                    MakeCallback(&RxRlcPDU));
}

/**
 * Function that connects UL PDCP and RLC traces to the corresponding trace sources.
 */
void
ConnectUlPdcpRlcTraces()
{
    Config::Connect("/NodeList/*/DeviceList/*/LteEnbRrc/UeMap/*/DataRadioBearerMap/*/LtePdcp/RxPDU",
                    MakeCallback(&RxPdcpPDU));

    Config::Connect("/NodeList/*/DeviceList/*/LteEnbRrc/UeMap/*/DataRadioBearerMap/*/LteRlc/RxPDU",
                    MakeCallback(&RxRlcPDU));
}