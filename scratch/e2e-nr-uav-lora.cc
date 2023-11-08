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

#include "ns3/command-line.h"
#include "ns3/core-module.h"
#include "ns3/lorawan-module.h"
#include "ns3/mobility-module.h"
#include "ns3/nr-module.h"

NS_LOG_COMPONENT_DEFINE("E2E_NR_UAV_LORA");

using namespace ns3;
using namespace lorawan;

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

    // NR settings
    uint16_t numerologyBwp1 = 4;
    double centralFrequencyBand1 = 28e9;
    double bandwidthBand1 = 100e6;

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
    }

    if (n_uavs < 1 || n_bss < 1 || n_loraeds < 1)
    {
        NS_LOG_ERROR("Number of nodes must be greater than 0");
        return 1;
    }

    uavNodes.Create(n_uavs);
    nrNodes.Create(n_bss);
    loraNodes.Create(n_loraeds);

    Ptr<PropagationDelayModel> delay = CreateObject<ConstantSpeedPropagationDelayModel>();
    Ptr<LogDistancePropagationLossModel> loss = CreateObject<LogDistancePropagationLossModel>();
    loss->SetPathLossExponent(3.76);
    loss->SetReference(1, 10.0);
    Ptr<LoraChannel> channel = CreateObject<LoraChannel>(loss, delay);

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

    return 0;
}