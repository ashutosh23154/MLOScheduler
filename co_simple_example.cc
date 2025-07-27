/*
 * Copyright (c) 2024 Indraprastha Institute of Information Technology Delhi
 * SPDX-License-Identifier: GPL-2.0-only
 */
#include "ns3/boolean.h"
#include "ns3/command-line.h"
#include "ns3/config.h"
#include "ns3/double.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/log.h"
#include "ns3/mobility-helper.h"
#include "ns3/mobility-model.h"
#include "ns3/multi-model-spectrum-channel.h"
#include "ns3/names.h"
#include "ns3/on-off-helper.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/ssid.h"
#include "ns3/string.h"
#include "ns3/uinteger.h"
#include "ns3/wifi-co-trace-helper.h"
#include "ns3/wifi-module.h"

#include <iomanip>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("WifiCoTraceSimpleExample");

/**
 * In this example, we enable WifiCoTraceHelper on two nodes of a network. We print WifiPhy State
 * durations on their links when simulation is over. In addition, we compute and print channel
 * occupancy on one of the links.
 */
int
main(int argc, char* argv[])
{
    Time duration{Seconds(4)};

    NodeContainer apNode(1);
    Names::Add("AP", apNode.Get(0));
    NodeContainer staNode(1);
    Names::Add("STA", staNode.Get(0));

    MobilityHelper mobility;
    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
    positionAlloc->Add(Vector(0.0, 0.0, 0.0));
    mobility.SetPositionAllocator(positionAlloc);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(apNode);
    positionAlloc = CreateObject<ListPositionAllocator>();
    double distance = 1; // meters
    positionAlloc->Add(Vector(distance, 0.0, 0.0));
    mobility.Install(staNode);

    WifiMacHelper mac;
    Ssid ssid = Ssid("ns-3-ssid");

    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211be);

    // Create multiple spectrum channels
    Ptr<MultiModelSpectrumChannel> spectrumChannel5Ghz = CreateObject<MultiModelSpectrumChannel>();
    Ptr<MultiModelSpectrumChannel> spectrumChannel6Ghz = CreateObject<MultiModelSpectrumChannel>();

    SpectrumWifiPhyHelper phy(2);
    phy.SetPcapDataLinkType(WifiPhyHelper::DLT_IEEE802_11_RADIO);
    phy.AddChannel(spectrumChannel5Ghz, WIFI_SPECTRUM_5_GHZ);
    phy.AddChannel(spectrumChannel6Ghz, WIFI_SPECTRUM_6_GHZ);

    // configure operating channel for each link
    phy.Set(0, "ChannelSettings", StringValue("{0, 40, BAND_5GHZ, 0}"));
    phy.Set(1, "ChannelSettings", StringValue("{0, 40, BAND_6GHZ, 0}"));

    // configure rate manager for each link
    wifi.SetRemoteStationManager(1,
                                 "ns3::ConstantRateWifiManager",
                                 "DataMode",
                                 StringValue("EhtMcs0"),
                                 "ControlMode",
                                 StringValue("OfdmRate24Mbps"));
    uint8_t linkId = 0;
    wifi.SetRemoteStationManager(linkId,
                                 "ns3::ConstantRateWifiManager",
                                 "DataMode",
                                 StringValue("EhtMcs0"),
                                 "ControlMode",
                                 StringValue("OfdmRate24Mbps"));

    mac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid));

    NetDeviceContainer allDevices;
    allDevices.Add(wifi.Install(phy, mac, apNode));
    mac.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ssid), "ActiveProbing", BooleanValue(false));
    allDevices.Add(wifi.Install(phy, mac, staNode));

    InternetStackHelper internet;
    internet.Install(apNode);
    internet.Install(staNode);

    Ipv4AddressHelper ipv4;
    ipv4.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer i = ipv4.Assign(allDevices);

    uint16_t portNumber = 9;
    auto ipv4ap = apNode.Get(0)->GetObject<Ipv4>();
    const auto address = ipv4ap->GetAddress(1, 0).GetLocal();

    ApplicationContainer sourceApplications;
    ApplicationContainer sinkApplications;

    InetSocketAddress sinkAddress(address, portNumber);
    PacketSinkHelper packetSinkHelper("ns3::UdpSocketFactory", sinkAddress);
    sinkApplications.Add(packetSinkHelper.Install(apNode.Get(0)));

    OnOffHelper onOffHelper("ns3::UdpSocketFactory", sinkAddress);
    onOffHelper.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
    onOffHelper.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));
    onOffHelper.SetAttribute("DataRate", DataRateValue(9000000)); // bits/sec
    onOffHelper.SetAttribute("PacketSize", UintegerValue(1472));  // bytes
    std::vector<uint8_t> tosValues = {0x70, 0x28, 0xb8, 0xc0};    // AC_BE, AC_BK, AC_VI, AC_VO
    onOffHelper.SetAttribute("Tos", UintegerValue(tosValues[0])); // AC_BE
    sourceApplications.Add(onOffHelper.Install(staNode.Get(0)));

    sinkApplications.Start(Seconds(0.0));
    sinkApplications.Stop(Seconds(6.0));
    sourceApplications.Start(Seconds(1.0));
    sourceApplications.Stop(Seconds(5.0));

    Time start{Seconds(1.0)};
    Time stop{Seconds(5.0)};
    WifiCoTraceHelper wifiCoTraceHelper(start, stop);

    wifiCoTraceHelper.Enable(apNode);
    wifiCoTraceHelper.Enable(staNode);

    Simulator::Stop(Seconds(6.0));
    Simulator::Run();

    // The following provide some examples of how to access and print the trace helper contents.
    std::cout << "*** Print statistics for all nodes using built-in print method:" << std::endl;
    wifiCoTraceHelper.PrintStatistics(std::cout);

    auto& records = wifiCoTraceHelper.GetDeviceRecords();

    size_t apNodeId = 0;
    auto apDeviceRecord = std::find_if(records.begin(), records.end(), [apNodeId](auto& x) {
        return x.m_nodeId == apNodeId;
    });
    NS_ASSERT_MSG(apDeviceRecord != records.end(), "Record not found for node: " << apNodeId);

    size_t link = 0;
    auto linkDurations = apDeviceRecord->m_linkStateDurations.find(link);
    NS_ASSERT_MSG(linkDurations != apDeviceRecord->m_linkStateDurations.end(),
                  "Link Statistics not found for link: " << link << " ,nodeId: " << apNodeId);

    auto& stats = linkDurations->second;
    double idle = 0;
    double total = 0;

    for (auto& entry : stats)
    {
        if (entry.first == WifiPhyState::IDLE)
        {
            idle = entry.second.GetDouble();
        }
        total += entry.second.GetDouble();
    }

    double channelOccupancy = (total - idle) / total;

    std::cout << std::fixed << std::showpoint << std::setprecision(4);
    std::cout << "*** Channel occupancy as observed by AP on Link#0 is: " << channelOccupancy
              << " ***\n";

    Simulator::Destroy();

    return 0;
}
