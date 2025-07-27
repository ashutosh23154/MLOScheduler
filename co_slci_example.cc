

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/multi-model-spectrum-channel.h"
#include "ns3/network-module.h"
#include "ns3/random-variable-stream.h"
#include "ns3/rng-seed-manager.h"
#include "ns3/ssid.h"
#include "ns3/wifi-co-trace-helper.h"
#include "ns3/wifi-module.h"
#include "ns3/yans-wifi-helper.h"

#include <algorithm>
#include <cassert>
#include <fstream>
#include <iomanip>
#include <iostream>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("MLD_FEEDBACK");

/**
 * In this example, we simulate a MLO scheduler that uses Single Link Less Congested Interface
 * (SLCI) algorithm. SLCI algorithm schedules a new flow on the least congested link. We simulate
 * SLCI on a network with two links and four nodes. We start four uplink flows, one on each node,
 * during the simulation run. We trace and log channel occupancies on both links to a file
 * periodically, the traces are analyzed in jupyter notebook to verify workings of SLCI.
 */

/* A helper function that computes channel occuapancy on a link as observed by a node. */
double
ComputeChannelOccupancy(WifiCoTraceHelper coHelper, size_t nodeId, size_t linkId)
{
    auto& records = coHelper.GetDeviceRecords();

    auto nodeRecord = std::find_if(records.begin(), records.end(), [nodeId](auto& x) {
        return x.m_nodeId == nodeId;
    });
    NS_ASSERT_MSG(nodeRecord != records.end(), "Record not found for node: " << nodeId);

    auto linkDurations = nodeRecord->m_linkStateDurations.find(linkId);
    NS_ASSERT_MSG(linkDurations != nodeRecord->m_linkStateDurations.end(),
                  "Link Statistics not found for linkId: " << linkId << " ,nodeId: " << nodeId);

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

    return (total - idle) / total;
}

/**
 * A helper function that traces and logs channel occupancies on all links.
 */
void
TraceChannelOccupancy(WifiCoTraceHelper& helper, std::ostream& out, Time& periodicInterval)
{
    // Assuming node with id equal to '0' is AP and it operates on two links.
    const auto nodeId = 0;
    auto co_link0 = ComputeChannelOccupancy(helper, nodeId, 0);
    auto co_link1 = ComputeChannelOccupancy(helper, nodeId, 1);

    out << Simulator::Now().GetSeconds() << "," << 0 << "," << co_link0 << "\n";
    out << Simulator::Now().GetSeconds() << "," << 1 << "," << co_link1 << "\n";

    helper.Reset();

    Simulator::Schedule(periodicInterval,
                        &TraceChannelOccupancy,
                        std::ref(helper),
                        std::ref(out),
                        periodicInterval);
}

/*
 * A helper function that configures an uplink flow.
 */
Ptr<PacketSocketClient>
GetClientApplication(const PacketSocketAddress& sockAddr,
                     const std::size_t pktSize,
                     const Time& interval,
                     const Time& startDelay,
                     const Time stop = Time::Max(),
                     const double p_tid3 = 0.5)
{
    auto client = CreateObject<PacketSocketClient>();
    client->SetAttribute("PacketSize", UintegerValue(pktSize));
    client->SetAttribute("MaxPackets", UintegerValue(0));
    client->SetAttribute("Interval", TimeValue(interval));

    /* Tid 0 & 3 are AC_BE. */
    auto tid0 = UintegerValue(0),
    tid3 = UintegerValue(3);
    client->SetAttribute("Priority", tid0);
    client->SetAttribute("OptionalTid", tid3);
    client->SetAttribute("OptionalTidPr",
    DoubleValue(p_tid3));

    client->SetRemote(sockAddr);
    client->SetStartTime(startDelay);
    client->SetStopTime(stop);
    return client;
}

/**
 * A helper function that enables tid-to-link mapping on a node.
 */
void
ConfigureTidToLinkMapping(NetDeviceContainer devices, std::string mapping)
{
    for (size_t i = 0; i < devices.GetN(); i++)
    {
        auto wifiDevice = DynamicCast<WifiNetDevice>(devices.Get(i));
        wifiDevice->GetMac()->GetEhtConfiguration()->SetAttribute(
            "TidToLinkMappingNegSupport",
            EnumValue(WifiTidToLinkMappingNegSupport::ANY_LINK_SET));

        wifiDevice->GetMac()->GetEhtConfiguration()->SetAttribute("TidToLinkMappingUl",
                                                                  StringValue(mapping));
        wifiDevice->GetMac()->GetEhtConfiguration()->SetAttribute("TidToLinkMappingDl",
                                                                  StringValue(mapping));
    }
}

/**
 * Install Packet Socket Server on a node. A Packet Socket client generates an uplink flow and sends
 * it to the server.
 */
void
InstallPacketSocketServer(Ptr<Node> node, Time startAfter, Time stopAfter)
{
    Ptr<WifiNetDevice> device = DynamicCast<WifiNetDevice>(node->GetDevice(0));
    PacketSocketAddress srvAddr;
    srvAddr.SetSingleDevice(device->GetIfIndex());
    srvAddr.SetProtocol(1);
    auto psServer = CreateObject<PacketSocketServer>();
    psServer->SetLocal(srvAddr);
    node->AddApplication(psServer);
    psServer->SetStartTime(startAfter);
    psServer->SetStopTime(stopAfter);
}

/**
 * Install packet socket client that generates an uplink flow on a node.
 */
void
InstallPacketSocketClient(Ptr<Node> client,
                          Ptr<Node> server,
                          double loadInMbps,
                          size_t link,
                          Time startAfter,
                          Time stopAfter)
{
    NS_LOG_INFO("Start Flow on node:" << client->GetId() << " at:" << Simulator::Now()
                                      << " on link:" << link << " with load:" << loadInMbps);

    Ptr<WifiNetDevice> staDevice = DynamicCast<WifiNetDevice>(client->GetDevice(0));

    PacketSocketAddress sockAddr;
    sockAddr.SetSingleDevice(staDevice->GetIfIndex());
    sockAddr.SetPhysicalAddress(server->GetDevice(0)->GetAddress());
    sockAddr.SetProtocol(1);

    size_t packetSizeInBytes = 1000;
    double packetInterval = packetSizeInBytes * 8.0 / loadInMbps;

    /* Find probability of generating tid-3 packets to
    achieve split-ratio of either 1:0 or 0:1 */
    double p_tid3 = (link == 0 ? 0.0 : 1.0);
    auto clientApp = GetClientApplication(sockAddr,
                                          1000,
                                          MicroSeconds(packetInterval),
                                          startAfter,
                                          stopAfter,
                                          p_tid3);
    staDevice->GetNode()->AddApplication(clientApp);
}

/**
 * It schedules a new flow to a link with lower channel occupancy.
 */
void
StartFlow(WifiCoTraceHelper& coHelper, Ptr<Node> sta, Ptr<Node> server, double load)
{
    NS_LOG_INFO("Start Flow at: " << Simulator::Now());

    auto nodeId = sta->GetId();

    auto co_link0 = ComputeChannelOccupancy(coHelper, nodeId, 0);
    auto co_link1 = ComputeChannelOccupancy(coHelper, nodeId, 1);
    auto selectedLinkId = (co_link0 < co_link1 ? 0 : 1);

    NS_LOG_INFO("Selected Link: " << selectedLinkId << " because co_link0: " << co_link0
                                  << " & co_link1: " << co_link1);

    /* Start a new flow on sta. */
    InstallPacketSocketClient(sta,
                              server,
                              load,
                              selectedLinkId,
                              Seconds(0.0) /* Start Immediately */,
                              Time::Max());
}

void
DisableAggregation(NodeContainer nodes)
{
    for (size_t i = 0; i < nodes.GetN(); i++)
    {
        for (uint32_t j = 0; j < nodes.Get(i)->GetNDevices(); j++)
        {
            auto device = DynamicCast<WifiNetDevice>(nodes.Get(i)->GetDevice(j));
            if (!device)
            {
                continue;
            }

            device->GetMac()->SetAttribute("BE_MaxAmpduSize", UintegerValue(0));
            device->GetMac()->SetAttribute("BK_MaxAmpduSize", UintegerValue(0));
            device->GetMac()->SetAttribute("VO_MaxAmpduSize", UintegerValue(0));
            device->GetMac()->SetAttribute("VI_MaxAmpduSize", UintegerValue(0));
        }
    }
}

/**
 * A network setup with configurable number of SLDs & MLDS on two links.
 * MCS = 11, BW = 40 MHz, links are on 5 and 6 GHz bands.
 * We use tid 0 & 3 to route traffic on the two links.
 */

int
main(int argc, char* argv[])
{
    LogComponentEnable("MLD_FEEDBACK", LOG_LEVEL_INFO);

    NS_LOG_INFO("[logs]: Simulation started");
    RngSeedManager::SetSeed(3);
    RngSeedManager::SetRun(2);

    Time m_start{Seconds(1.0)};
    Time m_stop{Seconds(10.0)};

    size_t nWifi{4};
    std::string coTraceFile{"/tmp/ns3_co.csv"};

    CommandLine cmd(__FILE__);

    cmd.AddValue("coTraceFile", "Path of channel occupancy trace file", coTraceFile);

    cmd.Parse(argc, argv);

    NodeContainer ap;
    ap.Create(1);

    NodeContainer sta;
    sta.Create(nWifi);

    NodeContainer m_nodes;
    m_nodes.Add(ap);
    m_nodes.Add(sta);

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
                                 StringValue("EhtMcs11"),
                                 "ControlMode",
                                 StringValue("OfdmRate24Mbps"));
    uint8_t linkId = 0;
    wifi.SetRemoteStationManager(linkId,
                                 "ns3::ConstantRateWifiManager",
                                 "DataMode",
                                 StringValue("EhtMcs11"),
                                 "ControlMode",
                                 StringValue("OfdmRate24Mbps"));

    mac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid));

    NetDeviceContainer m_devices;
    m_devices.Add(wifi.Install(phy, mac, ap));
    mac.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ssid), "ActiveProbing", BooleanValue(false));
    m_devices.Add(wifi.Install(phy, mac, sta));

    MobilityHelper mobility;
    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
    positionAlloc->Add(Vector(0.0, 0.0, 0.0));
    auto distance = 0.1;
    positionAlloc->Add(Vector(distance, 0.0, 0.0));
    mobility.SetPositionAllocator(positionAlloc);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(m_nodes);

    /* TID to link mapping to configure sld on link1, sld on link2 and mld */
    std::string mldMapping = "0 0; 1,2,3,4,5,6,7 1";
    ConfigureTidToLinkMapping(m_devices, mldMapping);

    /* Disable aggregation and set guard interval */
    DisableAggregation(m_nodes);
    int gi = 800; // Guard Interval in nano seconds
    Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/HeConfiguration/GuardInterval",
                TimeValue(NanoSeconds(gi)));

    // Install packet socket helper on all nodes
    PacketSocketHelper packetSocket;
    packetSocket.Install(m_nodes);

    InstallPacketSocketServer(ap.Get(0), m_start, m_stop);

    /* Start uplink flows at nodes every 2 seconds */
    std::vector<double> loads{10.0, 7.0, 7.0, 6.0};
    WifiCoTraceHelper m_wificohelper;
    m_wificohelper.Enable(sta);

    for (size_t i = 1; i <= nWifi; i++)
    {
        Simulator::Schedule(Seconds(i * 2.0 - 0.5),
                            &WifiCoTraceHelper::Reset,
                            std::ref(m_wificohelper));
        Simulator::Schedule(Seconds(i * 2.0),
                            &StartFlow,
                            std::ref(m_wificohelper),
                            sta.Get(i - 1),
                            ap.Get(0),
                            loads.at(i - 1));
    }

    /* Trace channel occupancies on both links during simulation */
    Time periodicity{Seconds(0.25)};
    std::ofstream coTrace{coTraceFile};
    coTrace << std::fixed << std::showpoint << std::setprecision(3);
    WifiCoTraceHelper apCoHelper;
    apCoHelper.Enable(m_nodes.Get(0));
    Simulator::Schedule(m_start,
                        &TraceChannelOccupancy,
                        std::ref(apCoHelper),
                        std::ref(coTrace),
                        periodicity);

    LogComponentEnable("MLD_FEEDBACK", LOG_LEVEL_INFO);

    Simulator::Stop(m_stop);
    Simulator::Run();
    Simulator::Destroy();

    return 0;
}
