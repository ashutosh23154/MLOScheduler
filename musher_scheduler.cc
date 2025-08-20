#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/multi-model-spectrum-channel.h"
#include "ns3/network-module.h"
#include "ns3/packet-socket.h"
#include "ns3/rng-seed-manager.h"
#include "ns3/ssid.h"
#include "ns3/wifi-module.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/wifi-mac-header.h"
#include "ns3/socket.h" // For SocketPriorityTag

#include <iomanip>
#include <iostream>
#include <string>
#include <map>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("ThroughputSchedulerExample");

// --------------------------------------------------------------------------------
// 1. GLOBAL DATA STRUCTURES
// --------------------------------------------------------------------------------
// Global map to store latest throughput [nodeId -> [linkId -> throughput Mbps]]
std::map<uint32_t, std::map<uint8_t, double>> g_throughputMap;
// Map to store received bytes during a calculation period [nodeId -> [linkId -> bytes]]
std::map<uint32_t, std::map<uint8_t, uint64_t>> g_bytesReceived;
// Utility map to resolve MAC address to Node ID
std::map<Mac48Address, uint32_t> g_macToNodeId;
// TID-to-Link mapping knowledge: TID 0 -> Link 0, TID 3 -> Link 1
std::map<uint16_t, uint8_t> g_tidToLinkMap = {{0, 0}, {3, 1}};

/**
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

    /* Tid 0 & 3 are AC_BE. We use them to route to Link 0 and Link 1. */
    client->SetAttribute("Priority", UintegerValue(0));    // Corresponds to TID 0
    client->SetAttribute("OptionalTid", UintegerValue(3)); // Corresponds to TID 3
    client->SetAttribute("OptionalTidPr", DoubleValue(p_tid3));

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

// --------------------------------------------------------------------------------
// 2. THROUGHPUT MONITORING CALLBACK (AT AP)
// --------------------------------------------------------------------------------
/**
 * This callback is connected to the Rx trace of the AP's PacketSocketServer.
 * It inspects every received packet to log bytes per node, per link.
 */

void
RxCallback(Ptr<const Packet> packet, const Address& from)
{
    PacketSocketAddress socketAddress = PacketSocketAddress::ConvertFrom(from);
    Mac48Address fromMac = Mac48Address::ConvertFrom(socketAddress.GetPhysicalAddress());
    if (g_macToNodeId.count(fromMac))
    {
        uint32_t nodeId = g_macToNodeId[fromMac];
        SocketPriorityTag priorityTag;
        if (packet->PeekPacketTag(priorityTag))
        {
            NS_LOG_INFO("Received packet from Node " << nodeId
                                                      << " with priority tag: "
                                                      << priorityTag.GetPriority());
            // The priority set by the client is equivalent to the TID for our mapping.
            uint16_t tid = priorityTag.GetPriority(); 
            if (g_tidToLinkMap.count(tid))
            {
                uint8_t linkId = g_tidToLinkMap[tid];
                g_bytesReceived[nodeId][linkId] += packet->GetSize();
            }
        } 
        // else {
        //     NS_LOG_INFO("Received packet from Node " << nodeId
        //                                               << " without priority tag");
        // }
    }
}

// --------------------------------------------------------------------------------
// 3. PERIODIC THROUGHPUT CALCULATOR
// --------------------------------------------------------------------------------
/**
 * This function is scheduled periodically to calculate throughput from the
 * collected byte counts and update the global throughput map.
 */
void
UpdateThroughputMap(Time period)
{
    for (auto const& [nodeId, linkMap] : g_bytesReceived)
    {
        for (auto const& [linkId, bytes] : linkMap)
        {
            double throughputMbps = (bytes * 8.0) / (period.GetSeconds() * 1e6);
            g_throughputMap[nodeId][linkId] = throughputMbps;
            NS_LOG_INFO(Simulator::Now().GetSeconds()
                        << "s, Update: Node " << nodeId << ", Link " << (int)linkId
                        << ", Throughput: " << throughputMbps << " Mbps");
        }
    }
    // Reset the byte counter for the next interval
    g_bytesReceived.clear();
    // Reschedule the next update
    Simulator::Schedule(period, &UpdateThroughputMap, period);
}

// --------------------------------------------------------------------------------
// 4. THE "MUSHER-LIKE" SCHEDULER (AT STA)
// --------------------------------------------------------------------------------
/**
 * Periodically adjusts the traffic split-ratio for a STA based on the
 * throughput values in the global map.
 */
void
MusherScheduler(Ptr<Node> sta, Ptr<PacketSocketClient> client, Time periodicity)
{
    uint32_t nodeId = sta->GetId();
    // Get throughput for link 0 and link 1 from the global map
    double tp_link0 = g_throughputMap.count(nodeId) && g_throughputMap[nodeId].count(0)
                          ? g_throughputMap[nodeId][0]
                          : 0.0;
    double tp_link1 = g_throughputMap.count(nodeId) && g_throughputMap[nodeId].count(1)
                          ? g_throughputMap[nodeId][1]
                          : 0.0;

    double total_tp = tp_link0 + tp_link1;
    // Probability for Optional TID (TID 3 -> Link 1) is proportional to link 1's throughput
    double p_tid3 = (total_tp > 0) ? (tp_link1 / total_tp) : 0.5; // Default to 50/50 split

    client->SetAttribute("OptionalTidPr", DoubleValue(p_tid3));

    NS_LOG_INFO(Simulator::Now().GetSeconds()
                << "s, Scheduler: Node " << nodeId << ", TP(L0/L1): " << tp_link0 << "/"
                << tp_link1 << " Mbps. Set new split ratio for Link 1 (p_tid3) to: " << p_tid3);

    // Reschedule the next scheduler decision
    Simulator::Schedule(periodicity, &MusherScheduler, sta, client, periodicity);
}

/**
 * Install Packet Socket Server on a node.
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
    // Connect our RxCallback to the server's Rx trace
    psServer->TraceConnectWithoutContext("Rx", MakeCallback(&RxCallback));
}

/**
 * Install packet socket client that generates an uplink flow on a node.
 */
Ptr<PacketSocketClient>
InstallPacketSocketClient(Ptr<Node> client,
                          Ptr<Node> server,
                          double loadInMbps,
                          Time startAfter,
                          Time stopAfter)
{
    Ptr<WifiNetDevice> staDevice = DynamicCast<WifiNetDevice>(client->GetDevice(0));
    PacketSocketAddress sockAddr;
    sockAddr.SetSingleDevice(staDevice->GetIfIndex());
    sockAddr.SetPhysicalAddress(server->GetDevice(0)->GetAddress());
    sockAddr.SetProtocol(1);

    size_t packetSizeInBytes = 1000;
    double packetInterval = packetSizeInBytes * 8.0 / loadInMbps;

    auto clientApp = GetClientApplication(sockAddr,
                                          packetSizeInBytes,
                                          MicroSeconds(packetInterval),
                                          startAfter,
                                          stopAfter);
    client->AddApplication(clientApp);
    return clientApp;
}

void
DisableAggregation(NodeContainer nodes)
{
    for (size_t i = 0; i < nodes.GetN(); i++)
    {
        for (uint32_t j = 0; j < nodes.Get(i)->GetNDevices(); j++)
        {
            auto device = DynamicCast<WifiNetDevice>(nodes.Get(i)->GetDevice(j));
            if (!device) continue;
            device->GetMac()->SetAttribute("BE_MaxAmpduSize", UintegerValue(0));
        }
    }
}

void
AddPriorityTag(Ptr<const Packet> pkt, WifiMacHeader const &hdr)
{
    // Check if the tag already exists to avoid duplicate tags
    SocketPriorityTag tag;
    NS_LOG_INFO("Adding priority tag to packet");
    if (!pkt->PeekPacketTag(tag)) {
        // For example, choose TID based on a per-node or per-app logic
        // Here, as an example: use TID 0 for even STAs, TID 3 for odd STAs
        uint16_t tid = (hdr.GetQosTid() == 0) ? 0 : 3;
        tag.SetPriority(tid);
        pkt->AddPacketTag(tag);
    }
}

/**
 * Main simulation setup.
 */
int
main(int argc, char* argv[])
{
    LogComponentEnable("ThroughputSchedulerExample", LOG_LEVEL_INFO);
    RngSeedManager::SetSeed(3);
    RngSeedManager::SetRun(2);

    Time m_start{Seconds(1.0)};
    Time m_stop{Seconds(10.0)};
    size_t nWifi{4};

    CommandLine cmd(__FILE__);
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

    Ptr<MultiModelSpectrumChannel> spectrumChannel5Ghz = CreateObject<MultiModelSpectrumChannel>();
    Ptr<MultiModelSpectrumChannel> spectrumChannel6Ghz = CreateObject<MultiModelSpectrumChannel>();

    SpectrumWifiPhyHelper phy(2);
    phy.AddChannel(spectrumChannel5Ghz, WIFI_SPECTRUM_5_GHZ);
    phy.AddChannel(spectrumChannel6Ghz, WIFI_SPECTRUM_6_GHZ);

    phy.Set(0, "ChannelSettings", StringValue("{0, 40, BAND_5GHZ, 0}"));
    phy.Set(1, "ChannelSettings", StringValue("{0, 40, BAND_6GHZ, 0}"));
    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                 "DataMode",
                                 StringValue("EhtMcs11"));

    mac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid));
    // mac.SetType("ns3::ApWifiMac",
    //         "Ssid", SsidValue(ssid),
    //         "QosSupported", BooleanValue(true));  // <-- added
    NetDeviceContainer ap_devices = wifi.Install(phy, mac, ap);

    mac.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ssid), "ActiveProbing", BooleanValue(false));
    // mac.SetType("ns3::StaWifiMac",
    //         "Ssid", SsidValue(ssid),
    //         "QosSupported", BooleanValue(true),   // <-- added
    //         "ActiveProbing", BooleanValue(false));
    NetDeviceContainer sta_devices = wifi.Install(phy, mac, sta);

    MobilityHelper mobility;
    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
    positionAlloc->Add(Vector(0.0, 0.0, 0.0));
    auto distance = 0.1;
    positionAlloc->Add(Vector(distance, 0.0, 0.0));
    mobility.SetPositionAllocator(positionAlloc);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    // mobility.SetPositionAllocator("ns3::GridPositionAllocator",
    //                               "MinX",
    //                               DoubleValue(1.0),
    //                               "MinY",
    //                               DoubleValue(1.0),
    //                               "DeltaX",
    //                               DoubleValue(1.0),
    //                               "DeltaY",
    //                               DoubleValue(1.0),
    //                               "GridWidth",
    //                               UintegerValue(5),
    //                               "LayoutType",
    //                               StringValue("RowFirst"));
    // mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(m_nodes);

    /* TID 0 maps to link 0; TIDs 1-7 map to link 1. We use TID 0 and TID 3. */
    std::string mldMapping = "0 0; 1,2,3,4,5,6,7 1";
    ConfigureTidToLinkMapping(ap_devices, mldMapping);
    ConfigureTidToLinkMapping(sta_devices, mldMapping);

    // InternetStackHelper stack;
    // stack.Install(m_nodes);

    DisableAggregation(m_nodes);
    Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/HeConfiguration/GuardInterval",
                TimeValue(NanoSeconds(800)));

    PacketSocketHelper packetSocket;
    packetSocket.Install(m_nodes);

    InstallPacketSocketServer(ap.Get(0), m_start, m_stop);

    // Populate the MAC-to-NodeID map for easy lookup in the RxCallback
    for (uint32_t i = 0; i < sta.GetN(); ++i)
    {
        Ptr<WifiNetDevice> dev = DynamicCast<WifiNetDevice>(sta.Get(i)->GetDevice(0));
        g_macToNodeId[dev->GetMac()->GetAddress()] = sta.Get(i)->GetId();
    }

    for (uint32_t i = 0; i < sta.GetN(); ++i)
    {
        Ptr<WifiNetDevice> dev = DynamicCast<WifiNetDevice>(sta.Get(i)->GetDevice(0));
        dev->GetMac()->TraceConnectWithoutContext("Tx", MakeCallback(&AddPriorityTag));
    }

    Time schedulerPeriodicity = Seconds(0.25);
    // Schedule the first call to the throughput calculator
    Simulator::Schedule(m_start + schedulerPeriodicity, &UpdateThroughputMap, schedulerPeriodicity);

    std::vector<double> loads{10.0, 7.0, 7.0, 6.0};
    for (size_t i = 1; i <= nWifi; i++)
    {
        Time startTime = i * Seconds(2.0);
        auto clientApp =
            InstallPacketSocketClient(sta.Get(i - 1), ap.Get(0), loads.at(i - 1), startTime, m_stop);

        // Schedule the Musher scheduler for this client to start and run periodically
        Simulator::Schedule(startTime,
                            &MusherScheduler,
                            sta.Get(i - 1),
                            clientApp,
                            schedulerPeriodicity);
    }

    Simulator::Stop(m_stop + Seconds(1.0));
    Simulator::Run();
    Simulator::Destroy();

    return 0;
}