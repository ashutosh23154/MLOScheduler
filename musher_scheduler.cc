// This is throughput and not goodput.
// If packet size is 1000 bytes, the mpdu size is coming as 1038 bytes. So, the overhead is 38 bytes.
// In the beginning of the simulation, some 28 bytes acknowledgement packets are also received.
// Although scheduler is independent of AC, but it's logic is based on TID to link mapping. So, if AC are changed, mapping and other things should be checked.
// PacketSocketClient can send only two TIDs. So, in this example, TID 0 is mapped to Link 0 and TID 3 is mapped to Link 1. We can't have more than two TIDs.
// Changing the p_tid3 is not making any difference. It should be checked

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
#include "ns3/packet-socket.h"
#include "ns3/socket.h" 

#include <algorithm>
#include <cassert>
#include <fstream>
#include <iomanip>
#include <map>
#include <iostream>
#include <fstream>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("MLD_FEEDBACK");

std::map<uint32_t, std::map<uint8_t, double>> g_throughputMap;
std::map<uint32_t, std::map<uint8_t, uint64_t>> g_bytesReceived;
std::map<Mac48Address, uint32_t> g_macToNodeId;
std::map<uint16_t, uint8_t> g_tidToLinkMap = {{0, 0}, {3, 1}};

// Add these with your other global variables
std::map<uint32_t, std::map<uint8_t, uint64_t>> g_totalBytesReceived;
std::map<uint32_t, Time> g_flowStartTimes;
std::map<uint32_t, Time> g_flowLastAckTimes;

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

    auto tid1 = UintegerValue(0), tid2 = UintegerValue(3);
    client->SetAttribute("Priority", tid1);
    client->SetAttribute("OptionalTid", tid2);
    client->SetAttribute("OptionalTidPr", DoubleValue(p_tid3));

    client->SetRemote(sockAddr);
    client->SetStartTime(startDelay);
    client->SetStopTime(stop);
    return client;
}

void
UpdateThroughputMap(Time period)
{
    std::ofstream outFile("throughput_with_scheduler.csv", std::ios::app);
    for (auto const& [nodeId, linkMap] : g_bytesReceived)
    {
        for (auto const& [linkId, bytes] : linkMap)
        {
            double throughputMbps = (bytes * 8.0) / (period.GetSeconds() * 1e6);
            g_throughputMap[nodeId][linkId] = throughputMbps;
            outFile << Simulator::Now().GetSeconds() << ","
                    << nodeId << ","
                    << (int)linkId << ","
                    << throughputMbps << std::endl;
            NS_LOG_INFO(Simulator::Now().GetSeconds()
                        << "s, Update: Node " << nodeId << ", Link " << (int)linkId
                        << ", Throughput: " << throughputMbps << " Mbps");
        }
    }
    g_bytesReceived.clear();
    Simulator::Schedule(period, &UpdateThroughputMap, period);
}

void
MusherScheduler(Ptr<Node> sta, Ptr<PacketSocketClient> client, Time periodicity)
{
    uint32_t nodeId = sta->GetId();
    double tp_link0 = g_throughputMap.count(nodeId) && g_throughputMap[nodeId].count(0)
                          ? g_throughputMap[nodeId][0]
                          : 0.0;
    double tp_link1 = g_throughputMap.count(nodeId) && g_throughputMap[nodeId].count(1)
                          ? g_throughputMap[nodeId][1]
                          : 0.0;

    double total_tp = tp_link0 + tp_link1;
    double p_tid3 = (total_tp > 0) ? (tp_link1 / total_tp) : 0.5;

    client->SetAttribute("OptionalTidPr", DoubleValue(p_tid3));

    NS_LOG_INFO(Simulator::Now().GetSeconds()
                << "s, Scheduler: Node " << nodeId << ", TP(L0/L1): " << tp_link0 << "/"
                << tp_link1 << " Mbps. Set new split ratio for Link 1 (p_tid3) to: " << p_tid3);

    Simulator::Schedule(periodicity, &MusherScheduler, sta, client, periodicity);
}

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

Ptr<PacketSocketClient>
InstallPacketSocketClient(Ptr<Node> client,
                          Ptr<Node> server,
                          double loadInMbps,
                          Time startAfter,
                          Time stopAfter)
{
    NS_LOG_INFO("Start Flow on node:" << client->GetId() << " at:" << Simulator::Now()
                                  << " with load:" << loadInMbps << " Mbps");

    Ptr<WifiNetDevice> staDevice = DynamicCast<WifiNetDevice>(client->GetDevice(0));

    PacketSocketAddress sockAddr;
    sockAddr.SetSingleDevice(staDevice->GetIfIndex());
    sockAddr.SetPhysicalAddress(server->GetDevice(0)->GetAddress());
    sockAddr.SetProtocol(1);

    size_t packetSizeInBytes = 1000;
    double packetInterval = packetSizeInBytes * 8.0 / loadInMbps;

    double p_tid3 = 0.5;
    auto clientApp = GetClientApplication(sockAddr,
                                          1000,
                                          MicroSeconds(packetInterval),
                                          startAfter,
                                          stopAfter,
                                          p_tid3);
    staDevice->GetNode()->AddApplication(clientApp);
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

void
AckedMpduCallback(Ptr<const WifiMpdu> mpdu)
{
    if (!mpdu->GetHeader().IsData() || mpdu->GetHeader().GetAddr1().IsGroup())
    {
        return;
    }

    Mac48Address senderMac = mpdu->GetHeader().GetAddr2();

    if (g_macToNodeId.count(senderMac))
    {
        uint32_t nodeId = g_macToNodeId[senderMac];
        std::set<uint8_t> links = mpdu->GetInFlightLinkIds();
        uint32_t mpduSize = mpdu->GetSize();

        g_flowLastAckTimes[nodeId] = Simulator::Now();

        if (!links.empty())
        {
            // This is the normal case for a correctly identified MLO frame.
            for (auto const& linkId : links)
            {
                // NS_LOG_INFO(Simulator::Now().GetSeconds()
                //             << "s, AckedMpdu (MLO): Node " << nodeId << ", Link " << (int)linkId
                //             << ", MPDU size: " << mpduSize << " bytes");
                g_bytesReceived[nodeId][linkId] += mpduSize;
                g_totalBytesReceived[nodeId][linkId] += mpduSize;
            }
        }
        else
        {
            // FALLBACK: This is a non-MLO frame. Check its TID to determine the intended link.
            if (mpdu->GetHeader().IsQosData())
            {
                uint8_t tid = mpdu->GetHeader().GetQosTid();
                if (g_tidToLinkMap.count(tid))
                {
                    uint8_t linkId = g_tidToLinkMap.at(tid);
                    // NS_LOG_INFO(Simulator::Now().GetSeconds()
                    //             << "s, AckedMpdu (non-MLO Fallback): Node " << nodeId << ", Link " << (int)linkId
                    //             << " (from TID " << (int)tid << "), MPDU size: " << mpduSize << " bytes");
                    g_bytesReceived[nodeId][linkId] += mpduSize;
                    g_totalBytesReceived[nodeId][linkId] += mpduSize;
                }
            }
        }
    }
}

void
PrintFinalResults()
{
    std::cout << "\n----------------------------------------------------" << std::endl;
    std::cout << "---           Final Simulation Results           ---" << std::endl;
    std::cout << "----------------------------------------------------" << std::endl;

    for (auto const& [nodeId, linkMap] : g_totalBytesReceived)
    {
        uint64_t totalBytesForNode = 0;
        for (auto const& [linkId, bytes] : linkMap)
        {
            totalBytesForNode += bytes;
        }

        Time startTime = g_flowStartTimes[nodeId];
        Time lastTime = g_flowLastAckTimes[nodeId];
        
        if (lastTime <= startTime) continue; // Avoid division by zero if no packets were acked

        Time duration = lastTime - startTime;
        double throughputMbps = (totalBytesForNode * 8.0) / (duration.GetSeconds() * 1e6);

        std::cout << "Node " << nodeId << ":" << std::endl;
        std::cout << "  - Total Bytes Received: " << totalBytesForNode << std::endl;
        std::cout << "  - Flow Duration:        " << duration.GetSeconds() << " s" << std::endl;
        std::cout << "  - Average Throughput:   " << throughputMbps << " Mbps" << std::endl;
        std::cout << std::endl;
    }
}

int
main(int argc, char* argv[])
{
    std::ofstream("throughput_with_scheduler.csv").close();
    LogComponentEnable("MLD_FEEDBACK", LOG_LEVEL_INFO);

    NS_LOG_INFO("[logs]: Simulation started");
    RngSeedManager::SetSeed(3);
    RngSeedManager::SetRun(2);

    Time m_start{Seconds(1.0)};
    Time m_stop{Seconds(20.0)};

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
    NetDeviceContainer ap_devices = wifi.Install(phy, mac, ap);

    mac.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ssid), "ActiveProbing", BooleanValue(false));
    NetDeviceContainer sta_devices = wifi.Install(phy, mac, sta);

    NetDeviceContainer m_devices;
    m_devices.Add(ap_devices);
    m_devices.Add(sta_devices);

    // Replace the entire mobility block with this:
    MobilityHelper mobility;
    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
    positionAlloc->Add(Vector(0.0, 0.0, 0.0)); // AP at the origin
    double distance = 5.0; // Start STAs 5 meters away from the AP
    for (size_t i = 0; i < nWifi; ++i)
    {
        // Place STAs at (5,0,0), (6,0,0), etc. to give them unique positions
        positionAlloc->Add(Vector(distance + i, 0.0, 0.0));
    }
    mobility.SetPositionAllocator(positionAlloc);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(m_nodes);

    std::string mldMapping = "0 0; 1,2,3,4,5,6,7 1";
    ConfigureTidToLinkMapping(m_devices, mldMapping);

    DisableAggregation(m_nodes);
    int gi = 800;
    Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/HeConfiguration/GuardInterval",
                TimeValue(NanoSeconds(gi)));

    PacketSocketHelper packetSocket;
    packetSocket.Install(m_nodes);

    // AFTER (in main function, after InstallPacketSocketServer call)
    InstallPacketSocketServer(ap.Get(0), m_start, m_stop);

    for (uint32_t i = 0; i < sta.GetN(); ++i)
    {
        Ptr<WifiNetDevice> dev = DynamicCast<WifiNetDevice>(sta.Get(i)->GetDevice(0));
        g_macToNodeId[dev->GetMac()->GetAddress()] = sta.Get(i)->GetId();
        for (uint8_t linkId = 0; linkId < dev->GetMac()->GetNLinks(); ++linkId)
        {
            Mac48Address linkAddress = dev->GetMac()->GetFrameExchangeManager(linkId)->GetAddress();
            g_macToNodeId[linkAddress] = sta.Get(i)->GetId();
            // NS_LOG_INFO("Mapping Node " << sta.Get(i)->GetId() << ", Link " << (int)linkId << " MAC " << linkAddress);
        }
        dev->GetMac()->TraceConnectWithoutContext("AckedMpdu", MakeCallback(&AckedMpduCallback));
    }

    Time schedulerPeriodicity = Seconds(0.25);
    Simulator::Schedule(m_start + schedulerPeriodicity, &UpdateThroughputMap, schedulerPeriodicity);

    std::vector<double> loads{8.0, 8.0, 8.0, 8.0};

    for (size_t i = 1; i <= nWifi; i++)
    {
        // Time startTime = i * Seconds(2.0);
        Time startTime = m_start + Seconds((i - 1) * 0.5);
        // Time startTime = m_start;

        g_flowStartTimes[sta.Get(i - 1)->GetId()] = startTime;

        auto clientApp =
            InstallPacketSocketClient(sta.Get(i - 1), ap.Get(0), loads.at(i - 1), startTime, m_stop);

        Simulator::Schedule(startTime + schedulerPeriodicity,
                            &MusherScheduler,
                            sta.Get(i - 1),
                            clientApp,
                            schedulerPeriodicity);
    }   

    Simulator::Schedule(m_stop, &PrintFinalResults);

    Simulator::Stop(m_stop);
    Simulator::Run();
    Simulator::Destroy();

    return 0;
}