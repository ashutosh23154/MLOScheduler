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
#include <map>
#include <iostream>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("MLD_FEEDBACK");

std::map<std::pair<uint32_t, uint8_t>, double> minRttPerNodeTid;

class MloPacketMarker : public Header {
public:
    MloPacketMarker() : m_isMloPacket(false), m_nodeId(0), m_tid(0) {}

    static TypeId GetTypeId() {
        static TypeId tid = TypeId("MloPacketMarker")
            .SetParent<Header>()
            .AddConstructor<MloPacketMarker>();
        return tid;
    }

    void SetMloPacket(bool val) { m_isMloPacket = val; }
    bool IsMloPacket() const { return m_isMloPacket; }

    void SetNodeId(uint32_t id) { m_nodeId = id; }
    uint32_t GetNodeId() const { return m_nodeId; }

    void SetTid(uint8_t tid) { m_tid = tid; }
    uint8_t GetTid() const { return m_tid; }

    TypeId GetInstanceTypeId() const override { return GetTypeId(); }

    void Serialize(Buffer::Iterator start) const override {
        start.WriteU8(m_isMloPacket);
        start.WriteU8(m_tid);
        start.WriteU32(m_nodeId);
    }

    uint32_t Deserialize(Buffer::Iterator start) override {
        m_isMloPacket = start.ReadU8();
        m_tid = start.ReadU8();
        m_nodeId = start.ReadU32();
        return GetSerializedSize();
    }

    uint32_t GetSerializedSize() const override { return 6; }

    void Print(std::ostream &os) const override {
        os << "MLO_Packet:" << m_isMloPacket << " TID:" << (int)m_tid << " Node:" << m_nodeId;
    }

private:
    bool m_isMloPacket;
    uint32_t m_nodeId;
    uint8_t m_tid;
};


class TimeHeader : public Header
{
private:
    double m_time;

public:
    TimeHeader() : m_time(0.0) {}
    TimeHeader(double time) : m_time(time) {}

    void SetTime(double time) { m_time = time; }
    double GetTime() const { return m_time; }

    static TypeId GetTypeId()
    {
        static TypeId tid = TypeId("TimeHeader")
                                .SetParent<Header>()
                                .AddConstructor<TimeHeader>();
        return tid;
    }

    TypeId GetInstanceTypeId() const override { return GetTypeId(); }

    void Serialize(Buffer::Iterator start) const override
    {
        start.WriteHtonU64(static_cast<uint64_t>(m_time * 1e9)); // microseconds
    }

    uint32_t Deserialize(Buffer::Iterator start) override
    {
        uint64_t nanos = start.ReadNtohU64();
        m_time = nanos / 1e9; // still stored internally as seconds
        return GetSerializedSize();
    }

    uint32_t GetSerializedSize() const override { return 8; }

    void Print(std::ostream& os) const override
    {
        os << "SendTime=" << m_time;
    }
};


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
    auto tid1 = UintegerValue(0), tid2 = UintegerValue(3);
    client->SetAttribute("Priority", tid1);
    client->SetAttribute("OptionalTid", tid2);
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

static void
OnPacketRx(Ptr<const Packet> p, const Address &from)
{
    Ptr<Packet> copy = p->Copy();

    MloPacketMarker marker;
    copy->RemoveHeader(marker);

    if (marker.IsMloPacket())
    {
        TimeHeader timeHdr;
        copy->RemoveHeader(timeHdr);

        double recvTime = Simulator::Now().GetSeconds();
        double sendTime = timeHdr.GetTime();
        double latencyNs = (recvTime - sendTime) * 1e9;

        uint32_t nodeId = marker.GetNodeId();
        uint8_t tid = marker.GetTid();
        auto key = std::make_pair(nodeId, tid);

        // Update min RTT
        if (minRttPerNodeTid.find(key) == minRttPerNodeTid.end()) {
            minRttPerNodeTid[key] = latencyNs;
        } else {
            // minRttPerNodeTid[key] = std::min(minRttPerNodeTid[key], latencyNs);
            minRttPerNodeTid[key] = latencyNs;
        }

        // 🪵 Enhanced logging output
        // NS_LOG_INFO("[RX] ACK received at: " << std::fixed << std::setprecision(6) << recvTime << " s"
        //               << " | From Node: " << nodeId
        //               << " | TID: " << static_cast<int>(tid)
        //               << " | RTT: " << std::fixed << std::setprecision(2) << latencyNs << " ns"
        //               << " | MinRTT[" << nodeId << "][TID " << static_cast<int>(tid) << "]: "
        //               << std::fixed << std::setprecision(2) << minRttPerNodeTid[key] << " ns");
    }
}

/**
 * Install Packet Socket Server on a node. A Packet Socket client generates an uplink flow and sends
 * it to the server.
 */
void InstallPacketSocketServer(Ptr<Node> node, Time startAfter, Time stopAfter) {
    Ptr<WifiNetDevice> device = DynamicCast<WifiNetDevice>(node->GetDevice(0));
    PacketSocketAddress srvAddr;
    srvAddr.SetSingleDevice(device->GetIfIndex());
    srvAddr.SetProtocol(1);

    auto psServer = CreateObject<PacketSocketServer>();
    psServer->TraceConnectWithoutContext("Rx", MakeCallback(&OnPacketRx));

    psServer->SetLocal(srvAddr);
    node->AddApplication(psServer);
    psServer->SetStartTime(startAfter);
    psServer->SetStopTime(stopAfter);
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
    NS_LOG_INFO("Start Flow on node:" << client->GetId() << " at:" << Simulator::Now()
                                      << " on link:" << link << " with load:" << loadInMbps);

    Ptr<WifiNetDevice> staDevice = DynamicCast<WifiNetDevice>(client->GetDevice(0));

    PacketSocketAddress sockAddr;
    sockAddr.SetSingleDevice(staDevice->GetIfIndex());
    sockAddr.SetPhysicalAddress(server->GetDevice(0)->GetAddress());
    sockAddr.SetProtocol(1);

    size_t packetSizeInBytes = 1000;
    double packetInterval = packetSizeInBytes * 8.0 / loadInMbps;

    double p_tid3 = 0.0;
    auto clientApp = GetClientApplication(sockAddr,
                                          1000,
                                          MicroSeconds(packetInterval),
                                          startAfter,
                                          stopAfter,
                                          p_tid3);
    staDevice->GetNode()->AddApplication(clientApp);
    return clientApp;
}

/**
 * It periodically adjusts split-ratio of a flow using channel occupancies.
 */
void
MCAB_Scheduler(WifiCoTraceHelper& coHelper, Ptr<Node> sta, Ptr<PacketSocketClient> client)
{
    NS_LOG_INFO("Adjust Flow at: " << Simulator::Now() << " for node: " << sta->GetId());

    uint32_t nodeId = sta->GetId();
    auto key0 = std::make_pair(nodeId, 0); // TID 0 maps to link 0
    auto key3 = std::make_pair(nodeId, 3); // TID 3 maps to link 1

    double rtt0 = minRttPerNodeTid.count(key0) ? minRttPerNodeTid[key0] : 1e9;
    double rtt3 = minRttPerNodeTid.count(key3) ? minRttPerNodeTid[key3] : 1e9;

    uint32_t chosenTid = (rtt0 < rtt3) ? 0 : 3;

    NS_LOG_INFO("RTT0: " << rtt0 << " ns, RTT3: " << rtt3 << " ns, ChosenTid: " << chosenTid);

    // Disable optional TID selection (force only one TID)
    client->SetAttribute("Priority", UintegerValue(chosenTid));
    client->SetAttribute("OptionalTid", UintegerValue(chosenTid));
    client->SetAttribute("OptionalTidPr", DoubleValue(1.0));

    Time periodicity{Seconds(0.25)};
    Simulator::Schedule(periodicity, &MCAB_Scheduler, std::ref(coHelper), sta, client);
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

void SendPacketToEachLink(Ptr<Node> senderNode, Ptr<Node> receiverNode, Time periodicInterval) {
    Ptr<WifiNetDevice> device = DynamicCast<WifiNetDevice>(senderNode->GetDevice(0));
    PacketSocketAddress sockAddr;
    sockAddr.SetSingleDevice(device->GetIfIndex());
    sockAddr.SetPhysicalAddress(receiverNode->GetDevice(0)->GetAddress());
    sockAddr.SetProtocol(1);

    Ptr<Socket> socket = Socket::CreateSocket(senderNode, PacketSocketFactory::GetTypeId());
    socket->Bind();
    socket->Connect(sockAddr);

    double sendTime = Simulator::Now().GetSeconds();

    // Loop for TID 0 and 3
    for (uint8_t tid : {0, 3}) {
        Ptr<Packet> packet = Create<Packet>(86); // Create new packet for each TID

        // Set timestamp
        TimeHeader timeHeader(sendTime);
        packet->AddHeader(timeHeader);

        // Set marker
        MloPacketMarker marker;
        marker.SetMloPacket(true);
        marker.SetTid(tid);
        marker.SetNodeId(senderNode->GetId());
        packet->AddHeader(marker);

        // socket->SetIpTos(tid << 5);
        socket->Send(packet);

        // NS_LOG_INFO("[TX] Time: " << std::fixed << std::setprecision(6) << sendTime << " s"
        //              << " | Node: " << senderNode->GetId()
        //              << " -> Node: " << receiverNode->GetId()
        //              << " | TID: " << (int)tid
        //              << " | Link: " << (tid == 0 ? 0 : 1));
    }

    socket->Close();

    // Reschedule
    Simulator::Schedule(periodicInterval, &SendPacketToEachLink, senderNode, receiverNode, periodicInterval);
}

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
    phy.AddChannel(spectrumChannel5Ghz, WIFI_SPECTRUM_2_4_GHZ);
    phy.AddChannel(spectrumChannel6Ghz, WIFI_SPECTRUM_5_GHZ);

    // configure operating channel for each link
    phy.Set(0, "ChannelSettings", StringValue("{0, 40, BAND_2_4GHZ, 0}"));
    phy.Set(1, "ChannelSettings", StringValue("{0, 40, BAND_5GHZ, 0}"));

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
    WifiCoTraceHelper m_wificohelper[4];

    for (size_t i = 1; i <= nWifi; i++)
    {
        auto clientApp = InstallPacketSocketClient(sta.Get(i - 1),
                                                   ap.Get(0),
                                                   loads.at(i - 1),
                                                   i * Seconds(2.0),
                                                   m_stop);
        m_wificohelper[i - 1].Enable(sta);
        Simulator::Schedule(Seconds(i * 2.0),
                            &MCAB_Scheduler,
                            std::ref(m_wificohelper[i - 1]),
                            sta.Get(i - 1),
                            clientApp);
    }   

    /* Trace channel occupancies on both links during simulation */
    Time periodicity{Seconds(0.25)};
    std::ofstream coTrace{coTraceFile};
    coTrace << std::fixed << std::showpoint << std::setprecision(3);
    WifiCoTraceHelper apCoHelper;
    apCoHelper.Enable(m_nodes.Get(0));
    // Simulator::Schedule(m_start,
    //                     &TraceChannelOccupancy,
    //                     std::ref(apCoHelper),
    //                     std::ref(coTrace),
    //                     periodicity);

    for(int i=0; i<(int)nWifi; i++){
        Simulator::Schedule(m_start,
                &SendPacketToEachLink,
                sta.Get(i),
                ap.Get(0),
                periodicity);
    }

    LogComponentEnable("MLD_FEEDBACK", LOG_LEVEL_INFO);

    Simulator::Stop(m_stop);
    Simulator::Run();
    Simulator::Destroy();

    return 0;
}
