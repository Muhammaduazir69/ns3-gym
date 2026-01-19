/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * ENHANCED VANET Link Quality Environment for ns3-gym
 *
 * This environment provides comprehensive link quality metrics for
 * neural network-based routing decisions in VANET scenarios.
 *
 * Metrics collected:
 * - RSSI (Received Signal Strength Indicator)
 * - SNR (Signal-to-Noise Ratio)
 * - Packet Loss Rate
 * - Distance between nodes
 * - Relative Speed
 * - OFDM Preamble/Payload errors
 *
 * Future NR enhancements ready:
 * - CQI (Channel Quality Indicator)
 * - MCS (Modulation and Coding Scheme)
 * - SINR (Signal-to-Interference-plus-Noise Ratio)
 * - Beamforming vector quality
 */

#ifndef VANET_LINK_ENV_H
#define VANET_LINK_ENV_H

#include "ns3/opengym-module.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include <vector>
#include <map>

namespace ns3 {

struct LinkMetrics {
    // Core PHY metrics
    double rssi = -100.0;           // Received Signal Strength Indicator (dBm)
    double snr = 0.0;               // Signal to Noise Ratio (dB)
    double packetLoss = 0.0;        // Packet loss rate [0,1]

    // Mobility metrics
    double distance = 0.0;          // Distance between nodes (meters)
    double relativeSpeed = 0.0;     // Relative velocity (m/s)

    // Error metrics (OFDM-specific, extensible to NR)
    double preambleError = 0.0;     // Preamble decoding error rate [0,1]
    double payloadError = 0.0;      // Payload decoding error rate [0,1]

    // Packet counters
    uint32_t packetsRx = 0;         // Successfully received packets
    uint32_t packetsTx = 0;         // Transmitted packets
    uint32_t preambleErrors = 0;    // Count of preamble failures
    uint32_t payloadErrors = 0;     // Count of payload failures

    // Timing
    Time lastUpdate = Seconds(0);   // Last metrics update timestamp

    // NR-ready extensions (for future use)
    double cqi = 15.0;              // Channel Quality Indicator (0-15) - future NR
    double mcs = 0.0;               // Modulation and Coding Scheme - future NR
    double linkQualityScore = 0.0;  // ML-computed link quality [0,1]

    LinkMetrics() = default;
};

class VanetLinkEnv : public OpenGymEnv
{
public:
    VanetLinkEnv();
    VanetLinkEnv(NodeContainer nodes);
    virtual ~VanetLinkEnv();
    static TypeId GetTypeId();
    virtual void DoDispose();

    // OpenGym interface
    virtual Ptr<OpenGymSpace> GetActionSpace();
    virtual Ptr<OpenGymSpace> GetObservationSpace();
    virtual bool GetGameOver();
    virtual Ptr<OpenGymDataContainer> GetObservation();
    virtual float GetReward();
    virtual std::string GetExtraInfo();
    virtual bool ExecuteActions(Ptr<OpenGymDataContainer> action);

    // VANET specific
    void CollectLinkMetrics();
    void SetNodes(NodeContainer nodes) { m_nodes = nodes; }
    void NotifyPacketTransmitted(uint32_t nodeId);
    void NotifyPacketReceived(uint32_t nodeId, double rssi, double snr);
    void NotifyPacketLost(uint32_t nodeId);
    void NotifyPreambleError(uint32_t nodeId);
    void NotifyPayloadError(uint32_t nodeId);

private:
    NodeContainer m_nodes;
    uint32_t m_observationSize;
    std::map<std::pair<uint32_t, uint32_t>, LinkMetrics> m_linkMetrics;
    std::vector<double> m_currentObservation;
    std::vector<int> m_routingDecisions;  // Routing decisions from NN
    double m_reward;
    bool m_gameOver;
};

} // namespace ns3

#endif /* VANET_LINK_ENV_H */