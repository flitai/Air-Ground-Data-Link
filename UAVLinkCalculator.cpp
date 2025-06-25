#include "UAVLinkCalculator.h"
#include "LinkConstants.h"
#include <iostream>
#include <stdexcept>

namespace LinkModel {

UAVLinkCalculator::UAVLinkCalculator(const LinkParameters& params) : m_params(params) {}

/**
 * @brief 运行完整的链路分析，遵循文档中的逻辑流程。
 */
void UAVLinkCalculator::runFullAnalysis() {
    // 1. 计算最大视距距离 (文档 3.1)
    m_results.maxLineOfSight_km = calculateMaxLineOfSight();

    // 2. 计算最大多普勒频移和相干时间 (文档 3.6)
    double f_m = calculateMaxDopplerShift();
    m_results.coherenceTime_s = calculateCoherenceTime(f_m);

    // 3. 对给定距离和速率，进行链路连通性检查
    bool losCheck = (m_params.linkDistance_km <= m_results.maxLineOfSight_km);
    if (!losCheck) {
        m_results.isConnected = false;
        m_results.statusMessage = "链路中断：超出最大视距范围。";
        return;
    }
    
    // 4. 计算总传播损耗 (文档 3.2)
    m_results.totalPropagationLoss_dB = calculateTotalPropagationLoss(m_params.linkDistance_km);

    // 5. 计算接收信号功率 (文档 3.3.1)
    m_results.receivedPower_dBW = calculateReceivedPower(m_results.totalPropagationLoss_dB);
    
    // 6. 计算接收机灵敏度 (门限电平) (文档 3.4)
    m_results.receiverSensitivity_dBW = calculateReceiverSensitivity();

    bool powerCheck = (m_results.receivedPower_dBW >= m_results.receiverSensitivity_dBW);
    if (!powerCheck) {
        m_results.isConnected = false;
        m_results.statusMessage = "链路中断：接收信号功率低于接收机灵敏度。";
        return;
    }

    // 7. 检查多普勒速率条件 (文档 3.7)
    m_results.requiredRateVsDoppler_bps = M_FACTOR_DOPPLER / m_results.coherenceTime_s;
    bool dopplerCheck = (m_params.transmissionRate_bps > m_results.requiredRateVsDoppler_bps);
    if (!dopplerCheck) {
        m_results.isConnected = false;
        m_results.statusMessage = "链路中断：传输速率过低，无法适应信道时变性（快衰落）。";
        return;
    }
    
    // 如果所有检查都通过，则链路判定为连通
    m_results.isConnected = true;
    m_results.statusMessage = "链路连通。";

    // 8. 计算最终的性能评估指标 (文档 4.2)
    m_results.maxEffectiveDistance_km = findMaxEffectiveDistance();
    m_results.maxEffectiveRate_bps = findMaxEffectiveRate();
}

LinkResults UAVLinkCalculator::getResults() const {
    return m_results;
}

// --- 私有辅助方法的实现 ---

/**
 * @brief 计算最大视距距离 (d_max)
 * @details 依据文档 3.1 节公式，考虑标准大气折射。
 */
double UAVLinkCalculator::calculateMaxLineOfSight() const {
    return LOS_FACTOR_STD_REFRACTION * (sqrt(m_params.groundStationAltitude_m) + sqrt(m_params.uavAltitude_m));
}

/**
 * @brief 计算总传播损耗 (L_Sigma)
 * @details 依据文档 3.2 节公式，是各损耗分量之和。
 */
double UAVLinkCalculator::calculateTotalPropagationLoss(double distance_km) const {
    double l_fs = calculateFreeSpacePathLoss(distance_km);
    double l_a = calculateAtmosphericAbsorptionLoss(distance_km);
    double l_r = calculateRainAttenuation(distance_km);
    // L_d (绕射损耗) 和 L_mp (多径损耗) 等被统一包含在 otherLosses_dB 中
    return l_fs + l_a + l_r + m_params.otherLosses_dB;
}

/**
 * @brief 计算自由空间路径损耗 (L_fs)
 * @details 依据文档 3.2.1 节公式。
 */
double UAVLinkCalculator::calculateFreeSpacePathLoss(double distance_km) const {
    if (distance_km <= 0) return 0;
    return 20.0 * log10(distance_km) + 20.0 * log10(m_params.frequency_MHz) + 32.45;
}

/**
 * @brief 计算大气吸收损耗 (L_a)
 * @details 依据文档 3.2.2 节公式，使用表格中的比衰减值进行简化计算。
 */
double UAVLinkCalculator::calculateAtmosphericAbsorptionLoss(double distance_km) const {
    double gamma_o, gamma_w;
    getAtmoParams(gamma_o, gamma_w);
    return (gamma_o + gamma_w) * distance_km;
}

/**
 * @brief 计算雨致衰减 (L_r)
 * @details 依据文档 3.2.3 节公式，使用表格中的 k 和 alpha 系数。
 */
double UAVLinkCalculator::calculateRainAttenuation(double distance_km) const {
    double R = 0.0;
    switch (m_params.weather) {
        case WeatherCondition::MODERATE_RAIN: R = 10.0; break; // 示例：中雨 10 mm/h
        case WeatherCondition::HEAVY_RAIN:    R = 25.0; break; // 示例：大雨 25 mm/h
        case WeatherCondition::CLEAR:
        default: return 0.0;
    }

    double k, alpha;
    getRainParams(k, alpha);
    double gamma_R = k * pow(R, alpha); // 特定衰减系数 (dB/km)
    return gamma_R * distance_km; // 假设有效路径长度等于总距离
}

/**
 * @brief 计算接收功率 (Pr)
 * @details 依据文档 3.3.1 节公式。
 */
double UAVLinkCalculator::calculateReceivedPower(double totalLoss_dB) const {
    return m_params.txEIRP_dBW + m_params.rxAntennaGain_dBi - totalLoss_dB;
}

/**
 * @brief 计算接收机灵敏度 ((S_i)_th)，单位 dBW
 * @details 依据文档 3.4.3 节公式。
 */
double UAVLinkCalculator::calculateReceiverSensitivity() const {
    // 噪声功率 (Ni = k * T0 * B * NF) (文档 3.4.1)
    double T0 = STANDARD_NOISE_TEMP;
    double k = BOLTZMANN_CONSTANT;
    // 假设噪声带宽 B 等于传输速率 Rb (BPSK调制的常用近似)
    double B_Hz = m_params.transmissionRate_bps;
    double NF_linear = pow(10, m_params.rxNoiseFigure_dB / 10.0);
    double Ni_watts = k * T0 * B_Hz * NF_linear;

    // 门限信噪比 (r_th) (文档 3.4.2)
    double snr_th_linear = calculateThresholdSnrFromBer();

    // 最小可接收功率 ((S_i)_th) (文档 3.4.3)
    double Si_th_watts = Ni_watts * snr_th_linear;

    return 10.0 * log10(Si_th_watts); // 转换为 dBW
}

/**
 * @brief 根据不同的调制方式，从BER计算所需的门限信噪比 (Eb/N0)
 * @details 对文档 3.4.2 节的公式进行求逆运算。
 */
double UAVLinkCalculator::calculateThresholdSnrFromBer() const {
    // 文档中的公式建立了 BER(Pb) 与 Eb/N0 的关系。
    // 信噪比 SNR = (S/N) = (Eb*Rb / N0*B)。如果我们假设 B=Rb，则 SNR = Eb/N0。
    // 此处计算所需的 Eb/N0。
    double EbN0_linear = 0;
    double P_b = m_params.maxAllowedBER;

    switch (m_params.modulation) {
        case ModulationType::BPSK: // 与 2ASK 公式相同
        case ModulationType::COHERENT_2ASK:
            // Pb = Q(sqrt(2 * Eb/N0)) => Eb/N0 = (Q_inv(Pb)^2) / 2
            EbN0_linear = pow(Q_inv(P_b), 2) / 2.0;
            break;
        case ModulationType::DPSK:
            // Pb = 0.5 * exp(-Eb/N0) => Eb/N0 = -ln(2 * Pb)
            if (P_b >= 0.5) return std::numeric_limits<double>::infinity();
            EbN0_linear = -log(2.0 * P_b);
            break;
        case ModulationType::COHERENT_2FSK:
            // Pb = Q(sqrt(Eb/N0)) => Eb/N0 = Q_inv(Pb)^2
            EbN0_linear = pow(Q_inv(P_b), 2);
            break;
        default:
            throw std::runtime_error("不支持的调制类型，无法计算SNR。");
    }
    return EbN0_linear;
}

/**
 * @brief 计算最大多普勒频移 (f_m)
 * @details 依据文档 3.6 节公式。
 */
double UAVLinkCalculator::calculateMaxDopplerShift() const {
    double wavelength = SPEED_OF_LIGHT / (m_params.frequency_MHz * 1e6);
    return m_params.uavSpeed_mps / wavelength;
}

/**
 * @brief 计算信道相干时间 (T_c)
 * @details 依据文档 3.6 节公式。
 */
double UAVLinkCalculator::calculateCoherenceTime(double maxDopplerShift) const {
    if (maxDopplerShift <= 0) return std::numeric_limits<double>::infinity();
    return 0.423 / maxDopplerShift;
}

/**
 * @brief 求解最大有效通信距离
 * @details 依据文档 4.2 节逻辑，该距离是视距距离和信噪比距离中的较小者。
 */
double UAVLinkCalculator::findMaxEffectiveDistance() {
    // 首先，检查当前速率是否满足多普勒条件
    if (m_results.requiredRateVsDoppler_bps >= m_params.transmissionRate_bps) {
        return 0.0; // 当前速率对于无人机动态来说太慢了
    }

    double d_los = m_results.maxLineOfSight_km;
    double sensitivity_dBW = m_results.receiverSensitivity_dBW;

    // 使用二分查找法，寻找接收功率恰好等于灵敏度时的距离 (D_SNR)
    double low = 0.0, high = d_los;
    double d_snr = 0.0;
    for(int i = 0; i < 100; ++i) { // 100次迭代以保证高精度
        double mid = low + (high - low) / 2.0;
        double loss = calculateTotalPropagationLoss(mid);
        double p_r = calculateReceivedPower(loss);

        if (p_r >= sensitivity_dBW) {
            d_snr = mid; // 此距离可达
            low = mid;
        } else {
            high = mid;
        }
    }

    return std::min(d_los, d_snr);
}

/**
 * @brief 求解最大有效传输速率
 * @details 依据文档 4.2 节逻辑，该速率需同时满足多普勒条件和链路预算（信噪比）条件。
 */
double UAVLinkCalculator::findMaxEffectiveRate() {
    // 速率的下限由多普勒效应决定
    double min_rate_doppler = m_results.requiredRateVsDoppler_bps;

    // 速率的上限由链路预算（信噪比）决定
    // Pr >= (Si)_th = (k*T0*B*NF) * r_th
    // 假设 B = Rb, 则 Pr >= k*T0*NF*r_th * Rb
    // 因此, Rb <= Pr / (k*T0*NF*r_th)

    double p_r_linear = pow(10, m_results.receivedPower_dBW / 10.0);
    double NF_linear = pow(10, m_params.rxNoiseFigure_dB / 10.0);
    double r_th_linear = calculateThresholdSnrFromBer();

    double denominator = BOLTZMANN_CONSTANT * STANDARD_NOISE_TEMP * NF_linear * r_th_linear;
    if (denominator == 0) return 0;
    
    double max_rate_snr = p_r_linear / denominator;
    
    // 实际的最大有效速率必须同时满足两个条件
    if (max_rate_snr > min_rate_doppler) {
        return max_rate_snr;
    } else {
        // 不存在既能满足多普勒要求，又能满足信噪比要求的速率
        return 0.0;
    }
}

// --- 工具函数的实现 ---

// 逆Q函数的简单多项式逼近，对于小的p值精度较高
double UAVLinkCalculator::Q_inv(double p) const {
    if (p <= 0.0 || p >= 1.0) return std::numeric_limits<double>::infinity();
    if (p > 0.5) return -Q_inv(1.0 - p);

    double t = sqrt(-2.0 * log(p));
    double c0 = 2.515517, c1 = 0.802853, c2 = 0.010328;
    double d1 = 1.432788, d2 = 0.189269, d3 = 0.001308;
    
    return t - (c0 + c1*t + c2*t*t) / (1 + d1*t + d2*t*t + d3*t*t*t);
}

// 根据文档 3.2.3 的表格提供雨衰系数 k 和 alpha
void UAVLinkCalculator::getRainParams(double& k, double& alpha) const {
    // 为简化起见，使用表格中水平极化的值。
    // 实际应用中可根据频率进行插值。
    if (m_params.frequency_MHz >= 1000 && m_params.frequency_MHz < 3000) { // 近似 2.4 GHz
        k = 0.000650; alpha = 1.121;
    } else if (m_params.frequency_MHz >= 8000 && m_params.frequency_MHz < 11000) { // 近似 10 GHz
        k = 0.0188; alpha = 1.217;
    } else { // 如果超出常用范围，默认使用 2.4 GHz 的值
        k = 0.000650; alpha = 1.121;
    }
}

// 根据文档 3.2.2 的表格提供大气吸收的比衰减系数
void UAVLinkCalculator::getAtmoParams(double& gamma_o, double& gamma_w) const {
    // 使用海平面、15°C 的典型值。
    // 完整的实现应使用 ITU-R P.676 模型。
    // 以下是 2.4 GHz 附近的代表性数值。
    gamma_o = 0.007; // dB/km (氧气)

    // 水蒸气吸收与湿度（天气状况）相关
    switch(m_params.weather) {
        case WeatherCondition::HEAVY_RAIN: // 潮湿
             gamma_w = 0.004; // (对应 15 g/m³ 的示例值)
             break;
        case WeatherCondition::MODERATE_RAIN: // 中等湿度
             gamma_w = 0.002; // (对应 7.5 g/m³ 的示例值)
             break;
        case WeatherCondition::CLEAR: // 干燥
        default:
             gamma_w = 0.0003; // (对应 1 g/m³ 的示例值)
             break;
    }
}

} // namespace LinkModel