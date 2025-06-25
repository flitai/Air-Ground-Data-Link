#ifndef LINK_PARAMETERS_H
#define LINK_PARAMETERS_H

#include <string>

namespace LinkModel {

    // 枚举类，用于表示文档中提到的调制方式
    enum class ModulationType {
        BPSK,       // 2PSK
        DPSK,
        COHERENT_2FSK,
        COHERENT_2ASK
    };

    // 枚举类，用于简化天气状况的参数选择
    enum class WeatherCondition {
        CLEAR,          // 晴天
        MODERATE_RAIN,  // 中雨 (例如 10 mm/h)
        HEAVY_RAIN      // 大雨 (例如 25 mm/h)
    };

    // 结构体，用于存储链路计算所需的所有输入参数
    struct LinkParameters {
        // 位置与动态参数
        double groundStationAltitude_m; // h1: 地面站天线高度 (米)
        double uavAltitude_m;           // h2: 无人机飞行高度 (米)
        double uavSpeed_mps;            // v: 无人机相对速度 (米/秒)
        double linkDistance_km;         // D: 当前链路距离 (公里)

        // 射频参数
        double frequency_MHz;           // f: 载波频率 (MHz)
        double txEIRP_dBW;              // 发射机等效各向同性辐射功率 (dBW)
        double rxAntennaGain_dBi;       // G_r: 接收天线增益 (dBi)
        double rxNoiseFigure_dB;        // N_F: 接收机噪声系数 (dB)
        double transmissionRate_bps;    // R_b: 传输速率 (bps)

        // 系统与环境参数
        ModulationType modulation;      // 调制方式
        WeatherCondition weather;       // 天气状况
        double maxAllowedBER;           // (BER)_max: 最大允许误码率
        double otherLosses_dB;          // L_o: 其他损耗总和 (dB)，如绕射、多径、馈线等
    };

    // 结构体，用于存储计算后的结果
    struct LinkResults {
        bool isConnected;               // 链路是否连通
        std::string statusMessage;      // 状态描述信息

        // 详细的中间计算结果
        double maxLineOfSight_km;       // 最大视距
        double totalPropagationLoss_dB; // 总传播损耗
        double receivedPower_dBW;       // 接收功率
        double receiverSensitivity_dBW; // 接收机灵敏度
        double coherenceTime_s;         // 相干时间
        double requiredRateVsDoppler_bps; // 多普勒效应要求的最低速率

        // 最终的性能指标
        double maxEffectiveDistance_km; // 最大有效通信距离
        double maxEffectiveRate_bps;    // 最大有效传输速率
    };
}

#endif // LINK_PARAMETERS_H