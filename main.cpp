#include <iostream>
#include <iomanip>
#include "UAVLinkCalculator.h"

using namespace LinkModel;

// 声明一个函数用于打印结果
void printResults(const LinkResults& results);

int main() {
    // 设置中文字符集，以便在终端正确显示中文
    #ifdef _WIN32
        system("chcp 65001");
    #endif

    // --- 为一个特定场景定义输入参数 ---
    // 这些参数基于文档中的示例和典型值进行设置。
    LinkParameters params;

    // 位置与动态参数
    params.groundStationAltitude_m = 20.0;     // h1: 地面站高度 (m)
    params.uavAltitude_m = 2000.0;             // h2: 无人机高度 (m)
    params.uavSpeed_mps = 150.0;               // 速度 v, 约 540 km/h
    params.linkDistance_km = 100.0;            // 需要进行计算的当前距离 D (km)

    // 射频参数 (示例值)
    params.frequency_MHz = 2400.0;             // 2.4 GHz S波段
    params.txEIRP_dBW = 30.0;                  // 例如 1 kW
    params.rxAntennaGain_dBi = 12.0;           // G_r
    params.rxNoiseFigure_dB = 4.0;             // N_F
    params.transmissionRate_bps = 2 * 1000000; // 2 Mbps

    // 系统与环境参数
    params.modulation = ModulationType::BPSK;  // BPSK调制
    params.weather = WeatherCondition::MODERATE_RAIN; // 中雨天气
    params.maxAllowedBER = 1e-5;               // 最大允许误码率
    params.otherLosses_dB = 5.0; // 其他损耗，如多径衰落余量等

    // --- 创建计算器实例并运行分析 ---
    UAVLinkCalculator calculator(params);
    calculator.runFullAnalysis();
    LinkResults results = calculator.getResults();

    // --- 显示计算结果 ---
    std::cout << "--- 无人机链路计算结果 ---" << std::endl;
    std::cout << "状态: " << results.statusMessage << std::endl;
    std::cout << "------------------------------------" << std::endl;
    printResults(results);

    return 0;
}

void printResults(const LinkResults& r) {
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "\n--- 基于给定参数的连通性分析 ---" << std::endl;
    std::cout << "最大视距:                   " << r.maxLineOfSight_km << " km" << std::endl;
    std::cout << "总传播损耗:                 " << r.totalPropagationLoss_dB << " dB" << std::endl;
    std::cout << "接收信号功率:               " << r.receivedPower_dBW << " dBW" << std::endl;
    std::cout << "接收机灵敏度:               " << r.receiverSensitivity_dBW << " dBW" << std::endl;
    std::cout << "信道相干时间:               " << r.coherenceTime_s * 1000 << " ms" << std::endl;
    std::cout << "多普勒效应要求的最低速率:   " << r.requiredRateVsDoppler_bps / 1e6 << " Mbps" << std::endl;

    std::cout << "\n--- 核心性能指标 ---" << std::endl;
    std::cout << "最大有效通信距离:           " << r.maxEffectiveDistance_km << " km" << std::endl;
    std::cout << "最大有效传输速率:           " << r.maxEffectiveRate_bps / 1e6 << " Mbps" << std::endl;
}