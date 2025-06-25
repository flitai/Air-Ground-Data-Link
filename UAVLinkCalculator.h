#ifndef UAV_LINK_CALCULATOR_H
#define UAV_LINK_CALCULATOR_H

#include "LinkParameters.h"

namespace LinkModel {

class UAVLinkCalculator {
public:
    // 构造函数，接收一个包含所有输入参数的结构体
    UAVLinkCalculator(const LinkParameters& params);

    // 主计算函数，执行完整的链路分析
    void runFullAnalysis();

    // 获取计算结果
    LinkResults getResults() const;

private:
    // 私有辅助方法，每个方法对应文档中的一个计算步骤
    double calculateMaxLineOfSight() const;
    double calculateTotalPropagationLoss(double distance_km) const;
    double calculateFreeSpacePathLoss(double distance_km) const;
    double calculateAtmosphericAbsorptionLoss(double distance_km) const;
    double calculateRainAttenuation(double distance_km) const;
    double calculateReceivedPower(double totalLoss_dB) const;
    double calculateReceiverSensitivity() const;
    double calculateThresholdSnrFromBer() const;
    double calculateMaxDopplerShift() const;
    double calculateCoherenceTime(double maxDopplerShift) const;
    
    // 用于求解最大有效距离和速率的函数
    double findMaxEffectiveDistance();
    double findMaxEffectiveRate();

    // 工具函数
    double Q_inv(double p) const; // 近似的逆Q函数
    void getRainParams(double& k, double& alpha) const; // 获取降雨衰减系数
    void getAtmoParams(double& gamma_o, double& gamma_w) const; // 获取大气吸收系数

    LinkParameters m_params;  // 存储输入的参数
    LinkResults m_results;    // 存储计算的结果
};

}

#endif // UAV_LINK_CALCULATOR_H