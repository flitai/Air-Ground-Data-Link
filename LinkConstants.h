#ifndef LINK_CONSTANTS_H
#define LINK_CONSTANTS_H

#include <cmath>

namespace LinkModel {
    // 物理常量
    const double SPEED_OF_LIGHT = 299792458.0;         // 光速 (m/s)
    const double BOLTZMANN_CONSTANT = 1.380649e-23;   // 玻尔兹曼常数 (J/K)
    const double STANDARD_NOISE_TEMP = 290.0;          // 标准噪声温度 (K, T0)

    // 来自文档的模型常量
    const double M_FACTOR_DOPPLER = 150.0;             // 多普勒判据中的 m 因子，典型值在100-200之间
    const double LOS_FACTOR_STD_REFRACTION = 4.12;     // 考虑标准大气折射的最大视距计算因子
}

#endif // LINK_CONSTANTS_H