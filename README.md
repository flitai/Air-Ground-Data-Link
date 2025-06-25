# 地空数据链计算模型

模型旨在通过综合考虑视距限制、信号传播特性、接收机灵敏度及多普勒频移等多种因素，来评估无人机（UAV）与地面控制站（GCS）之间数据链路的连通性与性能。其核心评估指标包括**比特误码率（BER）**、**最大有效传输速率**和**最大有效通信距离**。

此C++实现划分为多个文件，以提高代码的模块化和可读性：

  * **`LinkConstants.h`**: 定义了物理常量和模型中使用的常量。
  * **`LinkParameters.h`**: 定义了用于存储输入参数和计算结果的结构体。
  * **`UAVLinkCalculator.h`**: 主计算器类的头文件，声明了类的结构和方法。
  * **`UAVLinkCalculator.cpp`**: 计算器类的实现文件，包含了文档中所有核心逻辑的具体实现。
  * **`main.cpp`**: 一个示例程序，展示了如何配置参数并调用计算器来执行链路分析。

-----

### 编译和运行

为了编译这个项目，需要一个C++编译器（例如 g++）。将以上所有四个代码文件（`LinkConstants.h`, `LinkParameters.h`, `UAVLinkCalculator.h`, `UAVLinkCalculator.cpp`, `main.cpp`）放置在同一个目录下，然后在终端中执行以下命令：

```sh
g++ -std=c++17 main.cpp UAVLinkCalculator.cpp -o uav_link_analyzer
```

接着，运行编译好的程序：

```sh
./uav_link_analyzer
```

程序将会根据`main.cpp`中设置的参数，输出一份详细的链路分析报告，包括链路的当前状态以及根据模型计算出的关键性能指标。
