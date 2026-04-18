# 20th-Intelligent-Vehicle-Competition---Complete-Model-Group

> 第二十届全国大学生智能汽车竞赛 - 完全模型组 | 百度 EdgeBoard + 英飞凌 TC264

**Multi-threaded pure vision processing without IMU.**

During the final, the road sign was not recognized due to reflection, and we regretfully finished with a **National Second Prize**.

*留作纪念 · Archived for keepsake.*

---

## 📸 车模帅照

<p align="center">
  <img width="206" alt="车模1" src="https://github.com/user-attachments/assets/ef3fb571-ebb0-4c0e-97eb-c729a8740d56" />
  <img width="205" alt="车模2" src="https://github.com/user-attachments/assets/76f6877f-564f-4796-bf2f-177cd31d0959" />
  <img width="203" alt="车模3" src="https://github.com/user-attachments/assets/3970ff76-e63c-4c2e-a7f7-1d778ce76908" />
</p>

---

## 🏁 算法可视化

<p align="center">
  <img width="100%" alt="可视化" src="https://github.com/user-attachments/assets/35394a32-5178-4af3-b618-c9600492dbc9" />
</p>

---

## 🛠️ 技术架构

| 层级 | 硬件/框架 | 说明 |
|:---|:---|:---|
| 上位机 | 百度 EdgeBoard | Paddle Lite 推理 + OpenCV 多线程图像处理 |
| 下位机 | 英飞凌 TC264 | 电机/舵机控制、编码器里程计 |
| 通信 | UART | 上下位机串口通信协议 |
| 感知 | 纯视觉（无 IMU） | YOLO 目标检测 + 传统视觉边线提取 |

---

## 📂 核心模块

### 基础模块
| 模块 | 功能 |
|:---|:---|
| `common` | 全局变量、状态枚举、公共数据结构定义 |
| `motion` | 速度控制、舵机打角、PID 控制器 |
| `tracking` | 边线追踪、逆透视变换、中线拟合、角点检测 |
| `uart` | 上下位机串口通信协议封装 |

### 特殊元素处理模块
| 模块 | 功能 |
|:---|:---|
| `catering` | 餐饮区（汉堡）— 左右判别 → 停车 → 驶出 |
| `crosswalk` | 斑马线检测与停车逻辑 |
| `layby` | 临时停车区 — `school`/`company` 路牌识别、消失后停车等待 |
| `obstacle` | 障碍物分类（锥桶/砖块/行人）与绕行决策 |
| `ramp` | 坡道上下坡状态切换与刹车控制 |
| `crossroad` | 十字路口 — 正入/斜入、远线角点搜索、半十字补线 |

---
- **收获**：玩玩C++，玩玩CV和Control，玩玩Deeplearning

---

## 📄 License

MIT License — 仅供纪念与学习交流。
