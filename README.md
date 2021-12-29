# Narwhal-volumetric-video-streaming-system
# 系统环境说明
VS2017+PCL1.9.1+QT 5.12
# 功能说明
## 主文件为PCLVisualizer.cpp 主要包含:
### 1.MPD文件解析模块(MPD文件需要按照一定格式生成)
### 2.切块存储信息模块
### 3.判断FoV内切块是否存在(is_tileinfov函数)
### 4.优化求解功能(需要配置cplex环境)
### 5.解码功能(需要V-PCC链接库)
### 6.buffer读入读取(QT多线程)
### 7.服务端通信功能(服务端采用nginx配置)
