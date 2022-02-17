# Narwhal-volumetric-video-streaming-system
# Requirement
---
VS2017+PCL1.9.1+QT 5.12

# Setup
---
### 1.Install PCL 1.9.1 and QT 5.12
### 2.Configuring PCL and QT in VS2017
### 3.Install nginx as server
### 4.Place the point cloud video in the server root directory
### 5.Generate the corresponding mainfest file
### 6.Enter the server ip address in the player to play

# Modules Description
---
## The main file is mainly included in PCLVISUALIZER.CPP:
### 1. MPD file resolution module (MPD file needs to be generated in a certain format)
### 2. Cut block storage information module
### 3. Judge whether there is any existence of the cut block in the FOV (IS_TILEINFOV function)
### 4. Optimize solve function (need to configure a CPLEX environment)
### 5. Decoding function (requires V-PCC link library)
### 6.Buffer Read (QT Multithread)
### 7. Server communication function (the server uses Nginx configuration)

# Refrence
---
```
@ARTICLE{li2022optimal,
  author={Li, Jie and Zhang, Cong and Liu, Zhi and Hong, Richang and Hu, Han},
  ournal={IEEE Transactions on Multimedia}, 
  title={Optimal Volumetric Video Streaming with Hybrid Saliency based Tiling}, 
  year={2022},
  number={}, 
  pages={}}

@ARTICLE{liu2021point,
  author={Liu, Zhi and Li, Qiyue and Chen, Xianfu and Wu, Celimuge and Ishihara, Susumu and Li, Jie and Ji, Yusheng},
  journal={IEEE Network}, 
  title={Point Cloud Video Streaming: challenges and Solutions}, 
  year={2021},
  volume={35},
  number={5},
  pages={202-209}}
  
@inproceedings{li_demo_2020,
	title = {Demo Abstract: Narwhal: a {DASH}-based Point Cloud Video Streaming System over Wireless Networks},
	pages = {1326--1327},
	booktitle = {{IEEE} {INFOCOM} 2020 - {IEEE} Conference on Computer Communications Workshops ({INFOCOM} {WKSHPS})},
	author = {Li, Jie and Zhang, Cong and Liu, Zhi and Sun, Wei and Hu, Wei and Li, Qiyue},
	date = {2020-07},
}
```


