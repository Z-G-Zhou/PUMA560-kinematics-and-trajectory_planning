# PUMA560机械臂建模与轨迹规划 
## 项目简介
本项目为归档项目包，不再更新维护，该项目可作为机器人相关专业的本科生和研究生进行学习和参考的资料。\\
项目首先对PUMA560机械臂的基本结构和参数进行了介绍，对其运动学特性进行了分析并给出了正逆运动学的完整算法。其次，建立了PUMA560的三维模型，制作了机器人的URDF网格可视化文件，并在Simulink中开发了机械臂的可视化仿真框架。最后，分别基于关节空间和笛卡尔空间对机械臂进行了轨迹规划并在Simulink中实现了典型轨迹的可视化仿真。
## 文件夹说明
word_pdf_html                                  ：PUMA560的word文档、PDF文档及网页文档\\
Kinematics_of_PUMA560		            	：PUMA560的正逆运动学函数和验证脚本m文件\\
PUMA560_SLDPRT_SW_Model		      ：PUMA560的Solidworks文件（已标定关节轴和坐标系）\\
PUMA560_URDF_from_SW	         	    ：由Solidworks文件导出的URDF文件\\
Simulation_animation_of_trajectory	：轨迹规划动态仿真过程的动画\\
SIMULINK_slx_by_simscape		          ：URDF文件通过Simscape导入Simulink后的 slx 文件\\
Trajectory_Planning_of_PUMA560	  ：PUMA560轨迹规划函数和验证、仿真的脚本\\
## 联系
项目文档：https://www.notion.so/zgzhou/PUMA560-a116cd9695d349aa9f338081f013a2a4?pvs=4 \\
联系邮箱：zhouzge@foxmail.com
