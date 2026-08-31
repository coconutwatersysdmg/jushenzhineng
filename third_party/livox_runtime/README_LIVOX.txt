这个压缩包是给 jushenzhineng_v2 上位机准备的 Livox 雷达运行小包。

一、放置方式

把本压缩包解压后，里面的内容放到：

C:\Users\15316\Desktop\jushenzhineng_v2

最终结构应该类似：

C:\Users\15316\Desktop\jushenzhineng_v2\services\livox_service.py
C:\Users\15316\Desktop\jushenzhineng_v2\tools\test_livox_capture.py
C:\Users\15316\Desktop\jushenzhineng_v2\config\livox_config.ini
C:\Users\15316\Desktop\jushenzhineng_v2\third_party\livox_runtime\livox_realtime_select_and_move.exe
C:\Users\15316\Desktop\jushenzhineng_v2\third_party\livox_runtime\livox_realtime_select_and_move.cpp
C:\Users\15316\Desktop\jushenzhineng_v2\third_party\livox_runtime\02_build_realtime_example.bat
C:\Users\15316\Desktop\jushenzhineng_v2\third_party\livox_runtime\livox_lidar_sdk_shared.dll
C:\Users\15316\Desktop\jushenzhineng_v2\third_party\livox_runtime\mid360s_config.json
C:\Users\15316\Desktop\jushenzhineng_v2\data\lidar\

二、主界面在线识别的点云保存方式

主界面点击“启动雷达识别”时，程序会让 Livox 采集程序临时生成一帧 PCD，
Python 读取到内存并完成角点提取后，会自动删除这帧临时 PCD。

也就是说，正常 UI 在线识别不会在 data\lidar 长期保存 scan_*.pcd。

如果后面需要留一帧点云做排查，可以临时在代码里调用：

pipeline.capture_and_extract(keep_pcd=True)

三、单独测试雷达采集

打开 VSCode 终端或 CMD：

cd /d C:\Users\15316\Desktop\jushenzhineng_v2
C:\Users\15316\anaconda3\envs\cv\python.exe tools\test_livox_capture.py

这个单独测试脚本用于检查 Livox 采集链路。如果成功，会在这里生成：

C:\Users\15316\Desktop\jushenzhineng_v2\data\lidar\scan_000001.pcd

后面继续采集会自动递增：

C:\Users\15316\Desktop\jushenzhineng_v2\data\lidar\scan_000002.pcd

其中：

scan_000001.pcd 是雷达原始采集点云。

单独测试脚本不会覆盖旧 PCD。每次采集都会按 data\lidar 目录里已有的最大编号继续保存。

四、重要参数

参数文件：

config\livox_config.ini

里面可以改：

capture_ms：采集时间，默认 3000ms，也就是 3 秒
max_points：保存点云上限，默认 600000 点
save_dir：临时点云/单独采集测试的保存目录，默认 data/lidar
exe_path：Livox 采集程序路径
config_path：Livox SDK2 网络配置路径

当前雷达接收服务只负责采集原始点云，不在接收阶段做 ROI 裁剪。主界面在线识别会读取临时 PCD 后立即删除，后续 ROI、地面过滤和底板拟合放到点云处理脚本里完成。

注意：如果你发现无论采集多少秒，PCD 都只有 300000 点，说明
third_party\livox_runtime\livox_realtime_select_and_move.exe 还是旧版。
需要用新版 livox_realtime_select_and_move.cpp 重新编译 exe，并替换这个目录里的旧 exe。

重新编译方式：

1. 打开 x64 Native Tools Command Prompt for VS 2022
2. 执行：

cd /d C:\Users\15316\Desktop\jushenzhineng_v2\livox_mid360s\third_party\livox_runtime
02_build_realtime_example.bat

3. 看到输出：

Build finished: livox_realtime_select_and_move.exe

再回到主界面点“启动雷达识别”。新日志里会显示：

Capturing fresh point cloud for 3000 ms...
Max buffered points: 600000

五、雷达网络配置

当前 mid360s_config.json 里面的 host_ip 是：

192.168.1.50

这个必须和你电脑连接雷达的网卡 IP 对得上。
如果你电脑网卡不是这个 IP，就打开：

third_party\livox_runtime\mid360s_config.json

修改 host_ip。

六、以后 UI 按钮怎么调用

后面在上位机按钮函数里可以这样调用：

from services.livox_service import LivoxService

service = LivoxService()
result = service.capture_once()

if result.success:
    print(result.pcd_path)
    print(result.point_count)

如果走主界面在线识别，请使用 services\lidar_corner_pipeline.py 里的 LidarCornerPipeline。
它默认处理完会删除临时 PCD，不长期保存点云文件。
