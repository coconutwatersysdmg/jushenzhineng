这个目录是给本项目（jushenzhineng_v3）准备的 Livox Mid360 运行小包。

一、放置方式

本目录已位于项目内：

  third_party\livox_runtime\

相关文件：

  services\livox_service.py
  config\livox_config.ini
  third_party\livox_runtime\livox_realtime_select_and_move.exe
  third_party\livox_runtime\livox_realtime_select_and_move.cpp
  third_party\livox_runtime\02_build_realtime_example.bat
  third_party\livox_runtime\livox_lidar_sdk_shared.dll
  third_party\livox_runtime\mid360s_config.json
  data\lidar\

二、主界面在线识别的点云保存方式

主界面点击“启动雷达识别”时，程序会让 Livox 采集程序临时生成一帧 PCD，
Python 读取到内存并完成角点提取后，会自动删除这帧临时 PCD。

也就是说，正常 UI 在线识别不会在 data\lidar 长期保存 scan_*.pcd。

如果后面需要留一帧点云做排查，可以临时在代码里调用：

pipeline.capture_and_extract(keep_pcd=True)

三、单独测试雷达采集

在项目根目录打开 CMD：

  runtime\python.exe tools\test_livox_capture.py

（若没有 test 脚本，通过主程序联调即可。）

成功时会在 data\lidar\ 下生成 scan_000001.pcd，后续递增。

四、重新编译采集程序

先设置 Livox-SDK2 路径，再运行：

  set SDK_ROOT=D:\deps\Livox-SDK2
  third_party\livox_runtime\02_build_realtime_example.bat

不要写死某台电脑的用户名路径。

五、现场注意

  - 确认电脑网卡 IP 与 mid360s_config.json / external_devices_config.py 一致
  - 防火墙放行 Livox 相关端口
  - SDK DLL 需与 exe 同目录（已随本目录交付）
