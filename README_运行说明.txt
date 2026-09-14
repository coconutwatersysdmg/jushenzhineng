巨神智能 / 具身智能装载数字孪生 v8
Windows 便携式运行说明
========================================

一、软件如何启动
----------------------------------------
把整个 jushenzhineng_v3 文件夹放到任意盘符/目录后：

  正常模式（推荐现场操作）：
    双击  启动软件.bat

  调试模式（看报错）：
    双击  启动软件_调试模式.bat

  旧入口仍可用：
    start_digital_twin.bat  （内部会转到 启动软件.bat）


二、正常启动方式
----------------------------------------
启动软件.bat 会：
  1. 自动切到本 bat 所在的项目根目录
  2. 使用项目自带的 runtime\pythonw.exe（或 python.exe）
  3. 启动 main.py 打开 Qt 界面
  4. 不调用系统 Python / Anaconda / venv


三、调试启动方式
----------------------------------------
启动软件_调试模式.bat 会：
  1. 保留黑色 CMD 窗口
  2. 打印 sys.executable（必须指向 ...\runtime\python.exe）
  3. 用 runtime\python.exe 启动
  4. 程序退出后 pause，方便查看 Traceback / ImportError / DLL 错误


四、runtime 是什么
----------------------------------------
runtime\ 是项目自己的便携 Python 运行环境，内含：
  python.exe / pythonw.exe
  标准库
  本项目所需第三方包（PySide6、torch、ultralytics、OpenCV 等）

正式运行必须使用：
  .\runtime\python.exe
  .\runtime\pythonw.exe

不要使用：
  python / py / conda / venv\Scripts\python.exe
  任何 Anaconda 绝对路径


五、为什么不能复制普通 venv
----------------------------------------
普通 venv 会在 pyvenv.cfg 里写死原电脑 Python，例如：
  home = D:\ProgramData\anaconda3
  executable = D:\ProgramData\anaconda3\python.exe

复制到另一台电脑后，如果原路径不存在，就会报：
  No Python at '"D:\ProgramData\anaconda3\python.exe'

本项目的 runtime 是完整安装到项目目录内的 Python，可随文件夹移动。


六、换电脑以后哪些东西不需要安装
----------------------------------------
通常不需要再装：
  - Anaconda
  - 系统 Python（只要带上已构建好的 runtime）
  - 重新创建 venv
  - Visual Studio / CUDA Toolkit（当前验证为 CPU 版 PyTorch）

项目自带（复制文件夹即可）：
  - Python runtime 与第三方包
  - 源代码、配置、模型、UI 资源
  - Livox 采集 exe/dll（third_party\livox_runtime）


七、哪些硬件驱动仍然可能需要安装
----------------------------------------
Python runtime 解决不了硬件驱动：

  1) Intel RealSense D435i
     - 需要：Windows RealSense 驱动 / RealSense Viewer 或官方 Runtime
     - 不等于：pip 里的 pyrealsense2 包（包已在 runtime 中，驱动仍要装）

  2) NVIDIA GPU（仅当你改用了 CUDA 版 PyTorch）
     - 需要：兼容的 NVIDIA 显卡驱动
     - 通常不需要：完整 CUDA Toolkit / Anaconda

  3) Livox Mid360
     - 需要：网卡、IP/网段正确；SDK DLL 已在 third_party\livox_runtime

  4) PLC（Modbus TCP）
     - 需要：工控机与 PLC 网口互通
     - IP/端口在 config\external_devices_config.py 中配置

  5) 若提示缺少 VC++ 运行库
     - 安装微软 Visual C++ Redistributable（x64）


八、D435i 需要什么
----------------------------------------
  A. 项目内：pyrealsense2 Python 包（随 runtime 安装）
  B. 系统内：Intel RealSense Windows 驱动/Runtime
  C. 硬件：USB3 口连接，相机供电正常

没有相机时，软件应能启动；真机采集会提示设备未连接/失败，
不应因“找不到 Python”而崩溃。


九、NVIDIA GPU 需要什么
----------------------------------------
当前交付默认按已验证的 CPU 版 PyTorch 构建。
  - 仅 CPU：目标机不必装 NVIDIA 驱动也可跑算法（更慢）
  - 若开发机重建为 CUDA 版 torch：目标机只需装匹配的 NVIDIA 驱动


十、PLC / 网口 / IP 是否需要现场配置
----------------------------------------
需要。复制项目不会自动改现场地址。

请现场工程师检查：
  config\external_devices_config.py
    - DEVICE_MODE = "real" 或 "mock"
    - PLC["ip"] / PLC["port"]
    - LIVOX 主机 IP、雷达相关配置
    - 相机分辨率/外参文件路径（相对项目根目录）

相关同步文件：
  third_party\plc_finished_app\gantry_settings.json
  third_party\livox_runtime\mid360s_config.json


十一、常见错误排查
----------------------------------------
1) No Python at 'D:\ProgramData\anaconda3\...'
   → 仍在用旧 venv。请用 启动软件.bat，确认存在 runtime\python.exe

2) 提示找不到 runtime\python.exe
   → 开发机先运行 tools\build_runtime.bat，再复制整个文件夹

3) ModuleNotFoundError: PySide6 / torch / ultralytics ...
   → runtime 未装全依赖。在开发机重新执行 build_runtime.bat

4) DLL load failed / 导入 cv2、torch 失败
   → 安装 VC++ x64 运行库；确认复制了完整 runtime 目录

5) RealSense 打不开
   → 先装 RealSense 驱动，用官方 Viewer 确认相机可用，再开本软件

6) PLC 连接失败
   → 查网线、IP、端口、防火墙；确认 DEVICE_MODE 与现场一致

7) 模型找不到
   → 确认 models\corner_service.pt 、 models\pallet_hole_best.pt 存在
   → 程序按项目根目录定位，不依赖你从哪个文件夹双击（bat 会先 cd 到项目根）

8) 想看详细环境
   → 运行：
        runtime\python.exe tools\check_runtime.py


十二、目录说明（改造后）
----------------------------------------
  runtime\     便携 Python（正式运行环境）
  workdir\     运行产生的业务数据/结果（可删，可重建）
  logs\        启动日志 startup.log / startup_fault.log
  models\      YOLO 等模型
  config\      配置（相对路径）
  venv\        旧虚拟环境（可删除，不再用于启动）


十三、开发机如何构建 runtime
----------------------------------------
在能上网的 Windows 开发电脑上：

  1. 双击或命令行运行：
       tools\build_runtime.bat
  2. 脚本会下载官方 Python 3.12 并安装到 runtime\
  3. 再按 requirements.txt 安装依赖
  4. 自动调用 tools\check_runtime.py 自检

构建完成后，把整个 jushenzhineng_v3 拷到现场即可。


十四、联系现场联调前请确认
----------------------------------------
  [ ] 已包含 runtime\python.exe
  [ ] 启动软件_调试模式.bat 打印的 sys.executable 在本项目 runtime 下
  [ ] models 与 config 文件齐全
  [ ] PLC IP / 雷达 IP / 相机驱动已按现场准备
