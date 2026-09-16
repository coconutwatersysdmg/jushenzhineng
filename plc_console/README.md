# PLC 运动控制台（独立模块）

与主数字孪生系统同仓、目录平级，不混入主业务包。

## 启动

```bat
runtime\python.exe -m plc_console
```

或：

```bat
runtime\python.exe plc_console\main.py
```

## 作用

1. 监听本地 IPC：`jushenzhineng-plc-console`
2. 接收主系统推送的 `plc_motion_cmd_v1` JSON
3. 连接实验室 PLC，按 XYZR 绝对定位
4. 到位轮询只在本窗口完成；主系统推送成功即可继续下一步

## 协议

见仓库根目录 `contracts/plc_motion_cmd_v1.schema.json`。

主系统落盘目录：`workdir/plc_commands/`。
