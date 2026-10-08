# -*- coding: utf-8 -*-
"""现场模式双相机：海康角点（SDK 采图）+ 梅卡托盘（直连优先，失败切工控机路由）。

海康/梅卡采图都不打开厂家界面；采图前请关闭 Viewer/MVS 占用。
"""
from __future__ import annotations

import socket
from typing import Any, Mapping

from core.digital_twin_state import DigitalTwinState
from devices.hikvision_capture import capture_one_frame, mvs_sdk_available
from devices.mechmind_capture import (
    capture_rgbd as mech_capture_rgbd,
    capture_rgbd_via_ipc,
    discover_ips,
    sdk_available as mech_sdk_available,
)


class FieldCameraHub:
    """现场双相机入口；对外仍提供 capture_*，便于流程沿用 camera_id。"""

    def __init__(self, twin: DigitalTwinState, config: Mapping[str, Any] | None = None):
        self.twin = twin
        self.config = dict(config or {})
        self.field_cfg = dict(self.config.get("field_cameras") or {})
        self.tagged_images: dict[str, str] = {}
        self.tagged_depths: dict[str, str] = {}
        self.allow_file_inputs = bool(self.config.get("allow_file_inputs", True))
        self.demo_enabled = False
        self.mock_probe = bool(self.config.get("mock_probe", False))

    def set_demo_enabled(self, enabled: bool):
        self.demo_enabled = bool(enabled)

    def set_tagged_image(self, tag: str, path: str):
        self.tagged_images[str(tag)] = str(path or "")

    def set_tagged_depth(self, tag: str, path: str):
        self.tagged_depths[str(tag)] = str(path or "")

    def connect(self) -> dict:
        """兼容旧单相机接口：同时探测两台，全部成功才算成功。"""
        results = [self.probe_camera("CAM_CORNER"), self.probe_camera("CAM_PALLET")]
        ok = all(r.get("success") for r in results)
        return {
            "success": ok,
            "device_id": "FIELD_CAMERAS",
            "message": "；".join(str(r.get("message") or "") for r in results),
            "cameras": {r.get("device_id"): r for r in results},
        }

    def probe(self) -> dict:
        return self.connect()

    def probe_camera(self, camera_id: str) -> dict:
        cid = str(camera_id or "").strip().upper()
        if self.mock_probe:
            title = "海康角点相机" if cid == "CAM_CORNER" else ("梅卡托盘相机" if cid == "CAM_PALLET" else cid)
            self.twin.update_camera(cid, status="ONLINE", task="MOCK_READY")
            return {"success": True, "device_id": cid, "message": f"Mock {title}就绪"}
        if cid == "CAM_CORNER":
            return self._probe_hikvision()
        if cid == "CAM_PALLET":
            return self._probe_mechmind()
        return {
            "success": False,
            "device_id": cid or "CAMERA",
            "message": f"未知现场相机：{camera_id}",
        }

    def _probe_hikvision(self) -> dict:
        device_id = "CAM_CORNER"
        cam_cfg = dict(self.field_cfg.get("CAM_CORNER") or {})
        ip = str(cam_cfg.get("ip") or "").strip()
        sdk_ok, sdk_note = mvs_sdk_available()

        ip_ok = None
        if ip:
            ip_ok = self._tcp_ping(ip, int(cam_cfg.get("port", 8000) or 8000), timeout_s=1.2)

        if sdk_ok and (ip_ok is True or not ip):
            self.twin.update_camera(device_id, status="ONLINE", task="HIKVISION_PROBE_OK")
            msg = f"海康角点相机就绪（{sdk_note}"
            if ip:
                msg += f"；IP {ip} " + ("可达" if ip_ok else "未测端口")
            msg += "）。本系统可直接采图，无需打开 MVS 界面（采图前请关闭 MVS 对该相机的连接）。"
            return {"success": True, "device_id": device_id, "message": msg, "ip": ip, "sdk": sdk_note}

        if sdk_ok and ip and ip_ok is False:
            self.twin.update_camera(device_id, status="OFFLINE", task="HIKVISION_IP_FAIL")
            return {
                "success": False,
                "device_id": device_id,
                "message": f"海康 SDK 已装，但相机 IP {ip} 不通。检查网线/IP 是否同网段。",
                "ip": ip,
            }

        self.twin.update_camera(device_id, status="OFFLINE", task="HIKVISION_NO_SDK")
        tip = "请先安装海康 MVS（含 Development/Samples/Python），网线连接 MV-CE300P-04T 后重试。"
        if ip:
            tip += f" 配置 IP={ip}。"
        return {"success": False, "device_id": device_id, "message": tip, "ip": ip}

    def _mech_cfg(self) -> dict:
        return dict(self.field_cfg.get("CAM_PALLET") or {})

    def _probe_mechmind(self) -> dict:
        """探测顺序与采图一致：prefer_direct 时先直连，失败再工控机。"""
        device_id = "CAM_PALLET"
        cam_cfg = self._mech_cfg()
        prefer_direct = bool(cam_cfg.get("prefer_direct", True))
        cam_ip = str(cam_cfg.get("ip") or "").strip()
        ipc_ip = str(cam_cfg.get("ipc_ip") or "").strip()

        if prefer_direct:
            sdk_ok, sdk_note = mech_sdk_available()
            if sdk_ok:
                ips = discover_ips()
                if cam_ip and cam_ip in ips:
                    self.twin.update_camera(device_id, status="ONLINE", task="MECHMIND_DIRECT_OK")
                    return {
                        "success": True,
                        "device_id": device_id,
                        "route": "direct",
                        "ip": cam_ip,
                        "message": f"梅卡直连就绪（发现 IP={cam_ip}）。",
                    }
                if ips and not cam_ip:
                    self.twin.update_camera(device_id, status="ONLINE", task="MECHMIND_DIRECT_OK")
                    return {
                        "success": True,
                        "device_id": device_id,
                        "route": "direct",
                        "ip": ips[0],
                        "message": f"梅卡直连就绪（未配置 ip，将用发现的 {ips[0]}）。TODO：把该 IP 写入 CAM_PALLET.ip",
                    }
                if ips and cam_ip and cam_ip not in ips:
                    # 直连网通了但配置 IP 对不上，仍可尝试枚举；探测记警告后继续试工控机
                    direct_note = f"直连发现 {ips}，与配置 ip={cam_ip} 不一致；"
                else:
                    direct_note = f"直连未发现相机（{sdk_note}；检查网线/同网段/Viewer 占用）；"
            else:
                direct_note = f"直连不可用（{sdk_note}）；"
        else:
            direct_note = "已关闭 prefer_direct，跳过直连；"

        ipc_ok = False
        if ipc_ip:
            ipc_ok = (
                self._tcp_ping(ipc_ip, 22, timeout_s=1.0)
                or self._tcp_ping(ipc_ip, 3389, timeout_s=1.0)
                or self._host_ping(ipc_ip)
            )
        if ipc_ok:
            self.twin.update_camera(device_id, status="ONLINE", task="MECHMIND_IPC_ONLINE")
            return {
                "success": True,
                "device_id": device_id,
                "route": "ipc",
                "ipc_ip": ipc_ip,
                "message": (
                    f"{direct_note}已切工控机路由，工控机 {ipc_ip} 在线。"
                    "TODO：远程采图未实现，请 Anydesk+Viewer 验图或在工控机跑本程序。"
                ),
            }

        self.twin.update_camera(device_id, status="OFFLINE", task="MECHMIND_OFFLINE")
        return {
            "success": False,
            "device_id": device_id,
            "route": "none",
            "ip": cam_ip,
            "ipc_ip": ipc_ip,
            "message": (
                f"{direct_note}工控机也不通（ipc_ip={ipc_ip or '未配置'}）。"
                "TODO：填 CAM_PALLET.ip 或核对 ipc_ip（1/2/3 线）。"
            ),
        }

    @staticmethod
    def _tcp_ping(host: str, port: int, timeout_s: float = 1.0) -> bool:
        try:
            with socket.create_connection((host, int(port)), timeout=float(timeout_s)):
                return True
        except Exception:
            return False

    @staticmethod
    def _host_ping(host: str) -> bool:
        """ICMP 不一定可用；再用几个常见端口碰一下。"""
        for port in (445, 139, 80, 443, 5900, 7070):
            if FieldCameraHub._tcp_ping(host, port, timeout_s=0.6):
                return True
        return False

    def capture_rgb(self, camera_id: str, tag: str = "") -> dict:
        path = self.tagged_images.get(str(tag), "")
        if path:
            self.twin.update_camera(camera_id, status="ONLINE", task=f"CAPTURE_RGB:{tag}")
            return {
                "success": True,
                "device_id": camera_id,
                "image_path": path,
                "rgb_path": path,
                "source": "tagged_file",
                "message": "使用调试预填图像",
            }
        cid = str(camera_id or "").strip().upper()
        if cid == "CAM_CORNER":
            cam_cfg = dict(self.field_cfg.get("CAM_CORNER") or {})
            result = capture_one_frame(
                ip=str(cam_cfg.get("ip") or "").strip(),
                tag=str(tag or "corner"),
            )
            result["device_id"] = cid
            if result.get("success"):
                self.twin.update_camera(cid, status="ONLINE", task=f"CAPTURE_RGB:{tag}")
            else:
                self.twin.update_camera(cid, status="ERROR", task=f"CAPTURE_FAIL:{tag}")
            return result
        return {
            "success": False,
            "device_id": camera_id,
            "message": f"{camera_id} 采图适配尚未完成：请先用厂家软件拍照验证",
        }

    def capture_rgbd(self, camera_id: str, tag: str = "") -> dict:
        rgb = self.tagged_images.get(str(tag), "")
        depth = self.tagged_depths.get(str(tag), "")
        if rgb and depth:
            self.twin.update_camera(camera_id, status="ONLINE", task=f"CAPTURE_RGBD:{tag}")
            return {
                "success": True,
                "device_id": camera_id,
                "rgb_path": rgb,
                "depth_path": depth,
                "image_path": rgb,
                "source": "tagged_file",
                "message": "使用调试预填 RGB-D",
            }
        cid = str(camera_id or "").strip().upper()
        if cid == "CAM_PALLET":
            return self._capture_mechmind_rgbd(tag)
        return {
            "success": False,
            "device_id": camera_id,
            "message": f"{camera_id} RGB-D 采图适配尚未完成：请先用厂家软件拍照验证",
        }

    def _capture_mechmind_rgbd(self, tag: str = "") -> dict:
        """梅卡：prefer_direct 时先直连，失败再走工控机路由。"""
        cam_cfg = self._mech_cfg()
        prefer_direct = bool(cam_cfg.get("prefer_direct", True))
        cam_ip = str(cam_cfg.get("ip") or "").strip()
        ipc_ip = str(cam_cfg.get("ipc_ip") or "").strip()
        use_tag = str(tag or "pallet")

        direct_result: dict[str, Any] = {"success": False, "message": "跳过直连"}
        if prefer_direct:
            direct_result = mech_capture_rgbd(ip=cam_ip, tag=use_tag)
            direct_result["device_id"] = "CAM_PALLET"
            if direct_result.get("success"):
                self.twin.update_camera("CAM_PALLET", status="ONLINE", task=f"CAPTURE_RGBD:{use_tag}")
                return direct_result

        ipc_result = capture_rgbd_via_ipc(ipc_ip=ipc_ip, tag=use_tag)
        ipc_result["device_id"] = "CAM_PALLET"
        ipc_result["direct_error"] = direct_result.get("message")
        self.twin.update_camera("CAM_PALLET", status="ERROR", task=f"CAPTURE_FAIL:{use_tag}")
        # 合并提示，方便现场看清走了哪条路
        ipc_result["message"] = (
            f"直连失败（{direct_result.get('message')}）→ {ipc_result.get('message')}"
            if prefer_direct
            else str(ipc_result.get("message"))
        )
        return ipc_result
