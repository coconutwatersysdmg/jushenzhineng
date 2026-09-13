from __future__ import annotations

import configparser
import re
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class LivoxCaptureResult:
    success: bool
    pcd_path: Path
    point_count: int | None
    return_code: int
    stdout: str
    stderr: str


class LivoxService:
    """Small Python wrapper for the Livox SDK2 capture executable.

    Put this file under the UI project's services/ folder. The UI can create one
    LivoxService object and call capture_once() when the user clicks a button.
    """

    def __init__(self, project_root: str | Path | None = None, config_file: str | Path | None = None) -> None:
        self.project_root = Path(project_root).resolve() if project_root else Path(__file__).resolve().parents[1]
        self.config_file = Path(config_file).resolve() if config_file else self.project_root / "config" / "livox_config.ini"
        self.settings = self._load_settings()

    def _load_settings(self) -> dict[str, str]:
        # 优先读统一外接设备配置；失败再回退 ini
        try:
            from config.external_devices_config import get_livox_runtime_settings, sync_livox_mid360_json

            sync_livox_mid360_json(self.project_root)
            return get_livox_runtime_settings()
        except Exception:
            pass
        parser = configparser.ConfigParser()
        if self.config_file.exists():
            parser.read(self.config_file, encoding="utf-8")
        values = parser["livox"] if parser.has_section("livox") else {}
        return {
            "exe_path": self._get_option(values, "exe_path", "third_party/livox_runtime/livox_realtime_select_and_move.exe"),
            "config_path": self._get_option(values, "config_path", "third_party/livox_runtime/mid360s_config.json"),
            "save_dir": self._get_option(values, "save_dir", "data/lidar"),
            "capture_ms": self._get_option(values, "capture_ms", "2000"),
            "max_points": self._get_option(values, "max_points", "300000"),
            "timeout_sec": self._get_option(values, "timeout_sec", "20"),
        }

    @staticmethod
    def _get_option(section: object, key: str, default: str) -> str:
        if not hasattr(section, "get"):
            return default
        candidates = (key, key.replace("_", " "), key.replace("_", "-"))
        for candidate in candidates:
            value = section.get(candidate)  # type: ignore[attr-defined]
            # ConfigParser 可能返回 None；空串也回退默认，避免后续路径拼接炸掉
            if value is not None and str(value).strip() != "":
                return str(value).strip()
        return default

    def _resolve_path(self, value: str | Path) -> Path:
        path = Path(value)
        if path.is_absolute():
            return path
        return self.project_root / path

    @property
    def exe_path(self) -> Path:
        return self._resolve_path(self.settings["exe_path"])

    @property
    def lidar_config_path(self) -> Path:
        return self._resolve_path(self.settings["config_path"])

    @property
    def save_dir(self) -> Path:
        return self._resolve_path(self.settings["save_dir"])

    def capture_once(
        self,
        filename: str | None = None,
        capture_ms: int | None = None,
        timeout_sec: int | None = None,
    ) -> LivoxCaptureResult:
        """Capture one PCD file and return the output path plus point count."""

        try:
            from config.external_devices_config import sync_livox_mid360_json

            sync_livox_mid360_json(self.project_root)
        except Exception:
            pass

        exe_path = self.exe_path
        config_path = self.lidar_config_path
        if not exe_path.exists():
            raise FileNotFoundError(f"找不到 Livox 采集程序：{exe_path}")
        if not config_path.exists():
            raise FileNotFoundError(f"找不到 Livox 配置文件：{config_path}")

        ms = int(capture_ms if capture_ms is not None else self.settings["capture_ms"])
        max_points = int(self.settings["max_points"])
        timeout = int(timeout_sec if timeout_sec is not None else self.settings["timeout_sec"])
        if ms <= 0:
            raise ValueError("capture_ms 必须大于 0")
        if max_points <= 0:
            raise ValueError("max_points 必须大于 0")

        self.save_dir.mkdir(parents=True, exist_ok=True)
        pcd_path = self._build_capture_path(filename)

        command = [
            str(exe_path),
            str(config_path),
            "--capture-pcd",
            str(pcd_path),
            "--capture-ms",
            str(ms),
            "--max-points",
            str(max_points),
        ]
        completed = subprocess.run(
            command,
            cwd=str(exe_path.parent),
            text=True,
            capture_output=True,
            timeout=timeout,
        )

        stdout = completed.stdout if completed.stdout is not None else ""
        stderr = completed.stderr if completed.stderr is not None else ""
        point_count = self._parse_point_count(stdout + "\n" + stderr)

        return LivoxCaptureResult(
            success=completed.returncode == 0 and pcd_path.exists(),
            pcd_path=pcd_path,
            point_count=point_count,
            return_code=int(completed.returncode),
            stdout=stdout,
            stderr=stderr,
        )

    def capture_timestamped(self, capture_ms: int | None = None) -> LivoxCaptureResult:
        filename = time.strftime("scan_%Y%m%d_%H%M%S.pcd")
        return self.capture_once(filename=filename, capture_ms=capture_ms)

    def _build_capture_path(self, filename: str | None = None) -> Path:
        if filename is None:
            return self._next_sequence_path()
        requested = self.save_dir / filename
        return self._avoid_overwrite(requested)

    def _next_sequence_path(self) -> Path:
        pattern = re.compile(r"^scan_(\d{6})\.pcd$", re.IGNORECASE)
        max_index = 0
        for path in self.save_dir.glob("scan_*.pcd"):
            match = pattern.match(path.name)
            if match:
                max_index = max(max_index, int(match.group(1)))

        next_index = max_index + 1
        while True:
            candidate = self.save_dir / f"scan_{next_index:06d}.pcd"
            if not candidate.exists():
                return candidate
            next_index += 1

    @staticmethod
    def _avoid_overwrite(path: Path) -> Path:
        if not path.exists():
            return path

        index = 1
        while True:
            candidate = path.with_name(f"{path.stem}_{index:06d}{path.suffix}")
            if not candidate.exists():
                return candidate
            index += 1

    @staticmethod
    def _parse_point_count(output: str) -> int | None:
        match = re.search(r"Captured points:\s*(\d+)", output)
        return int(match.group(1)) if match else None

if __name__ == "__main__":
    service = LivoxService()
    result = service.capture_once()
    print(f"success: {result.success}")
    print(f"pcd_path: {result.pcd_path}")
    print(f"point_count: {result.point_count}")
    print(f"return_code: {result.return_code}")
    if result.stdout:
        print(result.stdout)
    if result.stderr:
        print(result.stderr)
