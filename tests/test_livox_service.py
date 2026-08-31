from __future__ import annotations

import sys
import tempfile
import unittest
import importlib.util
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SERVICE_PATH = PROJECT_ROOT / "services" / "livox_service.py"

spec = importlib.util.spec_from_file_location("livox_service_test_module", SERVICE_PATH)
livox_service_module = importlib.util.module_from_spec(spec)
assert spec.loader is not None
sys.modules["livox_service_test_module"] = livox_service_module
spec.loader.exec_module(livox_service_module)
LivoxService = livox_service_module.LivoxService


class LivoxServiceTest(unittest.TestCase):
    def test_default_capture_names_pcds_in_order_without_overwriting(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            fake_capture = root / "fake_capture.py"
            fake_capture.write_text(
                "\n".join(
                    [
                        "from pathlib import Path",
                        "import sys",
                        "output = Path(sys.argv[sys.argv.index('--capture-pcd') + 1])",
                        "output.parent.mkdir(parents=True, exist_ok=True)",
                        "output.write_text('\\n'.join([",
                        "    '# .PCD v0.7 - Point Cloud Data file format',",
                        "    'VERSION 0.7',",
                        "    'FIELDS x y z',",
                        "    'SIZE 4 4 4',",
                        "    'TYPE F F F',",
                        "    'COUNT 1 1 1',",
                        "    'WIDTH 1',",
                        "    'HEIGHT 1',",
                        "    'VIEWPOINT 0 0 0 1 0 0 0',",
                        "    'POINTS 1',",
                        "    'DATA ascii',",
                        "    '0 0 0',",
                        "]) + '\\n', encoding='utf-8')",
                        "print('Captured points: 1')",
                    ]
                ),
                encoding="utf-8",
            )

            config_dir = root / "config"
            config_dir.mkdir()
            (config_dir / "livox_config.ini").write_text(
                "\n".join(
                    [
                        "[livox]",
                        f"exe_path = {sys.executable}",
                        f"config_path = {fake_capture}",
                        "save_dir = data/lidar",
                        "capture_ms = 10",
                        "timeout_sec = 5",
                        "",
                    ]
                ),
                encoding="utf-8",
            )

            service = LivoxService(root)
            first = service.capture_once()
            second = service.capture_once()

            self.assertTrue(first.success)
            self.assertTrue(second.success)
            self.assertEqual(first.pcd_path.name, "scan_000001.pcd")
            self.assertEqual(second.pcd_path.name, "scan_000002.pcd")
            self.assertTrue(first.pcd_path.exists())
            self.assertTrue(second.pcd_path.exists())
            self.assertNotEqual(first.pcd_path, second.pcd_path)

    def test_capture_command_uses_configured_three_seconds_and_max_points(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            fake_capture = root / "fake_capture.py"
            args_path = root / "args.txt"
            fake_capture.write_text(
                "\n".join(
                    [
                        "from pathlib import Path",
                        "import sys",
                        f"Path({str(args_path)!r}).write_text('\\n'.join(sys.argv), encoding='utf-8')",
                        "output = Path(sys.argv[sys.argv.index('--capture-pcd') + 1])",
                        "output.parent.mkdir(parents=True, exist_ok=True)",
                        "output.write_text('POINTS 1\\nDATA ascii\\n0 0 0\\n', encoding='utf-8')",
                        "print('Captured points: 1')",
                    ]
                ),
                encoding="utf-8",
            )

            config_dir = root / "config"
            config_dir.mkdir()
            (config_dir / "livox_config.ini").write_text(
                "\n".join(
                    [
                        "[livox]",
                        f"exe_path = {sys.executable}",
                        f"config_path = {fake_capture}",
                        "save_dir = data/lidar",
                        "capture_ms = 3000",
                        "max_points = 600000",
                        "timeout_sec = 20",
                        "",
                    ]
                ),
                encoding="utf-8",
            )

            service = LivoxService(root)
            result = service.capture_once()

            args = args_path.read_text(encoding="utf-8").splitlines()
            self.assertTrue(result.success)
            self.assertIn("--capture-ms", args)
            self.assertEqual(args[args.index("--capture-ms") + 1], "3000")
            self.assertIn("--max-points", args)
            self.assertEqual(args[args.index("--max-points") + 1], "600000")


if __name__ == "__main__":
    unittest.main()
