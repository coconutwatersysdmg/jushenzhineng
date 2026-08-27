import io
import os
from pathlib import Path
from typing import Optional, Tuple

import numpy as np
import torch

from .models.pointnet2_part_seg_msg import get_model


DEFAULT_NUM_POINT = 16384
DEFAULT_NUM_PART = 4
DEFAULT_NUM_CLASSES = 1


def pc_normalize(pc: np.ndarray) -> np.ndarray:
    pc = np.asarray(pc, dtype=np.float32)
    centroid = np.mean(pc, axis=0)
    pc = pc - centroid
    m = np.max(np.sqrt(np.sum(pc ** 2, axis=1)))
    if m > 0:
        pc = pc / m
    return pc


def _pcd_numpy_dtype(size: int, ptype: str):
    if ptype == "F":
        return np.float32 if size == 4 else np.float64
    if ptype == "I":
        return {1: np.int8, 2: np.int16, 4: np.int32, 8: np.int64}.get(size, np.int32)
    if ptype == "U":
        return {1: np.uint8, 2: np.uint16, 4: np.uint32, 8: np.uint64}.get(size, np.uint32)
    return np.float32


def _ply_numpy_dtype(ptype: str, endian: str = "<"):
    mapping = {
        "char": "i1", "int8": "i1",
        "uchar": "u1", "uint8": "u1",
        "short": endian + "i2", "int16": endian + "i2",
        "ushort": endian + "u2", "uint16": endian + "u2",
        "int": endian + "i4", "int32": endian + "i4",
        "uint": endian + "u4", "uint32": endian + "u4",
        "float": endian + "f4", "float32": endian + "f4",
        "double": endian + "f8", "float64": endian + "f8",
    }
    if ptype not in mapping:
        raise ValueError(f"不支持的 PLY 属性类型: {ptype}")
    return np.dtype(mapping[ptype])


def load_txt(path: str) -> np.ndarray:
    data = np.loadtxt(path).astype(np.float32)
    if data.ndim == 1:
        data = data.reshape(1, -1)
    if data.shape[1] < 3:
        raise ValueError(f"TXT 文件少于 3 列，无法读取 xyz: {path}")
    return data[:, :3].copy()


def load_pcd(path: str) -> np.ndarray:
    header_lines = []
    with open(path, "rb") as f:
        while True:
            line = f.readline()
            if not line:
                raise ValueError(f"PCD 文件缺少 DATA 行: {path}")
            decoded = line.decode("utf-8", errors="ignore").strip()
            header_lines.append(decoded)
            if decoded.upper().startswith("DATA"):
                data_bytes = f.read()
                break

    items = {}
    for line in header_lines:
        if not line or line.startswith("#"):
            continue
        parts = line.split()
        if len(parts) >= 2:
            items[parts[0].upper()] = parts[1:]

    fields = items.get("FIELDS", [])
    sizes = [int(x) for x in items.get("SIZE", ["4"] * len(fields))]
    types = items.get("TYPE", ["F"] * len(fields))
    counts = [int(x) for x in items.get("COUNT", ["1"] * len(fields))]
    width = int(items.get("WIDTH", ["0"])[0])
    height = int(items.get("HEIGHT", ["1"])[0])
    points = int(items.get("POINTS", [str(width * height)])[0])
    data_type = items.get("DATA", ["ascii"])[0].lower()

    if data_type == "ascii":
        text = data_bytes.decode("utf-8", errors="ignore")
        data = np.loadtxt(io.StringIO(text)).astype(np.float32)
        if data.ndim == 1:
            data = data.reshape(1, -1)
    elif data_type == "binary":
        dtype_fields = []
        for field, size, ptype, count in zip(fields, sizes, types, counts):
            dt = _pcd_numpy_dtype(size, ptype)
            dtype_fields.append((field, dt) if count == 1 else (field, dt, (count,)))
        dtype = np.dtype(dtype_fields)
        arr = np.frombuffer(data_bytes, dtype=dtype, count=points)
        cols = []
        for field, count in zip(fields, counts):
            col = arr[field]
            cols.append(col.reshape(-1, 1) if count == 1 else col.reshape(points, count))
        data = np.hstack(cols).astype(np.float32)
    else:
        raise ValueError(f"不支持的 PCD DATA 类型: {data_type}")

    starts = {}
    offset = 0
    for field, count in zip(fields, counts):
        starts[field] = offset
        offset += count

    if not all(k in starts for k in ("x", "y", "z")):
        raise ValueError(f"PCD 文件缺少 x y z 字段: {path}")

    return data[:, [starts["x"], starts["y"], starts["z"]]].copy()


def load_ply(path: str) -> np.ndarray:
    header_lines = []
    with open(path, "rb") as f:
        while True:
            line = f.readline()
            if not line:
                raise ValueError(f"PLY 文件缺少 end_header: {path}")
            decoded = line.decode("utf-8", errors="ignore").strip()
            header_lines.append(decoded)
            if decoded == "end_header":
                data_bytes = f.read()
                break

    fmt = None
    vertex_count = None
    vertex_properties = []
    current_element = None

    for line in header_lines:
        parts = line.split()
        if not parts:
            continue
        if parts[0] == "format":
            fmt = parts[1]
        elif parts[0] == "element":
            current_element = parts[1]
            if current_element == "vertex":
                vertex_count = int(parts[2])
        elif parts[0] == "property" and current_element == "vertex":
            if parts[1] == "list":
                raise ValueError(f"PLY vertex 暂不支持 list 属性: {path}")
            vertex_properties.append((parts[1], parts[2]))

    if fmt is None or vertex_count is None:
        raise ValueError(f"PLY 头文件不完整: {path}")

    if fmt == "ascii":
        text = data_bytes.decode("utf-8", errors="ignore")
        lines = text.splitlines()[:vertex_count]
        data = np.loadtxt(io.StringIO("\n".join(lines))).astype(np.float32)
        if data.ndim == 1:
            data = data.reshape(1, -1)
    elif fmt in ("binary_little_endian", "binary_big_endian"):
        endian = "<" if fmt == "binary_little_endian" else ">"
        dtype = np.dtype([(name, _ply_numpy_dtype(ptype, endian)) for ptype, name in vertex_properties])
        arr = np.frombuffer(data_bytes, dtype=dtype, count=vertex_count)
        data = np.hstack([arr[name].reshape(-1, 1) for _, name in vertex_properties]).astype(np.float32)
    else:
        raise ValueError(f"不支持的 PLY 格式: {fmt}")

    names = [name for _, name in vertex_properties]
    if not all(k in names for k in ("x", "y", "z")):
        raise ValueError(f"PLY 文件缺少 x y z 字段: {path}")

    return data[:, [names.index("x"), names.index("y"), names.index("z")]].copy()


def load_point_cloud(path: str) -> np.ndarray:
    ext = Path(path).suffix.lower()
    if ext == ".txt":
        return load_txt(path)
    if ext == ".pcd":
        return load_pcd(path)
    if ext == ".ply":
        return load_ply(path)
    raise ValueError(f"不支持的文件格式: {path}")


class PointNet2Segmenter:
    """PointNet++ 4 类点云分割推理封装。"""

    def __init__(
        self,
        checkpoint_path: Optional[str] = None,
        device: Optional[str] = None,
        num_point: int = DEFAULT_NUM_POINT,
        num_votes: int = 1,
        seed: int = 42,
    ):
        package_dir = Path(__file__).resolve().parent
        if checkpoint_path is None:
            checkpoint_path = str(package_dir / "checkpoints" / "best_model.pth")

        self.num_point = int(num_point)
        self.num_votes = int(num_votes)
        self.seed = int(seed)
        self.rng = np.random.default_rng(self.seed)

        if device is None:
            self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        else:
            self.device = torch.device(device)

        np.random.seed(self.seed)
        torch.manual_seed(self.seed)
        if torch.cuda.is_available():
            torch.cuda.manual_seed_all(self.seed)

        self.model = get_model(DEFAULT_NUM_PART, normal_channel=False).to(self.device)

        try:
            state_dict = torch.load(checkpoint_path, map_location=self.device, weights_only=True)
        except TypeError:
            state_dict = torch.load(checkpoint_path, map_location=self.device)

        self.model.load_state_dict(state_dict)
        self.model.eval()
        self.checkpoint_path = checkpoint_path

    def _sample_indices(self, n: int) -> np.ndarray:
        if n <= 0:
            raise ValueError("输入点云为空")
        replace = n < self.num_point
        return self.rng.choice(n, self.num_point, replace=replace).astype(np.int64)

    @staticmethod
    def _validate_xyz(raw_xyz: np.ndarray) -> np.ndarray:
        raw_xyz = np.asarray(raw_xyz, dtype=np.float32)
        if raw_xyz.ndim != 2 or raw_xyz.shape[1] != 3:
            raise ValueError(f"raw_xyz 必须是 [N,3]，当前形状: {raw_xyz.shape}")
        return raw_xyz

    def _predict_indices(
        self,
        raw_xyz: np.ndarray,
        choice: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray]:
        sampled_xyz, pred_label, _ = self._predict_indices_with_scores(
            raw_xyz,
            choice,
        )
        return sampled_xyz, pred_label

    def _predict_indices_with_scores(
        self,
        raw_xyz: np.ndarray,
        choice: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Predict labels and keep the averaged per-class model scores."""

        norm_xyz_all = pc_normalize(raw_xyz.copy())
        sampled_xyz = raw_xyz[choice]
        norm_xyz = norm_xyz_all[choice]

        points = torch.from_numpy(norm_xyz).float().unsqueeze(0).to(self.device)
        points = points.transpose(2, 1)
        cls_label = torch.zeros((1, 1), dtype=torch.long, device=self.device)
        cls_one_hot = torch.eye(DEFAULT_NUM_CLASSES, device=self.device)[cls_label.cpu().numpy()]

        vote_pool = torch.zeros(1, self.num_point, DEFAULT_NUM_PART, device=self.device)

        with torch.inference_mode():
            for _ in range(max(self.num_votes, 1)):
                seg_pred, _ = self.model(points, cls_one_hot)
                vote_pool += seg_pred

        averaged_scores = vote_pool / max(self.num_votes, 1)
        pred_label = torch.argmax(averaged_scores, dim=2)[0]
        pred_label = pred_label.cpu().numpy().astype(np.int32)
        pred_scores = averaged_scores[0].cpu().numpy().astype(np.float32)

        return sampled_xyz.astype(np.float32), pred_label, pred_scores

    def predict_xyz_with_indices(
        self,
        raw_xyz: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Predict sampled points and retain their exact source row indices."""

        raw_xyz = self._validate_xyz(raw_xyz)
        choice = self._sample_indices(raw_xyz.shape[0])
        sampled_xyz, pred_label = self._predict_indices(raw_xyz, choice)
        return sampled_xyz, pred_label, choice

    def predict_xyz_with_indices_and_scores(
        self,
        raw_xyz: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Predict sampled points and return source indices plus class scores.

        Returns:
            sampled_xyz: [num_point, 3]
            pred_label:   [num_point]
            choice:       [num_point] source-row indices in ``raw_xyz``
            pred_scores:  [num_point, 4] averaged model scores before argmax
        """

        raw_xyz = self._validate_xyz(raw_xyz)
        choice = self._sample_indices(raw_xyz.shape[0])
        sampled_xyz, pred_label, pred_scores = (
            self._predict_indices_with_scores(raw_xyz, choice)
        )
        return sampled_xyz, pred_label, choice, pred_scores

    def predict_xyz(self, raw_xyz: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        输入:
            raw_xyz: [N,3] 原始 XYZ 点云

        返回:
            sampled_xyz: [num_point,3] 采样后的原始 XYZ
            pred_label:  [num_point]    预测标签 0~3
        """

        sampled_xyz, pred_label, _ = self.predict_xyz_with_indices(raw_xyz)
        return sampled_xyz, pred_label

    def predict_file(self, input_path: str, save_path: Optional[str] = None):
        """
        对 TXT / PCD / PLY 推理。

        save_path 不为空时保存格式：
            x y z pred_label

        该输出可直接交给后续 Edge_line_extraction 使用。
        """
        raw_xyz = load_point_cloud(input_path)
        sampled_xyz, pred_label = self.predict_xyz(raw_xyz)

        result = np.hstack([
            sampled_xyz,
            pred_label.reshape(-1, 1).astype(np.float32),
        ])

        if save_path is not None:
            save_path = str(save_path)
            Path(save_path).parent.mkdir(parents=True, exist_ok=True)
            np.savetxt(save_path, result, fmt="%.6f")

        counts = np.bincount(pred_label, minlength=DEFAULT_NUM_PART)
        return result, counts

    def warmup(self):
        """软件启动阶段可调用一次，用于 CUDA / 模型预热。"""
        dummy = np.zeros((self.num_point, 3), dtype=np.float32)
        dummy[:, 0] = np.linspace(-1.0, 1.0, self.num_point, dtype=np.float32)
        self.predict_xyz(dummy)
