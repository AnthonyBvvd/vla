"""RoArm-M3 的 LeRobot 数据字段、写入配对和 ACT 数据检查。"""

import json
from pathlib import Path

import cv2
import numpy as np
from lerobot.datasets import LeRobotDataset

JOINTS = ["base.pos", "shoulder.pos", "elbow.pos", "wrist_tilt.pos", "wrist_roll.pos", "gripper.pos"]
IMAGE = "observation.images.front"
FPS = 5
SIZE = (640, 360)


def features():
    return {
        "observation.state": {"dtype": "float32", "shape": (6,), "names": JOINTS},
        "action": {"dtype": "float32", "shape": (6,), "names": JOINTS},
        IMAGE: {"dtype": "video", "shape": (SIZE[1], SIZE[0], 3),
                "names": ("height", "width", "channels")},
    }


def read_state(arm):
    state = np.asarray(arm.joints_angle_get(), dtype=np.float32)
    if state.shape != (6,) or not np.isfinite(state).all():
        raise ValueError(f"关节反馈检查：{state!r}")
    return state


def add_pair(dataset, previous, current, task):
    dataset.add_frame({
        "observation.state": previous["state"].copy(),
        "action": current["state"].copy(),
        IMAGE: previous["image"],
        "task": task,
    })


def save_pending(dataset):
    if dataset.has_pending_frames():
        dataset.save_episode()


def read_info(root):
    return json.loads((root / "capture/info.json").read_text(encoding="utf-8"))


def check(args):
    import torch
    from torch.utils.data import DataLoader
    from lerobot.configs import FeatureType
    from lerobot.policies.act import ACTConfig, ACTPolicy
    from lerobot.policies.act.processor_act import make_act_pre_post_processors
    from lerobot.utils.feature_utils import dataset_to_policy_features

    root = args.root.resolve()
    info = read_info(root)
    dataset = LeRobotDataset(info["repo_id"], root=root, video_backend="torchcodec")
    assert dataset.fps == FPS
    traces = sorted((root / "capture").glob("episode_*.json"))
    assert len(traces) == dataset.num_episodes
    cursor = 0
    steps = []
    gaps = []
    for trace_path in traces:
        trace = json.loads(trace_path.read_text())
        samples = trace["samples"]
        for index in range(len(samples) - 1):
            row = dataset[cursor]
            np.testing.assert_allclose(row["observation.state"].numpy(), samples[index]["state"], atol=1e-6)
            np.testing.assert_allclose(row["action"].numpy(), samples[index + 1]["state"], atol=1e-6)
            assert row["episode_index"].item() == trace["episode"]
            assert abs(row["timestamp"].item() - index / FPS) < 1e-5
            assert row[IMAGE].shape == (3, SIZE[1], SIZE[0])
            assert row[IMAGE].dtype == torch.float32 and torch.isfinite(row[IMAGE]).all()
            steps.append(np.abs(np.asarray(samples[index + 1]["state"]) - samples[index]["state"]))
            gaps.append(samples[index + 1]["time_s"] - samples[index]["time_s"])
            cursor += 1
    assert cursor == len(dataset) and cursor > 0
    for key in ("observation.state", "action", IMAGE):
        assert all(np.isfinite(value).all() for value in dataset.meta.stats[key].values())
    chunked = LeRobotDataset(info["repo_id"], root=root, video_backend="torchcodec",
                            delta_timestamps={"action": [index / FPS for index in range(10)]})
    assert chunked[len(dataset) - 1]["action_is_pad"].tolist() == [False] + [True] * 9
    policy_features = dataset_to_policy_features(dataset.features)
    config = ACTConfig(input_features={k: v for k, v in policy_features.items() if v.type is not FeatureType.ACTION},
                       output_features={"action": policy_features["action"]}, device="cpu",
                       chunk_size=10, n_action_steps=1, pretrained_backbone_weights=None,
                       dim_model=32, n_heads=4, dim_feedforward=64,
                       n_encoder_layers=1, n_decoder_layers=1, latent_dim=4, n_vae_encoder_layers=1)
    config.validate_features()
    result = {"repo_id": info["repo_id"], "root": str(root), "episodes": dataset.num_episodes,
              "frames": len(dataset), "fps": dataset.fps, "image_shape": list(dataset[0][IMAGE].shape),
              "joint_names": JOINTS, "action_source": info["action_source"],
              "next_state_pairing": True, "act_chunk_shape": list(chunked[0]["action"].shape),
              "last_chunk_padding": chunked[len(dataset) - 1]["action_is_pad"].tolist(),
              "joint_range_degrees": (np.asarray(dataset.meta.stats["observation.state"]["max"]) -
                                       dataset.meta.stats["observation.state"]["min"]).tolist(),
              "max_step_degrees": np.max(steps, axis=0).tolist(),
              "sample_interval_seconds": {"min": float(min(gaps)), "max": float(max(gaps)),
                                          "mean": float(np.mean(gaps))},
              "first_state": dataset[0]["observation.state"].tolist(),
              "last_target": dataset[len(dataset) - 1]["action"].tolist()}
    if args.act_forward:
        torch.set_num_threads(4)
        policy = ACTPolicy(config)
        preprocessor, _ = make_act_pre_post_processors(config, dataset_stats=dataset.meta.stats)
        batch = next(iter(DataLoader(chunked, batch_size=1, num_workers=0)))
        loss, _ = policy.forward(preprocessor(batch))
        assert torch.isfinite(loss)
        loss.backward()
        result["act_forward_backward"] = True
        result["act_test_loss"] = float(loss.detach())
    preview = (dataset[0][IMAGE].permute(1, 2, 0).numpy() * 255).clip(0, 255).astype(np.uint8)
    assert cv2.imwrite(str(root / "capture/first_frame.jpg"), cv2.cvtColor(preview, cv2.COLOR_RGB2BGR))
    report = json.dumps(result, ensure_ascii=False, indent=2)
    (root / "capture/dataset_check.json").write_text(report + "\n", encoding="utf-8")
    print(report)

