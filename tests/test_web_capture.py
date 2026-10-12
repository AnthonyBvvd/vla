import argparse
from contextlib import redirect_stdout
import io
import json
import os
from pathlib import Path
import pty
import tempfile
import unittest
from unittest.mock import MagicMock, patch

import numpy as np
import torch
from lerobot.configs.video import RGBEncoderConfig
from lerobot.datasets import LeRobotDataset

from scripts import web_capture
from roarm_keyboard.control import Link


class WebCaptureTest(unittest.TestCase):
    def test_replay_compares_feedback_with_last_sent_target(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            (root / "capture").mkdir()
            (root / "capture/info.json").write_text(json.dumps({"repo_id": "local/test"}))
            config = root / "config.json"
            config.write_text("{}")
            initial = [0, 20, 100, 0, 0, 10]
            first = [0, 16.2, 100, 0, 0, 10]
            second = [0, 12.4, 100, 0, 0, 10]
            dataset = MagicMock()
            dataset.hf_dataset = {"observation.state": [initial, first], "action": [first, second]}
            arm = MagicMock()
            arm.joints_angle_get.side_effect = [initial, initial, [0, 21.5, 100, 0, 0, 10],
                                               [0, 17.7, 100, 0, 0, 10], second]
            args = argparse.Namespace(root=root, episode=0, max_step=5, max_start=3,
                                      execute=True, config=config, speed=10, countdown=0, rate=0.25)
            with patch.object(web_capture, "LeRobotDataset", return_value=dataset), \
                 patch.object(web_capture, "open_arm", return_value=arm), \
                 patch.object(web_capture, "send_target") as send, \
                 patch.object(web_capture.time, "sleep"), redirect_stdout(io.StringIO()):
                web_capture.replay(args)
            self.assertEqual(send.call_count, 3)
            arm.disconnect.assert_called_once()
            report = json.loads((root / "capture/replay_0000.json").read_text())
            self.assertEqual(report["frames_sent"], 2)
            self.assertAlmostEqual(report["readings"][0]["tracking_error_degrees"][1], 1.5)

    def test_sdk_serial_connection_uses_one_program(self):
        master, slave = pty.openpty()
        port = os.ttyname(slave)
        link = Link(port=port)
        try:
            with self.assertRaises(ValueError):
                web_capture.open_arm({"port": port, "baudrate": 115200})
            link.close()
            arm = web_capture.open_arm({"port": port, "baudrate": 115200})
            try:
                self.assertTrue(arm._serial_port.exclusive)
            finally:
                arm.disconnect()
        finally:
            link.close()
            os.close(master)
            os.close(slave)

    def test_pairs_episodes_and_act_chunks(self):
        cache = web_capture.ROOT / ".cache/record-dev"
        cache.mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory(dir=cache) as temporary:
            root = Path(temporary) / "dataset"
            repo_id = "local/web-capture-test"
            dataset = LeRobotDataset.create(
                repo_id, root=root, fps=web_capture.FPS,
                features=web_capture.features(), robot_type="roarm_m3_follower",
                rgb_encoder=RGBEncoderConfig(vcodec="h264", preset="ultrafast", crf=15),
                encoder_threads=2, image_writer_threads=2,
            )
            capture_dir = root / "capture"
            capture_dir.mkdir()
            (capture_dir / "info.json").write_text(json.dumps({
                "repo_id": repo_id, "fps": web_capture.FPS,
                "action_source": "next_observed_joint_state",
            }), encoding="utf-8")
            episodes = []
            try:
                for episode in range(2):
                    samples = []
                    for index in range(4):
                        state = np.array(
                            [episode * 20 + index * 2, index * 3, index * 4,
                             index * 5, index * 6, 30 + index], dtype=np.float32,
                        )
                        color = np.array([60 + episode * 80 + index * 15, 20 + index * 10,
                                          40 + episode * 20], dtype=np.uint8)
                        image = np.broadcast_to(color, (web_capture.SIZE[1], web_capture.SIZE[0], 3)).copy()
                        samples.append({"state": state, "image": image})
                    episodes.append(samples)
                    for previous, current in zip(samples, samples[1:]):
                        web_capture.add_pair(dataset, previous, current, f"演示 {episode}")
                    web_capture.save_pending(dataset)
                    self.assertFalse(dataset.has_pending_frames())
                    trace = {
                        "episode": episode,
                        "samples": [{"state": sample["state"].tolist(), "time_s": index / web_capture.FPS}
                                    for index, sample in enumerate(samples)],
                    }
                    (capture_dir / f"episode_{episode:04d}.json").write_text(
                        json.dumps(trace), encoding="utf-8",
                    )
            finally:
                dataset.finalize()

            loaded = LeRobotDataset(repo_id, root=root, video_backend="torchcodec")
            self.assertEqual((len(loaded), loaded.num_episodes, loaded.fps), (6, 2, 5))
            for episode, samples in enumerate(episodes):
                for index, previous in enumerate(samples[:-1]):
                    row = loaded[episode * 3 + index]
                    np.testing.assert_array_equal(row["observation.state"].numpy(), previous["state"])
                    np.testing.assert_array_equal(row["action"].numpy(), samples[index + 1]["state"])
                    self.assertEqual(row["episode_index"].item(), episode)
                    self.assertAlmostEqual(row["timestamp"].item(), index / 5, places=6)
                    self.assertEqual(row[web_capture.IMAGE].dtype, torch.float32)
                    self.assertEqual(tuple(row[web_capture.IMAGE].shape), (3, 360, 640))
                    self.assertTrue(torch.isfinite(row[web_capture.IMAGE]).all())
                    np.testing.assert_allclose(
                        row[web_capture.IMAGE].mean(dim=(1, 2)).numpy() * 255,
                        previous["image"][0, 0], atol=4,
                    )

            chunked = LeRobotDataset(
                repo_id, root=root, video_backend="torchcodec",
                delta_timestamps={"action": [index / 5 for index in range(4)]},
            )
            for episode in range(2):
                tail = chunked[episode * 3 + 2]
                self.assertEqual(tail["action_is_pad"].tolist(), [False, True, True, True])
                np.testing.assert_array_equal(
                    tail["action"].numpy(), np.tile(episodes[episode][-1]["state"], (4, 1)),
                )
            self.assertEqual(chunked[1]["action_is_pad"].tolist(), [False, False, True, True])
            np.testing.assert_array_equal(chunked[3]["action"][0].numpy(), episodes[1][1]["state"])

            with patch.object(web_capture, "open_arm") as open_arm, redirect_stdout(io.StringIO()):
                web_capture.check(argparse.Namespace(root=root, act_forward=False))
                open_arm.assert_not_called()
            report = json.loads((capture_dir / "dataset_check.json").read_text())
            self.assertEqual((report["frames"], report["episodes"]), (6, 2))
            self.assertTrue(report["next_state_pairing"])
            self.assertEqual(report["last_chunk_padding"], [False] + [True] * 9)
            self.assertTrue((capture_dir / "first_frame.jpg").is_file())


if __name__ == "__main__":
    unittest.main()
