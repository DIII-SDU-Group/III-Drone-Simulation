from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import subprocess
from typing import Sequence


@dataclass(frozen=True)
class SimulationArtifact:
    path: Path
    description: str


class GazeboSimulationControl:
    """Scriptable Gazebo control helpers for agent inspection workflows."""

    def __init__(self, artifact_dir: str | Path):
        self.artifact_dir = Path(artifact_dir)
        self.artifact_dir.mkdir(parents=True, exist_ok=True)

    def list_topics(self) -> list[str]:
        result = self._run(["gz", "topic", "--list"])
        return [line.strip() for line in result.stdout.splitlines() if line.strip()]

    def list_services(self) -> list[str]:
        result = self._run(["gz", "service", "--list"])
        return [line.strip() for line in result.stdout.splitlines() if line.strip()]

    def capture_topic_snapshot(self, topic: str, filename: str, timeout_sec: float = 2.0) -> SimulationArtifact:
        path = self.artifact_dir / filename
        result = self._run(
            ["gz", "topic", "-e", "-t", topic, "-n", "1"],
            timeout_sec=timeout_sec,
            check=False,
        )
        path.write_text(result.stdout, encoding="utf-8")
        return SimulationArtifact(path=path, description=f"Gazebo topic snapshot for {topic}")

    def capture_image_topic(self, topic: str, filename: str, timeout_sec: float = 2.0) -> SimulationArtifact:
        path = self.artifact_dir / filename
        result = self._run(
            ["gz", "topic", "-e", "-t", topic, "-n", "1"],
            timeout_sec=timeout_sec,
            check=False,
        )
        path.write_bytes(result.stdout.encode("utf-8"))
        return SimulationArtifact(path=path, description=f"Gazebo image topic capture for {topic}")

    def set_camera_pose(
        self,
        *,
        x: float,
        y: float,
        z: float,
        qx: float = 0.0,
        qy: float = 0.0,
        qz: float = 0.0,
        qw: float = 1.0,
        service: str = "/gui/move_to/pose",
        timeout_sec: float = 2.0,
    ) -> subprocess.CompletedProcess[str]:
        request = (
            f"pose {{ position {{ x: {x} y: {y} z: {z} }} "
            f"orientation {{ x: {qx} y: {qy} z: {qz} w: {qw} }} }}"
        )
        return self._run(
            [
                "gz",
                "service",
                "-s",
                service,
                "--reqtype",
                "gz.msgs.GUICamera",
                "--reptype",
                "gz.msgs.Boolean",
                "--timeout",
                str(int(timeout_sec * 1000)),
                "--req",
                request,
            ],
            timeout_sec=timeout_sec + 1.0,
            check=False,
        )

    def _run(self, command: Sequence[str], timeout_sec: float | None = None, check: bool = True) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            list(command),
            check=check,
            text=True,
            capture_output=True,
            timeout=timeout_sec,
        )
