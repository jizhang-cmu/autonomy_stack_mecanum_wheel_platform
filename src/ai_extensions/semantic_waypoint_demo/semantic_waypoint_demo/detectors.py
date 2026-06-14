from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Protocol


@dataclass(frozen=True)
class Detection:
    """Normalized image-space target detection."""

    label: str
    confidence: float
    center_x: float
    center_y: float
    width: float
    height: float


class DetectorBackend(Protocol):
    """Interface for pluggable detector implementations."""

    def detect(self, image_msg: object, image_width: int, image_height: int) -> Optional[Detection]:
        """Return the best target detection, or None if no target is visible."""


class MockDetector:
    """Detector used to validate the AI-to-waypoint contract without a model."""

    def __init__(
        self,
        label: str,
        confidence: float,
        center_x: float,
        center_y: float,
        width: float,
        height: float,
    ) -> None:
        self._detection = Detection(
            label=label,
            confidence=confidence,
            center_x=_clamp01(center_x),
            center_y=_clamp01(center_y),
            width=_clamp01(width),
            height=_clamp01(height),
        )

    def detect(self, image_msg: object, image_width: int, image_height: int) -> Optional[Detection]:
        del image_msg, image_width, image_height
        return self._detection


class ExternalDetectorPlaceholder:
    """Placeholder documenting where a real open-vocabulary detector plugs in."""

    def __init__(self, backend_name: str, target_label: str) -> None:
        self._backend_name = backend_name
        self._target_label = target_label

    def detect(self, image_msg: object, image_width: int, image_height: int) -> Optional[Detection]:
        del image_msg, image_width, image_height
        raise RuntimeError(
            f"Detector backend '{self._backend_name}' is not implemented in this POC. "
            f"Load the model for target '{self._target_label}' here and return a Detection."
        )


def create_detector(
    backend: str,
    target_label: str,
    mock_confidence: float,
    mock_center_x: float,
    mock_center_y: float,
    mock_width: float,
    mock_height: float,
) -> DetectorBackend:
    if backend == "mock":
        return MockDetector(
            label=target_label,
            confidence=mock_confidence,
            center_x=mock_center_x,
            center_y=mock_center_y,
            width=mock_width,
            height=mock_height,
        )
    if backend in {"yolo_world", "grounding_dino", "owl_vit", "custom"}:
        return ExternalDetectorPlaceholder(backend, target_label)
    raise ValueError(
        f"Unsupported detector_backend '{backend}'. "
        "Use one of: mock, yolo_world, grounding_dino, owl_vit, custom."
    )


def _clamp01(value: float) -> float:
    return max(0.0, min(1.0, float(value)))
