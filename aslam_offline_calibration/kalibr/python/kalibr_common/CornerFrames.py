"""Frame selection for basalt_dump_corners JSON files (see TargetExtractor.importCornersFromJson)."""

from typing import Any, Dict, List

CornerFrame = Dict[str, Any]


def selectCameraFrames(frames: List[CornerFrame], cam_id: int, every: int) -> List[CornerFrame]:
    """Frames of camera `cam_id` sorted by frame_id, keeping every `every`-th timestamp of the file.

    The stride runs over the timestamps of all cameras in the file, not over this camera's own
    frames, so all cameras keep the same timestamps even when one of them has no entry for a frame.
    """
    timestamps = sorted({fr["frame_id"] for fr in frames})
    kept = set(timestamps[::every])
    return sorted(
        (fr for fr in frames if fr["cam_id"] == cam_id and fr["frame_id"] in kept),
        key=lambda fr: fr["frame_id"],
    )
