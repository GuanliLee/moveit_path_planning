"""Pure rules for paired grasp/place data collection."""

from __future__ import annotations

import os


def derive_place_data_dir(grasp_data_dir: str, place_root: str) -> str:
    """Map a grasp configuration directory to one place configuration directory."""
    grasp = os.path.normpath(os.path.expanduser(str(grasp_data_dir).strip()))
    root = os.path.normpath(os.path.expanduser(str(place_root).strip()))
    config_name = os.path.basename(grasp)
    if not config_name or config_name in {os.path.sep, ".", ".."}:
        raise ValueError(f"invalid grasp configuration directory: {grasp_data_dir!r}")
    if not root or root == ".":
        raise ValueError(f"invalid place data root: {place_root!r}")
    return os.path.join(root, config_name)


def two_phase_event_action(capture_phase: str, runtime_phase: str, event: str) -> str:
    """Return the only valid action for a two-phase runtime state and event."""
    return {
        ("grasp", "idle", "start"): "start_grasp",
        ("grasp", "recording", "grasp_end"): "save_grasp",
        ("place", "idle", "place_start"): "start_place",
        ("place", "recording", "end"): "save_place",
    }.get((capture_phase, runtime_phase, event), "ignore")


def event_context_is_current(
    received_capture_phase: str,
    received_runtime_phase: str,
    current_capture_phase: str,
    current_runtime_phase: str,
) -> bool:
    """Reject a queued ROS event after the coordinator has changed state."""
    return (
        received_capture_phase == current_capture_phase
        and received_runtime_phase == current_runtime_phase
    )


def should_latch_place_start(
    event: str,
    pending_review_phase: str,
    received_capture_phase: str,
    received_runtime_phase: str,
) -> bool:
    """Latch a queued place-start around the current grasp-to-review transition."""
    return bool(
        event == "place_start"
        and pending_review_phase == "grasp"
        and received_capture_phase == "grasp"
        and received_runtime_phase in {"recording", "saving", "reviewing"}
    )


def should_release_latched_place_start(
    latched: bool,
    saved_phase: str,
    outcome: str,
    next_phase: str,
) -> bool:
    """Release one latched start only after an accepted grasp-to-place transition."""
    return bool(
        latched
        and saved_phase == "grasp"
        and outcome == "accepted"
        and next_phase == "place"
    )


def review_outcome(status: dict, saved_episode: int | str) -> str:
    """Classify the completed collection review without guessing partial states."""
    if status.get("capture_running") or status.get("quality_review_pending"):
        return "incomplete"

    saved = str(saved_episode).strip()
    marker = str(status.get("last_saved_episode", "")).strip()
    current = str(status.get("episode", "")).strip()
    if saved.isdigit() and current.isdigit() and marker == saved:
        if int(current) == int(saved) + 1:
            return "accepted"
    if saved.isdigit() and not marker and current == saved:
        return "discarded"
    return "incomplete"


def capture_target_after_review(
    capture_phase: str,
    outcome: str,
    saved_episode: int,
) -> tuple[str, int]:
    """Return the next physical phase and paired logical episode."""
    transitions = {
        ("grasp", "accepted"): ("place", saved_episode),
        ("grasp", "discarded"): ("grasp", saved_episode),
        ("place", "accepted"): ("grasp", saved_episode + 1),
        ("place", "discarded"): ("place", saved_episode),
    }
    try:
        return transitions[(capture_phase, outcome)]
    except KeyError as exc:
        raise ValueError(
            f"invalid two-phase review transition: phase={capture_phase!r}, outcome={outcome!r}"
        ) from exc


def review_target_if_complete(
    status: dict,
    pending_capture_phase: str,
    pending_episode: int,
) -> tuple[str, str, int] | None:
    """Resolve a pending review only after its durable status is complete."""
    outcome = review_outcome(status, pending_episode)
    if outcome == "incomplete":
        return None
    next_phase, next_episode = capture_target_after_review(
        pending_capture_phase,
        outcome,
        pending_episode,
    )
    return outcome, next_phase, next_episode


def review_transition_or_cached(
    cached_transition: tuple[str, str, int, str] | None,
    status: dict,
    pending_capture_phase: str,
    pending_episode: int,
    grasp_data_dir: str,
    place_data_dir: str,
) -> tuple[str, str, int, str] | None:
    """Freeze a completed review transition before applying its target config.

    Applying ``/config`` and confirming it are separate HTTP operations.  If the
    apply succeeds but confirmation fails, the collection status no longer
    describes the old review.  Keeping this resolved target makes the retry
    idempotent instead of trying to classify the already-switched status again.
    """
    if cached_transition is not None:
        return cached_transition
    transition = review_target_if_complete(
        status,
        pending_capture_phase,
        pending_episode,
    )
    if transition is None:
        return None
    outcome, next_phase, next_episode = transition
    target_dir = grasp_data_dir if next_phase == "grasp" else place_data_dir
    return outcome, next_phase, next_episode, target_dir
