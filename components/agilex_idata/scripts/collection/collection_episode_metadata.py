from __future__ import annotations

import argparse
import json
import re
import sys
import unicodedata
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from collection_targets import grasp_instruction


B_REASON_LABELS_EN = {
    "minor_collision": "minor collision",
    "unsmooth_motion": "unsmooth motion",
    "retry_success": "retry success",
    "minor_visual_issue": "minor visual issue",
    "others": "others",
}

F_REASON_LABELS_EN = {
    "grasp_failure": "grasp failure",
    "object_dropped": "object dropped",
    "wrong_placement": "wrong placement",
    "wrong_target": "wrong target",
    "object_knocked_over": "object knocked over",
    "task_abandoned": "task abandoned",
    "others": "others",
}

REASON_LABELS_EN = {**B_REASON_LABELS_EN, **F_REASON_LABELS_EN}
_UNSET = object()


def normalise_text(value: Any) -> str:
    text = unicodedata.normalize("NFKC", str(value or "")).casefold()
    return "".join(ch for ch in text if ch.isalnum() or "\u4e00" <= ch <= "\u9fff")


def split_text(value: Any) -> list[str]:
    if isinstance(value, list):
        raw_items = value
    else:
        raw_items = [value]
    out: list[str] = []
    for raw in raw_items:
        for item in re.split(r"[;,，；\s]+", str(raw or "")):
            text = item.strip()
            if text:
                out.append(text)
    return out


def unique(items: list[str]) -> list[str]:
    out: list[str] = []
    for item in items:
        if item and item not in out:
            out.append(item)
    return out


def load_json(path: Path) -> dict[str, Any]:
    if not path.is_file():
        return {}
    with path.open("r", encoding="utf-8") as handle:
        data = json.load(handle)
    return data if isinstance(data, dict) else {}


def write_json_atomic(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(f"{path.name}.tmp")
    with tmp.open("w", encoding="utf-8") as handle:
        json.dump(payload, handle, ensure_ascii=False, indent=2)
        handle.write("\n")
    tmp.replace(path)


def info_path_for(data_dir: Path, episode: int) -> Path:
    return data_dir / f"episode{episode}" / f"episode{episode}_0_info.json"


def aloha_info_path_for(aloha_dir: Path, episode: int) -> Path:
    return aloha_dir / f"episode{episode}" / f"episode{episode}_0_info.json"


def candidate_names(item: dict[str, Any]) -> list[str]:
    names: list[str] = []
    for key in ("product_cn", "product_en", "name", "label"):
        value = item.get(key)
        if value:
            names.append(str(value))
    aliases = item.get("aliases")
    if isinstance(aliases, list):
        names.extend(str(alias) for alias in aliases if str(alias).strip())
    return names


def match_score(target: str, candidate: str) -> int:
    target_norm = normalise_text(target)
    candidate_norm = normalise_text(candidate)
    if not target_norm or not candidate_norm:
        return 0
    if target_norm == candidate_norm:
        return 100
    if target_norm in candidate_norm or candidate_norm in target_norm:
        return 90
    target_parts = {normalise_text(part) for part in re.split(r"[-_/\s]+", target) if normalise_text(part)}
    candidate_parts = {normalise_text(part) for part in re.split(r"[-_/\s]+", candidate) if normalise_text(part)}
    if target_parts and target_parts.issubset(candidate_parts):
        return 80
    return 0


def matched_scene_items(inventory: dict[str, Any], targets: list[str]) -> list[dict[str, Any]]:
    items = inventory.get("items")
    if not isinstance(items, list):
        return []
    selected: list[dict[str, Any]] = []
    selected_keys: set[str] = set()
    for target in [item for item in targets if str(item or "").strip()]:
        ranked: list[tuple[int, int, dict[str, Any]]] = []
        for idx, raw_item in enumerate(items):
            if not isinstance(raw_item, dict):
                continue
            score = max((match_score(target, name) for name in candidate_names(raw_item)), default=0)
            if score > 0:
                ranked.append((score, -idx, raw_item))
        if not ranked:
            continue
        best_score = max(score for score, _idx, _item in ranked)
        for score, _idx, item in ranked:
            if score != best_score:
                continue
            key = json.dumps(
                {
                    "shelf": item.get("shelf"),
                    "level": item.get("level"),
                    "product_cn": item.get("product_cn"),
                    "product_en": item.get("product_en"),
                },
                ensure_ascii=False,
                sort_keys=True,
            )
            if key not in selected_keys:
                selected.append(item)
                selected_keys.add(key)
    return selected


def reason_codes_for_grade(grade: str, reason_codes: list[str]) -> list[str]:
    grade = grade.upper()
    if grade == "A":
        return []
    allowed = B_REASON_LABELS_EN if grade == "B" else F_REASON_LABELS_EN if grade == "F" else {}
    if not allowed:
        raise ValueError("grade must be A, B, or F")
    codes = unique([str(code).strip() for code in reason_codes if str(code).strip()])
    invalid = [code for code in codes if code not in allowed]
    if invalid:
        raise ValueError(f"invalid reason code for grade {grade}: {', '.join(invalid)}")
    return codes


def quality_payload(grade: str, reason_codes: list[str], reason_note: str, source: str) -> dict[str, Any]:
    grade = grade.upper()
    if grade not in {"A", "B", "F"}:
        raise ValueError("grade must be A, B, or F")
    codes = reason_codes_for_grade(grade, reason_codes)
    return {
        "schema_version": 1,
        "grade": grade,
        "status": "rejected" if grade == "F" else "accepted",
        "reason_code": codes[0] if codes else "",
        "reason_codes": codes,
        "reason_note": str(reason_note or "").strip(),
        "source": source,
        "marked_at": datetime.now(timezone.utc).isoformat(),
    }


def merge_scene_payload(payload: dict[str, Any], inventory: dict[str, Any], targets: list[str]) -> None:
    if not inventory:
        return
    if inventory.get("scene") is not None:
        payload["scene"] = inventory.get("scene")
    level_definition = inventory.get("level_definition")
    if isinstance(level_definition, dict):
        payload["level_definition"] = level_definition
    payload["items"] = matched_scene_items(inventory, targets)


def default_instruction(
    targets: list[str],
    left_target: object = _UNSET,
    right_target: object = _UNSET,
) -> str:
    if left_target is not _UNSET or right_target is not _UNSET:
        left = "" if left_target is _UNSET else left_target
        right = "" if right_target is _UNSET else right_target
        return grasp_instruction(left, right)
    clean = [str(target).strip() for target in targets if str(target).strip()]
    if len(clean) >= 2:
        return f"Grasp {clean[0]} with the left hand, then grasp {clean[1]} with the right hand."
    if len(clean) == 1:
        return f"Grasp {clean[0]} with the left hand."
    return "Collect the target object."


def ensure_info_payload(
    path: Path,
    targets: list[str],
    left_target: object = _UNSET,
    right_target: object = _UNSET,
) -> dict[str, Any]:
    payload = load_json(path)
    if not payload:
        instruction = default_instruction(targets, left_target, right_target)
        payload = {
            "mark": {
                "full-instructions-zh": [instruction],
                "full-instructions-en": [instruction],
                "segment-instructions": [],
            }
        }
    return payload


def write_payload_to_existing_sidecars(data_dir: Path, aloha_dir: Path, episode: int, payload: dict[str, Any]) -> list[str]:
    written: list[str] = []
    raw_info_path = info_path_for(data_dir, episode)
    write_json_atomic(raw_info_path, payload)
    written.append(str(raw_info_path))
    aloha_info_path = aloha_info_path_for(aloha_dir, episode)
    if aloha_info_path.exists():
        aloha_payload = load_json(aloha_info_path)
        aloha_payload.update(payload)
        write_json_atomic(aloha_info_path, aloha_payload)
        written.append(str(aloha_info_path))
    return written


def review_episode(
    *,
    data_dir: Path,
    aloha_dir: Path,
    episode: int,
    grade: str,
    reason_codes: list[str],
    reason_note: str,
    inventory_json: Path | None,
    targets: list[str],
    left_target: object = _UNSET,
    right_target: object = _UNSET,
    source: str = "collection_web",
) -> dict[str, Any]:
    data_dir = Path(data_dir)
    aloha_dir = Path(aloha_dir)
    episode = int(episode)
    raw_info_path = info_path_for(data_dir, episode)
    payload = ensure_info_payload(raw_info_path, targets, left_target, right_target)
    payload["collection_quality"] = quality_payload(grade, reason_codes, reason_note, source)
    inventory = load_json(Path(inventory_json)) if inventory_json else {}
    merge_scene_payload(payload, inventory, targets)
    written = write_payload_to_existing_sidecars(data_dir, aloha_dir, episode, payload)
    return {
        "ok": True,
        "episode": episode,
        "grade": payload["collection_quality"]["grade"],
        "status": payload["collection_quality"]["status"],
        "reason_codes": payload["collection_quality"]["reason_codes"],
        "written": written,
    }


def prepare_episode(
    *,
    data_dir: Path,
    episode: int,
    targets: list[str],
    left_target: object = _UNSET,
    right_target: object = _UNSET,
) -> dict[str, Any]:
    info_path = info_path_for(Path(data_dir), int(episode))
    payload = ensure_info_payload(info_path, targets, left_target, right_target)
    write_json_atomic(info_path, payload)
    return {"ok": True, "episode": int(episode), "written": [str(info_path)]}


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Update collection episode metadata sidecars.")
    subparsers = parser.add_subparsers(dest="command", required=True)

    prepare = subparsers.add_parser("prepare")
    prepare.add_argument("--data-dir", required=True)
    prepare.add_argument("--episode", type=int, required=True)
    prepare.add_argument("--target", action="append", default=[])
    prepare.add_argument("--left-target", default=argparse.SUPPRESS)
    prepare.add_argument("--right-target", default=argparse.SUPPRESS)

    review = subparsers.add_parser("review")
    review.add_argument("--data-dir", required=True)
    review.add_argument("--aloha-dir", required=True)
    review.add_argument("--episode", type=int, required=True)
    review.add_argument("--grade", required=True)
    review.add_argument("--reason-code", action="append", default=[])
    review.add_argument("--reason-note", default="")
    review.add_argument("--inventory-json", default="")
    review.add_argument("--target", action="append", default=[])
    review.add_argument("--left-target", default=argparse.SUPPRESS)
    review.add_argument("--right-target", default=argparse.SUPPRESS)
    review.add_argument("--source", default="collection_web")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        if args.command == "prepare":
            result = prepare_episode(
                data_dir=Path(args.data_dir),
                episode=args.episode,
                targets=args.target,
                left_target=getattr(args, "left_target", _UNSET),
                right_target=getattr(args, "right_target", _UNSET),
            )
        else:
            inventory_path = Path(args.inventory_json) if args.inventory_json else None
            result = review_episode(
                data_dir=Path(args.data_dir),
                aloha_dir=Path(args.aloha_dir),
                episode=args.episode,
                grade=args.grade,
                reason_codes=args.reason_code,
                reason_note=args.reason_note,
                inventory_json=inventory_path,
                targets=args.target,
                left_target=getattr(args, "left_target", _UNSET),
                right_target=getattr(args, "right_target", _UNSET),
                source=args.source,
            )
    except Exception as exc:  # noqa: BLE001 - shell caller needs a concise error.
        print(json.dumps({"ok": False, "error": str(exc)}, ensure_ascii=False), file=sys.stderr)
        return 1
    print(json.dumps(result, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
