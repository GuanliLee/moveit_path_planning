#!/usr/bin/env python3
"""OpenPI policy server wrapper with runtime PyTorch compile/device overrides."""

from __future__ import annotations

import dataclasses
import enum
import logging
import socket

import tyro

from openpi.policies import policy as _policy
from openpi.policies import policy_config as _policy_config
from openpi.serving import websocket_policy_server
from openpi.training import config as _config


class EnvMode(enum.Enum):
    ALOHA = "aloha"
    ALOHA_SIM = "aloha_sim"


@dataclasses.dataclass
class Checkpoint:
    config: str
    dir: str


@dataclasses.dataclass
class Default:
    pass


@dataclasses.dataclass
class Args:
    env: EnvMode = EnvMode.ALOHA
    default_prompt: str | None = None
    port: int = 8000
    record: bool = False
    pytorch_device: str | None = "cuda"
    pytorch_compile_mode: str | None = None
    policy: Checkpoint | Default = dataclasses.field(default_factory=Default)


DEFAULT_CHECKPOINT: dict[EnvMode, Checkpoint] = {
    EnvMode.ALOHA: Checkpoint(
        config="pi05_aloha",
        dir="gs://openpi-assets/checkpoints/pi05_base",
    ),
    EnvMode.ALOHA_SIM: Checkpoint(
        config="pi0_aloha_sim",
        dir="gs://openpi-assets/checkpoints/pi0_aloha_sim",
    ),
}


def normalize_compile_mode(value: str | None) -> str | None:
    if value is None:
        return None
    lowered = value.lower()
    if lowered in {"", "none", "no", "false", "off", "0"}:
        return None
    valid = {"default", "reduce-overhead", "max-autotune", "max-autotune-no-cudagraphs"}
    if value not in valid:
        raise ValueError(f"Invalid pytorch_compile_mode {value!r}; expected one of {sorted(valid)} or none.")
    return value


def get_train_config(checkpoint: Checkpoint, compile_mode: str | None) -> _config.TrainConfig:
    train_config = _config.get_config(checkpoint.config)
    compile_mode = normalize_compile_mode(compile_mode)
    train_config = dataclasses.replace(
        train_config,
        model=dataclasses.replace(train_config.model, pytorch_compile_mode=compile_mode),
    )
    logging.info("train_config: %s", train_config)
    return train_config


def create_policy(args: Args) -> _policy.Policy:
    match args.policy:
        case Checkpoint():
            checkpoint = args.policy
        case Default():
            checkpoint = DEFAULT_CHECKPOINT[args.env]

    return _policy_config.create_trained_policy(
        get_train_config(checkpoint, args.pytorch_compile_mode),
        checkpoint.dir,
        default_prompt=args.default_prompt,
        pytorch_device=args.pytorch_device,
    )


def main(args: Args) -> None:
    policy = create_policy(args)
    policy_metadata = policy.metadata

    if args.record:
        policy = _policy.PolicyRecorder(policy, "policy_records")

    hostname = socket.gethostname()
    local_ip = socket.gethostbyname(hostname)
    logging.info("Creating server (host: %s, ip: %s, port: %s)", hostname, local_ip, args.port)
    server = websocket_policy_server.WebsocketPolicyServer(
        policy=policy,
        host="0.0.0.0",
        port=args.port,
        metadata=policy_metadata,
    )
    server.serve_forever()


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, force=True)
    main(tyro.cli(Args))
