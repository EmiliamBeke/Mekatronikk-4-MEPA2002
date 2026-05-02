#!/usr/bin/env python3
import os
import shlex
import sys

try:
    import yaml
except Exception as exc:
    print(
        f"echo '[robot-cal] Missing python yaml support: {exc}. Install python3-yaml.' >&2",
        file=sys.stdout,
    )
    print("exit 1", file=sys.stdout)
    sys.exit(0)


REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
DEFAULT_CONFIG = os.path.join(REPO_ROOT, "config", "robot_calibration.yaml")


def to_shell(value):
    if isinstance(value, bool):
        return "1" if value else "0"
    return str(value)


def pick(env_name, default):
    value = os.environ.get(env_name)
    if value is not None and value != "":
        return value
    return default


def main():
    config_path = (
        sys.argv[1]
        if len(sys.argv) > 1
        else os.environ.get("ROBOT_CALIBRATION_FILE", DEFAULT_CONFIG)
    )

    values = {"ROBOT_CALIBRATION_FILE": config_path}

    if os.path.exists(config_path):
        with open(config_path, "r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle) or {}

        mega = data.get("mega_driver", {})
        values.update(
            {
                "SWAP_SIDES": pick("SWAP_SIDES", 1 if mega.get("swap_sides", False) else 0),
                "LEFT_CMD_SIGN": pick("LEFT_CMD_SIGN", mega.get("left_cmd_sign", 1)),
                "RIGHT_CMD_SIGN": pick("RIGHT_CMD_SIGN", mega.get("right_cmd_sign", 1)),
                "ANGULAR_CMD_SIGN": pick("ANGULAR_CMD_SIGN", mega.get("angular_cmd_sign", 1)),
                "MIN_NONZERO_PWM": pick("MIN_NONZERO_PWM", mega.get("min_nonzero_pwm", 55)),
                "LEFT_CMD_SCALE": pick("LEFT_CMD_SCALE", mega.get("left_cmd_scale", 1.0)),
                "RIGHT_CMD_SCALE": pick("RIGHT_CMD_SCALE", mega.get("right_cmd_scale", 1.0)),
                "LEFT_TICK_SIGN": pick("LEFT_TICK_SIGN", mega.get("left_tick_sign", 1)),
                "RIGHT_TICK_SIGN": pick("RIGHT_TICK_SIGN", mega.get("right_tick_sign", 1)),
                "LEFT_M_PER_TICK": pick("LEFT_M_PER_TICK", mega.get("left_m_per_tick", 0.0)),
                "RIGHT_M_PER_TICK": pick("RIGHT_M_PER_TICK", mega.get("right_m_per_tick", 0.0)),
                "TRACK_WIDTH_EFF_M": pick(
                    "TRACK_WIDTH_EFF_M", mega.get("track_width_eff_m", 0.35)
                ),
            }
        )

    for key, value in values.items():
        print(f"export {key}={shlex.quote(to_shell(value))}")


if __name__ == "__main__":
    main()
