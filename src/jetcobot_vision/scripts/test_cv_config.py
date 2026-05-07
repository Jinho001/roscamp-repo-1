#!/usr/bin/env python3
"""
CV Detect Server /config 엔드포인트 테스트

Usage:
    python3 test_cv_config.py tray
    python3 test_cv_config.py receiving_zone
    python3 test_cv_config.py --help
"""

import argparse
import json
import sys
import os
import requests
import yaml
from pathlib import Path

def load_profiles() -> dict:
    """pick_place_profiles.yaml 로드."""
    script_dir = Path(__file__).parent
    config_path = script_dir.parent / "config" / "pick_place_profiles.yaml"
    if not config_path.exists():
        raise FileNotFoundError(f"Config not found: {config_path}")
    with open(config_path) as f:
        return yaml.safe_load(f)

def send_config(server_url: str, profile: dict) -> bool:
    """cv_detect_server /config에 파라미터 전송."""
    cfg = {}
    for key in ("hsv_lower", "hsv_upper", "min_w", "max_w", "min_h", "max_h",
                "min_area", "max_area", "morph_k"):
        if key in profile and profile[key] is not None:
            cfg[key] = profile[key]

    if not cfg:
        print(f"[WARN] 파라미터 없음 (observe_pose 없는 location?)")
        return False

    try:
        url = server_url.rstrip("/") + "/config"
        print(f"[POST] {url}")
        print(f"[BODY] {json.dumps(cfg, indent=2)}")
        resp = requests.post(url, json=cfg, timeout=2.0)
        if resp.status_code == 200:
            print(f"[OK] {resp.json()}")
            return True
        else:
            print(f"[ERR] HTTP {resp.status_code}: {resp.text}")
            return False
    except Exception as exc:
        print(f"[ERR] {exc}")
        return False

def main():
    parser = argparse.ArgumentParser(
        description="Test CV Detect Server /config endpoint"
    )
    parser.add_argument(
        "location",
        help="Location name (tray, receiving_zone, pinky_tray_place, etc.)"
    )
    parser.add_argument(
        "--server",
        default="http://192.168.1.4:8000",
        help="CV Detect Server URL (default: http://192.168.1.4:8000)"
    )
    parser.add_argument(
        "--list",
        action="store_true",
        help="List all available locations"
    )

    args = parser.parse_args()

    try:
        profiles = load_profiles()
    except FileNotFoundError as e:
        print(f"[ERR] {e}")
        sys.exit(1)

    if args.list:
        print("Available locations:")
        for loc in sorted(profiles.keys()):
            print(f"  - {loc}")
        return

    if args.location not in profiles:
        print(f"[ERR] Unknown location: {args.location}")
        print(f"Available: {', '.join(sorted(profiles.keys()))}")
        sys.exit(1)

    profile = profiles[args.location]
    print(f"\n[LOCATION] {args.location}")
    print(f"[SERVER] {args.server}")
    print(f"[PROFILE]")
    for key in ("observe_pose", "z_surface_mm", "hsv_lower", "hsv_upper",
                "min_w", "max_w", "min_h", "max_h"):
        if key in profile:
            print(f"  {key}: {profile[key]}")
    print()

    success = send_config(args.server, profile)
    sys.exit(0 if success else 1)

if __name__ == "__main__":
    main()
