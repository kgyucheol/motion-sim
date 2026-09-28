"""integrations manifest에 적힌 로봇 URDF와 Decoupled WBC 자산의 크기·SHA-256을 검증한다."""

import argparse
import hashlib
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def check_models() -> list[str]:
    failures = []
    models = json.loads((ROOT / "integrations/robot-models.json").read_text())["models"]
    for name, model in models.items():
        path = ROOT / model["urdf"]
        if not path.is_file():
            hint = " (저장소에 포함하지 않는 자산)" if model.get("distribution", "").startswith("not-in-git") else ""
            failures.append(f"{name}: 파일 없음 {model['urdf']}{hint}")
        elif sha256(path) != model["urdf_sha256"]:
            failures.append(f"{name}: SHA-256 불일치 {model['urdf']}")
    return failures


def check_decoupled_wbc() -> list[str]:
    failures = []
    manifest = json.loads((ROOT / "integrations/decoupled-wbc-assets.json").read_text())
    for asset in manifest["assets"]:
        path = ROOT / manifest["target_dir"] / asset["file"]
        if not path.is_file():
            failures.append(f"decoupled-wbc: 파일 없음 {path.relative_to(ROOT)}")
        elif path.stat().st_size != asset["bytes"] or sha256(path) != asset["sha256"]:
            failures.append(f"decoupled-wbc: 크기 또는 SHA-256 불일치 {path.relative_to(ROOT)}")
    return failures


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--only", choices=["models", "decoupled-wbc"])
    args = parser.parse_args()

    failures = []
    if args.only in (None, "models"):
        failures += check_models()
    if args.only in (None, "decoupled-wbc"):
        failures += check_decoupled_wbc()

    for failure in failures:
        print("FAIL", failure)
    print("OK" if not failures else f"{len(failures)}개 실패")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
