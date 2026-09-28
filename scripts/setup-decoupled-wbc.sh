#!/usr/bin/env bash
# GR00T-WholeBodyControl checkout에서 Decoupled WBC 자산을 external/decoupled-wbc로 복사한다.
# ONNX는 git LFS로 관리되므로 작업 트리의 실제 파일을 복사하고, 버전은 revision이 아니라 SHA-256으로 검증한다.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."

source_root="${1:?사용법: scripts/setup-decoupled-wbc.sh /path/to/GR00T-WholeBodyControl}"
asset_root="$source_root/decoupled_wbc/sim2mujoco/resources/robots/g1"
target_root="external/decoupled-wbc"

if [[ ! -f "$asset_root/g1_gear_wbc.xml" ]]; then
  echo "GR00T-WholeBodyControl 자산을 찾을 수 없습니다: $asset_root" >&2
  exit 1
fi

mkdir -p "$target_root"
cp "$asset_root/g1_gear_wbc.xml" "$asset_root/g1_gear_wbc.yaml" "$target_root/"
cp "$asset_root/policy/GR00T-WholeBodyControl-Balance.onnx" "$asset_root/policy/GR00T-WholeBodyControl-Walk.onnx" "$target_root/"
cp "$source_root/LICENSE" "$target_root/LICENSE"

python scripts/verify_assets.py --only decoupled-wbc
echo "Decoupled WBC 자산 준비 완료: $target_root"
