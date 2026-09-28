#!/usr/bin/env bash
# motionsim 전용 conda env를 만든다. 다른 프로젝트의 env는 건드리지 않는다.
# 설치 결과의 정확한 버전은 requirements.lock.txt에 있다.
set -euo pipefail

env_name="${MOTIONSIM_ENV:-motionsim}"
source "$(conda info --base)/etc/profile.d/conda.sh"

if conda env list | awk '{print $1}' | grep -qx "$env_name"; then
  echo "conda env '$env_name'이(가) 이미 있습니다." >&2
  exit 1
fi

conda create -y -n "$env_name" python=3.11
conda activate "$env_name"
pip install -U pip

pip install "isaacsim[all,extscache]==5.1.0" --extra-index-url https://pypi.nvidia.com

# isaacsim[all]은 PyPI의 torch 2.7.0 cu126 빌드를 설치한다. RTX PRO 6000(Blackwell, sm_120)은
# cu128 빌드가 필요하다. 버전 번호가 같아 일반 설치는 건너뛰므로 강제로 다시 설치한 뒤 CUDA 의존성을 맞춘다.
torch_packages=(torch==2.7.0 torchvision==0.22.0 torchaudio==2.7.0)
pip install --force-reinstall --no-deps "${torch_packages[@]}" --index-url https://download.pytorch.org/whl/cu128
pip install "${torch_packages[@]}" --index-url https://download.pytorch.org/whl/cu128 --extra-index-url https://pypi.org/simple

# isaacsim이 고정한 numpy 1.26과 torch를 유지한다. 최신 pin은 numpy와 충돌하므로 2.7.0을 쓴다.
# viser는 타임라인 API가 있는 kimodo-viser fork를 commit으로 고정한다. 첫 실행 때 웹 클라이언트를 빌드한다.
constraints="$(mktemp)"
trap 'rm -f "$constraints"' EXIT
pip freeze | grep -iE '^(numpy|torch|torchvision|torchaudio)==' > "$constraints"
pip install -c "$constraints" \
  pin==2.7.0 \
  onnxruntime==1.30.0 \
  pytest==8.4.2 \
  fast_simplification==0.2.0 \
  "viser @ git+https://github.com/nv-tlabs/kimodo-viser.git@7c82ad8f8640bad9dff8ded5c5eee908eeb08f11"

python - <<'PY'
import torch
arch = torch.cuda.get_arch_list()
assert torch.version.cuda == "12.8" and "sm_120" in arch, (torch.version.cuda, arch)
print("torch", torch.__version__, "cuda", torch.version.cuda, "ok")
PY
echo "알려진 경고: isaacsim-kernel은 websockets==12.0을 요구하지만 viser 때문에 15.x가 설치된다 (headless 물리 검증 완료)."
