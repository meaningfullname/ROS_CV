#!/usr/bin/env bash
set -euo pipefail
tabletop_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$tabletop_root"
mkdir -p .tools
if [[ ! -x .tools/bin/micromamba ]]; then
    curl -fL --retry 2 https://micro.mamba.pm/api/micromamba/linux-64/latest \
        -o .tools/micromamba.tar.bz2
    tar -xjf .tools/micromamba.tar.bz2 -C .tools bin/micromamba
fi
export PIP_CACHE_DIR="$tabletop_root/.tools/pip-cache"
if [[ -f .ros-env/conda-meta/history ]]; then
    .tools/bin/micromamba --no-rc install -y \
        --root-prefix "$tabletop_root/.mamba" --prefix "$tabletop_root/.ros-env" \
        --strict-channel-priority -f environment.yml
else
    .tools/bin/micromamba --no-rc create -y \
        --root-prefix "$tabletop_root/.mamba" --prefix "$tabletop_root/.ros-env" \
        --strict-channel-priority -f environment.yml
fi
.tools/bin/micromamba run -p "$tabletop_root/.ros-env" python -m pip check
printf 'Environment ready. Run: source scripts/activate.sh\n'
