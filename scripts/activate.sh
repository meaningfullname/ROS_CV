#!/usr/bin/env bash
# Source this file from any directory; no global shell configuration changes.
_tabletop_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export MAMBA_ROOT_PREFIX="${_tabletop_root}/.mamba"
eval "$("${_tabletop_root}/.tools/bin/micromamba" shell hook --shell bash)"
micromamba activate "${_tabletop_root}/.ros-env"
if [[ -f "${_tabletop_root}/install/setup.bash" ]]; then
    source "${_tabletop_root}/install/setup.bash"
fi
unset _tabletop_root
