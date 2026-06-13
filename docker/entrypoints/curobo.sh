#!/usr/bin/env bash
# Entrypoint for the `curobo` service: sanity-check the GPU, ensure the harness is
# importable, then exec the cmd.
set -e
python3 -c "import torch; print('torch', torch.__version__, 'cuda', torch.cuda.is_available(), torch.cuda.get_device_name(0) if torch.cuda.is_available() else 'NO GPU')" || true
pip install -q -e /workspace/benchmark >/dev/null 2>&1 || true
exec "$@"
