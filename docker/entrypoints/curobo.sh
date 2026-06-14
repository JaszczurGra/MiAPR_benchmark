#!/usr/bin/env bash
# Entrypoint for the `curobo` service: sanity-check the GPU, ensure the harness is
# importable, then exec the cmd.
set -e
python3 -c "import torch; print('torch', torch.__version__, 'cuda', torch.cuda.is_available(), torch.cuda.get_device_name(0) if torch.cuda.is_available() else 'NO GPU')" || true
# Refresh ONLY the harness code from the (possibly host-mounted) source. --no-deps is
# critical: a full reinstall re-resolves dependencies and drags NumPy back to 2.x, which
# breaks torch 2.2.2 ("_ARRAY_API not found"). All deps are already pinned in the image.
# Non-editable (`-e` needs setuptools>=64 / PEP 660).
pip install -q --no-deps --force-reinstall /workspace/benchmark || true
exec "$@"
