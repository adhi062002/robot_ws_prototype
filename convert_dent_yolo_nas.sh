#!/usr/bin/env bash
#
# Export a trained SuperGradients YOLO-NAS-S checkpoint to ONNX,
# then build a TensorRT FP16 engine on the Jetson.
#
# Usage:
#   ./convert_dent_yolo_nas.sh
#   ./convert_dent_yolo_nas.sh /path/to/ckpt_best.pth
#
# Optional environment overrides:
#   PYTHON=python3
#   MODEL_DIR=/home/nvidia/robot_ws/models
#   ONNX_PATH=/path/to/output.onnx
#   ENGINE_PATH=/path/to/output.engine
#   TRTEXEC=/path/to/trtexec
#   TRT_WORKSPACE_MB=2048

set -Eeuo pipefail

PYTHON="${PYTHON:-python3}"
MODEL_DIR="${MODEL_DIR:-$HOME/robot_ws/models}"
CHECKPOINT="${1:-$MODEL_DIR/ckpt_best.pth}"
ONNX_PATH="${ONNX_PATH:-$MODEL_DIR/dent_yolo_nas_s.onnx}"
ENGINE_PATH="${ENGINE_PATH:-$MODEL_DIR/dent_yolo_nas_s_fp16.engine}"
TRT_WORKSPACE_MB="${TRT_WORKSPACE_MB:-2048}"

log() {
    printf '\n[%s] %s\n' "$(date '+%Y-%m-%d %H:%M:%S')" "$*"
}

die() {
    printf '\nERROR: %s\n' "$*" >&2
    exit 1
}

[[ -f "$CHECKPOINT" ]] || die "Checkpoint not found: $CHECKPOINT"
mkdir -p "$(dirname "$ONNX_PATH")" "$(dirname "$ENGINE_PATH")"
CHECKPOINT="$(realpath "$CHECKPOINT")"
ONNX_PATH="$(realpath -m "$ONNX_PATH")"
ENGINE_PATH="$(realpath -m "$ENGINE_PATH")"

log "Checkpoint: $CHECKPOINT"
log "ONNX output: $ONNX_PATH"
log "TensorRT output: $ENGINE_PATH"

# Do not install or upgrade PyTorch automatically: Jetson PyTorch builds
# are tied to the JetPack/CUDA stack.
log "Checking PyTorch and SuperGradients..."
"$PYTHON" - <<'PY'
import sys
try:
    import torch
    print("PyTorch:", torch.__version__)
except Exception as exc:
    print("Cannot import PyTorch:", exc)
    sys.exit(1)

try:
    import super_gradients
    print("SuperGradients:", getattr(super_gradients, "__version__", "version not exposed"))
except Exception as exc:
    print("Cannot import SuperGradients:", exc)
    print("Install a SuperGradients version compatible with this Jetson/PyTorch environment.")
    sys.exit(2)
PY
status=$?
[[ "$status" -eq 0 ]] || die "Python dependency check failed (exit code $status)."

TRTEXEC_BIN="${TRTEXEC:-}"
if [[ -z "$TRTEXEC_BIN" ]]; then
    TRTEXEC_BIN="$(command -v trtexec || true)"
fi
if [[ -z "$TRTEXEC_BIN" ]]; then
    for candidate in \
        /usr/src/tensorrt/bin/trtexec \
        /usr/local/tensorrt/bin/trtexec \
        /opt/tensorrt/bin/trtexec \
        /usr/bin/trtexec
    do
        if [[ -x "$candidate" ]]; then
            TRTEXEC_BIN="$candidate"
            break
        fi
    done
fi
if [[ -z "$TRTEXEC_BIN" ]]; then
    die "trtexec was not found. TensorRT's Python package does not necessarily include the trtexec CLI. Check /usr/src/tensorrt/bin/trtexec or install the TensorRT command-line tools for your JetPack/TensorRT version. You can also rerun with TRTEXEC=/full/path/to/trtexec."
fi
[[ -x "$TRTEXEC_BIN" ]] || die "TRTEXEC is not executable: $TRTEXEC_BIN"

log "Using TensorRT compiler: $TRTEXEC_BIN"
"$TRTEXEC_BIN" --version 2>&1 | head -n 5 || true

log "Exporting YOLO-NAS-S checkpoint to ONNX..."
CHECKPOINT_PATH="$CHECKPOINT" ONNX_OUTPUT="$ONNX_PATH" "$PYTHON" - <<'PY'
import os
import torch

from super_gradients.common.object_names import Models
from super_gradients.training import models
from super_gradients.conversion import DetectionOutputFormatMode

checkpoint_path = os.environ["CHECKPOINT_PATH"]
onnx_output = os.environ["ONNX_OUTPUT"]

print("Loading YOLO-NAS-S checkpoint:", checkpoint_path, flush=True)
model = models.get(
    Models.YOLO_NAS_S,
    num_classes=1,
    checkpoint_path=checkpoint_path,
)
model.eval()

checkpoint = torch.load(checkpoint_path, map_location="cpu")
processing_params = checkpoint.get("processing_params", {})
class_names = processing_params.get("class_names")
if class_names:
    print("Checkpoint classes:", class_names, flush=True)
    if class_names != ["dent"]:
        raise RuntimeError(
            f"Expected exactly ['dent'] in checkpoint metadata, got {class_names!r}"
        )

print("Exporting with batch size 1 and 640x640 input.", flush=True)
print("Using checkpoint processing parameters and flat detection output.", flush=True)

result = model.export(
    onnx_output,
    confidence_threshold=float(processing_params.get("conf", 0.5)),
    nms_threshold=float(processing_params.get("iou", 0.65)),
    output_predictions_format=DetectionOutputFormatMode.FLAT_FORMAT,
)

exported_path = getattr(result, "output", None) or onnx_output
if not os.path.isfile(exported_path):
    raise RuntimeError(f"Export API did not create ONNX file: {exported_path}")

print("ONNX export complete:", exported_path, flush=True)
print("ONNX size: %.2f MiB" % (os.path.getsize(exported_path) / (1024 * 1024)), flush=True)
PY

[[ -s "$ONNX_PATH" ]] || die "ONNX export did not create a non-empty file: $ONNX_PATH"

log "Building TensorRT FP16 engine (this can take several minutes)..."
"$TRTEXEC_BIN" \
    "--onnx=$ONNX_PATH" \
    "--saveEngine=$ENGINE_PATH" \
    --fp16 \
    "--memPoolSize=workspace:${TRT_WORKSPACE_MB}" \
    2>&1 | tee "${ENGINE_PATH}.build.log"

[[ -s "$ENGINE_PATH" ]] || die "TensorRT build finished without producing an engine: $ENGINE_PATH"

log "Conversion completed successfully."
printf '\nArtifacts:\n'
printf '  ONNX:      %s\n' "$ONNX_PATH"
printf '  TensorRT:  %s\n' "$ENGINE_PATH"
printf '  Build log: %s\n' "${ENGINE_PATH}.build.log"
printf '\nAfter replacing ckpt_best.pth with a newly trained checkpoint, run this same script again.\n'
