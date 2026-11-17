#!/usr/bin/env bash
# Build a TensorRT FP16 engine from an ONNX model on the Jetson.
# No PyTorch or SuperGradients dependency is required.
#
# Usage:
#   ./build_dent_yolo_engine.sh
#   ./build_dent_yolo_engine.sh /path/to/model.onnx
#   ./build_dent_yolo_engine.sh /path/to/model.onnx /path/to/model.engine
#
# Optional:
#   TRTEXEC=/usr/src/tensorrt/bin/trtexec
#   TRT_WORKSPACE_MB=2048
#   TRT_PRECISION=fp16                 # fp16 or fp32
#   TRT_MIN_SHAPES='images:1x3x640x640'
#   TRT_OPT_SHAPES='images:1x3x640x640'
#   TRT_MAX_SHAPES='images:1x3x640x640'
#
# Leave shape variables unset for a static-shape ONNX model.
# For a dynamic-shape model, set all three using its actual input tensor name.

set -Eeuo pipefail

MODEL_DIR="${MODEL_DIR:-$HOME/robot_ws/models}"
ONNX_PATH="${1:-$MODEL_DIR/dent_yolonas_m.onnx}"
ENGINE_PATH="${2:-$MODEL_DIR/dent_yolo_nas_s_fp16.engine}"
TRTEXEC="${TRTEXEC:-}"
TRT_WORKSPACE_MB="${TRT_WORKSPACE_MB:-2048}"
TRT_PRECISION="${TRT_PRECISION:-fp16}"

log() { printf '\n[%s] %s\n' "$(date '+%Y-%m-%d %H:%M:%S')" "$*"; }
die() { printf '\nERROR: %s\n' "$*" >&2; exit 1; }

[[ -f "$ONNX_PATH" ]] || die "ONNX model not found: $ONNX_PATH"
[[ -s "$ONNX_PATH" ]] || die "ONNX model is empty: $ONNX_PATH"

mkdir -p "$(dirname "$ENGINE_PATH")"
ONNX_PATH="$(realpath "$ONNX_PATH")"
ENGINE_PATH="$(realpath -m "$ENGINE_PATH")"
LOG_PATH="${ENGINE_PATH}.build.log"

if [[ -z "$TRTEXEC" ]]; then
    TRTEXEC="$(command -v trtexec || true)"
fi
if [[ -z "$TRTEXEC" ]]; then
    for candidate in /usr/src/tensorrt/bin/trtexec /usr/local/tensorrt/bin/trtexec /opt/tensorrt/bin/trtexec /usr/bin/trtexec; do
        if [[ -x "$candidate" ]]; then TRTEXEC="$candidate"; break; fi
    done
fi
[[ -n "$TRTEXEC" ]] || die "trtexec not found. Expected /usr/src/tensorrt/bin/trtexec on this Jetson."
[[ -x "$TRTEXEC" ]] || die "trtexec is not executable: $TRTEXEC"

case "$TRT_PRECISION" in
    fp16|FP16) PRECISION_FLAG=(--fp16) ;;
    fp32|FP32) PRECISION_FLAG=() ;;
    *) die "TRT_PRECISION must be fp16 or fp32." ;;
esac

if [[ -n "${TRT_MIN_SHAPES:-}" || -n "${TRT_OPT_SHAPES:-}" || -n "${TRT_MAX_SHAPES:-}" ]]; then
    [[ -n "${TRT_MIN_SHAPES:-}" && -n "${TRT_OPT_SHAPES:-}" && -n "${TRT_MAX_SHAPES:-}" ]] ||
        die "For dynamic ONNX inputs, set TRT_MIN_SHAPES, TRT_OPT_SHAPES and TRT_MAX_SHAPES."
    SHAPE_FLAGS=("--minShapes=${TRT_MIN_SHAPES}" "--optShapes=${TRT_OPT_SHAPES}" "--maxShapes=${TRT_MAX_SHAPES}")
else
    SHAPE_FLAGS=()
fi

log "Input ONNX: $ONNX_PATH"
log "Output engine: $ENGINE_PATH"
log "TensorRT builder: $TRTEXEC"
log "Precision: $TRT_PRECISION"
log "Workspace: ${TRT_WORKSPACE_MB} MiB"
log "TensorRT version:"
"$TRTEXEC" --version 2>&1 | head -n 8 || true

log "Building TensorRT engine. This may take several minutes..."
"$TRTEXEC" \
    "--onnx=$ONNX_PATH" \
    "--saveEngine=$ENGINE_PATH" \
    "${PRECISION_FLAG[@]}" \
    "--memPoolSize=workspace:${TRT_WORKSPACE_MB}" \
    "${SHAPE_FLAGS[@]}" \
    2>&1 | tee "$LOG_PATH"

[[ -s "$ENGINE_PATH" ]] || die "TensorRT did not produce a non-empty engine. Check $LOG_PATH"

log "TensorRT engine built successfully."
printf '\nGenerated files:\n'
printf '  ONNX model: %s\n' "$ONNX_PATH"
printf '  Engine:     %s\n' "$ENGINE_PATH"
printf '  Build log:  %s\n' "$LOG_PATH"
printf '\nBuild the engine on the target Jetson; TensorRT engines are hardware/software specific.\n'
