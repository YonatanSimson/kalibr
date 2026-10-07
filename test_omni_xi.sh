#!/bin/bash
# Test matrix for the configurable / fixed omni xi patch (OMNI_FIXED_XI.md).
#
#   ./test_omni_xi.sh <bag dir> <aprilgrid.yaml> [cam0|cam1] [case ...]
#
# <bag dir> holds insta360.bag (run_insta360_kalibr.sh output, e.g.
# ~/data/calibration/insta360/BXEA3ABHFQ7YSX/3008_24). Each case writes
# <bag dir>/omni_<case>/<cam>-camchain.yaml (+ results/report/log). Cases (default: all):
#   stock         stock image, stock behaviour                      (reference)
#   patched       patched image, no variables                       (must equal stock)
#   seed2.455     patched, KALIBR_OMNI_XI_INIT=2.45543              (free xi, X6 factory seed)
#   fixed2.455    patched, KALIBR_OMNI_XI_INIT=2.45543 KALIBR_OMNI_XI_FIXED=1
#   fixed2        patched, KALIBR_OMNI_XI_INIT=2.0     KALIBR_OMNI_XI_FIXED=1   (X4/X5 convention)
# Finished cases are skipped; delete the folder to redo one.

set -eo pipefail
BAGDIR="$(cd "$1" && pwd -P)"; TARGET="$2"; CAM="${3:-cam0}"; shift 3 || true
CASES=("$@"); [[ ${#CASES[@]} -gt 0 ]] || CASES=(stock patched seed2.455 fixed2.455 fixed2)
[[ -f "$BAGDIR/insta360.bag" && -f "$TARGET" ]] || { echo "usage: $0 <bag dir> <aprilgrid.yaml> [cam] [case ...]"; exit 1; }

run_case() {
    local name="$1" image="$2" env="$3" d="$BAGDIR/omni_$1"
    mkdir -p "$d"; [[ -e "$d/$CAM.bag" ]] || ln -s ../insta360.bag "$d/$CAM.bag"
    if [[ -f "$d/$CAM-camchain.yaml" ]]; then echo "[$name] done, skipping"; return; fi
    echo "[$name] image=$image env='$env'"
    docker run --rm -i --init -v "$HOME:$HOME" -w "$d" "$image" bash -c \
        "source /catkin_ws/devel/setup.bash && export MPLBACKEND=Agg PYTHONUNBUFFERED=1 $env && \
         xvfb-run -a rosrun kalibr kalibr_calibrate_cameras --bag '$d/$CAM.bag' --topics /$CAM/image_raw \
         --models omni-radtan --target '$TARGET' --bag-freq 6 --dont-show-report" </dev/null \
        > "$d/$CAM.log" 2>&1 || true
    if [[ -f "$d/$CAM-camchain.yaml" ]]; then
        echo "[$name] $(grep -A1 'intrinsics' "$d/$CAM-camchain.yaml" | tail -1)"
    else
        echo "[$name] FAILED, see $d/$CAM.log"
    fi
}

for c in "${CASES[@]}"; do
    case "$c" in
        stock)      run_case stock      kalibr_ubuntu2004        "" ;;
        patched)    run_case patched    kalibr_ubuntu2004_omnixi "" ;;
        seed2.455)  run_case seed2.455  kalibr_ubuntu2004_omnixi "KALIBR_OMNI_XI_INIT=2.45543" ;;
        fixed2.455) run_case fixed2.455 kalibr_ubuntu2004_omnixi "KALIBR_OMNI_XI_INIT=2.45543 KALIBR_OMNI_XI_FIXED=1" ;;
        fixed2)     run_case fixed2     kalibr_ubuntu2004_omnixi "KALIBR_OMNI_XI_INIT=2.0 KALIBR_OMNI_XI_FIXED=1" ;;
        *) echo "unknown case $c"; exit 1 ;;
    esac
done
