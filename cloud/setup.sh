#!/usr/bin/env bash
# Set up a GPU machine to run CARLA 0.9.15 and Bench2Drive with this repository's
# controllers. Written for an NVIDIA Brev instance in VM mode (Ubuntu, NVIDIA
# driver, sudo); any Ubuntu 22.04 or 24.04 machine with an NVIDIA GPU should do.
#
#   cloud/setup.sh               everything, then the checks (cloud/check.sh)
#   cloud/setup.sh --no-maps     without the additional maps (Town06/07/11/12/13/15, a 7.4 GB download)
#   cloud/setup.sh --no-carla    without CARLA: only the build and the Python environments
#   cloud/setup.sh --no-system   without the apt packages (needs no sudo)
#   cloud/setup.sh --no-check    without the checks at the end
#
# A step that is already done is skipped, so run the script again after a
# failure or an interrupted download.
#
# What it installs, and where:
#   apt packages                     compiler, CMake, Eigen, the libraries CARLA needs
#   uv (~/.local/bin)                creates the Python environments, downloading Python itself
#   .venv, build/                    this repository's own environment (Python 3.12) and build
#   .venv-b2d, build-b2d/            Python 3.8 with the CARLA client and Bench2Drive's packages,
#                                    and the controllers (overtake_core) built for it
#   external/carla                   CARLA 0.9.15 and its additional maps
#   external/Bench2Drive             the benchmark, cloned at the commit below
# Set OV_EXTERNAL to keep external/ somewhere else. CARLA_URL and CARLA_MAPS_URL
# override the download addresses.
set -euo pipefail
cd "$(dirname "$0")/.."

B2D_URL=https://github.com/Thinklab-SJTU/Bench2Drive.git
B2D_COMMIT=7ec25d1c9f7522d923ce5f3420986cef1cb2d956  # release 0.0.4
CARLA_URL=${CARLA_URL:-https://carla-releases.s3.us-east-005.backblazeb2.com/Linux/CARLA_0.9.15.tar.gz}
CARLA_MAPS_URL=${CARLA_MAPS_URL:-https://carla-releases.s3.us-east-005.backblazeb2.com/Linux/AdditionalMaps_0.9.15.tar.gz}
DISK_NEEDED_GB=60  # an estimate: both archives, unpacked, with the larger archive still on disk

system=1
carla=1
maps=1
check=1
while [[ $# -gt 0 ]]; do
    case "$1" in
        --no-system) system=0 ;;
        --no-carla) carla=0 ;;
        --no-maps) maps=0 ;;
        --no-check) check=0 ;;
        -h|--help) sed -n '2,24p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *) echo "unknown option: $1 (see --help)" >&2; exit 2 ;;
    esac
    shift
done

# shellcheck source=cloud/env.sh
source cloud/env.sh

step() { echo; echo "== $*"; }
die() { echo "error: $*" >&2; exit 1; }

# Download to `file` from `url`, continuing a partial download.
download() {
    local file=$1 url=$2
    [[ -f $file ]] && return 0
    mkdir -p "$(dirname "$file")"
    curl -fL --retry 5 --retry-delay 10 -C - -o "$file.part" "$url" || die "download failed: $url (run the script again to continue it)"
    mv "$file.part" "$file"
}

# ---- the machine

step "machine"
# shellcheck disable=SC1091
( . /etc/os-release 2>/dev/null && echo "   $PRETTY_NAME" ) || true
if command -v nvidia-smi >/dev/null; then
    nvidia-smi --query-gpu=name,memory.total,driver_version --format=csv,noheader | sed 's/^/   GPU: /'
else
    echo "   warning: nvidia-smi not found. CARLA needs an NVIDIA GPU with its driver installed."
fi
if [[ $EUID -eq 0 ]]; then
    echo "   warning: running as root. CARLA refuses to start as root; use an ordinary user"
    echo "   (on Brev: an instance in VM mode, where the user is ubuntu)."
fi

# ---- system packages

if [[ $system == 1 ]]; then
    step "system packages (apt)"
    if [[ $EUID -eq 0 ]]; then
        sudo=()
    elif command -v sudo >/dev/null; then
        sudo=(sudo)
    else
        die "no sudo; install the packages listed in cloud/setup.sh yourself and pass --no-system"
    fi
    "${sudo[@]}" apt-get update -q
    # build tools; then what CARLA's server needs (as in its Docker image) and Vulkan
    # diagnostics; then what OpenCV in Bench2Drive's Python packages loads
    "${sudo[@]}" env DEBIAN_FRONTEND=noninteractive apt-get install -y -q --no-install-recommends \
        build-essential cmake git curl ca-certificates libeigen3-dev \
        libomp5 libvulkan1 vulkan-tools xdg-user-dirs libsdl2-2.0-0 \
        libgl1 libglib2.0-0
fi

for tool in git curl cmake c++; do
    command -v $tool >/dev/null || die "$tool not found (run without --no-system, or install it)"
done

# ---- uv

export PATH="$HOME/.local/bin:$PATH"
if ! command -v uv >/dev/null; then
    step "uv"
    curl -LsSf https://astral.sh/uv/install.sh | sh
    command -v uv >/dev/null || die "uv was installed but is not on PATH"
fi

# ---- this repository's own environment and build

step "repository environment and build (.venv, build/)"
[[ -x .venv/bin/python ]] || uv venv --python 3.12 .venv
./run.sh --setup-only

# ---- Bench2Drive

step "Bench2Drive ($B2D_ROOT)"
if [[ $(git -C "$B2D_ROOT" rev-parse HEAD 2>/dev/null) == "$B2D_COMMIT" ]]; then
    echo "   already at ${B2D_COMMIT:0:7}"
else
    # fetched at one commit, so results do not change when the benchmark is updated
    mkdir -p "$B2D_ROOT"
    git -C "$B2D_ROOT" init -q
    git -C "$B2D_ROOT" remote remove origin 2>/dev/null || true
    git -C "$B2D_ROOT" remote add origin "$B2D_URL"
    git -C "$B2D_ROOT" fetch -q --depth 1 origin "$B2D_COMMIT"
    git -C "$B2D_ROOT" checkout -q FETCH_HEAD
    echo "   checked out ${B2D_COMMIT:0:7}"
fi

# ---- Python 3.8 environment for the CARLA client and the evaluator

step "Bench2Drive environment (.venv-b2d, Python 3.8)"
[[ -x $B2D_PYTHON ]] || uv venv --python 3.8 .venv-b2d
if $B2D_PYTHON -c "import carla, casadi, pybind11, py_trees, shapely, networkx, scipy, pkg_resources" 2>/dev/null; then
    echo "   packages already installed"
else
    # Bench2Drive's own requirement files, so its pinned versions decide; then the
    # CARLA client, and CasADi and pybind11 for the controllers. The evaluator
    # imports pkg_resources, which setuptools provides.
    uv pip install --python "$B2D_PYTHON" \
        -r "$B2D_ROOT/leaderboard/requirements.txt" \
        -r "$B2D_ROOT/scenario_runner/requirements.txt" \
        "carla==0.9.15" "casadi>=3.6,<3.8" "pybind11>=2.11" scipy "setuptools<81"
fi

step "controllers for Python 3.8 (build-b2d/)"
[[ -f build-b2d/Makefile ]] || cmake -S . -B build-b2d -DPython3_EXECUTABLE="$B2D_PYTHON" >/dev/null
cmake --build build-b2d -j"$(nproc)" --target overtake_core | grep -vE '^\[|Built target|Consolidate|Entering|Leaving' || true
$B2D_PYTHON -c "import overtake_core" || die "overtake_core does not import in .venv-b2d"

# ---- CARLA

if [[ $carla == 1 ]]; then
    step "CARLA 0.9.15 ($CARLA_ROOT)"
    if [[ -f $CARLA_ROOT/.carla-0.9.15 ]]; then
        echo "   already installed"
    else
        mkdir -p "$CARLA_ROOT"
        free_gb=$(df -Pk "$CARLA_ROOT" | awk 'NR==2 {print int($4 / 1048576)}')
        if (( free_gb < DISK_NEEDED_GB )); then
            echo "   warning: $free_gb GB free; CARLA with its additional maps needs about $DISK_NEEDED_GB GB"
        fi
        archive=$OV_EXTERNAL/downloads/CARLA_0.9.15.tar.gz
        echo "   downloading (8.4 GB)"
        download "$archive" "$CARLA_URL"
        echo "   unpacking"
        tar -xzf "$archive" -C "$CARLA_ROOT"
        [[ -f $CARLA_ROOT/CarlaUE4.sh ]] || die "CarlaUE4.sh is not in the archive from $CARLA_URL"
        touch "$CARLA_ROOT/.carla-0.9.15"
        rm -f "$archive"
    fi

    if [[ $maps == 1 ]]; then
        step "CARLA additional maps"
        if [[ -f $CARLA_ROOT/.additional-maps-0.9.15 ]]; then
            echo "   already installed"
        else
            archive=$OV_EXTERNAL/downloads/AdditionalMaps_0.9.15.tar.gz
            echo "   downloading (7.4 GB)"
            download "$archive" "$CARLA_MAPS_URL"
            echo "   unpacking"
            # Unpacked over the CARLA folder, as CARLA's ImportAssets.sh does. Its
            # --keep-newer-files is left out: GNU tar then reports every folder that
            # already exists as an error, and the maps archive is the newer one anyway.
            tar -xzf "$archive" -C "$CARLA_ROOT"
            touch "$CARLA_ROOT/.additional-maps-0.9.15"
            rm -f "$archive"
        fi
    fi
fi

# ---- checks

if [[ $check == 1 ]]; then
    step "checks"
    check_args=()
    [[ $carla == 1 ]] || check_args+=(--no-carla)
    if ! cloud/check.sh "${check_args[@]}"; then
        echo
        echo "== installed, but a check failed (above). Fix it and run cloud/check.sh again."
        exit 1
    fi
fi

cat <<EOF

== set up
   cloud/check.sh --route             drive one Bench2Drive route with its sample agent
   cloud/b2d.sh --routes 24906        the same, with the results in results/b2d/
   source cloud/env.sh                CARLA_ROOT, PYTHONPATH and the rest in this shell
   ./run.sh --no-animate              this repository's own simulation
EOF
