#!/bin/bash
#
# setup.sh - one-shot setup of the Mecatron ArduPilot SITL environments.
#
# Usage:
#   mkdir -p ~/ardupilot && cd ~/ardupilot
#   wget https://github.com/NTU-Mecatron/miscellaneous/releases/download/ardupilot/setup.sh
#   chmod +x setup.sh
#   ./setup.sh
#
# It clones uuv-sub-4.5, usv-sub-4.5 and copter-4.5 into ~/ardupilot, builds a
# single shared Docker image `ardupilot-dev`, and configures + builds SITL for
# all three.

set -euo pipefail

ARDUPILOT_BASE="$HOME/ardupilot"
REPO_URL="https://github.com/NTU-Mecatron/ardupilot.git"
IMAGE="ardupilot-dev"

# dir | branch | waf target
VEHICLES=(
    "uuv-sub-4.5|Sub-4.5|sub"
    "usv-sub-4.5|Sub-4.5|sub"
    "copter-4.5|Copter-4.5|copter"
)

# The Docker image is built once, from this repo, and shared by all three.
PRIMARY_DIR="uuv-sub-4.5"

# ---------------------------------------------------------------- logging ---

if [ -t 1 ]; then
    C_INFO=$'\e[1;34m'; C_OK=$'\e[1;32m'; C_WARN=$'\e[1;33m'; C_ERR=$'\e[1;31m'; C_OFF=$'\e[0m'
else
    C_INFO=""; C_OK=""; C_WARN=""; C_ERR=""; C_OFF=""
fi

info() { echo "${C_INFO}==>${C_OFF} $*"; }
ok()   { echo "${C_OK}==>${C_OFF} $*"; }
warn() { echo "${C_WARN}warning:${C_OFF} $*" >&2; }
die()  { echo "${C_ERR}error:${C_OFF} $*" >&2; exit 1; }

# Docker needs a TTY only when we actually have one (so CI / piped runs work).
DOCKER_TTY=()
[ -t 0 ] && [ -t 1 ] && DOCKER_TTY=(-it)

# ---------------------------------------------------------- prerequisites ---

check_prereqs() {
    info "Checking prerequisites"

    command -v git >/dev/null 2>&1 || die "git is not installed. Install it with: sudo apt install git"

    if ! command -v docker >/dev/null 2>&1; then
        die "docker is not installed. Install Docker Engine natively (not Docker Desktop):
    https://docs.docker.com/engine/install/ubuntu/"
    fi

    if ! docker info >/dev/null 2>&1; then
        die "cannot talk to the Docker daemon.

  If this is a permissions problem, add yourself to the docker group:
      sudo usermod -aG docker \$USER
  then log out and back in (or restart the machine) and re-run ./setup.sh

  If the daemon is not running:
      sudo systemctl start docker"
    fi

    ok "Prerequisites look good"
}

# ------------------------------------------------------------------ steps ---

clone_repo() {
    local dir="$1" branch="$2"
    local path="$ARDUPILOT_BASE/$dir"

    if [ -d "$path/.git" ]; then
        info "$dir already exists, skipping clone"
        return
    fi
    if [ -e "$path" ]; then
        die "$path exists but is not a git repository. Remove or rename it and re-run."
    fi

    info "Cloning $dir (branch $branch)"
    git clone --recursive -b "$branch" "$REPO_URL" "$path"
}

build_image() {
    info "Building Docker image '$IMAGE' (this takes a while the first time)"
    # Match the container user to the host user so files written into the mounted
    # volume (build/, logs/) are owned by us instead of uid 1000.
    docker build -t "$IMAGE" \
        --build-arg USER_UID="$(id -u)" \
        --build-arg USER_GID="$(id -g)" \
        "$ARDUPILOT_BASE/$PRIMARY_DIR"
    ok "Docker image '$IMAGE' ready"
}

waf() {
    local path="$1"; shift
    docker run --rm "${DOCKER_TTY[@]}" -v "$path:/ardupilot" "$IMAGE" ./waf "$@"
}

build_vehicle() {
    local dir="$1" branch="$2" target="$3"
    local path="$ARDUPILOT_BASE/$dir"

    echo
    info "===== $dir ($branch, ./waf $target) ====="

    info "Configuring SITL build for $dir"
    waf "$path" configure --board=sitl

    info "Building $target for $dir (this takes several minutes)"
    waf "$path" "$target"

    [ -f "$path/run_sitl.sh" ] && chmod +x "$path/run_sitl.sh"

    ok "$dir is ready"
}

# ------------------------------------------------------------------- main ---

main() {
    check_prereqs

    mkdir -p "$ARDUPILOT_BASE"

    # Clone everything first, so the shared Docker image can be built from
    # uuv-sub-4.5 before any vehicle is compiled.
    for entry in "${VEHICLES[@]}"; do
        IFS='|' read -r dir branch _ <<< "$entry"
        clone_repo "$dir" "$branch"
    done

    build_image

    for entry in "${VEHICLES[@]}"; do
        IFS='|' read -r dir branch target <<< "$entry"
        build_vehicle "$dir" "$branch" "$target"
    done

    echo
    ok "All three vehicles are set up:"
    for entry in "${VEHICLES[@]}"; do
        IFS='|' read -r dir _ _ <<< "$entry"
        echo "    $ARDUPILOT_BASE/$dir"
    done
    cat <<EOF

Next steps — run SITL from the root of whichever vehicle you want:

    cd $ARDUPILOT_BASE/$PRIMARY_DIR
    ./run_sitl.sh

Edit run_sitl.sh first if you need to change JSON_BACKEND_SIM_IP or the GCS IP.
Use './run_sitl.sh -I 1' to run a second vehicle at the same time.

After changing code, rebuild with:

    docker run --rm -it -v \$PWD:/ardupilot $IMAGE ./waf sub      # or ./waf copter
EOF
}

main "$@"
