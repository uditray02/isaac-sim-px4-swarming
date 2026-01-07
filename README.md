# Isaac-Sim-with-PX4-Swarming


# Step 1 : Installation of Isaac Sim + Drone Simulator

### a. Install system dependencies
```bash
# Go to the home directory
cd ~

# Create a new directory to store the Isaac Sim installation
mkdir -p isaacsim
cd isaacsim

# Download the zip file containing the Isaac Sim installation
wget https://download.isaacsim.omniverse.nvidia.com/isaac-sim-standalone-5.1.0-linux-x86_64.zip

# Unzip the file
unzip isaac-sim-standalone-5.1.0-linux-x86_64.zip

# Run the post-installation scripts
./post_install.sh
./isaac-sim.selector.sh

# Delete the zip file
rm isaac-sim-standalone-5.1.0-linux-x86_64.zip
```
### b. Adding in .bashrc
```bash
# ---------------------------
# ISAAC SIM SETUP
# ---------------------------
# Isaac Sim root directory
export ISAACSIM_PATH="${HOME}/isaacsim"
# Isaac Sim python executable
export ISAACSIM_PYTHON="${ISAACSIM_PATH}/python.sh"
# Isaac Sim app
export ISAACSIM="${ISAACSIM_PATH}/isaac-sim.sh"

# Define an auxiliary function to launch Isaac Sim or run scripts with Isaac Sim's python
# This is done to avoid conflicts between ROS 2 and Isaac Sim's Python environment
isaac_run() {

    # ------------------
    # === VALIDATION ===
    # ------------------
    if [ ! -x "$ISAACSIM_PYTHON" ]; then
        echo "❌ IsaacSim python.sh not found at: $ISAACSIM_PYTHON"
        return 1
    fi
    if [ ! -x "$ISAACSIM" ]; then
        echo "❌ IsaacSim launcher not found at: $ISAACSIM"
        return 1
    fi

    # -------------------------
    # === CLEAN ENVIRONMENT ===
    # -------------------------
    # Unset ROS 2 environment variables to avoid conflicts with Isaac's Python 3.11
    unset ROS_VERSION ROS_PYTHON_VERSION ROS_DISTRO AMENT_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH CMAKE_PREFIX_PATH

    # Remove ROS 2 paths from LD_LIBRARY_PATH if present
    local ros_paths=("/opt/ros/humble" "/opt/ros/jazzy" "/opt/ros/iron")
    for ros_path in "${ros_paths[@]}"; do
        export LD_LIBRARY_PATH=$(echo "$LD_LIBRARY_PATH" | tr ':' '\n' | grep -v "^${ros_path}" | paste -sd':' -)
    done

    # -----------------------------
    # === UBUNTU VERSION CHECK ===
    # -----------------------------

    if [ -f /etc/os-release ]; then
        UBUNTU_VERSION=$(grep "^VERSION_ID=" /etc/os-release | cut -d'"' -f2)
    fi

    # If Ubuntu 24.04 -> use the Isaac Sim internal ROS2 Jazzy (ROS2 Jazzy bridge)
    if [[ "$UBUNTU_VERSION" == "24.04" ]]; then
        export ROS_DISTRO=jazzy
        export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
        export LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:${ISAACSIM_PATH}/exts/isaacsim.ros2.bridge/jazzy/lib"
        echo "🧩 Detected Ubuntu 24.04 -> Using ROS_DISTRO=jazzy"
    # If Ubuntu 22.04 -> use the Isaac Sim internal ROS2 Humble (ROS2 Humble bridge)
    else
        export ROS_DISTRO=humble
        export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
        export LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:${ISAACSIM_PATH}/exts/isaacsim.ros2.bridge/humble/lib"
        echo "🧩 Detected Ubuntu ${UBUNTU_VERSION:-unknown} -> Using ROS_DISTRO=humble"
    fi

    # ---------------------
    # === RUN ISAAC SIM ===
    # ---------------------
    if [ $# -eq 0 ]; then
        # No args → Launch full Isaac Sim GUI
        echo "🧠 Launching Isaac Sim GUI..."
        "${ISAACSIM}"

    elif [[ "$1" == --* ]]; then
        # Arguments start with "--" → pass them to Isaac Sim executable
        echo "⚙️  Launching Isaac Sim with options: $*"
        "${ISAACSIM}" "$@"

    elif [ -f "$1" ]; then
        # First argument is a Python file → run with Isaac Sim's Python
        local SCRIPT_PATH="$1"
        shift
        echo "🚀 Running Python script with Isaac Sim: $SCRIPT_PATH"
        "${ISAACSIM_PYTHON}" "$SCRIPT_PATH" "$@"

    else
        # Unrecognized input
        echo "❌ Unknown argument or file not found: '$1'"
        echo "Usage:"
        echo "  isaac_run                 → launch GUI"
        echo "  isaac_run my_script.py    → run script with IsaacSim Python"
        echo "  isaac_run --headless ...  → launch IsaacSim with CLI flags"
        return 1
    fi
}
```







# Step 2 : MAVSDK Server – Build, Install, and Run (Linux)

## Prerequisites

- Ubuntu 20.04 / 22.04  
- Python 3.10  
- GCC / Make toolchain  
- Internet connection  

---

## Installation

### 1. Install system dependencies

```bash
sudo apt update
sudo apt install -y \
    git cmake build-essential \
    libssl-dev protobuf-compiler \
    libprotobuf-dev libcurl4-openssl-dev


cd ~
git clone https://github.com/mavlink/MAVSDK.git
cd MAVSDK

mkdir build
cd build

cmake ..
make -j$(nproc)

Verification
3. Locate mavsdk_server
find . -type f -name mavsdk_server


Expected output:

./src/mavsdk_server/mavsdk_server


(or)

./src/mavsdk_server

4. Python MAVSDK server location (required for MAVSDK-Python)

The server binary must exist at:

~/.local/lib/python3.10/site-packages/mavsdk/bin/mavsdk_server


If it is missing, install MAVSDK for Python:

pip3 install --user mavsdk


Verify:

~/.local/lib/python3.10/site-packages/mavsdk/bin/mavsdk_server --version

Running the Server
5. Start mavsdk_server

Example configuration:

MAVSDK gRPC port: 50040

MAVLink UDP port: 14540

~/.local/lib/python3.10/site-packages/mavsdk/bin/mavsdk_server \
-p 50040 udp://:14540

~/.local/lib/python3.10/site-packages/mavsdk/bin/mavsdk_server -p 50040 udp://:14540 &
~/.local/lib/python3.10/site-packages/mavsdk/bin/mavsdk_server -p 50041 udp://:14541 &
~/.local/lib/python3.10/site-packages/mavsdk/bin/mavsdk_server -p 50042 udp://:14542 &
~/.local/lib/python3.10/site-packages/mavsdk/bin/mavsdk_server -p 50043 udp://:14543 &
~/.local/lib/python3.10/site-packages/mavsdk/bin/mavsdk_server -p 50044 udp://:14544 &



Port Usage Rules

One MAVSDK server per vehicle

Each server must use a unique gRPC port

MAVLink UDP ports must match PX4 / SITL / simulator configuration

Do not mix system-built and pip-installed MAVSDK binaries

