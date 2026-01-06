# Isaac-Sim-with-ROS2-PX4-Ardupilot








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

