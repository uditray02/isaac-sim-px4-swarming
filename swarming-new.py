import asyncio
import math
from mavsdk import System
from mavsdk.offboard import VelocityNedYaw, OffboardError

# ---------------- CONFIG ----------------
SERVERS = [
    (50040, 1),
    (50041, 2),
    (50042, 3),  # LEADER
    (50043, 4),
    (50044, 5),
]

LEADER_IDX = 2

TAKEOFF_ALT = 15.0
TAKEOFF_SPEED = 1.0

HZ = 10
DT = 1 / HZ

FORWARD_SPEED = 2.0
KP_POS = 1.2
MAX_VEL = 3.0

# LINE FORMATION (meters behind leader)
LINE_SPACING = {
    0: 4.0,
    1: 2.0,
    3: 6.0,
    4: 8.0,
}
# ---------------------------------------

class Vehicle:
    def __init__(self, drone, idx):
        self.drone = drone
        self.idx = idx
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.yaw = 0.0

# ---------------- YAW HELPER ----------------
def yaw_from_velocity(vn, ve, prev_yaw, alpha=0.2):
    if abs(vn) < 0.05 and abs(ve) < 0.05:
        return prev_yaw
    desired = math.atan2(ve, vn)
    return prev_yaw + alpha * (desired - prev_yaw)

# ---------------- CONNECT ----------------
async def connect_vehicle(server_port, idx):
    drone = System(mavsdk_server_address="127.0.0.1", port=server_port)
    await drone.connect()

    async for s in drone.core.connection_state():
        if s.is_connected:
            print(f"[CONNECTED] Vehicle {idx}")
            break

    async for h in drone.telemetry.health():
        if h.is_local_position_ok and h.is_armable:
            print(f"[READY] Vehicle {idx}")
            break
        await asyncio.sleep(0.2)

    return Vehicle(drone, idx)

# ---------------- OFFBOARD STREAM ----------------
async def offboard_stream(v: Vehicle):
    while True:
        await v.drone.offboard.set_velocity_ned(
            VelocityNedYaw(v.vx, v.vy, v.vz, v.yaw)
        )
        await asyncio.sleep(DT)

# ---------------- TAKEOFF ----------------
async def arm_and_takeoff(v: Vehicle):
    await asyncio.sleep(1.0)

    try:
        await v.drone.offboard.start()
        print(f"[OFFBOARD] Vehicle {v.idx}")
    except OffboardError as e:
        print(f"[OFFBOARD FAIL] Vehicle {v.idx}: {e}")
        return

    await v.drone.action.arm()
    print(f"[ARMED] Vehicle {v.idx}")

    climbed = 0.0
    while climbed < TAKEOFF_ALT:
        v.vz = -TAKEOFF_SPEED
        await asyncio.sleep(DT)
        climbed += TAKEOFF_SPEED * DT

    v.vz = 0.0
    print(f"[TAKEOFF COMPLETE] Vehicle {v.idx}")

# ---------------- ALTITUDE HOLD (CRITICAL FIX) ----------------
async def altitude_hold(vehicles):
    while True:
        for v in vehicles:
            v.vz = 0.0   # HARD LOCK ALTITUDE
        await asyncio.sleep(DT)

# ---------------- LEADER MOTION ----------------
async def leader_forward(leader: Vehicle):
    print("[LEADER] Moving straight forward")
    while True:
        leader.vx = FORWARD_SPEED
        leader.vy = 0.0
        leader.yaw = yaw_from_velocity(leader.vx, leader.vy, leader.yaw)
        await asyncio.sleep(DT)


async def align_yaw_north(v: Vehicle):
    v.yaw = 0.0
    for _ in range(30):   # ~3 seconds
        await asyncio.sleep(DT)

# ---------------- FOLLOWER CONTROL (1-D CONVOY) ----------------
async def follower_control(vehicles):
    leader = vehicles[LEADER_IDX]

    while True:
        async for pv in leader.drone.telemetry.position_velocity_ned():
            lp = pv.position
            lv = pv.velocity
            break

        for i, v in enumerate(vehicles):
            if i == LEADER_IDX:
                continue

            async for pv in v.drone.telemetry.position_velocity_ned():
                fp = pv.position
                break

            desired_north = lp.north_m - LINE_SPACING[i]
            err_n = desired_north - fp.north_m

            # ✅ FIXED: use north_m_s
            vn = KP_POS * err_n + lv.north_m_s
            vn = max(min(vn, MAX_VEL), -MAX_VEL)

            v.vx = vn
            v.vy = 0.0
            v.yaw = leader.yaw

        await asyncio.sleep(DT)


# ---------------- MAIN ----------------
async def main():
    vehicles = []

    for port, idx in SERVERS:
        vehicles.append(await connect_vehicle(port, idx))

    # Start OFFBOARD streams FIRST
    for v in vehicles:
        asyncio.create_task(offboard_stream(v))

    # Arm + takeoff
    await asyncio.gather(*(arm_and_takeoff(v) for v in vehicles))

    # 🔒 ALTITUDE HOLD (prevents descent)
    asyncio.create_task(altitude_hold(vehicles))

    print("\n[SWARM] Line formation convoy\n")

    # Align yaw of all vehicles to North
    await asyncio.gather(*(align_yaw_north(v) for v in vehicles))

    print("[ALIGNMENT] All vehicles aligned to North")

    asyncio.create_task(follower_control(vehicles))
    await leader_forward(vehicles[LEADER_IDX])


asyncio.run(main())
