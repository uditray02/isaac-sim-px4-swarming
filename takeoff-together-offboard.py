import asyncio
from mavsdk import System
from mavsdk.offboard import VelocityNedYaw, OffboardError

# (mavsdk_server_port, vehicle_index)
SERVERS = [
    (50040, 1),
    (50041, 2),
    (50042, 3),
    (50043, 4),
    (50044, 5),
]

HZ = 10  # OFFBOARD setpoint rate (>=2 Hz required)

async def offboard_takeoff(server_port: int, idx: int):
    drone = System(
        mavsdk_server_address="127.0.0.1",
        port=server_port
    )
    await drone.connect()

    # 1) Connection
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"[CONNECTED] Vehicle {idx} (server {server_port})")
            break

    # 2) Health gate
    async for health in drone.telemetry.health():
        if health.is_local_position_ok and health.is_armable:
            print(f"[READY] Vehicle {idx}")
            break
        await asyncio.sleep(0.2)

    # 3) Pre-setpoints (MANDATORY)
    for _ in range(20):
        await drone.offboard.set_velocity_ned(
            VelocityNedYaw(0.0, 0.0, 0.0, 0.0)
        )
        await asyncio.sleep(1.0 / HZ)

    # 4) Start OFFBOARD
    try:
        await drone.offboard.start()
        print(f"[OFFBOARD] Vehicle {idx}")
    except OffboardError as e:
        print(f"[OFFBOARD FAIL] Vehicle {idx}: {e}")
        return

    # 5) Arm
    await drone.action.arm()
    print(f"[ARMED] Vehicle {idx}")

    # 6) Takeoff using VELOCITY (PX4-safe for swarms)
    while True:
        await drone.offboard.set_velocity_ned(
            VelocityNedYaw(0.0, 0.0, -1.0, 0.0)  # up at 1 m/s
        )
        await asyncio.sleep(1.0 / HZ)

async def main():
    tasks = [
        asyncio.create_task(offboard_takeoff(port, idx))
        for port, idx in SERVERS
    ]
    await asyncio.gather(*tasks)

if __name__ == "__main__":
    asyncio.run(main())
