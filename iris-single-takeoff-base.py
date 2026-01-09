#!/usr/bin/env python3

import asyncio
import logging
from mavsdk import System

logging.basicConfig(level=logging.INFO)


async def print_status_text(drone):
    try:
        async for status_text in drone.telemetry.status_text():
            print(f"STATUS [{status_text.type}]: {status_text.text}")
    except asyncio.CancelledError:
        return


async def run():
    drone = System()
    await drone.connect(system_address="udpin://0.0.0.0:14540")

    status_task = asyncio.create_task(print_status_text(drone))

    print("Waiting for connection...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Connected")
            break

    print("Waiting for global position & home position...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("Position OK")
            break

    print("Setting takeoff altitude to 2.0 m")
    await drone.action.set_takeoff_altitude(2.0)

    print("Arming")
    await drone.action.arm()

    print("Taking off to 2.0 m")
    await drone.action.takeoff()

    print("Holding at 2.0 m (no land command)")
    while True:
        await asyncio.sleep(1)


if __name__ == "__main__":
    asyncio.run(run())
