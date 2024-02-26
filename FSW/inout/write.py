import mavsdk
from mavsdk import System


def connect_to_drone_eth():
    drone = System()
    await drone.connect(system_address="udp://:14540")
    return drone


def write_to_pixhawk_eth(drone):
    pass
