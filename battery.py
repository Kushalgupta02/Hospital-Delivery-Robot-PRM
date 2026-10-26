from config import *


class BatteryModel:
    def __init__(self, level=BATTERY_CAPACITY):
        self.level = level
        self.charges = 0

    def consume(self, dist, dt):
        self.level = max(0.0, self.level - dist * ENERGY_PER_M - dt * ENERGY_IDLE_PER_S)

    @staticmethod
    def cost(dist):
        return dist * ENERGY_PER_M

    def charge_time(self):
        return (BATTERY_CAPACITY - self.level) / CHARGE_RATE

    def full(self):
        self.level = BATTERY_CAPACITY
        self.charges += 1
