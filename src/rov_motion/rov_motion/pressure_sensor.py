import time

class PressureSensor():
    def __init__(self, bus=1):
        self.is_connected = False
        self.sensor= None

        try:
            import ms5837
            self.sensor= ms5837.MS5837_30BA(bus)
            if self.sensor.init():
                self.sensor.setFluidDensity(ms5837.DENSITY_FRESHWATER)
                self.is_connected=True
        except Exception:
            self.is_connected=False

    def depth(self):

        if self.is_connected and self.sensor is not None:
            try:
                #take depth reading from sensor
                if self.sensor.read():
                    return self.sensor.depth()
            except Exception:
                pass

        return 30.0 #fallback to 30 when testing without physical hardware

    def pressure(self):

        if self.is_connected and self.sensor is not None:
            try:
                #take pressure reading from sensor
                if self.sensor.read():
                    return self.sensor.pressure()
            except Exception:
                pass
        return 3955.0

    def temperature(self):
        if self.is_connected and self.sensor is not None:
            try:
                #take temperature reading from sensor
                if self.sensor.read():
                    return self.sensor.temperature()
            except Exception:
                pass
        return 20.0






# class PressureSensor(Node):
#     def __init__(self, bus=1, default_depth=30):
#         self.bus = bus
#         self.default_depth=float(default_depth)
#         self.is_connected= False
#         self.sensor= None

#         try:
#             self.sensor=ms5837.MS5837_30BA(bus)
#             if self.sensor.init():
#                 self.sensor.setFluidDensity(ms5837.DENSITY_FRESHWATER)
#                 self.is_connected=True
#         except Exception:
#             self.is_connected=False


#     def read_depth(self):
#         if self.is_connected and self.sensor is not None:
#             try:
#                 if self.sensor.read_depth():
#                     return round(float(self.sensor.depth()),3)
#             except Exception:
#                 pass
#             return self.default_depth

#     def read_pressure()


