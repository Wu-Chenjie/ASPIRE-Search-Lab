"""Propagate terminal PX4/map-session faults; a healthy report cannot reset them."""
import math


class FlightSessionFault:
    def __init__(self, drone):
        self.drone = drone
        self.reason = None
        self.time = None
        self.last_report = -math.inf

    def observe(self, packet, now):
        """True exactly once per node lifetime, on a fresh local terminal fault.

        No claim of braking or relocalization: consumers retire this session and
        must not interpret loss of commands as proof that the vehicle stopped.
        """
        try:
            stamp = float(packet['time'])
            if (type(packet['drone']) is not int or packet['drone'] != self.drone or
                    not -.1 <= now-stamp <= 2. or stamp < self.last_report):
                return False
            self.last_report = stamp
            reason = packet.get('flight_session_fault') or packet.get('alignment_fault')
            if not isinstance(reason, str) or not reason:
                reason = ('px4_failsafe_requires_new_session' if
                          packet.get('failsafe_latched') is True or
                          (packet.get('armed') is True and packet.get('failsafe') is True)
                          else None)
            if reason and self.reason is None:
                self.reason, self.time = reason, stamp
                return True
        except (KeyError, TypeError, ValueError, OverflowError):
            pass
        return False
