"""Delivery queue: emergency requests first, otherwise nearest-first (by planned path length)."""
from dataclasses import dataclass


@dataclass
class Request:
    ward: str
    priority: int = 0        # 1 = emergency
    arrival: float = 0.0     # simulated time at which the request appears


def parse_request(text):
    """'Ward4' | 'Ward4:E' | 'Ward4:E@30'  (E = emergency, @t = arrival time in s)"""
    arrival, prio = 0.0, 0
    if '@' in text:
        text, a = text.split('@'); arrival = float(a)
    if ':' in text:
        text, flag = text.split(':'); prio = 1 if flag.upper() == 'E' else 0
    return Request(text, prio, arrival)


class DeliveryQueue:
    def __init__(self, requests=()):
        self.items = list(requests)

    def __len__(self):
        return len(self.items)

    def pop_next(self, now, here_xy, stops, cost):
        ready = [r for r in self.items if r.arrival <= now]
        if not ready:
            return None
        top = max(r.priority for r in ready)
        cand = [r for r in ready if r.priority == top]
        best = min(cand, key=lambda r: cost(here_xy, stops[r.ward]))
        self.items.remove(best)
        return best
