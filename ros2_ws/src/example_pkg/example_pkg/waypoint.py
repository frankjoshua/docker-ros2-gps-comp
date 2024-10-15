from dataclasses import dataclass

@dataclass
class Waypoint:
    latitude: float
    longitude: float
    description: str
