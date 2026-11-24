import math

OPPONENT_COUNT = 4
OPPONENT_RANGE = 6.0
OPPONENT_SPEED_SCALE = 6.0
OPPONENT_INPUTS = OPPONENT_COUNT * 5


def observe_opponents(world, opponents):
    """Nearest cars in the driver's frame: presence, position, relative velocity."""
    body = world.vehicle.body
    cosine = math.cos(world.vehicle.heading)
    sine = math.sin(world.vehicle.heading)
    nearby = []
    for other in opponents:
        dx = other.body.position.x - body.position.x
        dy = other.body.position.y - body.position.y
        distance = math.hypot(dx, dy)
        if distance <= OPPONENT_RANGE:
            nearby.append((distance, dx, dy, other))
    nearby.sort(key=lambda item: item[0])

    features = []
    for _, dx, dy, other in nearby[:OPPONENT_COUNT]:
        dvx = other.body.velocity.x - body.velocity.x
        dvy = other.body.velocity.y - body.velocity.y
        features.extend([
            1.0,
            (dx * cosine + dy * sine) / OPPONENT_RANGE,
            (-dx * sine + dy * cosine) / OPPONENT_RANGE,
            max(-1.0, min(1.0, (dvx * cosine + dvy * sine) / OPPONENT_SPEED_SCALE)),
            max(-1.0, min(1.0, (-dvx * sine + dvy * cosine) / OPPONENT_SPEED_SCALE)),
        ])
    features.extend([0.0] * (OPPONENT_INPUTS - len(features)))
    return features
