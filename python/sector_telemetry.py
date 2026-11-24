import csv
class SectorTelemetry:
    def __init__(self, lap_length):
        self.lap_length = lap_length
        self.times = []
        self.previous_distance = 0.0
        self.previous_time = 0.0
        self.last_crossing_time = 0.0
        self.samples = []

    def update(self, status, controls):
        if len(self.times) == 3:
                return

        distance = status.lap_distance
        elapsed = status.lap_time

        if status.last_lap is not None:
            distance = self.lap_length
            elapsed = status.last_lap
        elif elapsed <= 0.0:
            return

        sector = min(3, int(distance / (self.lap_length / 3.0)) + 1)
        self.samples.append({
            "lap_time": elapsed,
            "lap_distance": distance,
            "sector": sector,
            "speed": status.speed,
            "throttle": controls.throttle,
            "steering": controls.steering,
        })

        if distance > self.previous_distance:
            while len(self.times) < 3:
                boundary = self.lap_length * ((len(self.times) + 1) / 3.0)
                if distance < boundary:
                    break

                fraction = (boundary - self.previous_distance) / (distance - self.previous_distance)
                crossing_time = self.previous_time + fraction * (elapsed - self.previous_time)

                self.times.append(crossing_time - self.last_crossing_time)
                self.last_crossing_time = crossing_time

        self.previous_distance = distance
        self.previous_time = elapsed
    def save_csv(self, path):
        if not self.samples:
            return

        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w", newline="", encoding="utf-8") as file:
            writer = csv.DictWriter(file, fieldnames=list(self.samples[0]))
            writer.writeheader()
            writer.writerows(self.samples)
