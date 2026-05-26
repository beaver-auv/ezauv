from ezauv.utils import Clock

class FakeClock(Clock):
    def __init__(self):
        self.current_time = 0

    def time(self):
        return self.current_time

    def perf_counter(self):
        return self.current_time
    
    def sleep(self, seconds):
        self.current_time += seconds

    def set_time(self, new_time):
        self.current_time = new_time