import time as t

class Clock:
    def time(self):
        return t.time()
    
    def sleep(self, seconds):
        t.sleep(seconds)

    def perf_counter(self):
        return t.perf_counter()