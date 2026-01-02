import time
from functools import partial, wraps


class _show_performance:
    def __init__(self, func, a, show_every):
        wraps(func)(self)
        self.func = func
        self.count = 0
        self.time_avg = 0.0
        self.a = a
        self.show_every = show_every

    def __call__(self, *args, **kwargs):
        start_time = time.perf_counter()

        # Execute the actual function
        result = self.func(*args, **kwargs)

        end_time = time.perf_counter()
        duration = end_time - start_time

        if self.count > 1 / self.a:
            a = self.a
        else:
            a = 1 / (self.count + 1)

        # Update state
        self.count += 1
        self.time_avg = self.time_avg * (1 - a) + duration * a

        if self.count % self.show_every == 0:
            print(f"[{time.strftime('%H:%M:%S')}] {self.func.__name__} Execution: {duration:.4f}s | "
                f"Avg: {self.time_avg:.4f}s | Calls: {self.count}")

        return result

    def __del__(self):
        print(f"[{time.strftime('%H:%M:%S')}] {self.func.__name__} Avg: {self.time_avg:.4f}s | Calls: {self.count}")


def show_performance(func, a=0.1, show_every=16):
    return partial(_show_performance, a=a, show_every=show_every)(func)
