class Fake(object):
    def __init__(self):
        pass
    def is_alive(self):
        return False

drone_num = 3

open_threads = [Fake()] * drone_num
print(open_threads[0].is_alive())