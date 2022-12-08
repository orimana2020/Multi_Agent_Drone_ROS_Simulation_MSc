import time

class experi(object):
    def __init__(self):
        self.x = 5


def change(cls):
    cls.x = 3
    

exp = experi()
change(exp)

print(exp.x)