import numpy as np


class A(object):
    def __init__(self):
        self.lst = 5


class B(object):
    def __init__(self, a):
        self.a = a
    def cng(self):
        self.a.lst = 2

a = A()
print(a.lst)

b = B(a)
b.cng()
print(a.lst)

