import logging
import numpy as np

# class Logger(object):
#     def __init__(self, file_name):
#         logging.basicConfig(level=logging.DEBUG, filename=file_name, filemode='w',
#                     format="%(asctime)s - %(levelname)s - %(message)s")
#         self.logger = logging.getLogger(file_name)
#         handler = logging.FileHandler(file_name)
#         formatter = logging.Formatter("%(asctime)s - %(levelname)s - %(message)s")
#         handler.setFormatter(formatter)
#         self.logger.addHandler(handler)
#         self.logger.setLevel(logging.DEBUG)
    
#     def log(self, msg):
#         self.logger.debug(msg)
#         print(msg)

# logger = Logger('my_logger')
# x = 3
# logger.log(f'bkabka {x}')
# logger.log('hellow')

print(round(70.565464656,2))