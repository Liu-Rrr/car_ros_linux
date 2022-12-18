#!/usr/bin/env python
from logger import logger
from threading import Thread


def loggerInto(array):
    logger.info(array)
    # print("jilu")
    print("jilu")

# loggerInto("sss")
# loggerInto("sss")
Thread(target=loggerInto,args=[[1,23,4,54,5,5,5,5,656]]).start()
