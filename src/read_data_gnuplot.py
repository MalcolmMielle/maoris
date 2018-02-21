#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import math

class Reader:
    def __init__(self):
        self.time = list()
    
    def read(self, file_name):
        f = open(file_name, 'r')
        for line in f:
            if(len(line) > 1):
                first_letter = line.split(None, 1)[0]
                print(line.split()[4])
                flag = True
                count = 0
                if first_letter != "#" and flag:
                    self.time.append(float(line.split()[4]) )
                else:
                    if count > 0:
                        flag = False
                    count += 1
    
    def mean_time(self):
        sum = 0
        for el in self.time:
            sum = sum + el
        return sum / len(self.time)


def main():
    reader = Reader()
    reader.read("data2.dat")
    print(reader.mean_time())


if __name__ == "__main__":
    main()
