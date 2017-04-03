#!/usr/bin/env python
import time

start_time = time.time()
calc = time.time()
end_time = time.time()

print (end_time - start_time)/2.0

start_time = time.clock()
calc = time.clock()
end_time = time.clock()

print (end_time - start_time)/2.0
