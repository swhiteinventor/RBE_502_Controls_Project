#!/bin/python
from Robot_State import Robot_State

past = Robot_State(0,0,0,0,0,0,0)
curr = Robot_State(1,1,1,1,1,1,1)
extra = past

print past
print curr
print extra

def a():
	return 1,2

c,d = a()
print c
print d