#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sun Feb 13 15:38:11 2022

@author: uas

You should be able to use the functions in this script to encode a serial message and 
send it via the "serial_send" command to the plane.
The plane should be able to decode the message and use it


"""

import numpy as np


def bytes_to_int(byte_array):
    result = 0
    for b in byte_array:
        result = result * 256 + int(b)
    return result

def int_to_bytes(value, length):
    byte_array = []
    for i in range(0, length):
        byte_array = [value%256] + byte_array
        value=value//256

    return byte_array

def testFunction():
    a = int_to_bytes(93143610,4)
    b = bytes_to_int(a)
    print("both values should be the same",a,b)
    
    
def state_encode(state_header,state_body):
    state = state_header
    # Assign 4 bytes to the header
    state += [0]*(4-len(state_header))
    nAgents = state_header[0]
    for i in range(np.size(state_body)/nAgents):
        state += int_to_bytes(int((-state_body[i*3]-30)*1e8),4)   # 30 and 136 are the first digits of the woomera location
        state += int_to_bytes(int((state_body[i*3+1]-136)*1e8),4)  
        state.append(int(state_body[i*3+2]))
    
    state += [0]*(70-len(state))
    
#    print("Encode is")
#    print(state_body)
    
    return state
    
def state_decode(state):
    state_header = state[0:4]
    state_body = []
    state_body_tmp = state[4:]
    nAgents = state[0]
    for i in range(nAgents):
        state_body.append(-(bytes_to_int(state_body_tmp[9*i:9*i+4])*1e-8+30)) 
        state_body.append(bytes_to_int(state_body_tmp[9*i+4:9*i+8])*1e-8+136) 
        state_body.append(state_body_tmp[9*i+8]) 

#    print("Decode is")
#    print(state_body)
        
    return state_header, state_body
    
    