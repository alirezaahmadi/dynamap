import sys
# print(sys.path)
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d import proj3d

from pytransform3d.rotations import *
from pyquaternion import Quaternion
import argparse
import os
import math


def rmse(predictions, targets):
    return np.sqrt(((predictions - targets) ** 2).mean())
    # rmse_val = rmse(np.array(d), np.array(p))


def rotate(X, theta, axis='x'):
    '''Rotate multidimensional array `X` `theta` degrees around axis `axis`'''
    c, s = np.cos(theta), np.sin(theta)
    if axis == 'x': return np.dot(X, np.array([
        [1.,  0,  0, 0],
        [0 ,  c, -s, 0],
        [0 ,  s,  c, 0],
        [0 ,  0,  0, 1]
        ]))
    elif axis == 'y': return np.dot(X, np.array([
        [c,  0,  -s, 0],
        [0,  1,   0, 0],
        [s,  0,   c, 0],
        [0 ,  0,  0, 1]
        ]))
    elif axis == 'z': return np.dot(X, np.array([
    [c, -s,  0, 0],
    [s,  c,  0, 0],
    [0,  0,  1, 0],
    [0 ,  0,  0, 1]
    ]))
def swapPositions(list, pos1, pos2): 
      
    list[pos1], list[pos2] = list[pos2], list[pos1] 
    return list

def read_file_list(filename):
    """
    Reads a trajectory from a text file. 
    
    File format:
    The file format is "stamp d1 d2 d3 ...", where stamp denotes the time stamp (to be matched)
    and "d1 d2 d3.." is arbitary data (e.g., a 3D position and 3D orientation) associated to this timestamp. 
    
    Input:
    filename -- File name
    
    Output:
    dict -- dictionary of (stamp,data) tuples
    
    """
    file = open(filename)
    data = file.read()
    lines = data.replace(","," ").replace("\t"," ").split("\n") 
    list = [[v.strip() for v in line.split(" ") if v.strip()!=""] for line in lines if len(line)>0 and line[0]!="#"]
    list = [(float(l[0]),l[1:]) for l in list if len(l)>1]
    return dict(list)

if __name__ == '__main__':
    # parse command line
    parser = argparse.ArgumentParser(description='''
    This script takes two data files with timestamps and associates them   
    ''')
    parser.add_argument('first_file', help='first text file (format: timestamp data)')
    parser.add_argument('second_file', help='second text file (format: timestamp data)')
    args = parser.parse_args()

    first_list = read_file_list(args.first_file)
    listofTuples = sorted(list(first_list.items()) )
    second_list = read_file_list(args.second_file )
    listofTuples_ref = sorted(list(second_list.items()) )

    p0 = np.array(listofTuples_ref[0][1][0:3], dtype=np.float)
    q0 = Quaternion(listofTuples_ref[0][1][6], listofTuples_ref[0][1][3], listofTuples_ref[0][1][4], listofTuples_ref[0][1][5])
    T0 = q0.transformation_matrix   # 4x4 transformation matrix
    T0[0,3] = p0[0]
    T0[1,3] = p0[1]
    T0[2,3] = p0[2]
    R_0= np.array(T0[0:3,0:3])
    print("T_ref: ")
    print(T0)

    startOffset = 0
    ax = plot_basis(R=R_0, s=0.1)
    for i in range(startOffset,len(listofTuples),30):  
    # for i in range(2): 
        p = np.array(listofTuples[i][1][0:3], dtype=np.float)
        q = Quaternion(listofTuples[i][1][6],listofTuples[i][1][3],listofTuples[i][1][4],listofTuples[i][1][5])
        T = q.transformation_matrix   # 4x4 transformation matrix
        tmp = p
        p[0] = tmp[0]
        p[1] = tmp[2]
        p[2] = tmp[1]
        T[0,3] = p[0]
        T[1,3] = p[1]
        T[2,3] = p[2]
        # print(p)
        R = np.array(T[0:3,0:3])
        R = np.dot(R_0, R)
        T = rotate(T, -math.pi/2, axis='z')
        # print("T: ")
        # print(T)
        
        axes = plot_basis(ax, R, p, s=0.1)
        
        index_ref = i * 8
        p_ref = np.array(listofTuples_ref[index_ref][1][0:3], dtype=np.float)
        p_ref = np.subtract(p_ref, p0)
        q_ref = Quaternion(listofTuples_ref[index_ref][1][6],listofTuples_ref[index_ref][1][3],listofTuples_ref[index_ref][1][4],listofTuples_ref[index_ref][1][5])
        T_ref = q_ref.transformation_matrix   # 4x4 transformation matrix
        # T_ref = np.dot(np.linalg.inv(T0), T_ref)
        R_ref = np.array(T_ref[0:3,0:3])
        # axes_ref = plot_basis(ax, R_ref, p_ref, s=0.05)

        # if i > 1:
        #     ax.set_axis_off()

        # plt.pause(0.05)
        plt.waitforbuttonpress() 
    # p_ref = []
    # for i in range(0,len(listofTuples_ref)):
    #     p_ref.append(listofTuples_ref[i][1][0:3])
    
    # npArray_ref = np.array(p_ref[:][:], dtype=np.float)
    # print("ref_len: ",npArray_ref[:,0].shape)
    
    # p = []
    # for i in range(0,len(listofTuples)):
    #     p.append(listofTuples[i][1][0:3])

    # npArray = np.array(p[:][:], dtype=np.float)
    # print("Traj_len: ",npArray[:,0].shape)

    # mpl.rcParams['legend.fontsize'] = 10

    # fig = plt.figure()
    # ax = fig.gca(projection='3d')
    
    # ax.plot(npArray[:,0], npArray[:,1], npArray[:,2], label='fr2_xyz', c='r')
    # ax.plot(npArray_ref[:,0], npArray_ref[:,1], npArray_ref[:,2], label='fr2_xyz_ref', c='b')
    # ax.legend()

    plt.show()
    
