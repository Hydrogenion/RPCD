import open3d as o3d 
import numpy as np 
import argparse
import os
from utils import *

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-i', type=str,help='input path')
    parser.add_argument('-t', type=str,help='type c:clean r:real')
    args = parser.parse_args()
    path = args.i    
    if(args.t =='r'):
        _type = 'real'
    elif(args.t == 'c'):
        _type = 'clean'
    else:
        red_print('Wrong args')
        exit()

    try:
        raw_point_cloud = o3d.io.read_point_cloud(os.path.join(path,f'{_type}_100k.ply'))
        green_print(f'Load {_type}_100k.ply')
    except:
        red_print(f'{os.path.join(path,f"{_type}_100k.ply")} not exists')
        exit
    try:
        f = open(os.path.join(path,f'{_type}_annotation.txt'),'r',encoding='utf-8')
        green_print(f'Load {_type}_annotation.ply')
    except:
        red_print(f'{os.path.join(path,f"{_type}_annotation.txt")} not exists')
        exit
    seg_pc = []
    data = f.readlines()
    dic = {}
    for key in data:
        k,v = key.replace('\n','').split(' ')
        if not dic.__contains__(v):
            dic[v] = []
        dic[v].append(int(k))
    for k in dic:
        pc = raw_point_cloud.select_down_sample(dic[k])
        pc = pc.paint_uniform_color(np.random.rand(3,1))
        seg_pc.append(pc)
    o3d.visualization.draw_geometries(seg_pc)

