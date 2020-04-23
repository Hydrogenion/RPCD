import open3d as o3d 
from utils import *
import numpy as np 
import time
import datetime
import os 
import logging
import argparse

class RPCDPrepreocess():
    def __init__(self,folder_path,save_path):
        self.folder_path = folder_path
        self.save_path = save_path
        self.noiseless_point_cloud = None
        self.sample_point_clouds_from_mesh = []
        self.sample_point_clouds_tmp = []
        self.sample_point_clouds_from_point_cloud = []
        self.reconstructed_mesh = None
        self.segmentation_point_cloud = []
        self.annotation_name = []
        self.annotation_dict = {}
        self.translate_matrix = None
        logging.basicConfig(filename='./log.txt',datefmt='%Y-%m-%d %H:%M:%S %p',level=logging.DEBUG,format='%(asctime)s-%(message)s')
    
    def load_point_cloud(self):
        for file in os.listdir(self.folder_path):
            if file=='.DS_Store':
                continue
            file_path = os.path.join(self.folder_path,file)
            ext = file.split('.')[-1]
            if ext == 'ply':
                self.raw_point_cloud = o3d.io.read_point_cloud(file_path)
                self.translate_matrix = 0 - self.raw_point_cloud.get_center()
            elif ext =='txt':
                np_points = np.loadtxt(file_path)
                pcd = o3d.geometry.PointCloud()
                pcd.points = o3d.utility.Vector3dVector(np_points[:,0:3])
                self.segmentation_point_cloud.append(pcd)
                self.annotation_name.append(file.split('-')[-1].split('.')[0])

    def print_info(self):
        try:
            yellow_print('File:')
            blue_print(self.folder_path)
        except:
            red_print('No input file')
        try:
            yellow_print('Raw point cloud:')
            blue_print(self.raw_point_cloud)
        except:
            red_print('No raw point cloud')
        try:
            yellow_print('Noiseless point cloud:')
            blue_print(self.noiseless_point_cloud)
        except:
            red_print('No raw point cloud')
        try:
            yellow_print('Reconstructed mesh info:')
            blue_print(self.reconstructed_mesh)
        except:
            red_print('No reconstructed mesh')
        yellow_print('Down sample times:')
        blue_print(f'{len(self.sample_point_clouds_from_point_cloud)}')
        for idx,(pc) in enumerate(self.sample_point_clouds_from_point_cloud):
            yellow_print(f'Down sampled pc from pc {idx}:')
            blue_print(pc)

    def wirte_info(self,path):
        f = open(os.path.join(path,'info.txt'),'w',encoding='utf-8')		#以'w'方式打开文件
        f.write(self.folder_path.split('/')[-1]+'\n')
        f.write('annotations:'+'\n')
        for index,(name) in enumerate(self.annotation_name):
            f.write(str(index)+' '+name+'\n')
        f.write(f'noiseless.ply:{self.noiseless_point_cloud}\n')
        f.write(f'clean_100k.ply:{self.sample_point_clouds_from_mesh[0]}\n')
        f.write(f'clean_10k.ply:{self.sample_point_clouds_from_mesh[1]}\n')
        f.write(f'clean_1k.ply:{self.sample_point_clouds_from_mesh[2]}\n')
        f.write(f'real_100k.ply:{self.sample_point_clouds_from_point_cloud[0]}\n')
        f.write(f'real_10k.ply:{self.sample_point_clouds_from_point_cloud[1]}\n')
        f.write(f'real_1k.ply:{self.sample_point_clouds_from_point_cloud[2]}\n')
        f.write(f'mesh.ply:{self.reconstructed_mesh}\n')
        f.close()
  

    def remove_noise(self):
        '''
        Function to remove points that have less than nb_points in a given sphere of a given radius
        
        The parameters of remove_radius_outlier:
        nb_points (int) – Number of points within the radius.
        radius (float) – Radius of the sphere.
        
        You can change the parameters of remove_radius_outlier
        '''
        self.noiseless_point_cloud,_ = self.raw_point_cloud.remove_radius_outlier(nb_points=5,radius=1.0)

    def reconstruct_mesh(self):
        '''
        Function that computes a triangle mesh from a oriented PointCloud pcd. 
        This implements the Screened Poisson Reconstruction proposed in Kazhdan and Hoppe, “Screened Poisson Surface Reconstruction”, 2013. 

        The parameters of o3d.geometry.TriangleMesh.create_from_point_cloud_poisson:
        pcd (open3d.geometry.PointCloud) – PointCloud from which the TriangleMesh surface is reconstructed. Has to contain normals.
        depth (int, optional, default=8) – Maximum depth of the tree that will be used for surface reconstruction. Running at depth d corresponds to solving on a grid whose resolution is no larger than 2^d x 2^d x 2^d. Note that since the reconstructor adapts the octree to the sampling density, the specified reconstruction depth is only an upper bound.
        width (int, optional, default=0) – Specifies the target width of the finest level octree cells. This parameter is ignored if depth is specified
        scale (float, optional, default=1.1) – Specifies the ratio between the diameter of the cube used for reconstruction and the diameter of the samples’ bounding cube.
        linear_fit (bool, optional, default=False) – If true, the reconstructor use linear interpolation to estimate the positions of iso-vertices.

        You can change the parameters of creatre_from_point_cloud_poisson
        '''
        [self.reconstructed_mesh,_] = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(self.noiseless_point_cloud)
        self.reconstructed_mesh = self.reconstructed_mesh.remove_degenerate_triangles()
        self.reconstructed_mesh = self.reconstructed_mesh.remove_duplicated_triangles()
        self.reconstructed_mesh = self.reconstructed_mesh.remove_duplicated_vertices()
        self.reconstructed_mesh = self.reconstructed_mesh.remove_non_manifold_edges()
        # self.reconstructed_mesh = self.reconstructed_mesh.merge_close_vertices(0.5)

    def down_sample_to_100k_10k_1k_from_mesh(self):
        down_sample_100k = self.reconstructed_mesh.sample_points_poisson_disk(100000)
        down_sample_10k = self.reconstructed_mesh.sample_points_poisson_disk(10000)
        down_sample_1k = self.reconstructed_mesh.sample_points_poisson_disk(1000)
        self.sample_point_clouds_from_mesh.append(down_sample_100k)
        self.sample_point_clouds_from_mesh.append(down_sample_10k)
        self.sample_point_clouds_from_mesh.append(down_sample_1k)

    def down_sample_to_100k_10k_1k_from_pointcloud(self):
        tmp_pc = self.noiseless_point_cloud
        self.voxel_down_sample(1/10)
        self.voxel_down_sample(2/10)
        voxel_size = 2
        while np.asarray(tmp_pc.points).shape[0]>500:
            if np.asarray(self.sample_point_clouds_tmp[-2].points).shape[0]>100000 and np.asarray(self.sample_point_clouds_tmp[-1].points).shape[0]<100000:
                self.sample_point_clouds_from_point_cloud.append(self.sample_point_clouds_tmp[-2])
            if np.asarray(self.sample_point_clouds_tmp[-2].points).shape[0]>10000 and np.asarray(self.sample_point_clouds_tmp[-1].points).shape[0]<10000:
                self.sample_point_clouds_from_point_cloud.append(self.sample_point_clouds_tmp[-2])
            if np.asarray(self.sample_point_clouds_tmp[-2].points).shape[0]>1000 and np.asarray(self.sample_point_clouds_tmp[-1].points).shape[0]<1000:
                self.sample_point_clouds_from_point_cloud.append(self.sample_point_clouds_tmp[-2])
            self.voxel_down_sample(voxel_size/10)
            tmp_pc = self.sample_point_clouds_tmp[-1]
            voxel_size+=1
    
    def voxel_down_sample(self,voxel_size):
        '''
        voxel_size (float) – Voxel size to downsample into.

        Function to downsample input pointcloud into output pointcloud with a voxel
        '''
        self.sample_point_clouds_tmp.append(self.noiseless_point_cloud.voxel_down_sample(voxel_size))

    def make_annotation(self):
        for anno_id,(target) in enumerate(self.segmentation_point_cloud):
            vt = self.sample_point_clouds_from_mesh[0].compute_point_cloud_distance(target)
            indices = []
            for i,(ky) in enumerate(vt):
                if ky<0.2:
                    indices.append(i)
                    self.annotation_dict[i] = anno_id
            pt = self.sample_point_clouds_from_mesh[0].select_down_sample(indices)
            # o3d.visualization.draw_geometries([pt])         
           


    def save_ply(self):
        green_print('Saving!')
        save_floder_name = self.folder_path.split('/')[-1]
        if os.path.exists(os.path.join(self.save_path,save_floder_name)):
            red_print(f'{os.path.join(self.save_path,save_floder_name)} already exists')
        os.mkdir(os.path.join(self.save_path,save_floder_name))
        o3d.io.write_point_cloud(os.path.join(self.save_path,save_floder_name,'noiseless.ply'),self.noiseless_point_cloud)
        o3d.io.write_point_cloud(os.path.join(self.save_path,save_floder_name,'clean_100k.ply'),self.sample_point_clouds_from_mesh[0])
        o3d.io.write_point_cloud(os.path.join(self.save_path,save_floder_name,'clean_10k.ply'),self.sample_point_clouds_from_mesh[1])
        o3d.io.write_point_cloud(os.path.join(self.save_path,save_floder_name,'clean_1k.ply'),self.sample_point_clouds_from_mesh[2])
        o3d.io.write_point_cloud(os.path.join(self.save_path,save_floder_name,'real_100k.ply'),self.sample_point_clouds_from_point_cloud[0])
        o3d.io.write_point_cloud(os.path.join(self.save_path,save_floder_name,'real_10k.ply'),self.sample_point_clouds_from_point_cloud[1])
        o3d.io.write_point_cloud(os.path.join(self.save_path,save_floder_name,'real_1k.ply'),self.sample_point_clouds_from_point_cloud[2])
        o3d.io.write_triangle_mesh(os.path.join(self.save_path,save_floder_name,'mesh.ply'),self.reconstructed_mesh)

        f = open(os.path.join(self.save_path,save_floder_name,'annotation.txt'),'w',encoding='utf-8')		#以'w'方式打开文件
        for k,v in self.annotation_dict.items():			# 遍历字典中的键值
            f.write(f'{k} {v}\n')	
        f.close()		

        self.wirte_info(os.path.join(self.save_path,save_floder_name))
        green_print('Done!')
    
    def move_to_origin(self):
        self.noiseless_point_cloud = self.noiseless_point_cloud.translate(self.translate_matrix)
        self.sample_point_clouds_from_mesh[0] = self.sample_point_clouds_from_mesh[0].translate(self.translate_matrix)
        self.sample_point_clouds_from_mesh[1] = self.sample_point_clouds_from_mesh[1].translate(self.translate_matrix)
        self.sample_point_clouds_from_mesh[2] = self.sample_point_clouds_from_mesh[2].translate(self.translate_matrix)
        self.sample_point_clouds_from_point_cloud[0] = self.sample_point_clouds_from_point_cloud[0].translate(self.translate_matrix)
        self.sample_point_clouds_from_point_cloud[1] = self.sample_point_clouds_from_point_cloud[1].translate(self.translate_matrix)
        self.sample_point_clouds_from_point_cloud[2] = self.sample_point_clouds_from_point_cloud[2].translate(self.translate_matrix)
        self.reconstructed_mesh = self.reconstructed_mesh.translate(self.translate_matrix)

    def run(self):
        green_print('Processing:')
        green_print(self.folder_path)
        self.load_point_cloud()
        logging.info(self.folder_path)
        green_print('Remove noise')
        logging.info('Remove noise')
        try:
            start = datetime.datetime.now()
            self.remove_noise()
            cost_time =(datetime.datetime.now()-start).seconds
            green_print(f'Remove noise finished, cost {cost_time}s')
            logging.info(f'Remove noise finished, cost {cost_time}s')
        except:
            red_print("Remove noise error")
            logging.warning("Remove noise error")

        green_print('Reconstruct mesh')
        logging.info('Reconstruct mesh')
        try:
            start = datetime.datetime.now()
            self.reconstruct_mesh()
            cost_time =(datetime.datetime.now()-start).seconds
            green_print(f'Reconstruct mesh finished, cost {cost_time}s')
            logging.info(f'Reconstruct mesh finished, cost {cost_time}s')
            # o3d.visualization.draw_geometries([self.reconstructed_mesh])
        except:
            red_print("Reconstruct mesh error")
            logging.warning("Reconstruct mesh error")

        green_print('Down sampling from mesh')
        try:
            start = datetime.datetime.now()
            self.down_sample_to_100k_10k_1k_from_mesh()
            green_print(f'Down sampling from mesh finished, cost {(datetime.datetime.now()-start).seconds}s')
            logging.info(f'Down sampling from mesh finished, cost {(datetime.datetime.now()-start).seconds}s')
        except:
            red_print('Down sampling from mesh error')
            logging.warning("Down sampling from mesh error")

        green_print('Down sampling from point cloud')
        try:
            start = datetime.datetime.now()
            self.down_sample_to_100k_10k_1k_from_pointcloud()
            green_print(f'Down sampling from point cloud finished, cost {(datetime.datetime.now()-start).seconds}s')
            logging.info(f'Down sampling from point cloud finished, cost {(datetime.datetime.now()-start).seconds}s')
        except:
            red_print('Down sampling error')
            logging.warning("Down sampling error")
        
        try:
            self.make_annotation()
        except:
            red_print('Make annotation error')
            logging.warning("Make annotation error")
        
        try:
            self.move_to_origin()
        except:
            red_print('Move to origin error')
            logging.warning("Move to origin error")
        
        self.print_info()

        try:
            self.save_ply()
        except:
            red_print('Save ply error')
            logging.warning("Save ply error")

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-s', type=str,help='src',default='data')
    parser.add_argument('-d', type=str,help='dst',default='res')
    args = parser.parse_args()
    dst_path = args.d 
    src_path = args.s 

    src_floder = []
    dst_floder = []
    count = 0
    for file in os.listdir(f'{dst_path}'):
        if file=='.DS_Store':
            continue
        dst_floder.append(file)

    for file in os.listdir(f'{src_path}'):
        if file=='.DS_Store':
            continue
        src_floder.append(file)

    blue_print(f'Existing files: {dst_floder}')
    blue_print(f'{len(dst_floder)} files already exist, skip!')

    blue_print(f'{len(src_floder)-len(dst_floder)} files are ready to go')

    start = datetime.datetime.now()
    for file in src_floder:
        if file in dst_floder:
            continue
        # rp = RPCDPrepreocess('./data/2-l.ply')
        rp = RPCDPrepreocess(os.path.join(f'{src_path}',file),f'{dst_path}')
        rp.run()
        count+=1
        blue_print(f'Finished {count}/{len(src_floder)-len(dst_floder)}\n')
        blue_print(f'Totally cost {(datetime.datetime.now()-start).seconds}s')


if __name__ == "__main__":
    main()