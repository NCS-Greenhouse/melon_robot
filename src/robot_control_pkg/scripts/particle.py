#
# Created on Sun Dec 10 2023
#
# Copyright (c) 2023 NCS-Greenhouse-Group
#
# Author:ShengDao Du, Email: duchengdao@gmail.com
# Github Page: https://github.com/Runnlion
# Personal Page: https://shengdao.me
#

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from robot_control_pkg.msg import nn_objects
from tqdm import tqdm
from sklearn.cluster import KMeans
from sklearn.metrics import silhouette_score

class ParticleFilter_Single_Object:

    def __init__(self, num_particles):
        self.num_particles = num_particles
        self.region_limits = None
        # self.particles = self.initialize_particles()
        self.bounding_generating_iter = 0
        self.MAXIMUM_BBOX_ITER = 3
        # Additional variables for storing current ranges
        self.field_names=("x", "y", "z")
        self.pp_iteration = 10
        # self.object_sub = rospy.Subscriber('melon_result_pc',PointCloud2,callback=particle_filter.object_points_callback)

        
    def initialize_particles(self):
        min_limits, max_limits = self.region_limits
        particles = np.random.uniform(min_limits, max_limits, size=(self.num_particles, 3))
        return particles

    def motion_model(self, particles, previous_positions, noise_scale=0.5):
        adjustments = noise_scale * np.random.randn(*particles.shape)
        predicted_positions = previous_positions + adjustments
        return predicted_positions

    def update_particles_from_pointcloud(self,particles, P_petiole):
        weights = np.zeros(len(particles))

        for i, particle in enumerate(particles):
            distances = np.linalg.norm(particle - P_petiole, axis=1)
            min_distance = np.min(distances)
            weights[i] = 1 / (min_distance + 1e-6)

        weights /= np.sum(weights)
        return weights

    def visualize_particles_with_weights(self, particles, weights, point_cloud=None):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        scaled_weights = 100 * weights
        scatter = ax.scatter(particles[:, 0], particles[:, 1], particles[:, 2], c=scaled_weights, cmap='viridis')
        
        if point_cloud is not None:
            # Visualize the point cloud in red
            ax.scatter(point_cloud[:, 0], point_cloud[:, 1], point_cloud[:, 2], c='red', marker='o', label='Point Cloud')
        
        cbar = plt.colorbar(scatter)
        cbar.set_label('Particle Weights')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_xlim([self.region_limits[0][0], self.region_limits[1][0]])
        ax.set_ylim([self.region_limits[0][1], self.region_limits[1][1]])
        ax.set_zlim([self.region_limits[0][2], self.region_limits[1][2]])
        ax.set_title('Particle Visualization with Weights')
        
        # Add a legend
        ax.legend()

        plt.show()


    def resample_particles(self,particles, weights, num_random_points=20, noise_scale=0.003):
        num_particles = len(particles)
        indices = np.arange(num_particles)
        resampled_indices = np.random.choice(indices, size=num_particles, replace=True, p=weights)
        resampled_particles = particles[resampled_indices]
        noise = noise_scale * np.random.randn(*resampled_particles.shape)
        resampled_particles += noise
        resampled_weights = np.ones(num_particles) / num_particles

        random_points = self.generate_random_points_3d(num_random_points)
        resampled_particles = np.vstack([resampled_particles, random_points])
        resampled_weights = np.concatenate([resampled_weights, np.ones(num_random_points) / (num_particles + num_random_points)])

        return resampled_particles, resampled_weights

    def _get_pointcloud(self)->np.ndarray:
        msg:PointCloud2 = PointCloud2()
        try:
            msg:PointCloud2 = rospy.wait_for_message('melon_result_pc',PointCloud2,timeout=5)
            cloud = pc2.read_points(msg, field_names=self.field_names, skip_nans=True)
            points = np.array(list(cloud), dtype=np.float32)
            return points
        except:
            return None

    def generate_random_points_3d(self, num_points):
        x_limit, y_limit, z_limit = zip(*self.region_limits)
        x_coordinates = np.random.uniform(low=x_limit[0], high=x_limit[1], size=num_points)
        y_coordinates = np.random.uniform(low=y_limit[0], high=y_limit[1], size=num_points)
        z_coordinates = np.random.uniform(low=z_limit[0], high=z_limit[1], size=num_points)
        coordinates = np.array([x_coordinates, y_coordinates, z_coordinates]).T
        return coordinates

    def generate_bounding_box(self, expanding_factor = 1.5)->bool:
        self.current_x_range = [0.,0.]
        self.current_y_range = [0.,0.]
        self.current_z_range = [0.,0.]
        while(self.bounding_generating_iter < self.MAXIMUM_BBOX_ITER):
            points =self._get_pointcloud()
            if(points is None): return False

            print("Received {} points.".format(len(points)))
            # Update current ranges based on the received points
            self.current_x_range[0] += np.min(points[:, 0])
            self.current_x_range[1] += np.max(points[:, 0])

            self.current_y_range[0] += np.min(points[:, 1])
            self.current_y_range[1] += np.max(points[:, 1])

            self.current_z_range[0] += np.min(points[:, 2])
            self.current_z_range[1] += np.max(points[:, 2])
            self.bounding_generating_iter += 1
            # del msg
        else:
            # Increment bounding box generation iteration
            print("Iter=",self.bounding_generating_iter)
            # Calculate average limitations
            avg_x_limit_l = np.mean(self.current_x_range[0])/self.MAXIMUM_BBOX_ITER
            avg_y_limit_l = np.mean(self.current_y_range[0])/self.MAXIMUM_BBOX_ITER
            avg_z_limit_l = np.mean(self.current_z_range[0])/self.MAXIMUM_BBOX_ITER
            avg_x_limit_u = np.mean(self.current_x_range[1])/self.MAXIMUM_BBOX_ITER 
            avg_y_limit_u = np.mean(self.current_y_range[1])/self.MAXIMUM_BBOX_ITER 
            avg_z_limit_u = np.mean(self.current_z_range[1])/self.MAXIMUM_BBOX_ITER
            margin_x = abs(avg_x_limit_u - avg_x_limit_l)/2. * expanding_factor
            margin_y = abs(avg_y_limit_u - avg_y_limit_l)/2. * expanding_factor
            margin_z = abs(avg_z_limit_u - avg_z_limit_l)/2. * expanding_factor
            avg_x_limit_l -= margin_x
            avg_y_limit_l -= margin_y
            avg_z_limit_l -= margin_z
            avg_x_limit_u += margin_x
            avg_y_limit_u += margin_y
            avg_z_limit_u += margin_z

            self.region_limits = ((avg_x_limit_l,avg_y_limit_l,avg_z_limit_l),
                                (avg_x_limit_u,avg_y_limit_u,avg_z_limit_u))
            rospy.loginfo("Region Limits Calculated:%s",str(self.region_limits))
            return True
        
    def run(self):
        particles = self.initialize_particles()
        for i in tqdm(range(self.pp_iteration)):
            # Example usage:
            previous_positions = particles  # Assuming particles have been initialized
            predicted_positions = self.motion_model(particles, previous_positions, noise_scale=0.3)

            # Update particle weights
            # For group
            objective = self._get_pointcloud()
            weights = self.update_particles_from_pointcloud(particles, objective)
            if(i <= self.pp_iteration - 1):
                resampled_particles, new_weights = self.resample_particles(particles, weights,num_random_points=0)
            else:
                continue
            # self.visualize_particles_with_weights(resampled_particles,weights, objective)

            particles = resampled_particles
            weights = new_weights
        self.visualize_particles_with_weights(resampled_particles,weights, objective)

class ParticleFilter_Multiple_Objects:

    def __init__(self, num_particles):
        self.num_particles = num_particles
        self.region_limits = None
        self.bounding_generating_iter = 0
        self.MAXIMUM_BBOX_ITER = 5
        # Additional variables for storing current ranges
        self.field_names=("x", "y", "z")
        self.pp_iteration = 10
        # self.object_sub = rospy.Subscriber('melon_result_pc',PointCloud2,callback=particle_filter.object_points_callback)
        self.group_ranges = []
        
    def initialize_particles(self, region_limits):
        min_limits, max_limits = region_limits
        particles = np.random.uniform(min_limits, max_limits, size=(self.num_particles, 3))
        return particles

    def motion_model(self, particles, previous_positions, noise_scale=0.5):
        adjustments = noise_scale * np.random.randn(*particles.shape)
        predicted_positions = previous_positions + adjustments
        return predicted_positions

    def update_particles_from_pointcloud(self,particles, P_petiole):
        weights = np.zeros(len(particles))

        for i, particle in enumerate(particles):
            distances = np.linalg.norm(particle - P_petiole, axis=1)
            min_distance = np.min(distances)
            weights[i] = 1 / (min_distance + 1e-6)

        weights /= np.sum(weights)
        return weights
    def visualize_particles(self, particles):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        # fig, axes = plt.subplots(1, len(self.group_ranges), subplot_kw={'projection': '3d'}, figsize=(15, 5))
        # for i, ax in enumerate(axes):
        for particle in particles:
            ax.scatter(particle[:, 0], particle[:, 1], particle[:, 2])
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.legend()
        plt.show()
    
    def visualize_particles_with_weights(self, particles, weights, region_limits,  point_cloud=None):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        scaled_weights = 100 * weights
        scatter = ax.scatter(particles[:, 0], particles[:, 1], particles[:, 2], c=scaled_weights, cmap='viridis')
        
        if point_cloud is not None:
            # Visualize the point cloud in red
            ax.scatter(point_cloud[:, 0], point_cloud[:, 1], point_cloud[:, 2], c='red', marker='o', label='Point Cloud')
        
        cbar = plt.colorbar(scatter)
        cbar.set_label('Particle Weights')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_xlim([region_limits[0][0], region_limits[1][0]])
        ax.set_ylim([region_limits[0][1], region_limits[1][1]])
        ax.set_zlim([region_limits[0][2], region_limits[1][2]])
        ax.set_title('Particle Visualization with Weights')
        
        # Add a legend
        ax.legend()

        plt.show()


    def resample_particles(self,particles, weights, region, num_random_points=20, noise_scale=0.003):
        num_particles = len(particles)
        indices = np.arange(num_particles)
        resampled_indices = np.random.choice(indices, size=num_particles, replace=True, p=weights)
        resampled_particles = particles[resampled_indices]
        noise = noise_scale * np.random.randn(*resampled_particles.shape)
        resampled_particles += noise
        resampled_weights = np.ones(num_particles) / num_particles

        random_points = self.generate_random_points_3d(num_random_points,region)
        resampled_particles = np.vstack([resampled_particles, random_points])
        resampled_weights = np.concatenate([resampled_weights, np.ones(num_random_points) / (num_particles + num_random_points)])

        return resampled_particles, resampled_weights

    def _get_pointcloud(self, obj_class = 0)->[np.ndarray]:
        msg:nn_objects = nn_objects()
        try:
            msg:nn_objects = rospy.wait_for_message('yolo_results',nn_objects,timeout=5)
            object_points = []
            for i in range(len(msg.object_clouds)):
                obj_pointcloud = msg.object_clouds[i]
                if(msg.object_classes[i].data == obj_class or obj_class == -1):
                    cloud = pc2.read_points(obj_pointcloud, field_names=self.field_names, skip_nans=True)
                    points = np.array(list(cloud), dtype=np.float32)
                    object_points.append(points)
            return object_points
        except:
            return None

    def generate_random_points_3d(self, num_points, region):
        x_limit, y_limit, z_limit = zip(*region)
        x_coordinates = np.random.uniform(low=x_limit[0], high=x_limit[1], size=num_points)
        y_coordinates = np.random.uniform(low=y_limit[0], high=y_limit[1], size=num_points)
        z_coordinates = np.random.uniform(low=z_limit[0], high=z_limit[1], size=num_points)
        coordinates = np.array([x_coordinates, y_coordinates, z_coordinates]).T
        return coordinates

    def generate_bounding_box(self, expanding_factor = 1.5)->bool:

        #collect the incoming data first
        rospy.loginfo("Collecting Detected Object Point Cloud.")
        detected_object_points_list = []
        for i in tqdm(range(self.MAXIMUM_BBOX_ITER)):
            cloud_list = self._get_pointcloud(obj_class=0)
            for cloud in cloud_list:
                detected_object_points_list.append(cloud)
        centroids = np.array([])
        for cloud in tqdm(detected_object_points_list):
            centroid = np.mean(cloud,axis=0)
            centroids = np.append(centroids, centroid, axis=0)
        centroids = centroids.reshape(-1, 3)
        # Try different values of n_clusters and calculate silhouette scores
        silhouette_scores = []
        for n_clusters in range(2, 5):
            kmeans = KMeans(n_clusters=n_clusters)
            kmeans.fit(centroids)
            labels = kmeans.labels_
            silhouette_avg = silhouette_score(centroids, labels)
            silhouette_scores.append(silhouette_avg)

        # Find the optimal number of clusters with the highest silhouette score
        optimal_n_clusters = np.argmax(silhouette_scores) + 2  # Add 2 because the loop starts from 2 clusters
        print(optimal_n_clusters)

        kmeans = KMeans(n_clusters=optimal_n_clusters)
        kmeans.fit(centroids)

        cluster_centers = kmeans.cluster_centers_
        self.cluster_centers = kmeans.cluster_centers_
        labels = kmeans.labels_
        self.pp_group_num = optimal_n_clusters

        print(cluster_centers, labels)
        
        for i in range(optimal_n_clusters):
            cluster_clouds = np.array(detected_object_points_list,dtype=object)[labels==i]
            current_x_range = [0.,0.]
            current_y_range = [0.,0.]
            current_z_range = [0.,0.]
            for cloud in cluster_clouds:
                current_x_range[0] += np.min(cloud[:, 0])
                current_x_range[1] += np.max(cloud[:, 0])
                current_y_range[0] += np.min(cloud[:, 1])
                current_y_range[1] += np.max(cloud[:, 1])
                current_z_range[0] += np.min(cloud[:, 2])
                current_z_range[1] += np.max(cloud[:, 2])
            avg_x_limit_l = np.mean(current_x_range[0])/self.MAXIMUM_BBOX_ITER
            avg_y_limit_l = np.mean(current_y_range[0])/self.MAXIMUM_BBOX_ITER
            avg_z_limit_l = np.mean(current_z_range[0])/self.MAXIMUM_BBOX_ITER
            avg_x_limit_u = np.mean(current_x_range[1])/self.MAXIMUM_BBOX_ITER 
            avg_y_limit_u = np.mean(current_y_range[1])/self.MAXIMUM_BBOX_ITER 
            avg_z_limit_u = np.mean(current_z_range[1])/self.MAXIMUM_BBOX_ITER
            margin_x = abs(avg_x_limit_u - avg_x_limit_l)/2. * expanding_factor
            margin_y = abs(avg_y_limit_u - avg_y_limit_l)/2. * expanding_factor
            margin_z = abs(avg_z_limit_u - avg_z_limit_l)/2. * expanding_factor
            avg_x_limit_l -= margin_x
            avg_y_limit_l -= margin_y
            avg_z_limit_l -= margin_z
            avg_x_limit_u += margin_x
            avg_y_limit_u += margin_y
            avg_z_limit_u += margin_z
            self.group_ranges.append(((avg_x_limit_l,avg_y_limit_l,avg_z_limit_l),
                                (avg_x_limit_u,avg_y_limit_u,avg_z_limit_u)))
        rospy.loginfo("Region Limits Calculated:%s",str(self.group_ranges))
        
    def run(self):
        self.particles = []
        iteration = []
        for region in self.group_ranges:
            self.particles.append(self.initialize_particles(region)) 
            iteration.append(0)

        # print("here")
        # particles = self.initialize_particles()
        for i in tqdm(range(self.pp_iteration)):
            object_clouds = self._get_pointcloud()
            for cloud in object_clouds:
                #make sure which group
                label = -1
                c = np.mean(cloud,axis=0)
                for j in range(len(self.cluster_centers)):
                    centroid = self.cluster_centers[j]
                    dist = np.linalg.norm(c - centroid)
                    if(dist < 0.01): 
                        label = j
                        break
                # print("Label:",label)

                # Example usage:
                previous_positions = self.particles[label]  # Assuming particles have been initialized
                predicted_positions = self.motion_model(self.particles[label], previous_positions, noise_scale=0.3)

                weights = self.update_particles_from_pointcloud(self.particles[label], cloud)

                if(i <= self.pp_iteration - 1):
                    resampled_particles, new_weights = self.resample_particles(self.particles[label], weights,self.group_ranges[label],num_random_points=0)
                else:
                    continue
                # self.visualize_particles_with_weights(resampled_particles,weights, self.group_ranges[label], cloud)

                self.particles[label] = resampled_particles
            # return
        
            # Update particle weights
            # For group
            # objective = self._get_pointcloud()
            # weights = self.update_particles_from_pointcloud(particles, objective)
            # if(i <= self.pp_iteration - 1):
            #     resampled_particles, new_weights = self.resample_particles(particles, weights,num_random_points=0)
            # else:
            #     continue
            # # self.visualize_particles_with_weights(resampled_particles,weights, objective)

            # particles = resampled_particles
            # weights = new_weights
        # self.visualize_particles_with_weights(resampled_particles,weights, objective)



rospy.init_node('particle_filter',anonymous=False)
# iteration = 20
# num_particles = 1000
# expanded_region_limits = ((-1., -1., 0.), (1.0, 1.0,  1.8)) # Adjust these limits based on your scenario
# particle_filter = ParticleFilter_Single_Object(num_particles)
# if(particle_filter.generate_bounding_box(expanding_factor=1.2)):
#     particle_filter.run()
#     pass

ppf = ParticleFilter_Multiple_Objects(1000)
ppf.generate_bounding_box()
ppf.run()
ppf.visualize_particles(ppf.particles)
# print(ppf._get_pointcloud())
# rospy.spin()

