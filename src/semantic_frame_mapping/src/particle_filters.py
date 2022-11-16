#! /usr/bin/python

from random import gauss, random

import rospy
import numpy as np
import copy
import math
import actionlib
# import PyKDL
# import tf

from scipy.stats import multivariate_normal
from scipy.spatial.transform import Rotation as R
from sklearn.mixture import GaussianMixture, BayesianGaussianMixture
from std_msgs.msg import Bool
from threading import RLock

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, PointStamped
from std_msgs.msg import Bool
from semantic_frame_mapping.msg import ObjectDetection
from apriltag_ros.msg import AprilTagDetectionArray



class StaticObject(object):
    def __init__(self, label, x, y, z):
        self.label = label
        self.x = x
        self.y = y
        self.z = z
        self.marker_pub = rospy.Publisher('/filter/static_object/{}'.format(label), MarkerArray, queue_size=10)

        self.marker_array = MarkerArray()
        self.marker = Marker()

        self.marker.header.frame_id = 'map'
        self.marker.id = 0
        self.marker.type = self.marker.CUBE
        self.marker.action = self.marker.ADD
        self.marker.scale.x = 0.2
        self.marker.scale.y = 0.2
        self.marker.scale.z = 0.2
        self.marker.color.a = 1
        self.marker.pose.orientation.w = 1.0
        self.marker.pose.position.x = self.x
        self.marker.pose.position.y = self.y
        self.marker.pose.position.z = self.z

        self.marker_array.markers.append(self.marker)
    
    def publish(self):
        self.marker_pub.publish(self.marker_array)
    
    def update_position(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
        self.marker.pose.position.x = self.x
        self.marker.pose.position.y = self.y
        self.marker.pose.position.z = self.z

class ParticleFilter(object):
    def __init__(self, n, label, valid_regions):
        self.n = n
        self.Sigma = np.array([[0.02, 0, 0], [0, 0.02, 0], [0, 0, 0.0]])
        self.particles = np.zeros([n,3])
        self.weights = 1.0/self.n * np.ones([n])
        self.valid_regions = {}
        self.negative_regions = []
        try:
            for region, weight in valid_regions.items():
                self.valid_regions[region] = weight
        except:
            # rospy.logwarn("{} has no valid reigons".format(label))
            self.valid_regions = None
        rospy.logwarn(label)
        try:
            for region, weight in self.valid_regions.items():
                rospy.logwarn("Region_id: {}, weight: {}, bounds:\n{}".format(region.name, weight, region.get_bounds()))
        except:
            pass

        self.lock = RLock()
        self.reinvigoration_idx = 0
        self.label = label
        for i in range(self.n):
            self.particles[i] = self.reinvigorate(self.particles[i])
        self.bgm = BayesianGaussianMixture(n_components=20, n_init=10, warm_start=False)
    
    def add_negative_region(self, region):
        self.negative_regions.append(region)
    
    def bgmm(self):
        with self.lock:
            bgm = self.bgm.fit(self.particles)
            return bgm.means_, bgm.covariances_, bgm.weights_
        
    def reinvigorate(self, particle, valid_region=False):
        # if self.valid_regions is None:
            # Random resample in entire map
        particle[0] = np.random.uniform(-3, 11.5)
        # -9 + np.random.random()*10
        particle[1] = np.random.uniform(-3, 10)
        # -5 + np.random.random()*6
        particle[2] = np.random.uniform(0, 2)# np.random.uniform(0, 3)
        # np.random.random()*2
        # else:
        #     # Random resample in a valid region
        #     r = np.random.random()
        #     prev_bounds = [0]
        #     for region, weight in self.valid_regions.items():
        #         if r >= 1 - weight - sum(prev_bounds):
        #             # region = self.valid_regions[self.reinvigoration_idx % len(self.valid_regions)]
        #             min_x, max_x, min_y, max_y, min_z, max_z = region.get_bounds()
        #             particle[0] = min_x + np.random.random()*(max_x - min_x)
        #             particle[1] = min_y + np.random.random()*(max_y - min_y)
        #             particle[2] = min_z + np.random.random()*(max_z - min_z)
        #             # self.reinvigoration_idx+=1                   
        return particle

    def systematic_resample(self):
        """ Performs the systemic resampling algorithm used by particle filters.

        This algorithm separates the sample space into N divisions. A single random
        offset is used to to choose where to sample from for all divisions. This
        guarantees that every sample is exactly 1/N apart.

        Parameters
        ----------
        weights : list-like of float
            list of weights as floats

        Returns
        -------

        indexes : ndarray of ints
            array of indexes into the weights defining the resample. i.e. the
            index of the zeroth resample is indexes[0], etc.
        """
        N = len(self.weights)

        # make N subdivisions, and choose positions with a consistent random offset
        positions = (np.random.random() + np.arange(N)) / N

        indexes = np.zeros(N, 'i')
        cumulative_sum = np.cumsum(self.weights)
        i, j = 0, 0
        while i < N:
            if positions[i] < cumulative_sum[j]:
                indexes[i] = j
                i += 1
            else:
                j += 1
        return indexes

    def resample(self):
        # Low Variance Resampling
        # particles_added = 0
        # temp_p = np.zeros([self.n, 3])
        # w = np.cumsum(self.weights)
        # r = np.random.rand(1) / self.n
        # j = 1
        # for i in range(self.n):
        #     u = r + (i-1) / self.n
        #     while u > w[j]:
        #         j += 1
        #     temp_p[i] = self.particles[j]
        #     particles_added +=1
        #     if particles_added > self.n*0.9:
        #         break
        # for i in range(particles_added, self.n):
        #     temp_p[i] = self.reinvigorate()
        # self.particles = copy.deepcopy(temp_p)
        # self.weights = 1/self.n * np.ones([self.n])

        ## all particles over 1/n weight get resampled all less than get reinvigorated
        count = 0
        temp_p = self.particles[:]
        temp_w = self.weights[:]
        for i in range(self.n):
            if temp_w[i] > float(1/self.n) and count < int(self.n*0.8):
                self.particles[i] = copy.deepcopy(temp_p[i])
                self.weights[i] = copy.deepcopy(temp_w[i])
                count += 1
            else:
                self.particles[i] = self.reinvigorate(copy.deepcopy(temp_p[i]))
                self.weights[i] = float(1/self.n)

        
        ## Some other resampling technique
        # self.particles[0, :] = copy.deepcopy(temp_p[np.argmax(self.weights)])
        # self.weights[0] = copy.deepcopy(temp_w[np.argmax(self.weights)])
        # self.particles[1, :] = self.jitter(copy.deepcopy(temp_p[np.argmax(self.weights)]))
        # self.weights[1] = copy.deepcopy(temp_w[np.argmax(self.weights)])
        # for i in range(len(draw)):
        #     self.particles[i+2] = copy.deepcopy(temp_p[draw[i]])
        #     self.weights[i+2] = copy.deepcopy(temp_w[draw[i]])

        # for j in range(2+len(draw), self.n):
        #     self.particles[j] = self.reinvigorate(self.particles[j])
        #     self.weights[j] = 1/self.n

        
    
    def jitter(self, particle):
        x_noise = np.random.normal(0, self.Sigma[0,0])
        y_noise = np.random.normal(0, self.Sigma[1,1])
        # z_noise = np.random.normal(0, self.Sigma[2,2])
        particle[0] += x_noise
        particle[1] += y_noise
        # particle[2] += z_noise
        return particle

    def update_filter(self):
        for k in range(self.n):
            self.particles[k, :]= self.jitter(self.particles[k, :])
            self.weights[k] = self.assign_weight(self.particles[k, :])
        self.weights = self.weights / np.sum(self.weights)
        self.resample()
    
    def publish(self):
        marker_array = MarkerArray()
        # highest_weight = max(self.weights)
        max_weight = max(self.weights)
        min_weight = min(self.weights)
        # rospy.logwarn(sum(self.weights))
        # if self.label == 'grasp_cup' or self.label == 'grasp_spoon' or self.label == 'stir_cup':
        #     rospy.logwarn("{}: max: {:.6f}, min: {:.6f}".format(self.label, max(self.weights), min(self.weights)))
        # rospy.logwarn("{}: {}".format(self.label, sum(self.weights)))
        # assert(sum(self.weights) == 1.0)
        # lowest_weight = min(self.weights)
        # threshold = lowest_weight + float((highest_weight-lowest_weight)*0.8)
        for i in range(self.n):
            marker = Marker()
            marker.id = i
            marker.header.frame_id = 'map'
            if self.label == 'cup':
                # red cube
                marker.color.r = 1
                marker.type = marker.CUBE
            elif self.label == 'spoon':
                # blue cube
                marker.color.b = 1
                marker.type = marker.CUBE
            elif self.label == 'bowl':
                # green cube
                marker.color.g = 1
                marker.type = marker.CUBE
            elif self.label == 'grasp_cup':
                # red sphere
                marker.color.r = 1
                marker.type = marker.SPHERE
            elif self.label == 'grasp_spoon':
                # blue sphere
                marker.color.b = 1
                marker.type = marker.SPHERE
            elif self.label == 'grasp_bowl':
                # green sphere
                marker.color.g = 1
                marker.type = marker.SPHERE
            elif self.label == 'pour_bowl':
                marker.color.r = 1
                marker.color.g = 1
                marker.type = marker.SPHERE  
            elif self.label == 'stir_bowl':
                marker.color.b = 1
                marker.color.g = 1
                marker.type = marker.SPHERE
            elif self.label == 'stir_cup':
                marker.color.b = 1
                marker.color.r = 1
                marker.type = marker.SPHERE
            elif self.label == 'token':
                marker.color.r = 1
                marker.color.g = 1
                marker.type = marker.SPHERE
                    
            
            marker.action = marker.ADD
            # marker.lifetime = rospy.Duration(10)
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 1
            # a = min(round((self.weights[i] - min_weight) / (max_weight - min_weight), 2) + 0.45, 1.0)
            # if a < 0.6:
            #     marker.color.a = 0
            # else:
            #     marker.color.a = a
            # marker.color.a = 0.7
            # if self.weights[i] <= float(highest_weight/4):
            #     marker.color.a = 0.25
            # if self.weights[i] <= float(highest_weight/2):
            #     marker.color.a = 0.25
            # elif self.weights[i] <= float(3*highest_weight/4):
            #     marker.color.a = 0.75
            # else:
            #     marker.color.a = 1.0
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = self.particles[i, 0]
            marker.pose.position.y = self.particles[i, 1]
            marker.pose.position.z = self.particles[i, 2]
            marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)

class ObjectParticleFilter(ParticleFilter):
    def __init__(self, n, valid_regions, label):
        super(ObjectParticleFilter, self).__init__(n, label, valid_regions)
        self.ar_to_obj_map = {0: 'mug', 1:'elevator_button'}
        self.observation_sub = rospy.Subscriber('scene/observations', ObjectDetection, self.add_observation, queue_size=1)
        self.apriltag_sub = rospy.Subscriber('tag_detections', AprilTagDetectionArray, self.handle_ar_detection, queue_size=1)
        
        self.marker_pub = rospy.Publisher('filter/particles/{}'.format(label), MarkerArray, queue_size=10)
        self.observations = []
        # if self.label == 'spoon':
        #     # 'dining_room': (4.5, 8.5, 0.5, 3.5, 0, 1.5),
        #     dummy_obs = StaticObject('spoon', 6.5, 2, 0.7)
        #     self.observations.append(dummy_obs)

        # if self.label == 'bottle':
        #     # 'kitchen': (6.5, 9.0, -5, 0, 0, 1.5),
        #     dummy_obs = StaticObject('bottle', 7, -3, 0.79)
        #     self.observations.append(dummy_obs)
        # if self.label == 'bowl':
        #     # 'kitchen': (6.5, 9.0, -5, 0, 0, 1.5),
        #     dummy_obs = StaticObject('bowl', 8, -1, 0.79)
        #     self.observations.append(dummy_obs)
        # elif self.label == 'mug':
        #     dummy_obs = StaticObject('mug', 2.48213674403, 2.11895497563, 0.798618733883 )
        #     other_mug = StaticObject('mug_2', 8.5, -4.0, 0.798618733883)
        #     self.observations.append(other_mug)
        #     self.observations.append(dummy_obs)
    
    def publish(self):
        super(ObjectParticleFilter, self).publish()
        for observation in self.observations:
            observation.publish()

    def add_valid_region(self, region, weight):
        # rospy.logwarn("Adding {} to valid regions ".format(region.name))
        self.valid_regions[region] = weight
    
    def remove_valid_region(self, region):
        del self.valid_regions[region]
    
    def handle_ar_detection(self, msg):
        return
        for detection in msg.detections:
            obj_label = self.ar_to_obj_map[detection.id[0]]
            if obj_label == self.label:
                new_obj = True
                for obs in self.observations:
                    if obs.label == obj_label:
                        obs.update_position(detection.pose.pose.pose.position.x, detection.pose.pose.pose.position.y, detection.pose.pose.pose.position.z)
                        new_obj = False
                if new_obj:
                    obj = StaticObject(obj_label, detection.pose.pose.pose.position.x, detection.pose.pose.pose.position.y, detection.pose.pose.pose.position.z)
                    self.observations.append(obj)
            
    def add_observation_from_config(self, x, y, z):
        self.observations.append(StaticObject(self.label, x, y, z))
    
    def add_observation_from_scene(self, obj_label, x, y, z):
        if obj_label == self.label:
            rospy.logwarn("{}_pf: Received observation at ({}, {}, {})".format(self.label, x, y, z))
            self.observations.append(StaticObject(obj_label, x, y, z))

    def add_observation(self, observation):
        if observation.type == self.label:
            rospy.loginfo("{}_pf: Received observation at ({}, {}, {})".format(self.label, observation.pose.position.x, observation.pose.position.y, observation.pose.position.z))
            obj = StaticObject(observation.label, observation.pose.position.x, observation.pose.position.y, observation.pose.position.z)
            self.observations.append(obj)
    
    
    def assign_weight(self, particle):
        # for region in self.negative_regions:
        #     min_x, max_x, min_y, max_y, min_z, max_z = region.get_bounds()
        #     if min_x <= particle[0] <= max_x and min_y <= particle[1] <= max_y and min_z <= particle[2] <= max_z:
        #         return 0
        if len(list(self.valid_regions.keys())) == 0:
            # rospy.logwarn_throttle(1, "no valid_regions for {}".format(self.label))
            return 1
        
        region_weight = 1e-3
        for region, weight in self.valid_regions.items():
            min_x, max_x, min_y, max_y, min_z, max_z = region.get_bounds()
            if min_x <= particle[0] <= max_x and min_y <= particle[1] <= max_y and min_z <= particle[2] <= max_z:
                # rospy.logwarn("Particle in region {}".format(i))
                region_weight = weight
                break
        if len(self.observations) == 0:
            return 1 * region_weight
        
        for i, observation in enumerate(self.observations):
            err_x = particle[0] - observation.x
            err_y = particle[1] - observation.y
            err_z = particle[2] - observation.z
            dist = math.sqrt(err_x**2 + err_y**2 + err_z**2)
            phi = math.exp(-100 * dist)
            if i == 0:
                max_phi = phi
            else:
                max_phi = max(max_phi, phi)
        return max_phi + region_weight
    
    def resample(self):
        if len(self.valid_regions.keys()) > 0:
            valid_regions = list(self.valid_regions.keys())
            for i in range(int(self.n*0.8)):
                region = valid_regions[i%len(valid_regions)]
                min_x, max_x, min_y, max_y, min_z, max_z = region.get_bounds()
                self.particles[i,0] = min_x + np.random.random()*(max_x - min_x)
                self.particles[i,1] = min_y + np.random.random()*(max_y - min_y)
                self.particles[i,2] = min_z + np.random.random()*(max_z - min_z)
            for j in range(i, self.n):
                self.particles[j] = self.reinvigorate(self.particles[j])

        elif len(self.observations) != 0:
            obs = self.observations[0]
            for i in range(int(self.n*0.7)):
                self.particles[i, 0] = obs.x
                self.particles[i, 1] = obs.y
                self.particles[i, 2] = obs.z 
            for j in range(i, self.n):
                self.particles[j] = self.reinvigorate(self.particles[j])


        else:
            for i in range(self.n):
                self.particles[i] = self.reinvigorate(self.particles[i])
        return


                
        if len(self.observations) == 0:
            temp_p = self.particles[:]
            temp_w = self.weights[:]
            for i in range(self.n):
                if temp_w[i] > float(1/self.n):
                    self.particles[i] = copy.deepcopy(temp_p[i])
                    self.weights[i] = copy.deepcopy(temp_w[i])
                else:
                    self.particles[i] = self.reinvigorate(copy.deepcopy(temp_p[i]))
                    self.weights[i] = float(1/self.n)
        else:
            # x % of particles evenly between observations
            temp_p = self.particles[:]
            temp_w = self.weights[:]
            # indexes = self.systematic_resample()
            # count = 0
            for i in range(int(self.n*0.4)):
                if self.weights[i] >= float(1.0/self.n):
                    continue
                self.particles[i] = self.reinvigorate(copy.deepcopy(temp_p[i]))
                # self.weights[i] = float(1/self.n)

            for j in range(i, self.n):
                obs = self.observations[j%len(self.observations)]
                self.particles[j,0] = obs.x
                self.particles[j,1] = obs.y
                self.particles[j,2] = obs.z
            assert(self.particles.shape[0] == self.n)
                
            # r = int(self.n * 0.2)
            # for i in range(r):
            #     obs = self.observations[i%len(self.observations)]
            #     self.particles[i, 0] = obs.x
            #     self.particles[i, 1] = obs.y
            #     self.particles[i, 2] = obs.z
            # for i in range(r, self.n):
            #     self.particles[i] = self.reinvigorate(copy.deepcopy(self.particles[i]))
            #     self.weights[i] = float(1/self.n)
            # for i in range(self.n):
            #     self.particles[i, 0] = self.observations[0].x
            #     self.particles[i, 1] = self.observations[0].y
            #     self.particles[i, 2] = self.observations[0].z


    def update_filter(self):
        count = 0
        self.resample()
        for k in range(self.n):
            self.particles[k, :]= self.jitter(self.particles[k, :])
            # weight = self.assign_weight(self.particles[k, :])
            # if weight == 0:
            #     count+=1
            # self.weights[k] = weight
        # rospy.logwarn("{} had {} particles w 0 weight".format(self.label, count))
        # self.weights = self.weights / np.sum(self.weights)
           
class FrameParticleFilter(ParticleFilter):
    def __init__(self, n, label, preconditions, core_frame_elements, valid_regions):
        super(FrameParticleFilter, self).__init__(n, label, valid_regions)
        
        self.marker_pub = rospy.Publisher('filter/particles/{}'.format(label), MarkerArray, queue_size=10)
        self.gauss_pub = rospy.Publisher('filter/gauss/{}'.format(label), MarkerArray, queue_size=10)
        self.frame_element_filters = {frame_element:None for frame_element in core_frame_elements}
        self.frame_elements = core_frame_elements
        if preconditions:
            self.precondition_filters = {precondition:None for precondition in preconditions}
            self.preconditions = preconditions
            self.next_precondition = preconditions[0]
        else:
            self.preconditions = None
        
    def publish(self, gauss_only=False):
        if not gauss_only:
            super(FrameParticleFilter, self).publish()
        arr = MarkerArray()
        means, covs, weights = self.bgmm()
        max_idx = np.argmax(weights)
        m, c = means[max_idx], covs[max_idx]
        def normalize(v):
                norm = np.linalg.norm(v)
                if norm == 0: 
                    return v
                return v / norm
        n = 2
        indices = (-weights).argsort()[:n]
        count = 0
        for idx in indices:
            m, c, _ = means[idx], covs[idx], weights[idx]


            (eigValues,eigVectors) = np.linalg.eig(c)
            eigx_n = np.array([eigVectors[0,0],eigVectors[0,1],eigVectors[0,2]]).reshape(1,3)
            eigy_n=-np.array([eigVectors[1,0],eigVectors[1,1],eigVectors[1,2]]).reshape(1,3)
            eigz_n= np.array([eigVectors[2,0],eigVectors[2,1],eigVectors[2,2]]).reshape(1,3)
            eigx_n = normalize(eigx_n)
            eigy_n = normalize(eigy_n)
            eigz_n = normalize(eigz_n)
            rot_mat = np.hstack([eigx_n.T, eigy_n.T, eigz_n.T])
            rot = R.from_matrix(rot_mat)
            quat = rot.as_quat()
            marker = Marker()
            marker.id = count
            count+=1
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = m[0]
            marker.pose.position.y = m[1]
            marker.pose.position.z = m[2]
            marker.header.frame_id = 'map'
            marker.pose.orientation.x = quat[0]
            marker.pose.orientation.y = quat[1]
            marker.pose.orientation.z = quat[2]
            marker.pose.orientation.w = quat[3]
            marker.scale.x = eigValues[0]*2
            marker.scale.y = eigValues[1]*2
            marker.scale.z = eigValues[2]*2
            foo = np.sqrt(marker.scale.x**2 + marker.scale.y**2 + marker.scale.z**2)
            if  foo > 20:
                break
            # rospy.logwarn("{} gauss # {} scale is: {}".format(self.label, count, foo))

            # marker.color.a = 0.5
            if count > 1:
                marker.lifetime = rospy.Duration(10)
            marker.color.a = 0.75 - count*0.2
            # count+=1
            if self.label == 'cup':
                # red cube
                marker.color.r = 1
 
            elif self.label == 'spoon':
                # blue cube
                marker.color.b = 1

            elif self.label == 'bowl':
                # green cube
                marker.color.g = 1

            elif self.label == 'grasp_cup':
                # red sphere
                marker.color.r = 1

            elif self.label == 'grasp_spoon':
                # blue sphere
                marker.color.b = 1

            elif self.label == 'grasp_bowl':
                # green sphere
                marker.color.g = 1
 
            elif self.label == 'pour_bowl':
                marker.color.r = 1
                marker.color.g = 1
 
            elif self.label == 'stir_bowl':
                marker.color.b = 1
                marker.color.g = 1

            elif self.label == 'stir_cup':
                marker.color.b = 1
                marker.color.r = 1
            # marker.lifetime = rospy.Duration(10)
            arr.markers.append(marker)
        
        self.gauss_pub.publish(arr)


        
        
    def gmm(self):
        with self.lock:
            good_particles = []
            max_weight = max(self.weights)
            min_weight = min(self.weights)
            threshold = min_weight + float((max_weight - min_weight)*0.9)
            for i in range(self.n):
                if self.weights[i] >=threshold:
                    good_particles.append(self.particles[i,:])
            gm = GaussianMixture(random_state=0).fit(good_particles)
            return  gm.means_[0], gm.covariances_
            
    def add_frame_element(self, frame_element_filter, frame_element_name):
        self.frame_element_filters[frame_element_name] = frame_element_filter
    
    def add_precondition(self, p_filter, p_name):
        self.precondition_filters[p_name] = p_filter

    
    def context_potential(self, particle, state):
        i = 0
        if self.preconditions:
            for precondition in self.preconditions:
                if precondition in state.action_history:
                    i += 1
                else:
                    break
            potential = 1
            while i < len(self.preconditions):
                subtask_filter = self.precondition_filters[self.preconditions[i]]
                for j in range(subtask_filter.n):
                    other_particle = subtask_filter.particles[j, :]
                    dist = np.sqrt((other_particle[0]-particle[0])**2 + (other_particle[1]-particle[1])**2 + (other_particle[2]-particle[2])**2)
                    phi = math.exp(-5*dist)
                    potential += (phi*subtask_filter.weights[j])
                i+=1
            return potential
        else:
            return 1
    
    def measurement_potential(self, particle, state):
        # determine state wrt preconditions
        i = 0
        if self.preconditions:
            for precondition in self.preconditions:
                if precondition not in state.action_history:
                    break
                else:
                    i+=1
        potential = 1
        core_elem_weight_mod = 1
        while i < len(self.frame_elements):
            core_element_filter = self.frame_element_filters[self.frame_elements[i]]
            for j in range(core_element_filter.n):
                other_particle = core_element_filter.particles[j, :]
                dist = np.sqrt((other_particle[0]-particle[0])**2 + (other_particle[1]-particle[1])**2 + (other_particle[2]-particle[2])**2)
                phi = math.exp(-5*dist)
                potential += (1/math.pow(core_elem_weight_mod,2))*(phi*core_element_filter.weights[j])
            i+=1
            core_elem_weight_mod+=1
        return potential

    def assign_weight(self, particle, state):
        measurement = self.measurement_potential(particle, state)
        context = self.context_potential(particle, state)
        return measurement*context 
    
    def update_filter(self, state):
        with self.lock:
            self.resample()
            for k in range(self.n):
                self.particles[k, :] = self.jitter(self.particles[k,:])
                self.weights[k] = self.assign_weight(self.particles[k,:], state)
            self.weights = self.weights / np.sum(self.weights)
            