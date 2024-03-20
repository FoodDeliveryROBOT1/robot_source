import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import math
import numpy as np

import yaml
from PIL import Image

# Fast SLAM covariance
Q = np.diag([0.1, 0.1]) ** 2
R = np.diag([1.0, np.deg2rad(5.0)]) ** 2


DT = 0.2  # time tick [s]

MAX_RANGE = 2.0  # maximum observation range
M_DIST_TH = 1.0  # Threshold of Mahalanobis distance for data association.
STATE_SIZE = 3  # State size [x,y,yaw]
LM_SIZE = 2  # LM state size [x,y]
N_PARTICLE = 25  # number of particle
NTH = N_PARTICLE / 5.0  # Number of particle for re-sampling

def euler_from_quaternion(x, y, z, w):
    t0 = 2.0 * (w * x + y * z)
    t1 = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    
    t2 = 2.0 * (w * y - z * x)
    t2 = 1.0 if t2 > 1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    
    t3 = 2.0 * (w * z +x * y)
    t4 = 1.0 - 2.0*(y * y + z * z)
    yaw = math.atan2(t3, t4)
    
    return yaw


class Particle:
    def __init__(self, N_LM):
        self.w = 1.0 / N_PARTICLE
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.P = np.eye(3)
        # landmark x-y positions
        self.lm = np.zeros((N_LM, LM_SIZE))
        # landmark position covariance
        self.lmP = np.zeros((N_LM * LM_SIZE, LM_SIZE))


def load_map(map_yaml, map_pgm):
    # Load map.yaml file to extract metadata
    with open(map_yaml, 'r') as yaml_file:
        yaml_data = yaml.load(yaml_file, Loader=yaml.FullLoader)

    # Extract origin and resolution from map.yaml
    origin = yaml_data.get('origin', [0, 0])
    resolution = yaml_data.get('resolution', 0.05)  # Default resolution if not specified

    # Load map.pgm file as an image
    img = Image.open(map_pgm)
    occupancy_grid = np.array(img)

    # Extract occupied_thresh from map.yaml
    occupied_thresh = yaml_data.get('occupied_thresh', 0.65)  # Default value if not specified

    # Find occupied points based on occupied_thresh
    occupied_points = np.where(occupancy_grid <= occupied_thresh)

    # Apply resolution and origin to the occupied points
    transformed_points = np.column_stack([
        occupied_points[1] * resolution + origin[0],
        occupied_points[0] * resolution + origin[1]
    ])

    return transformed_points, origin, resolution  # Return transformed occupied points, the origin, and the resolution



class slam(Node):
    def __init__(self):
        super().__init__('slam')
        self.init_slam()
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        self.laser_sub = self.create_subscription(
            LaserScan, 'scan', self.laser_callback, qos_profile)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.control_sub = self.create_subscription(
            Twist, '/cmd_vel', self.control_callback, 10)
        
        self.timerr = self.create_timer(0.2, self.slam_loop)


        self.marker_pub = self.create_publisher(MarkerArray, 'laser_markers', 10)
        self.seq = 0
        self.start_slam = False
        

        

    def odom_callback(self,odom_msg):
        self.pos_x = odom_msg.pose.pose.position.x
        self.pos_y = odom_msg.pose.pose.position.y
        self.odom_yaw =  euler_from_quaternion(odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w)
        self.yaw = self.odom_yaw
        self.z_odom = np.array([[self.pos_x],[self.pos_y],[self.odom_yaw]])
        # print(self.z_odom)

    def control_callback(self, control_msg):
        # control_msg = Twist()
        self.u_vx = control_msg.linear.x
        self.u_w = control_msg.angular.z
        self.ud = np.array([[self.u_vx], [self.u_w]])


    def laser_callback(self, msg):
        markers = self.convert_to_markers(msg)
        self.marker_pub.publish(markers)
        self.start_slam = True

    def convert_to_markers(self, msg):
        markers = MarkerArray()
        get_point = np.array([0, 0])
        for i, r in enumerate(msg.ranges):
            if not math.isnan(r):
                angle = msg.angle_min + i * msg.angle_increment
                x = r * math.cos(angle)
                y = r * math.sin(angle)

                marker = Marker() 
                
                marker.header.frame_id = "base_link"  # Set the header for each marker
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "laser_scan"
                marker.id = i
                marker.type = Marker.POINTS
                marker.action = Marker.ADD
                marker.pose.orientation.w = 1.0
                marker.scale.x = 0.1
                marker.scale.y = 0.1
                marker.color.r = 1.0
                marker.color.a = 1.0

                p = Point()
                p.x = x
                p.y = y
                get_point = np.vstack((get_point , np.array([x,y])))
                
                p.z = 0.0
                marker.points.append(p)

                markers.markers.append(marker)

        self.point = get_point[1:]
        return markers
    
    ## part slam

    def init_slam(self):
        # RFID positions [x, y]
        map_yaml = 'maps/map.yaml'
        map_pgm = 'maps/map.pgm'
        occupied_points, origin, resolution = load_map(map_yaml, map_pgm)
        print("Transformed Points:", occupied_points)
        print("Origin:", origin)
        print("Resolution:", resolution)

        self.RFID = occupied_points

        n_landmark = self.RFID.shape[0]
        print(n_landmark)
        self.xEst = np.zeros((STATE_SIZE, 1))  # SLAM estimation
        self.xTrue = np.zeros((STATE_SIZE, 1))  # True state
        self.xDR = np.zeros((STATE_SIZE, 1))  # Dead reckoning
        
        self.particles = [Particle(n_landmark) for _ in range(N_PARTICLE)]

        ## state of slam
        self.ud = np.array([[0.0], [0.0]])
        self.z_odom = np.array([[0.0], [0.0] , [0.0] ])

        self.point = []

    def slam_loop(self): 
        if self.start_slam :
            z = self.observation(self.z_odom , self.point)

            self.particles = self.fast_slam2(self.particles, self.ud, z)

            xEst = self.calc_final_state(self.particles)

            x_state = xEst[0: STATE_SIZE]

            print(x_state)



    def fast_slam2(self, particles, u, z):
        particles = self.predict_particles(particles, u)

        particles = self.update_with_observation(particles, z)

        particles = self.resampling(particles)

        return particles
    
    def normalize_weight(self, particles):
        sum_w = sum([p.w for p in particles])

        
        for i in range(N_PARTICLE):
                if sum_w != 0 :
                    particles[i].w /= sum_w
                else :
                    particles[i].w = 1.0 / N_PARTICLE
                    return particles

        return particles
    
    def calc_final_state(self, particles):
        xEst = np.zeros((STATE_SIZE, 1))

        particles = self.normalize_weight(particles)

        for i in range(N_PARTICLE):
            xEst[0, 0] += particles[i].w * particles[i].x
            xEst[1, 0] += particles[i].w * particles[i].y
            xEst[2, 0] += particles[i].w * particles[i].yaw

        xEst[2, 0] = self.pi_2_pi(xEst[2, 0])

        return xEst
    
    def predict_particles(self, particles, u):
        for i in range(N_PARTICLE):
            px = np.zeros((STATE_SIZE, 1))
            px[0, 0] = particles[i].x
            px[1, 0] = particles[i].y
            px[2, 0] = particles[i].yaw
            ud = u + (np.random.randn(1, 2) @ R ** 0.5).T  # add noise
            px = self.motion_model(px, ud)
            particles[i].x = px[0, 0]
            particles[i].y = px[1, 0]
            particles[i].yaw = px[2, 0]

        return particles
    
    def add_new_lm(self, particle, z, Q_cov):
        r = z[0]
        b = z[1]
        lm_id = int(z[2])

        s = math.sin(self.pi_2_pi(particle.yaw + b))
        c = math.cos(self.pi_2_pi(particle.yaw + b))

        particle.lm[lm_id, 0] = particle.x + r * c
        particle.lm[lm_id, 1] = particle.y + r * s

        # covariance
        dx = r * c
        dy = r * s
        d2 = dx ** 2 + dy ** 2
        d = math.sqrt(d2)
        Gz = np.array([[dx / d, dy / d],
                    [-dy / d2, dx / d2]])
        particle.lmP[2 * lm_id:2 * lm_id + 2] = np.linalg.inv(
            Gz) @ Q_cov @ np.linalg.inv(Gz.T)

        return particle
    
    def compute_jacobians(self, particle, xf, Pf, Q_cov):
        dx = xf[0, 0] - particle.x
        dy = xf[1, 0] - particle.y
        d2 = dx ** 2 + dy ** 2
        d = math.sqrt(d2)

        zp = np.array(
            [d, self.pi_2_pi(math.atan2(dy, dx) - particle.yaw)]).reshape(2, 1)

        Hv = np.array([[-dx / d, -dy / d, 0.0],
                    [dy / d2, -dx / d2, -1.0]])

        Hf = np.array([[dx / d, dy / d],
                    [-dy / d2, dx / d2]])

        Sf = Hf @ Pf @ Hf.T + Q_cov

        return zp, Hv, Hf, Sf
    
    def update_kf_with_cholesky(self, xf, Pf, v, Q_cov, Hf):
        PHt = Pf @ Hf.T
        S = Hf @ PHt + Q_cov

        S = (S + S.T) * 0.5
        SChol = np.linalg.cholesky(S).T
        SCholInv = np.linalg.inv(SChol)
        W1 = PHt @ SCholInv
        W = W1 @ SCholInv.T

        x = xf + W @ v
        P = Pf - W1 @ W1.T

        return x, P
    
    def update_landmark(self, particle, z, Q_cov):
        lm_id = int(z[2])
        xf = np.array(particle.lm[lm_id, :]).reshape(2, 1)
        Pf = np.array(particle.lmP[2 * lm_id:2 * lm_id + 2])

        zp, Hv, Hf, Sf = self.compute_jacobians(particle, xf, Pf, Q_cov)

        dz = z[0:2].reshape(2, 1) - zp
        dz[1, 0] = self.pi_2_pi(dz[1, 0])

        xf, Pf = self.update_kf_with_cholesky(xf, Pf, dz, Q, Hf)

        particle.lm[lm_id, :] = xf.T
        particle.lmP[2 * lm_id:2 * lm_id + 2, :] = Pf

        return particle
    
    def compute_weight(self, particle, z, Q_cov):
        lm_id = int(z[2])
        xf = np.array(particle.lm[lm_id, :]).reshape(2, 1)
        Pf = np.array(particle.lmP[2 * lm_id:2 * lm_id + 2])
        zp, Hv, Hf, Sf = self.compute_jacobians(particle, xf, Pf, Q_cov)

        dz = z[0:2].reshape(2, 1) - zp
        dz[1, 0] = self.pi_2_pi(dz[1, 0])

        try:
            invS = np.linalg.inv(Sf)
        except np.linalg.linalg.LinAlgError:
            return 1.0

        num = np.exp(-0.5 * dz.T @ invS @ dz)[0, 0]
        den = 2.0 * math.pi * math.sqrt(np.linalg.det(Sf))

        w = num / den

        return w

    def proposal_sampling(self, particle, z, Q_cov):
        lm_id = int(z[2])
        xf = particle.lm[lm_id, :].reshape(2, 1)
        Pf = particle.lmP[2 * lm_id:2 * lm_id + 2]
        # State
        x = np.array([particle.x, particle.y, particle.yaw]).reshape(3, 1)
        P = particle.P
        zp, Hv, Hf, Sf = self.compute_jacobians(particle, xf, Pf, Q_cov)

        Sfi = np.linalg.inv(Sf)
        dz = z[0:2].reshape(2, 1) - zp
        dz[1] = self.pi_2_pi(dz[1])

        Pi = np.linalg.inv(P)

        particle.P = np.linalg.inv(Hv.T @ Sfi @ Hv + Pi)  # proposal covariance
        x += particle.P @ Hv.T @ Sfi @ dz  # proposal mean

        particle.x = x[0, 0]
        particle.y = x[1, 0]
        particle.yaw = x[2, 0]

        return particle
    
    def update_with_observation(self, particles, z):
        for iz in range(len(z[0, :])):
            landmark_id = int(z[2, iz])

            for ip in range(N_PARTICLE):
                # new landmark
                if abs(particles[ip].lm[landmark_id, 0]) <= 0.01:
                    particles[ip] = self.add_new_lm(particles[ip], z[:, iz], Q)
                # known landmark
                else:
                    w = self.compute_weight(particles[ip], z[:, iz], Q)
                    particles[ip].w *= w

                    particles[ip] = self.update_landmark(particles[ip], z[:, iz], Q)
                    particles[ip] = self.proposal_sampling(particles[ip], z[:, iz], Q)

        return particles
    
    def resampling(self, particles):
        """
        low variance re-sampling
        """

        particles = self.normalize_weight(particles)

        pw = []
        for i in range(N_PARTICLE):
            pw.append(particles[i].w)

        pw = np.array(pw)

        n_eff = 1.0 / (pw @ pw.T)  # Effective particle number

        if n_eff < NTH:  # resampling
            w_cum = np.cumsum(pw)
            base = np.cumsum(pw * 0.0 + 1 / N_PARTICLE) - 1 / N_PARTICLE
            resample_id = base + np.random.rand(base.shape[0]) / N_PARTICLE

            inds = []
            ind = 0
            for ip in range(N_PARTICLE):
                while (ind < w_cum.shape[0] - 1) \
                        and (resample_id[ip] > w_cum[ind]):
                    ind += 1
                inds.append(ind)

            tmp_particles = particles[:]
            for i in range(len(inds)):
                particles[i].x = tmp_particles[inds[i]].x
                particles[i].y = tmp_particles[inds[i]].y
                particles[i].yaw = tmp_particles[inds[i]].yaw
                particles[i].lm = tmp_particles[inds[i]].lm[:, :]
                particles[i].lmP = tmp_particles[inds[i]].lmP[:, :]
                particles[i].w = 1.0 / N_PARTICLE

        return particles
    

    def observation(self, xTrue, RFID):
        # add noise to range observation
        z = np.zeros((3, 0))

        for i in range(len(RFID[:, 0])):

            dx = RFID[i, 0] - xTrue[0, 0]
            dy = RFID[i, 1] - xTrue[1, 0]
            d = math.hypot(dx, dy)
            angle = self.pi_2_pi(math.atan2(dy, dx) - xTrue[2, 0])
            if d <= MAX_RANGE:
                zi = np.array([d, self.pi_2_pi(angle), i]).reshape(3, 1)
                z = np.hstack((z, zi))

        return z
    
    def motion_model(self, x, u):
        F = np.array([[1.0, 0, 0],
                    [0, 1.0, 0],
                    [0, 0, 1.0]])

        B = np.array([[DT * math.cos(x[2, 0]), 0],
                    [DT * math.sin(x[2, 0]), 0],
                    [0.0, DT]])

        x = F @ x + B @ u

        x[2, 0] = self.pi_2_pi(x[2, 0])

        return x
    def pi_2_pi(self, angle):
        return self.angle_mod(angle)


    def angle_mod(self, x, zero_2_2pi=False, degree=False):
        """
        Angle modulo operation
        Default angle modulo range is [-pi, pi)

        Parameters
        ----------
        x : float or array_like
            A angle or an array of angles. This array is flattened for
            the calculation. When an angle is provided, a float angle is returned.
        zero_2_2pi : bool, optional
            Change angle modulo range to [0, 2pi)
            Default is False.
        degree : bool, optional
            If True, then the given angles are assumed to be in degrees.
            Default is False.

        Returns
        -------
        ret : float or ndarray
            an angle or an array of modulated angle.

        Examples
        --------
        >>> angle_mod(-4.0)
        2.28318531

        >>> angle_mod([-4.0])
        np.array(2.28318531)

        >>> angle_mod([-150.0, 190.0, 350], degree=True)
        array([-150., -170.,  -10.])

        >>> angle_mod(-60.0, zero_2_2pi=True, degree=True)
        array([300.])

        """
        if isinstance(x, float):
            is_float = True
        else:
            is_float = False

        x = np.asarray(x).flatten()
        if degree:
            x = np.deg2rad(x)

        if zero_2_2pi:
            mod_angle = x % (2 * np.pi)
        else:
            mod_angle = (x + np.pi) % (2 * np.pi) - np.pi

        if degree:
            mod_angle = np.rad2deg(mod_angle)

        if is_float:
            return mod_angle.item()
        else:
            return mod_angle
        





def main(args=None):
    rclpy.init(args=args)
    converter_node = slam()
    rclpy.spin(converter_node)
    converter_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
