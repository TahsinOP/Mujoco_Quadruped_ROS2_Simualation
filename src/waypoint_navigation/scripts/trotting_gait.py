import numpy as np
from numpy.linalg import inv, norm
from numpy import array, asarray, matrix
from math import atan, pi, radians, degrees, cos, sin, acos, asin
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R  # For alternative rotation matrix implementation

def point_to_rad(p1, p2):
    """
    Converts 2D cartesian points to polar angles in range 0 - 2pi
    Similar to atan2 but ensures output in [0, 2π] range
    """
    if (p1 > 0 and p2 >= 0): return atan(p2/p1)
    elif (p1 == 0 and p2 >= 0): return pi/2
    elif (p1 < 0 and p2 >= 0): return -abs(atan(p2/p1)) + pi
    elif (p1 < 0 and p2 < 0): return atan(p2/p1) + pi
    elif (p1 > 0 and p2 < 0): return -abs(atan(p2/p1)) + 2*pi
    elif (p1 == 0 and p2 < 0): return pi * 3/2
    elif (p1 == 0 and p2 == 0): return pi * 3/2  # edge case

def RotMatrix3D(rotation=[0,0,0], is_radians=True, order='xyz'):
    """
    Creates a 3D rotation matrix based on roll, pitch, and yaw angles
    Parameters:
    rotation: [roll, pitch, yaw] angles
    is_radians: Whether the input angles are in radians (True) or degrees (False)
    order: The order of rotations to apply
    """
    roll, pitch, yaw = rotation[0], rotation[1], rotation[2]
    
    # Convert to radians if the input is in degrees
    if not is_radians: 
        roll = radians(roll)
        pitch = radians(pitch)
        yaw = radians(yaw)
    
    # Rotation matrix about each axis
    rotX = np.matrix([[1, 0, 0], [0, cos(roll), -sin(roll)], [0, sin(roll), cos(roll)]])
    rotY = np.matrix([[cos(pitch), 0, sin(pitch)], [0, 1, 0], [-sin(pitch), 0, cos(pitch)]])
    rotZ = np.matrix([[cos(yaw), -sin(yaw), 0], [sin(yaw), cos(yaw), 0], [0, 0, 1]])
    
    # Rotation matrix order (default: pitch -> roll -> yaw)
    if order == 'xyz': rotationMatrix = rotZ * rotY * rotX
    elif order == 'xzy': rotationMatrix = rotY * rotZ * rotX
    elif order == 'yxz': rotationMatrix = rotZ * rotX * rotY
    elif order == 'yzx': rotationMatrix = rotX * rotZ * rotY
    elif order == 'zxy': rotationMatrix = rotY * rotX * rotZ
    elif order == 'zyx': rotationMatrix = rotX * rotY * rotZ
    
    return rotationMatrix

def RotMatrix3D_scipy(rotation=[0,0,0], is_radians=True, order='xyz'):
    """
    Alternative implementation using scipy's Rotation class
    More efficient and potentially more robust
    """
    # Convert to radians if needed
    if not is_radians:
        rotation = [radians(angle) for angle in rotation]
    
    # Create rotation object - note scipy uses ZYX order by default, so we need to reverse
    rot_order = order[::-1].upper()
    rot = R.from_euler(rot_order, rotation)
    
    # Get rotation matrix
    return np.matrix(rot.as_matrix())

class Kinematics:
    
    def __init__(self):
        # Leg IDs
        left_front = 0
        left_back = 1
        right_front = 2
        right_back = 3
        
        self.right_legs = [right_front, right_back]
        
        # Link lengths (in meters)
        self.link_1 = 0.11   # Hip to thigh joint
        self.link_2 = 0.32   # Thigh length
        self.link_3 = 0.35    # Calf length
        self.phi = radians(90) # Fixed angle
        
        # Body dimensions
        self.length = 0.65
        self.width = 0.14
        self.height = 0.0
        
        # Leg origins (left_f, left_b, right_b, right_f), i.e., the coordinate of j1
        self.leg_origins = np.matrix([
            [self.length/2, self.width/2, 0],     # Left front
            [-self.length/2, self.width/2, 0],    # Left back
            [-self.length/2, -self.width/2, 0],   # Right back
            [self.length/2, -self.width/2, 0],    # Right front
            [self.length/2, self.width/2, 0]      # Extra point to close the rectangle
        ])
        
    def leg_IK(self, xyz, rot=[0,0,0], legID=0, is_radians=True, center_offset=[0,0,0]):
        """
        Calculates inverse kinematics for a specific leg
        Adjusts inputs by adding rotation and offset of that rotation from the center of the robot
        
        Parameters:
        xyz: Target foot position
        rot: Body rotation [roll, pitch, yaw]
        legID: ID of the leg (0-3)
        is_radians: Whether rotation inputs are in radians
        center_offset: Offset from the center of rotation
        
        Returns:
        List containing [theta_1, theta_2, theta_3, j1, j2, j3, j4]
        """
        # Check if the leg is from the right side 
        is_right = (legID in self.right_legs)
        
        # Add offset of each leg from the axis of rotation
        XYZ = asarray((inv(RotMatrix3D(rot, is_radians)) * 
                      ((array(xyz) + self.leg_origins[legID,:] - array(center_offset)).transpose())).transpose())
        
        # Subtract the offset between the leg and the center of rotation 
        # so that the resultant coordinate is relative to the origin (j1) of the leg
        xyz_ = asarray(XYZ - self.leg_origins[legID,:] + array(center_offset)).flatten()

        # Calculate the angles and coordinates of the leg relative to the origin of the leg
        return self.leg_IK_calc(xyz_, is_right)

    def leg_IK_calc(self, xyz, is_right=False):
        """
        Core IK calculator function
        
        Parameters:
        xyz: Target foot position relative to leg origin
        is_right: Boolean indicating if this is a right leg
        
        Returns:
        List containing [theta_1, theta_2, theta_3, j1, j2, j3, j4]
        """
        x, y, z = xyz[0], xyz[1], xyz[2]    # Unpack coordinates
        
        # Length of vector projected on the YZ plane. equiv. to len_A = sqrt(y**2 + z**2)
        len_A = norm([0, y, z])   
        
        # a_1: Angle from the positive y-axis to the end-effector (0 <= a_1 < 2pi)
        # a_2: Angle between len_A and leg's projection line on YZ plane
        # a_3: Angle between link1 and length len_A
        a_1 = point_to_rad(y, z)                     
        a_2 = asin(sin(self.phi) * self.link_1 / len_A) 
        a_3 = pi - a_2 - self.phi                   
        
        # Angle of link1 about the x-axis 
        if is_right: 
            theta_1 = a_1 - a_3
        else: 
            theta_1 = a_1 + a_3
            if theta_1 >= 2*pi: 
                theta_1 -= 2*pi
        
        # Calculate position of joint 2
        j2 = array([0, self.link_1 * cos(theta_1), self.link_1 * sin(theta_1)])
        j4 = array(xyz)
        j4_2_vec = j4 - j2  # Vector from j2 to j4
        
        # Calculate rotation angle to work in a new 2D plane
        if is_right: 
            R = theta_1 - self.phi - pi/2
        else: 
            R = theta_1 + self.phi - pi/2
        
        # Create rotation matrix to work on a new 2D plane (XZ_)
        rot_mtx = RotMatrix3D([-R, 0, 0], is_radians=True)
        j4_2_vec_ = rot_mtx * (np.reshape(j4_2_vec, [3, 1]))
        
        # xyz in the rotated coordinate system + offset due to link_1 removed
        x_, y_, z_ = j4_2_vec_[0], j4_2_vec_[1], j4_2_vec_[2]
        
        # Distance from j2 to j4
        len_B = norm([x_, z_])
        
        # Handle mathematically invalid input, i.e., point too far away to reach
        if len_B >= (self.link_2 + self.link_3): 
            len_B = (self.link_2 + self.link_3) * 0.99999
            print('Target coordinate: [%f %f %f] too far away' % (x, y, z))
        
        # b_1: Angle between +ve x-axis and len_B (0 <= b_1 < 2pi)
        # b_2: Angle between len_B and link_2
        # b_3: Angle between link_2 and link_3
        b_1 = point_to_rad(x_, z_)  
        b_2 = acos((self.link_2**2 + len_B**2 - self.link_3**2) / (2 * self.link_2 * len_B)) 
        b_3 = acos((self.link_2**2 + self.link_3**2 - len_B**2) / (2 * self.link_2 * self.link_3))  
        
        # Assuming theta_2 = 0 when the leg is pointing down (i.e., 270 degrees offset from the +ve x-axis)
        theta_2 = b_1 - b_2    
        theta_3 = pi - b_3
        
        # Calculate the coordinates of the joints for visualization
        j1 = np.array([0, 0, 0])
        
        # Calculate joint 3
        j3_ = np.reshape(np.array([self.link_2 * cos(theta_2), 0, self.link_2 * sin(theta_2)]), [3, 1])
        j3 = np.asarray(j2 + np.reshape(np.linalg.inv(rot_mtx) * j3_, [1, 3])).flatten()
        
        # Calculate joint 4
        j4_ = j3_ + np.reshape(np.array([self.link_3 * cos(theta_2 + theta_3), 0, 
                                         self.link_3 * sin(theta_2 + theta_3)]), [3, 1])
        j4 = np.asarray(j2 + np.reshape(np.linalg.inv(rot_mtx) * j4_, [1, 3])).flatten()
        
        # Modify angles to match robot's configuration (i.e., adding offsets)
        # angles = self.angle_corrector(angles=[theta_1, theta_2, theta_3], is_right=is_right)
        # return [angles[0], angles[1], angles[2], j1, j2, j3, j4]
        return [theta_1, theta_2,theta_3, j1, j2, j3, j4]
    
    def base_pose(self, rot=[0,0,0], is_radians=True, center_offset=[0,0,0]):
        """
        Calculate the position of the robot's base with applied rotation
        """
        # Offset due to non-centered axes of rotation
        offset = RotMatrix3D(rot, is_radians) * \
            (matrix(center_offset).transpose()) - matrix(center_offset).transpose()
        
        # Rotate the base around the center of rotation
        rotated_base = RotMatrix3D(rot, is_radians) * self.leg_origins.transpose() - offset
        return rotated_base.transpose()
       
    def leg_pose(self, xyz, rot, legID, is_radians, center_offset=[0,0,0]):
        """
        Get coordinates of leg joints relative to j1
        """
        # Get the coordinates of each joints relative to the leg's origin
        pose_relative = self.leg_IK(xyz, rot, legID, is_radians, center_offset)[3:]
        
        # Adjust the coordinates according to the robot's orientation (roll, pitch, yaw)
        pose_true = RotMatrix3D(rot, is_radians) * (array(pose_relative).transpose())
        return pose_true.transpose()
    
    def plot_base(self, ax, rot=[0,0,0], is_radians=True, center_offset=[0,0,0]):
        """
        Plot rectangular base where each corner represents the origin of leg
        """
        # Get coordinates
        p = (self.base_pose(rot, is_radians, center_offset)).transpose()     
        # Plot coordinates
        ax.plot3D(asarray(p[0,:]).flatten(), asarray(p[1,:]).flatten(), 
                 asarray(p[2,:]).flatten(), 'r')
        return
       
    def plot_leg(self, ax, xyz, rot=[0,0,0], legID=0, is_radians=True, center_offset=[0,0,0]):
        """
        Plot an individual leg
        """
        # Get coordinates
        p = ((self.leg_pose(xyz, rot, legID, is_radians, center_offset) \
                + self.base_pose(rot, is_radians, center_offset)[legID]).transpose())
        # Plot coordinates
        ax.plot3D(asarray(p[0,:]).flatten(), asarray(p[1,:]).flatten(), 
                 asarray(p[2,:]).flatten(), 'b')
        return

    def plot_robot(self, xyz, rot=[0,0,0], leg_N=4, is_radians=True, limit=0.250, center_offset=[0,0,0]):
        """
        Plot the entire robot with all legs
        """
        ax = self.ax_view(limit)  # Set the view
        self.plot_base(ax, rot, is_radians, center_offset)  # Plot base

        # Plot legs
        for leg in range(leg_N):
            self.plot_leg(ax, xyz[leg], rot, leg, is_radians, center_offset) 
        
        # Show figure
        plt.show()
        return
        
    def angle_corrector(self, angles=[0,0,0], is_right=True):
        """
        Adjust angles for specific configurations of motors, including orientation
        This will vary for each robot (possibly for each leg as well)
        """
        angles[1] -= 1.5*pi  # Add offset 
        
        if is_right:
            theta_1 = angles[0] - pi
            theta_2 = angles[1] + 45*pi/180  # 45 degrees initial offset
        else: 
            if angles[0] > pi:  
                theta_1 = angles[0] - 2*pi
            else: 
                theta_1 = angles[0]
            
            theta_2 = -angles[1] - 45*pi/180
        
        theta_3 = -angles[2] + 45*pi/180
        return [theta_1, theta_2, theta_3]
        
    @staticmethod
    def ax_view(limit):
        """
        Set up a 3D plot with proper axis limits
        """
        ax = plt.axes(projection="3d")
        ax.set_xlim(-limit, limit)
        ax.set_ylim(-limit, limit)
        ax.set_zlim(-limit, limit)
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        return ax

# Example usage
if __name__ == "__main__":
    # Create kinematics object
    kin = Kinematics()
    
    # Define desired foot positions for each leg
    # Format: [x, y, z] for each leg in meters
    foot_positions = [
        [0.29, 0.23, -0.51],  # Left front
        [-0.37, 0.22, -0.51], # Left back
        [-0.37, -0.22, -0.51],# Right back
        [0.289, -0.23, -0.51]  # Right front
    ]
    
    # Calculate IK for each leg
    for i, pos in enumerate(foot_positions):
        angles = kin.leg_IK(pos, legID=i)[:3]
        print(f"Leg {i} angles (degrees): Hip={degrees(angles[0]):.2f}, Thigh={degrees(angles[1]):.2f}, Calf={degrees(angles[2]):.2f}")
    
    # Visualize the robot
    kin.plot_robot(foot_positions)