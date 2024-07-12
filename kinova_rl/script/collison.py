#!/usr/bin/env python3
import pybullet as p
import pybullet_data
import numpy as np

class BoundingVolume:
    def __init__(self, center, radius):
        self.center = center
        self.radius = radius

class BVHNode:
    def __init__(self, bv, link_name, left=None, right=None):
        self.bv = bv
        self.link_name = link_name
        self.left = left
        self.right = right

def construct_bvh_from_urdf(urdf_file):
    # Initialize PyBullet and load URDF
    physicsClient = p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    robot_id = p.loadURDF(urdf_file)

    # Get link information
    num_joints = p.getNumJoints(robot_id)
    link_name_to_index = {p.getJointInfo(robot_id, i)[12].decode('UTF-8'): i for i in range(num_joints)}
    
    # Construct BVH for each link
    bvh_nodes = {}
    root = None
    
    for link_name, link_index in link_name_to_index.items():
        # Compute bounding volume (simple AABB for demonstration)
        link_pose = p.getLinkState(robot_id, link_index)[0]
        link_extent = p.getAABB(robot_id, link_index)
        link_center = 0.5 * (np.array(link_extent[0]) + np.array(link_extent[1]))
        link_radius = np.linalg.norm(np.array(link_extent[1]) - np.array(link_extent[0])) / 2.0
        bv = BoundingVolume(center=link_center, radius=link_radius)
        
        # Create BVH node
        bvh_node = BVHNode(bv, link_name)
        bvh_nodes[link_name] = bvh_node
        
        # Set root if not already set
        if not root:
            root = bvh_node
    
    # Build BVH structure for self-collision detection
    for link_name, link_index in link_name_to_index.items():
        parent_name = p.getJointInfo(robot_id, link_index)[12].decode('UTF-8')
        if parent_name in bvh_nodes:
            if bvh_nodes[parent_name].left is None:
                bvh_nodes[parent_name].left = bvh_nodes[link_name]
            else:
                bvh_nodes[parent_name].right = bvh_nodes[link_name]
    
    p.disconnect()
    return root, bvh_nodes

def check_self_collision_with_bvh(root_node):
    def traverse_bvh_for_self_collision(node):
        if node is None:
            return False
        
        # Check collision with sibling nodes
        if node.left and intersect(node.bv, node.left.bv):
            return True
        
        if node.right and intersect(node.bv, node.right.bv):
            return True
        
        # Recursively check children
        return (traverse_bvh_for_self_collision(node.left) or
                traverse_bvh_for_self_collision(node.right))
    
    return traverse_bvh_for_self_collision(root_node)

def intersect(bv1, bv2):
    # Simple AABB intersection check
    dist = np.linalg.norm(bv1.center - bv2.center)
    return dist < (bv1.radius + bv2.radius)

if __name__ == "__main__":
    # Example usage
    urdf_file = r"/home/vr-lab/kinova_arm/src/kinova-ros/kinova_description/urdf/j2s7s300_standalone.urdf"  # Replace with your URDF file path
    robot_bvh, bvh_nodes = construct_bvh_from_urdf(urdf_file)
    
    # Check self-collision
    self_collision_detected = check_self_collision_with_bvh(robot_bvh)
    
    if self_collision_detected:
        print("Self-collision detected!")
    else:
        print("No self-collision detected.")
