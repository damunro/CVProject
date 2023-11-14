################################################
######### FILE IS NOT COMPLETE #################
################################################
# TODO: VERYIFY intrinsics matrix
# TODO: Verity translation matrix

from typing import List, Union

import numpy as np
from scipy.spatial.transform import Rotation
import cv2


class TransformationMatrix:
    """
    Transformation Matrix Class
    """
    def __init__(self, R=np.eye(3), t = np.zeros(3), H: np.ndarray=None) -> None:
        if H is None:
            self.matrix = TransformationMatrix.__transformation_matrix(t, R)
        else:
            assert H.shape == (4,4)
            self.matrix = H

    def set_rotation(self, R=np.eye(3)) -> None:
        assert R.shape == (3,3)
        self.matrix = TransformationMatrix.__transformation_matrix(self.matrix[:3, 3], R)

    def set_translation(self, t=np.zeros(3)) -> None:
        assert len(t) == 3
        self.matrix = TransformationMatrix.__transformation_matrix(t, self.matrix[:3,:3])

    def transform_point(self, pose=np.zeros(3)) -> np.ndarray:
        assert len(pose) == 3 or len(pose) == 4
        
        if len(pose) == 3:
            pose = np.append(pose, [1])
        else:
            assert pose[3] == 1
        
        return (self.matrix @ pose)[:3]

    def inverse_transform(self, pose=np.zeros(3)):
        mat_copy = self.matrix.copy()

        self.matrix = np.linalg.inv(self.matrix)
        pose = self.transform(pose)

        self.matrix = mat_copy

        return pose

    def as_mat(self):
        return self.matrix
    
    def as_pos_and_quat(self):
        R = self.matrix[:3,:3]
        orientation = Rotation.from_matrix(R).as_quat()
        pose = self.matrix[:3,3]
        return pose, orientation

    @ staticmethod
    def transform_transform(H1=None, H2=None):
        assert H1 is not None
        if isinstance(H1, list):

            matrix = TransformationMatrix.__transformation_matrix()
            for mat in H1:
                assert isinstance(mat, TransformationMatrix)
                matrix = mat.matrix @ matrix
        elif isinstance(H1, TransformationMatrix) and isinstance(H2, TransformationMatrix):
            return H1 @ H2
        else:
            raise TypeError("Must input Transformation Matrix or List if transformation matricies")

    @ staticmethod
    def __transformation_matrix(translation : np.ndarray = np.zeros(3), rotation : np.ndarray = np.eye(3)) -> np.ndarray:
        """
        assembles transformation matrix from rotation and translation

        Args:
            translation (np.ndarray): 3x1 translation matrix
            rotation (np.ndarray): 3x3 rotation matrix
        Returns:
            np.ndarray: 4x4 translation matrix
        """
        # ensure rotation matrix is orthonormal
        assert np.allclose(np.dot(rotation, rotation.T), np.eye(3))

        matrix = np.eye(4, dtype=np.float32)
        matrix[:3, 3] =  translation# translation
        matrix[:3, :3] = rotation
        return matrix


class IntrinsicsMatrix:
    '''
    Class defining intrinsics matrix
    contains 2 methods
    calc_pixels: calculate pixel values from positions
    calc_position: calculates positon from pixel coordinates
    '''

    def __init__(self, fov_x = 84, dimensions = (720,1280), degrees=True) -> None:
        '''
        Contstructor to create intrinsics matrix
        Default parameters mirror the d405 camera with 720p resolution
        args:
            fov_x: field of view in the horizintal direction of the camera
            dimensions (tuple): pair o heigh and width of image 
        '''
        assert len(dimensions) == 2
        assert fov_x > 0

        h, w = dimensions
        cy, cx = h / 2, w / 2
        
        if degrees:
            fov_x = np.deg2rad(fov_x)
        
        fl = (w/2) * np.tan(fov_x / 2)

        self.matrix = np.array([
            [fl,    0,      cx],
            [0,     fl,     cy],
            [0,     0,      1 ],
        ], dtype=np.float32)

        assert self.matrix.shape == (3,3)

    def calc_pixels(self, pose=[0,0,1]):
        # print(pose)
        assert len(pose) == 3
        assert pose[2] > 0

        pose = np.array(pose, dtype=np.float32)
        coords = self.matrix @ pose

        pixels = (coords / coords[2])[:2]
        return [round(p) for p in pixels]
    
    def calc_position(self, pixels: tuple = (640, 360), depth=1):
        '''
        Calculates position based on position and depth
        If no pixels give, assume default images center
        If no depth is give, assume the depth is 1 (unit)
        Ards:
            pixels (tuple): (x,y) coordinate on the image
            depth: depth of the point from the camera
        Returns:
            np.array: (3x3) position (x,y,z)
        '''
        assert len(pixels) == 2
        assert depth > 0

        pixels = depth * np.array(list(pixels) + [1])
        return np.linalg.inv(self.matrix) @ pixels


def calculate_matrix(x: float, y: float, z: float, angle_mount: float = 0, angle_cap: float = 0) -> TransformationMatrix:
    """
    Calculates the transformation matrix from the camera to the fuel cap using the data collection rig
    Assumes fuel cap coordinates to be positive x in the left direction on wall, positive y down, positive z towards camera
    Positive fuel cap rotation are counter-clockwise, positve camera mount rotations are counter-clockwise (based on coordinates)
    
    Args:
        x (float): horizontal distance along the wall
        y (float): virtical distance along the wall
        z (float): distance from the rig to the wall
        angle_mount (float): angle measured at the camera mount
        angle_cap (float): angle of the fuel cap on the wall (use protractor)
    
    Returns:
        np.ndarray: translation matrix from camera to fuel cap
    """
    R_cap = Rotation.from_euler('z',angle_cap, degrees=True).as_matrix()
    H_cap = TransformationMatrix(R=R_cap)

    t_cap_to_mount = np.array([x,y,z])
    R_cap_to_mount = Rotation.from_euler('y',angle_mount+180, degrees=True).as_matrix()
    H_cap_to_mount = TransformationMatrix(R=R_cap_to_mount, t=t_cap_to_mount)

    t_mount_to_cam = __mount_to_camera_translation()
    H_mount_to_cam = TransformationMatrix(t=t_mount_to_cam)

    H_cap_to_cam = H_cap.as_mat() @ H_cap_to_mount.as_mat() @ H_mount_to_cam.as_mat()
    return TransformationMatrix(H=np.linalg.inv(H_cap_to_cam))


def annotate_img(img: np.ndarray, H: TransformationMatrix, K: IntrinsicsMatrix, line_width=5, axis_len = 5) -> np.ndarray:
    """
    Plots coordinate axis on image

    Args:
        img (np.ndarray): image array
        translation (np.ndarray): 4x4 translation matrix
        K (np.ndarray): camera intrinsic matrix

    Returns:
        np.ndarray: annotated image
    """
    # initialize points for coordinate axis
    points = [
        np.array([0, 0, 0, 1]),
        np.array([axis_len, 0, 0, 1]),
        np.array([0, axis_len, 0, 1]),
        np.array([0, 0, axis_len, 1])
    ]

    pixels = [K.calc_pixels(H.transform_point(p)) for p in points]

    o,x,y,z = pixels

    thickness = line_width
    cv2.line(img, o, x, (0,0,255), thickness)
    cv2.line(img, o, y, (0,255,0), thickness)
    cv2.line(img, o, z, (255,0,0), thickness)

    return img


def __mount_to_camera_translation(cm=True) -> np.ndarray:
    """
    camera translation based on datasheet information
    camera origin is located on left camera, inset 3.07 mm
    Default to return translation in centimeters
    """
    x = -9          # cameras are 18 mm apart
    y = 42 / 2          # cameras are located on middle of camera in y, and cam is 42 mm tall
    z = -3.7

    trans = np.array([x,y,z], dtype = np.float32)
    if cm:
        return trans / 10
    return trans


def main():
    K = IntrinsicsMatrix()
    pos = [1, 1, 2]
    pixels = (0,0)
    depth = 3
    target_pos = [-3.33183754, -1.87415862,  3.]
    tol = 1e-5

    assert K.calc_pixels() == [640, 360]
    assert K.calc_pixels(pos) == [928, 648]
    assert [int(i) for i in list(K.calc_position())] == [0, 0, 1]
    pos = K.calc_position(pixels, depth)
    assert np.sum(pos - np.array(target_pos)) < tol


    img = cv2.imread("data//saved_img.png")
    
    K = IntrinsicsMatrix()
    translation = calculate_matrix(-6, 19.05, 45.72, angle_mount=-10, angle_cap=20)
    pos, orien = translation.as_pos_and_quat()
    img = annotate_img(img, translation, K)

    cv2.imshow("Annotated Image", img)
    cv2.waitKey(0)

    cv2.imwrite("annotated_img.png", img)


if __name__ == "__main__":
    main()