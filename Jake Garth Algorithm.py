import numpy as np

import open3d as o3d

#Note to engineer: I have seperated the different steps in visualising the point clouds in to different methods.
#Calculating the rotation matrix: make_rotation_matrix()
#Transforming a data cloud based on the rotation and translation: bun_transformation()
#Visualising the data clouds: visualisation_after_transformation()
#I have put some additional methods down the bottom, just so the code is easier to read.


def main():
    #These are the input arrays for the Euler Angles and the Translation
    rotation_Euler = [-0.0642381, -45.1938717, 1.1853354]
    translation_matrix = [-0.00646017, -0.0000136122, -0.0129064]

    #rotation_matrix is the resulting rotation matrix R, it is calculated using the Euler Angles
    rotation_matrix = make_rotation_matrix(rotation_Euler)

    #pcd_315 is the point cloud from bun315.txt, bun315_points is the numpy array
    
    pcd_315 = o3d.io.read_point_cloud(r"bun315.txt", format='xyz')
    bun315_points = np.asarray(pcd_315.points)

    #reframed_bunf315 is a numpy array of data points after being rotated and translated
    reframed_bun315 = bun_transformation(bun315_points, translation_matrix, rotation_matrix)

    #plots all the data clouds in the one window.
    visualisation_after_transformation(reframed_bun315)




#Returns the rotation matrix corresponding the given Euler angles
def make_rotation_matrix(Euler_Angles):
    rotation_Radians = np.radians(Euler_Angles)

    #Retrieve the corresponding matrices for roll, pitch and yaw.
    x_rotation_matrix = make_x_axis_rotation_matrix(rotation_Radians[0])
    y_rotation_matrix = make_y_axis_rotation_matrix(rotation_Radians[1])
    z_rotation_matrix = make_z_axis_rotation_matrix(rotation_Radians[2])

    #The complete rotation matrix is the result of a matrix multiplication of the yaw, pitch and roll matrices.
    rotation_matrix = np.matmul(z_rotation_matrix, y_rotation_matrix, x_rotation_matrix)

    #NOTE: I could have just hard coded the rotation matrix, and that would save some computation time.
    #I could also just make it such that there is no need for matrix multiplication or calculating the yaw/roll/pitch matrices.
    #However, I thought I would just do it all automatically.
    return rotation_matrix


#Takes in a point_cloud numpy array, and rotates it around the rotation matrix
#and translates it by the translation matrix.
#Returns the transformed point cloud as a numpy array.
def bun_transformation(points_array, translation_matrix, rotation_matrix):
    #Every point in the point cloud gets rotated by the rotation matrix
    for i in range(len(points_array)):
        points_array[i] = np.dot(rotation_matrix, points_array[i])

    #And then every point gets translated by the corresponding translation.
    points_array[:, 0] += translation_matrix[0]
    points_array[:, 1] += translation_matrix[1]
    points_array[:, 2] += translation_matrix[2]

    #Returns a numpy array of the transformed points
    return points_array



#Visualise all data clouds in the one window. Includes the "before" and "after" transformation of bun315 points.
def visualisation_after_transformation(reframed_bun315):


    #making a data cloud of the bun000 points
    pcd_000 = o3d.io.read_point_cloud(r"bun000.txt", format='xyz')
    pcd_000.paint_uniform_color([0, 1, 0])

    #making a data cloud of the reframed bun315 points
    pcd_315_reframed = o3d.geometry.PointCloud()
    pcd_315_reframed.points = o3d.utility.Vector3dVector(reframed_bun315)
    pcd_315_reframed.paint_uniform_color([1, 0, 0])

    #making a data cloud of the bun000 points and translating it by 0.2 units in the 'y' direction
    pcd_000_translated = o3d.io.read_point_cloud(r"bun000.txt", format='xyz')
    pcd_000_translated.paint_uniform_color([0, 1, 0])
    pcd_000_translated.translate((0, 0.2, 0))


    #making a data cloud of the original bun315 points and translating it by 0.2 units in the 'y' direction
    pcd_315_translated = o3d.io.read_point_cloud(r"bun315.txt", format='xyz')
    pcd_315_translated.paint_uniform_color([1, 0, 0])
    pcd_315_translated.translate((0, 0.2, 0))

    #This visualises all the data clouds.
    o3d.visualization.draw_geometries([pcd_315_reframed, pcd_315_translated, pcd_000, pcd_000_translated], left = 0, top = 40)


#These functions are here because I thought it made the code look neater to seperate them.
#This is just the standard 'roll' matrix around the 'x' axist
def make_x_axis_rotation_matrix(angle):
     return np.array([1, 0, 0, 0, np.cos(angle), -np.sin(angle), 0, np.sin(angle), np.cos(angle)]).reshape(3,3)

#This is just the standard 'pitch' matrix around the 'y' axist
def make_y_axis_rotation_matrix(angle):
     return np.array([np.cos(angle), 0, np.sin(angle), 0, 1, 0, -np.sin(angle), 0, np.cos(angle)]).reshape(3,3)

#This is just the standard 'yaw' matrix around the 'x' axist
def make_z_axis_rotation_matrix(angle):
     return np.array([np.cos(angle), -np.sin(angle), 0, np.sin(angle), np.cos(angle), 0, 0, 0, 1]).reshape(3,3)

if __name__ == "__main__":
    main()
