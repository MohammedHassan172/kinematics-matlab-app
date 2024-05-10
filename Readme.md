# GUI app for robots kinematics using matlab app designer

This application was created using MATLAB App Designer for a college project, and I'm sharing it to potentially assist others in developing their own MATLAB apps or to aid in studying robot kinematics.


## Main features:

1. Forward Kinematics : 
    * Using DH parameters, you can compute the position of the robot arm and plot it.

2. Inverse Kinematics : 
    * Using the numerical method of pseudo-inverse Jacobian, you can compute the values of the joints (revolute or prismatic) required to reach a specified position. Additionally, you can show various solutions on a single plot by changing the initial values of the joints.

 3. Workgspace: 
    * You can visualize the Workgspace of a robotic arm by defining the lower and upper limits of each joint.

### Example video:

<video width="720" height="360" controls>
  <source src="Example_video.mp4" type="video/mp4">
</video>

## installation :
Simply drag and drop the MATLAB package file into the MATLAB console, then navigate to your apps to use it.

![gif](gif.gif)

# ! Disclaimer !:

This is a very early build, so some bugs may be present. hopefully I will include more updates and fixes soon. Thank you for your understanding.