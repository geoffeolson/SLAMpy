# Covariance Matrix
I have done another review of the covariance matrices coming from EKF, and here are my conclusions.

### 1. MOTION COVARIANCE MATRIX
***Our current solution for this seems good.*** As shown from the code snippet below, sigma_control is the motion covariance matrix in the control space [right, left].  It depends on the control values of left and right.  So the greater the left the greater the variance in the left coordinate. [left, right] is not the space used to optimize the pose, however sigma_control is multiplied by dg_dcontrol, which converts the covariance from [left, right] space to the correct 3D pose space. The values used to create sigma_control and dg_dcontrol do not change when the optimizer changes the pose, so this covariance matrix does not need to be recalculated with each iteration.
```
left, right = control
a1 = self.control_motion_factor
a2 = self.control_turn_factor
Gl2 = (a1 * left)**2 + (a2 * (left - right))**2
Gr2 = (a1 * right)**2 + (a2 * (left - right))**2
sigma_control = diag([Gl2, Gr2])
Vt = self.dg_dcontrol(self.state, control, self.robot_width)
Rt = np.dot(Vt, np.dot(sigma_control, Vt.T))
```
 
 
### 2. OBSERVATION COVARIANCE MATRIX
***Our current solution needs to change as described below.  Please review the changes in uploaded files.*** As shown from the code snippet below, the current covariance matrix for the observation in EKF is in polar coordinates:
```
 sigma_r = self.measurement_distance_stddev
 sigma_a = self.measurement_angle_stddev
 Q = np.diag([sigma_r^2, sigma_a^2])
 ```
 This makes sense, because the error in distance will be different than error in angle.  Distance and angle are also likely to be independent (i.e. uncorrelated). This means the GraphSLAM class should be modified to change the following in polar coordinates:
 
 - (z) measurement
 - (e) error
 - (J) jacobian matrix
 - (Sigma/Omega) covariance/information matrix
 
 Because of this we should reuse some code from EKF in GraphSLAM. This goes against our original design intent, but these calculations may be complex, so they should not be duplicated.  I think you may have been illuding to this in some of your code proposals, but I was not understanding the significance.  
 
 The functions we need from EKF are h() and dh_dstate().  These two functions are static functions of EKF, so internal changes to EKF objects will not effect these functions.  Both these functions take the constant parameter scanner_displacement.  GraphSLAM will have a member varible scanner_displacement that can be set when the the object is initialized.

 Please review changes in the uploaded files... 

