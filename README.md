# ARDVARC
Team ARDVARC's github repository

## Team Members (open a pr to put urs below)
* Tim
* Lyon
* Aidan

## How to make a contribution
1. Create a new issue
2. Link a branch to the issue (main should always be the source branch)
3. Checkout that branch locally
4. Make your changes, add, commit, push
5. Make a new PR to pull your working branch into main
6. Assign someone to review and complete your pr
7. Bother them until it goes through

## General coding policy
- When creating a new computational capability (rotation matrix, least squares estimator, filtering, etc.) please make a completely new  
function properly named to make re-use easier in the future. **If you think someone might use it somewhere else make it a function!**
- Try to comment, we will be going over each others code frequently and it can be hard to follow otherwise. 
- Communicate what you are working on to avoid conflicts
- Don't complete other people's PRs
- Don't resolve other peoples comments
- Small, refined, and often changes are better

## Vector and Frame Naming Convention
#### Vectors
* `vec_a2b_in_frame`
    * `vec` - denotes it's a vector
    * `a2b` - denotes the vectors originates at `a` and terminates at `b`
    * `in_frame` - denotes the frame that the vector is expressed in

Example:

`vec_uas2rgv1_in_enu` - a vector from the UAS to RGV1 expressed in the ENU reference frame


#### Matrices
* `trix` - Matrix (2D)
* `trixN` - Matrix (N-dimensional)
    * `trix3` for 3D matrix, `trix4` for 4D matrix, etc.

Examples:

`trix_cameraImage` - a matrix containing a camera image

`trix3_pressure` - a 3D matrix containing pressure values throughout a 3D space

# Nesting

If a higher dimension object exists to contain many similar lower dimension objects, show that in the name

* `trix_vec_a2b_frame`
    * `trix` - the variable is a 2D matrix
    * `vec_a2b_frame` - each row is a similar vector from a to b in frame
* `trix3_trix_info`
    * `trix3` - the variable is a 3D matrix
    * `trix_info` - each page is a similar 2D matrix of "info"

Examples:

* `trix_vec_uas2rgv_uasBody` - a 2D matrix where each row is a different vector from the UAS to the RGV in the UAS body frame. Each row may correspond to a different time
* `trix3_dcm_uasBody2ned` - a 3D matrix where each page is a different direction cosine matrix from the UAS body frame to the NED frame. Each page may correspond to a different time

Notes:

* In general, each item should vary in the last dimension. For a 3D matrix containing 2D matrices, this means each 2D matrix is in a different page. For a 2D matrix containing vectors, this means each vector is in a different row.

#### Rotations
* `object_a2b`
    * `object` - denotes the type of rotation parameter:
        * `q` - for unit quaternion
        * `mrp` - for modified rodrigues parameter
        * `dcm` - for direction cosine matrix
    * `a2b` - denotes that the rotation parameter transforms a vector from being expressed in frame `a` to being expressed in frame `b`
* `theta_x` - denotes the frame transfer angle around the `x` axis

Examples: 

`theta_1_cam2uasBody` - an angle describing the rotation about the camera primary-axis to rotate it to the uasBody frame

`quat_enu2uasBody` - an attitude quaternion that takes a vector expressed in the ENU frame to the UAS body frame

`vec_uas2rgv1_in_uasBody = quat_enu2uasBody * vec_uas2rgv1_in_enu * quat_enu2uasBody.conj() `


#### Reference Frames
* `uasBody` - frame fixed at UAS center of mass (TBR) with x-axis out the front (TBR) and y-axis out the (TBD)
* `inert` - non-rotating inertial coordinate frame used as a reference for angular rotation measurements
* `bluetooth` - frame fixed (TBD) and aligned with (TBD)
* `enu` - Earth-fixed frame with origin at Southwest corner of the mission area with x-axis aligned with East, y-axis aligned with North, z-axis pointing straight-down
* `cam` - frame fixed at (TBD) with z-x plane aligned with camera's principal point and center


#### General items
* `camExtr` - Referring to camera extrinsics
* `camIntr` - Referring to camera intrinsics


### TODO
- [ ] quaternion conventions
- [ ] convention for rates
- [ ] define all frames used

