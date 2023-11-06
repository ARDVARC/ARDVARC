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

## Vector and Frame Naming Convention
#### Vectors
* `vec_a2b_in_frame`
    * `vec` - dentotes it's a vector
    * `a2b` - denotes the vectors originates at `a` and terminates at `b`
    * `in_frame` - denotes the frame that the vector is expressed in

Example:

`vec_uas2rgv1_in_enu` - a vector from the UAS to RGV1 expressed in the ENU reference frame



#### Rotations
* `object_a2b`
    * `object` - denotes the type of rotation parameter:
        * `q` - for unit quaternion
        * `mrp` - for modified rodrigues parameter
        * `dcm` - for direction cosine matrix
    * `a2b` - denotes that the rotation parameter transforms a vector from being expressed in frame `a` to being expressed in frame `b`
* `theta_x` - denotes the frame tranfer angle around the `x` axis

Example: 
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
* `camExtr` - Refering to camera extrinsics
* `camIntr` - Refering to camera intrinsics


### TODO
- [ ] quaternion conventions
- [ ] convention for rates
- [ ] define all frames used

