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

Example: 

`quat_enu2uasBody` - an attitude quaternion that takes a vector expressed in the ENU frame to the UAS body frame

`vec_uas2rgv1_in_uasBody = quat_enu2uasBody * vec_uas2rgv1_in_enu * quat_enu2uasBody.conj() `


#### Reference Frames
* `uasBody` - frame fixed at UAS center of mass (TBR) with x-axis out the front (TBR) and y-axis out the (TBD)
* `inert` - Non-rotating inertial coordinate frame used as areference for angular rotation measurements
* `bluetooth` - frame fixed (TBD) and aligned with (TBD)

### TODO
- [ ] quaternion conventions
- [ ] convention for rates
- [ ] define all frames used

