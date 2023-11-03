# ARDVARC
Team ARDVARC's github repository

## Team Members (open a pr to put urs below)
* Tim
* Lyon


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
`vec_a2b_in_frame` \ 
`vec` - dentotes it's a vector \ 
`a2b` - denotes the vectors originates at `a` and terminates at `b` \ 
`in_frame` - denotes the frame that the vector is expressed in \ 

#### Rotations
`object_a2b` \ 
`object` - denotes the type of rotation parameter: \ 
    `q` - for unit quaternion \ 
    `mrp` - for modified rodrigues parameter \ 
    `dcm` - for direction cosine matrix \ 
`a2b` - denotes that the rotation parameter transforms a vector from being expressed in frame `a` to being expressed in frame `b` \ 

### TODO
- [ ] quaternion conventions
- [ ] convention for rates

