extern crate nalgebra as na;
use crate::utils::{adj_se3, exp_se3};
use na::{Matrix4, Matrix6xX, RealField, Vector3, Vector6};
use urdf_rs::{Joint, Robot};

pub fn parse_from_root(path: String, name: String) -> (Robot, Vec<Joint>) {
    // parse URDF curtesy of urdf_rs
    let urdf = urdf_rs::read_file(path).unwrap();
    let urdf_joints: Vec<Joint> = urdf.joints.clone();

    // find root joint TODO error handling
    let root_joint = urdf_joints.iter().find(|&x| x.name == name).unwrap();

    // vector of joints ordered sequentally
    let mut joints: Vec<Joint> = vec![root_joint.clone()];

    // parse the rest by looking at connectivity
    for joint in urdf_joints {
        if joint.parent.link == joints.last().unwrap().child.link {
            joints.push(joint.clone())
        }
    }
    (urdf, joints)
}

pub fn revolute_joint_screw<T: RealField + Copy>(
    xf_joint: &Matrix4<T>,
    rev_axis: [T; 3],
) -> Vector6<T> {
    //get screw axis for revolute joint////////////////////////////////////////
    let origin = Vector3::<T>::from(xf_joint.fixed_slice::<3, 1>(0, 3));
    let rot_axis = Vector3::<T>::from(rev_axis);
    let axis = Vector3::<T>::from(xf_joint.fixed_slice::<3, 3>(0, 0) * rot_axis); // rotation axis of joint
    let rho = Vector3::<T>::from(-axis.cross(&origin));
    Vector6::<T>::new(axis[0], axis[1], axis[2], rho[0], rho[1], rho[2])
}

pub fn _fk<T: RealField + Copy>(
    screws: &Vec<Vector6<T>>,
    xf_home: &Matrix4<T>,
    config: &[T],
) -> Matrix4<T> {
    let mut xf = Matrix4::<T>::identity();
    for idx in 0..config.len() {
        xf *= exp_se3(&screws[idx], config[idx]);
    }
    xf * xf_home
}

pub fn _jac<T: RealField + Copy>(screws: &Vec<Vector6<T>>, config: &[T]) -> Matrix6xX<T> {
    let mut jac = Matrix6xX::<T>::from_element(screws.len(), T::zero());
    let mut xf = Matrix4::<T>::identity();
    for idx in 0..config.len() {
        xf *= exp_se3(&screws[idx], config[idx]);
        let adj = adj_se3(&xf);
        jac.index_mut((.., idx)).copy_from(&(adj * screws[idx]));
    }
    jac
}
