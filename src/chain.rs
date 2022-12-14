extern crate nalgebra as na;
extern crate urdf_rs;

use na::{IsometryMatrix3, Matrix4, Matrix6xX, RealField, Rotation3, Translation3, Vector6};
use rand::distributions::Uniform;
use rand::Rng;
use std::collections::HashMap;
use urdf_rs::{JointType, Robot};

use crate::kinematics::{_fk, _jac, parse_from_root, revolute_joint_screw};
use crate::utils::{adj_se3, exp_se3};

pub struct Chain<T: RealField + Copy> {
    pub urdf: Robot,
    pub xfs_home: HashMap<String, Matrix4<T>>,
    pub screws: HashMap<String, Vector6<T>>,
    pub parents: HashMap<String, Vec<String>>,
    pub active_joints: Vec<String>,
}

impl<T: RealField + Copy> Chain<T> {
    pub fn new(path: String, root_name: String) -> Self {
        // get parsed urdf object and ordered vec of all joints
        let (urdf, joints) = parse_from_root(path, root_name);

        // vector containing an identity matrix
        let mut xf = IsometryMatrix3::identity().to_matrix();
        let mut xfs_home: HashMap<String, Matrix4<T>> = HashMap::new();
        let mut screws: HashMap<String, Vector6<T>> = HashMap::new();
        let mut parents: HashMap<String, Vec<String>> = HashMap::new();
        let mut active_joints = Vec::<String>::new();

        for joint in &joints {
            // load rpy and xyz w.r.t, previous joint + local actuation axis
            let rpy = joint.origin.rpy.map(|x| T::from_f64(x).unwrap());
            let xyz = joint.origin.xyz.map(|x| T::from_f64(x).unwrap());
            let joint_axis = joint.axis.xyz.map(|x| T::from_f64(x).unwrap());

            // get rotation and transform matrix
            let rot_rel = Rotation3::<T>::from_euler_angles(rpy[0], rpy[1], rpy[2]);
            let trans_rel = Translation3::<T>::from(xyz);
            let xf_rel = IsometryMatrix3::<T>::from_parts(trans_rel, rot_rel).to_matrix();

            // add joint pose to list of joint poses at zero config
            xf = xf * xf_rel;
            xfs_home.insert(joint.name.clone(), xf.clone());

            parents.insert(joint.parent.link.clone(), active_joints.clone());

            // get screw vector for that particular joint if active
            if joint.joint_type == JointType::Revolute {
                active_joints.push(joint.name.clone());
                screws.insert(joint.name.clone(), revolute_joint_screw(&xf, joint_axis));
            }

            parents.insert(joint.name.clone(), active_joints.clone());
        }

        parents.insert(
            joints.last().unwrap().child.link.clone(),
            active_joints.clone(),
        );

        Chain {
            urdf: urdf,
            xfs_home: xfs_home,
            screws: screws,
            parents: parents,
            active_joints: active_joints,
        }
    }

    pub fn fk(&self, config: &Vec<T>, target: &String) -> Matrix4<T> {
        // Perform forward kinematics for target frame given config vector./////
        let screws = self.parents[target]
            .iter()
            .map(|k| self.screws[k])
            .collect();
        _fk(&screws, &self.xfs_home[target], &config[..screws.len()])
    }

    pub fn jacobian(&self, config: &Vec<T>, target: &String) -> Matrix6xX<T> {
        // Get Jacobian for target frame given config vector./////
        let screws = self.parents[target]
            .iter()
            .map(|k| self.screws[k])
            .collect();
        _jac(&screws, &config[..screws.len()])
    }

    pub fn fk_and_jac(&self, config: &Vec<T>, target: &String) -> (Matrix4<T>, Matrix6xX<T>) {
        // Get FK and Jacobian for target frame given config vector./////
        let mut jac = Matrix6xX::<T>::from_element(self.parents[target].len(), T::zero());
        let mut xf = Matrix4::<T>::identity();
        for (idx, joint_name) in self.parents[target].iter().enumerate() {
            xf *= exp_se3(&self.screws[joint_name], config[idx]);
            let adj = adj_se3(&xf);
            jac.index_mut((.., idx))
                .copy_from(&(adj * self.screws[joint_name]));
        }
        (xf * self.xfs_home[target], jac)
    }

    pub fn dof(&self) -> usize {
        self.screws.len()
    }

    pub fn random_config(&self) -> Vec<T> {
        let mut rng = rand::thread_rng();
        let range = Uniform::new(-std::f64::consts::PI, std::f64::consts::PI);
        let mut config = Vec::<T>::with_capacity(self.dof());

        for _ in 0..self.screws.len() {
            config.push(T::from_f64(rng.sample(&range)).unwrap());
        }
        config
    }
}
