extern crate nalgebra as na;
use na::{Vector3,
         Vector6,
         Matrix3,
         Matrix4,
         Matrix6,
         RealField,
         Rotation3,
         Isometry3
};

use urdf_rs::{Joint, Robot};

pub fn wedge<T: RealField>(w:&Vector3<T>) -> Matrix3<T> {
    return w.cross_matrix();
}

pub fn left_jacobian_so3<T: RealField + Copy>(axis: &Vector3<T>, angle: T) -> Matrix3<T>{
    // Computes left jacobian of SO3 matrices
    // NOTE implement small angle rule
    let ax_wedge = wedge(axis);

    if angle.abs() < T::from_f32(1e-6).unwrap() {
        Matrix3::identity() + ax_wedge*T::from_f32(0.5).unwrap()
    } else {
        Matrix3::identity()*(angle.sin()/angle) + ax_wedge*((T::one() - angle.cos())/(angle)) + (axis*axis.transpose())*((T::from_f32(1.0).unwrap() - angle.sin())/(angle))
    }
}

pub fn exp_so3<T: RealField + Copy>(axis: &Vector3<T>, angle: T) -> Matrix3<T> {
    // Computes an SO3 matrix from angle + rotation axis representation
    // DEPRECATED explicit implementation
    // let ax_wedge = wedge(&axis);
    // Matrix3::identity() + ax_wedge*((1.0 - angle.cos())/(angle.powi(2))) + ((angle - angle.sin())/(angle.powi(3)))*(ax_wedge*ax_wedge)

    Rotation3::new(axis*angle).into_inner()
}

pub fn exp_se3<T: RealField + Copy>(screw: &Vector6<T>, angle: T) -> Matrix4<T> {
    // Computes an SE3 matrix from angle + screw axis representation
    // NOTE passing by reference because we only borrow screws and angles
    let rho = Vector3::<T>::new(screw[3], screw[4], screw[5]);
    let axis = Vector3::<T>::new(screw[0], screw[1], screw[2]);
    let jac = left_jacobian_so3(&axis, angle);
    Isometry3::new(jac*rho*angle, axis*angle).to_matrix()
}

pub fn adj_se3<T: RealField + Copy>(xf: &Matrix4<T>) -> Matrix6<T> {
    // Constructs SE3 adjoint matrix
    let rot_mat = xf.fixed_slice::<3,3>(0,0);
    let p = xf.fixed_slice::<3,1>(0,3);
    let p_wedge = wedge(&Vector3::new(p[0], p[1], p[2]));
    let mut adj: Matrix6<T> = Matrix6::from_element(T::zero());
    adj.index_mut((..3,..3)).copy_from(&rot_mat);
    adj.index_mut((3..,..3)).copy_from(&(p_wedge*rot_mat));
    adj.index_mut((3..,3..)).copy_from(&rot_mat);
    adj
}

pub fn parse_from_root(path: String, name: String) -> (Robot, Vec<Joint>) {
    // parse URDF curtesy of urdf_rs
    let urdf = urdf_rs::read_file(path).unwrap();
    let urdf_joints: Vec<Joint> = urdf.joints.clone();

    // find root joint TODO error handling
    let root_joint = urdf_joints.iter()
                                .find(|&x| x.name == name)
                                .unwrap();

    // vector of joints ordered sequentally
    let mut joints: Vec<Joint> = vec![root_joint.clone()];

    // parse the rest by looking at connectivity
    for joint in urdf_joints {
        if joint.parent.link == joints.last().unwrap().child.link {
            joints.push(joint.clone())
        }
    };
    (urdf, joints)
}

pub fn revolute_joint_screw<T: RealField + Copy>(xf_joint: &Matrix4::<T>, rev_axis: [T; 3]) -> Vector6::<T> {
        //get screw axis for revolute joint////////////////////////////////////////
        let origin = Vector3::<T>::from(xf_joint.fixed_slice::<3, 1>(0,3));
        let rot_axis = Vector3::<T>::from(rev_axis);
        let axis = Vector3::<T>::from(xf_joint.fixed_slice::<3, 3>(0,0)*rot_axis); // rotation axis of joint
        let rho = Vector3::<T>::from(-axis.cross(&origin));
        Vector6::<T>::new(axis[0], axis[1], axis[2], rho[0], rho[1], rho[2])
}
