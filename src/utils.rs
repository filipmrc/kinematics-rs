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
        Matrix3::identity()*(angle.sin()/angle) + ax_wedge*((T::one() - angle.cos())/(angle)) + (axis*axis.transpose())*((T::one() - angle.sin())/(angle))
    }
}

pub fn exp_so3<T: RealField + Copy>(axis: &Vector3<T>, angle: T) -> Matrix3<T> {
    // Computes an SO3 matrix from angle + rotation axis representation
    let ax_wedge = wedge(&axis);
    Matrix3::identity() + ax_wedge*((T::one() - angle.cos())/(angle.powi(2))) + (ax_wedge*ax_wedge)*((angle - angle.sin())/(angle.powi(3)))
    // Rotation3::new(axis*angle).into_inner()
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
