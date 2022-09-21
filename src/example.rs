use rustik::{Chain};

fn main() {
    // Initialize Chain object that loads the robot URDF
    let robot = Chain::<f32>::new(String::from("models/kuka_iiwr.urdf"), String::from("world_joint"));

    // Random joint configuration
    let config = vec![-0.57, 0.87, -0.65, -0.04, 2.27, 0.83, -2.44];

    // Forward kinematics for "ee_fixed_joint"
    let xf_target = robot.fk(&config, &String::from("ee_fixed_joint"));
    println!("{:}", xf_target);

    // Jacobian for "ee_fixed_joint"
    let jac_target = robot.jacobian(&config, &String::from("ee_fixed_joint"));
    println!("{:}", jac_target);
}
