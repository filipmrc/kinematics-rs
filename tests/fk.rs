use nalgebra::{dmatrix};
use rustik::{Chain};

// NOTE tests are for now based on FK results from GRAPHIK, should be expanded

#[test]
fn test_screws() {
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Test joint screws parsed by the library and compare to ground truth obtained from the GRAPHIK python library. //
    // NOTE: GT from GRAPHIK is modified due to its canoncial base frame being moved to the first actuated joint insead of
    // the physical base frame.
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // UR10
    let gt_screws = dmatrix![0., 0., 0., 0., 1., 0.;
                        0., 1., 1., 1., 0., 1.;
                        1., 0., 0., 0. ,0., 0.;
                        0., 0., -0.612, -1.1843 , 0., -1.1843;
                        0., 0., 0., 0. , 1.1843, 0.;
                        0., 0., 0., 0. , -0.163941, 0.1157;
    ];
    let robot = Chain::new(String::from("models/ur10_mod.urdf"), String::from("world_joint"));
    for (idx, name) in robot.active_joints.iter().enumerate() {
        assert!((gt_screws.column(idx) - robot.screws[name]).abs().sum() < 1.0e-6);
    };

    // KUKA IIWR
    let gt_screws = dmatrix![0., 0.,        0.,  0.,     0., 0., 0.;
                             0., 1.,        0., -1.,     0., 1., 0.;
                             1., 0.,        1.,  0. ,    1., 0., 1.;
                             0., -0.36,        0.,  0.78, 0., -1.18, 0.;
                             0., 0.,        0.,  0. ,    0., 0., 0.;
                             0., 0.,   0.,  0.,    -0., 0., 0.;
    ];
    let robot = Chain::<f64>::new(String::from("models/kuka_iiwr.urdf"), String::from("world_joint"));
    for (idx, name) in robot.active_joints.iter().enumerate() {
        assert!((gt_screws.column(idx) - robot.screws[name]).abs().sum() < 1.0e-6);
    };
}

#[test]
fn test_fk() {
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Test forward kineamtics for random config and compare to ground truth obtained from the GRAPHIK python library.
    // NOTE: GT from GRAPHIK is modified due to its canoncial base frame being moved to the first actuated joint insead of
    // the physical base frame.
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // UR10
    let config = vec![-1.6333352864886936, 1.9904832183698629, -1.058632445175561, 0.9097905013220551, 0.8112221073340007, -2.0020893277255416];
    let T_gt = dmatrix![-0.30469028,  0.70209945,  0.64359941,  0.16125383;
                        -0.01531924,  0.67202942, -0.74036598,-1.06391064;
                        -0.95232828, -0.23544177, -0.1940052,  -0.03745314;
                        0.0, 0.0, 0.0, 1.0];
    let robot = Chain::new(String::from("models/ur10_mod.urdf"), String::from("world_joint"));
    assert!((T_gt - robot.fk(&config, &String::from("ee_fixed_joint"))).abs().sum() < 1.0e-6);

    // KUKA IIWR
    let config = vec![-0.5770057653280771, 0.8797884979496464, -0.6500974986020784, -0.04409327129573626, 2.270870130215024, 0.8326346174333068, -2.44789448732016];
    let T_gt = dmatrix![ 0.43628054,  0.33805164,  0.8338947,   0.59856079;
                        -0.8799765,   0.35380273,  0.31696211, -0.33269581;
                        -0.18788466, -0.87209215,  0.45183475,  0.90813286;
                        0.0, 0.0, 0.0, 1.0];
    let robot = Chain::new(String::from("models/kuka_iiwr.urdf"), String::from("world_joint"));
    assert!((T_gt - robot.fk(&config, &String::from("ee_fixed_joint"))).abs().sum() < 1.0e-6);
}

#[test]
fn test_jacobian() {
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Test jacobian for random config and compare to ground truth obtained from the GRAPHIK python library.
    // NOTE: GT from GRAPHIK is modified due to its canoncial base frame being moved to the first actuated joint insead of
    // the physical base frame.
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // UR10
    let config = vec![-1.6333352864886936, 1.9904832183698629, -1.058632445175561, 0.9097905013220551, 0.8112221073340007, -2.0020893277255416];
    let J_gt = dmatrix![0.0,  9.98045077e-01,  9.98045077e-01, 9.98045077e-01,  1.67211236e-02,  6.43599407e-01;
                        0.0, -6.24982014e-02, -6.24982014e-02,-6.24982014e-02,  2.67022646e-01, -7.40365980e-01;
                        1.0,  0.0,  0.0, 0.0, -9.63545178e-01, -1.94005203e-01;
                        0.0,  0.0, -1.55854525e-02, 5.74460418e-03,  9.64577219e-01,  1.78675165e-01;
                        0.0,  0.0, -2.48886909e-01, 9.17366226e-02,  9.78715501e-02,  7.17926100e-03;
                        0.0,  0.0,  5.58888540e-01, 1.01828807e+00,  4.38617059e-02,  5.65345401e-01;];
    let robot = Chain::new(String::from("models/ur10_mod.urdf"), String::from("world_joint"));
    assert!((J_gt - robot.jacobian(&config, &String::from("ee_fixed_joint"))).abs().sum() < 1.0e-6);

    // KUKA IIWR
    let config = vec![-0.5770057653280771, 0.8797884979496464, -0.6500974986020784, -0.04409327129573626, 2.270870130215024, 0.8326346174333068, -2.44789448732016];

    let J_gt = dmatrix![0., 0.5455169184296017, 0.6458431523016438, -0.7575363936562216, 0.6494029826015326, -0.5388746630729301, 0.8338947036306463;
                        0., 0.8380998101103896, -0.4203775755351518, -0.45671871805228875, -0.45452788899460106, 0.29060956745529626, 0.31696211271820174;
                        1.0, 0.000000000004896588860146748, 0.6373141427995015, 0.46641893708017446, 0.6096557752653093, 0.7906707132564617, 0.4518347511623808;
                        0.0, -0.3017159316397403, 0.1513359271921429, 0.20431925776231916, 0.17765444283663986, -0.5366286472582854, -0.4381672383002741;
                        0.0, 0.19638609063465665, 0.23250353482897537, -0.6020023979873821, 0.2422403866758025, -0.889505984816935, 0.48683662155391083;
                        0.0, 0.0, 0.0, -0.2576363870336632,-0.008634894770328545, -0.038798239868654334, 0.4671543626596568;];

    let robot = Chain::new(String::from("models/kuka_iiwr.urdf"), String::from("world_joint"));
    assert!((J_gt - robot.jacobian(&config, &String::from("ee_fixed_joint"))).abs().sum() < 1.0e-6);
}
