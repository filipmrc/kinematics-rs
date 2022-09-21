extern crate nalgebra as na;

use criterion::{black_box, criterion_group, criterion_main, Criterion, BenchmarkId};
use rustik::{Chain};
use na::{Vector6};

pub fn bench_fk(c: &mut Criterion) {
    let config = vec![-1.6333352864886936, 1.9904832183698629, -1.058632445175561, 0.9097905013220551, 0.8112221073340007, -2.0020893277255416];
    let robot = Chain::new(String::from("models/ur10_mod.urdf"), String::from("world_joint"));
    let mut screws: Vec<Vector6<f64>> = vec![];
    for joint_name in &robot.parents["ee_fixed_joint"] {
        screws.push(robot.screws[joint_name])
    }
    c.bench_with_input(BenchmarkId::new("robot", "ur10"),
                     &config,
                     |b, c| {b.iter(|| robot.fk(c, black_box(&String::from("ee_fixed_joint"))))});

    let config = vec![-0.5770057653280771, 0.8797884979496464, -0.6500974986020784, -0.04409327129573626, 2.270870130215024, 0.8326346174333068, -2.44789448732016];
    let robot = Chain::new(String::from("models/kuka_iiwr.urdf"), String::from("world_joint"));
    let mut screws: Vec<Vector6<f64>> = vec![];
    for joint_name in &robot.parents["ee_fixed_joint"] {
        screws.push(robot.screws[joint_name])
    }

    c.bench_with_input(BenchmarkId::new("robot", "kuka"),
                     &config,
                     |b, c| {b.iter(|| robot.fk(c, black_box(&String::from("ee_fixed_joint"))))});
}

criterion_group!(benches, bench_fk);
criterion_main!(benches);
