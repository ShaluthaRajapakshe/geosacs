extern crate nalgebra as na;
use na::{Vector3, UnitQuaternion, Quaternion, Rotation3, Unit, Translation3};
use std::convert::TryFrom;

// use ncollide3d::math::Point as OtherPoint;
// use ncollide3d::nalgebra::Translation3 as OtherTranslation;
// use ncollide3d::nalgebra::UnitQuaternion as OtherUnitQuaternion;
// use ncollide3d::nalgebra::Quaternion as OtherQuaternion;
// use ncollide3d::math::Vector as OtherVector;
// use ncollide3d::shape::{Polyline,Segment,Ball,Cuboid,ConvexHull};
// use ncollide3d::query;
// use ncollide3d::query::ClosestPoints;
// use ncollide3d::nalgebra::geometry::Isometry3;

use optimization_engine::constraints::{Constraint, Rectangle};
use optimization_engine::{Optimizer, Problem, SolverError, panoc::*,core::ExitStatus};

use std::borrow::BorrowMut;
use std::panic::catch_unwind;
use std::sync::{Mutex};
use std::cmp;

extern crate libc;
use libc::c_char;
use std::ffi::CStr;
use std::str;

use rand::Rng;

use k::*;

type Robot = k::Chain<f64>;

struct SolverState {
    robot_panoc_cache : PANOCCache,
    robot : Robot
}

static mut STATE: Option<Mutex<SolverState>> = None;

fn charp_to_str<'a>(p : *mut c_char) -> &'a str {
    let c_str : &CStr = unsafe {CStr::from_ptr(p)};
    c_str.to_str().unwrap()
}

fn _init(urdf: &str) {
    let robot = k::Chain::<f64>::from_urdf_file(urdf).unwrap();
    // Create a set of joints from end joint
    // let robot_panoc_cache = PANOCCache::new(7, 1e-6, 100000);
    let robot_panoc_cache = PANOCCache::new(6, 1e-6, 100000);


    unsafe {
        *STATE.borrow_mut() = Some(
            Mutex::new(
                SolverState {robot_panoc_cache, robot}
            )
        );
    }
}

#[no_mangle]
pub extern "C" fn init(urdf_p: *mut c_char) -> bool {
    
    let urdf = charp_to_str(urdf_p);
    match catch_unwind(|| {
        _init(urdf);
    }) {
        Err(err) => {
            println!("{:?}", err);
            false
        },
        _ => {
            return true;
        }
    }
}

fn get_state<'a>() -> &'a mut Mutex<SolverState> {
    unsafe { STATE.as_mut().unwrap() }
}

fn get_gaussian(x: f64, c: f64, k: i32) -> f64{
    (-(x).powi(2)/(2.*c.powi(2))).powi(k).exp()
}


pub fn groove_loss(x_val: f64, offset: f64, top_width: f64, bottom_width: i32, depth: f64, poly_factor: f64, g: i32) -> f64 {
    2.*depth*depth-( (-((x_val - offset)/top_width).powi(2*bottom_width)) * (2.0 * depth.powi(2) ) ).exp() + poly_factor * (x_val - offset).powi(g)
}

fn position_cost(current_position: &Vector3<f64>, desired_position: &Vector3<f64>) -> f64 {

    // println!("curr position  =  {}", current_position);
    // println!("desired position  =  {}", desired_position);
    let n = (current_position - desired_position).norm();
    // println!("position cost =  {}", n);
    n * n
}

fn movement_cost(state: &[f64], init_state: &[f64],lb: &[f64], hb: &[f64]) -> f64 {
    let mut n = 0.0;
    // for i in 0..7 {
    for i in 0..6 {
        n+=((state[i]-init_state[i])/(hb[i]-lb[i])).powi(2);
    }
    // println!("movement_cost  =  {}", n);
    n
}

fn norm_angle(a: f64) -> f64{
    (a+3.*std::f64::consts::PI)%(2.*std::f64::consts::PI)-std::f64::consts::PI
}

fn diff_angle(a1: f64, a2: f64)-> f64 {
    norm_angle(norm_angle(a1)-(a2))
}

fn joint_limit_cost(state: &[f64], lb: &[f64], hb: &[f64]) -> f64 {
    let mut n = 0.0;
    // for i in 0..7 {
    for i in 0..6 {
        let x = ((state[i]-lb[i])/(hb[i]-lb[i]) - 0.5).abs();
        n+=0.2*(1./(0.5-x).sqrt() - (1./0.5 as f64).sqrt());                           //(groove_loss((state[i]-lb[i])/(hb[i]-lb[i]), 0.5,0.5,12,2.,2.,2);
    }
    println!("joint_limit_cost =  {}", n);
    n
}

fn rotation_cost(current_rotation: &UnitQuaternion<f64>, desired_rotation: &UnitQuaternion<f64>) -> f64 {
    // println!("curr rotation  =  {}", current_rotation);
    // println!("desired rotation  =  {}", desired_rotation);
    let a = current_rotation.angle_to(desired_rotation);
    // println!("rotation_cost  =  {}", a);
    a*a
}

fn robot_finite_difference(f: &dyn Fn(&[f64], &mut f64) -> Result<(), SolverError>, u: &[f64], grad: &mut [f64]) -> Result<(), SolverError> {
    let h = 1000.0 * f64::EPSILON;
    let mut f0 = 0.0;
    f(u, &mut f0).unwrap();

    // let mut x = [0.0,0.0,0.0,0.0,0.0,0.0,0.0];
    let mut x = [0.0,0.0,0.0,0.0,0.0,0.0];
    // for i in 0..7 {
    for i in 0..6 {
        x[i] = u[i];
    }

    // for i in 0..7 {
    for i in 0..6 {
        let mut fi = 0.0;
        x[i] += h;
        f(&x, &mut fi).unwrap();
        grad[i] = (fi - f0) / h;
        x[i] -= h;
    }

    Ok(())
}

#[no_mangle]
pub extern "C" fn solve(robot_start: *mut [f64;6], link_name: *mut c_char, 
                        goal_x: *mut [f64;3], goal_q: *mut [f64;4], vel: *mut [f64;3], errors_start: *mut [bool;4], w: f64) {
       
    let position = unsafe{Vector3::from(std::ptr::read(goal_x))};
    let orientation = unsafe{UnitQuaternion::from_quaternion(Quaternion::from(std::ptr::read(goal_q)))};
    let velocity = unsafe{Vector3::from(std::ptr::read(vel))};
    let mut errors = unsafe{std::ptr::read(errors_start).clone()};

    // println!("Inside cost, position =  {}", position);
    // println!("Inside cost, weight =  {}", w);

    let state = get_state().get_mut().unwrap();
    let robot = &mut state.robot;
    let name = charp_to_str(link_name); // lio_tcp_joint

    match robot.find(&name) {
        None => {

            for node in robot.iter() {
                println!("{:?}", (*node.joint()).name);
            }
            println!("Couldn't find: {}", name);
        },
        Some(_v) => {

        }
    };
    let init_state = unsafe{std::ptr::read(robot_start).clone()};
    robot.set_joint_positions_clamped(&init_state);
    robot.update_transforms();

    // println!("Inside Solver");

    let robot_state = optimize_robot(robot_start, &position, &orientation, &mut errors, robot, name, w);
    
    robot.set_joint_positions_clamped(&robot_state);
    robot.update_transforms();
    let trans = robot.find(&name).unwrap().world_transform().unwrap();

    unsafe {
        *robot_start = robot_state;
        *goal_x = [trans.translation.x,trans.translation.y,trans.translation.z];
        *errors_start = errors;
    }
}

fn optimize_robot(robot_start: *mut [f64;6], position: &Vector3<f64>, orientation: &UnitQuaternion<f64>, errors: &mut [bool;4],robot: &mut Robot, name: &str, w: f64) -> [f64; 6] {
    let state = get_state().get_mut().unwrap();

    let panoc_cache = &mut state.robot_panoc_cache;
    // println!("Inside optimize, weight =  {}", w);


    // let mut lb = [
    //     //Panda Joint
    //     -1.0472, //-60
    //     -1.0472, //-60
    //     -1.0472, //-60
    //     -1.0472, //-60
    //     -1.0472, //-60
    //     -1.0472, //-60
    // ];

    // let mut ub = [
    //     1.0472, //60
    //     1.0472,  //60
    //     1.0472, //60
    //     1.0472,  //60
    //     1.0472, //60
    //     1.0472,  //60
    // ];

    // let mut lb = [
    //     //Panda Joint
    //     -2.8973,
    //     -1.7628,
    //     -2.8973,
    //     -2.8973,
    //     -0.0175,
    //     -2.8973,
    // ];

    // let mut ub = [
    //     2.8973,
    //     1.7628,
    //     2.8973,
    //     2.8973,
    //     3.7525,
    //     2.8973,
    // ];

    let mut lb = [
        //lio Joint
        -2.792527,
        -1.745329,
        -1.745329,
        -2.792527,
        -1.745329,
        -2.792527,
    ];
    
    let mut ub = [
        2.792527,
        1.745329,
        1.745329,
        2.792527,
        1.745329,
        2.792527,
    ];

    let mut middle = [0.;6];
    for i in 0..6 {
        middle[i] = (ub[i] + lb[i]) / 2.;
    }

    for i in 0..6 {
        lb[i] += 1e-1;
        ub[i] -= 1e-1;
    }
    let bounds = Rectangle::new(Some(&lb), Some(&ub));

    let mut robot_state = unsafe{std::ptr::read(robot_start).clone()};
    let init_state = unsafe{std::ptr::read(robot_start).clone()};

    // println!("Inside Optimser, Roboto  State {:#?}", init_state);

    let cost = |u: &[f64], c: &mut f64| {
        for i in 0..6{
            if u[i].is_nan(){
                *c = 10.;
                return Ok(())
            }
        }
        robot.set_joint_positions_clamped(&u);
        robot.update_transforms();
        let trans = robot.find(&name).unwrap().world_transform().unwrap();

        *c = 100.0 * position_cost(&trans.translation.vector, &position);
        *c += w*rotation_cost(&trans.rotation, &orientation);
        *c += movement_cost(&u, &init_state, &lb, &ub);
        // *c += 0.1 * joint_limit_cost(&u, &lb, &ub);
        

        // println!("Inside cost, cost =  {}", *c);

        Ok(())
    };

    let dcost = |u: &[f64], grad: &mut [f64]| {
        robot_finite_difference(&cost, u, grad)
    };

    
    
    let mut cur_cost: f64 = 0.0;
    cost(&robot_state, &mut cur_cost).unwrap();
    let problem = Problem::new(&bounds, dcost, cost);
    let mut opt = PANOCOptimizer::new(problem, panoc_cache).with_max_iter(50).with_max_duration(std::time::Duration::from_millis(7));

    bounds.project(&mut robot_state);
    let _status = opt.solve(&mut robot_state);
    if _status.is_err(){
        errors[0]=true;
    }
    else{
        if _status.unwrap().exit_status() == ExitStatus::NotConvergedOutOfTime {
            errors[1]=true;
        }
        else{}
        if _status.unwrap().exit_status() == ExitStatus::NotConvergedIterations {
            errors[1]=true;
        }
        else{}
    }
    robot_state
}