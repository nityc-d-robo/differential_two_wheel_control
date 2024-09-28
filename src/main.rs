use num_traits::abs;
use safe_drive::topic::publisher;
use safe_drive::{
    context::Context, error::DynError, logger::Logger, pr_info, topic::publisher::Publisher
};
use safe_drive::msg::common_interfaces::geometry_msgs::msg;
use drobo_interfaces::msg::DiffDrive;
use core::f64::consts::PI;



const  WHEEL_DIA: f64 = 0.100;      //ホイールの直径［m］
const  ROBOT_CENTER_TO_WHEEL_DISTANCE: f64 = 0.37;  //ロボットの中心からホイールまでの距離[m]
const  STEERING_GEAR_RATIO :u16  = 3;

enum WheelAdress {
    Left    = 1,
    Right   = 2
}
enum Mode{
    CURRENT = 2,
    SPEED = 3,
    ANGLE = 4
}
// struct Component {
//     x: f64,
//     y: f64
// }

struct WheelOrder {
    phase:  bool,
    speed:   u16
}

fn main() -> Result<(), DynError> {
    // Create a context.
    let ctx = Context::new()?;

    // Create a node.
    let node = ctx.create_node("differential_two_wheel_control", None, Default::default())?;

    //Create a subscriber.
    let subscriber = node.create_subscriber::<msg::Twist>("cmd_vel", None)?;

    // Create a publisher.
    let diff_publisher = node.create_publisher::<DiffDrive>("/diff", None)?;

    // Create a logger.
    let logger = Logger::new("differential_two_wheel_control");

    let mut selector = ctx.create_selector()?;

    // let mut left_now_angle  = 0.0;
    // let mut right_now_angle = 0.0;

    selector.add_subscriber(
        subscriber, 
        Box::new(move |msg| {
        
            let (left_order, right_order) = move_chassis(-msg.linear.x, msg.linear.y, msg.angular.z);
            pr_info!(logger, "left_order:{}  {}  right_order:{}  {}", left_order.phase, left_order.speed, right_order.phase, right_order.speed);
            
            // send_speed(WheelAdress::Left as u8,0,  left_order.phase, left_order.speed, 0, &publisher);
            // send_speed(WheelAdress::Right as u8 , 0, right_order.phase, right_order.speed, 0, &publisher);

            send_pwm(left_order.phase, left_order.speed, right_order.phase, right_order.speed,&diff_publisher);

            // move_wheel(WheelAdress::LeftPower, WheelAdress::LeftSteering, left_order, &mut left_now_angle, &publisher);
            // move_wheel(WheelAdress::RightPower, WheelAdress::RightSteering, right_order, &mut right_now_angle, &publisher);
        }),
    );

    loop {
        selector.wait()?;
    }
}



fn move_chassis(_xrpm: f64, _yrpm: f64, _yaw: f64) -> (WheelOrder, WheelOrder) {
    // let speed_abs: f64 = (_xrpm*_xrpm +_yrpm*_yrpm).sqrt();
    // let rad: f64 = _yrpm.atan2(_xrpm);
    
    //回転成分　θ・R _yawの０基準を前方にしてロボットからの距離をかける
    let mut rotation_component: f64 = _yaw * 0.04 * ROBOT_CENTER_TO_WHEEL_DISTANCE;

    if (_xrpm < 10.0) {
        rotation_component *= 1.5;
    }
    let left_speed: i32 = (_xrpm + rotation_component) as i32;
    let right_speed: i32 = (_xrpm - rotation_component) as i32;

    let left_order = WheelOrder{
        phase : (left_speed < 0) as bool,
        speed : (abs(left_speed)  as f32 *3.5* 1.0) as u16
    };

    let right_order = WheelOrder{
        phase : (right_speed < 0) as bool,
        speed : (abs(right_speed) as f32 * 3.5* 1.0) as u16
    };
    (left_order, right_order)
}

fn send_pwm(_phase0: bool, _power0: u16, _phase1: bool, _power1: u16, _diff_publisher:  &Publisher<DiffDrive>){
    //dmotor_rosでsendPwmをするようにパブリッシュ
    let mut diff_msg =  DiffDrive::new().unwrap();
    diff_msg.left = if _phase0 {_power0 as i16} else {-1 *_power0 as i16};
    diff_msg.right = if _phase1 {-1 * _power1 as i16} else {_power1 as i16};

    let logger = Logger::new("differential_two_wheel_control");
    pr_info!(logger, "{:?} {:?}", diff_msg.left, diff_msg.right);
    let _ =_diff_publisher.send(&diff_msg);
}
