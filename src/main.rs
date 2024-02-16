use drobo_interfaces::msg::MdLibMsg;
use num_traits::abs;
use safe_drive::topic::publisher;
use safe_drive::{
    context::Context, error::DynError, logger::Logger, pr_info, topic::publisher::Publisher
};
use safe_drive::msg::common_interfaces::geometry_msgs::msg;
use core::f64::consts::PI;



const  WHEEL_DIA: f64 = 0.100;      //ホイールの直径［m］
const  ROBOT_CENTER_TO_WHEEL_DISTANCE: f64 = 0.37;  //ロボットの中心からホイールまでの距離[m]
const  STEERING_GEAR_RATIO :u16  = 3;

enum WheelAdress {
    Left    = 0,
    Right   = 2
}
enum Mode{
    PWM = 2,
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
    let publisher = node.create_publisher::<drobo_interfaces::msg::MdLibMsg>("md_driver_topic", None)?;

    // Create a logger.
    let logger = Logger::new("differential_two_wheel_control");

    let mut msg = drobo_interfaces::msg::MdLibMsg::new().unwrap();

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

            send_pwm(WheelAdress::Left as u8, 0,left_order.phase, left_order.speed, &publisher);
            send_pwm(WheelAdress::Right as u8, 0, left_order.phase, right_order.speed, &publisher);

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
    let rotation_component: f64 = _yaw * 0.1 * ROBOT_CENTER_TO_WHEEL_DISTANCE;

    let left_speed: i32 = (_xrpm + rotation_component) as i32;
    let right_speed: i32 = (_xrpm - rotation_component) as i32;

    // //各ホイールのx,y成分を出す。並進成分と回転成分を合成
    // let left_component = Component{
    //     x : (_xrpm) as f64,
    //     y : (_yrpm - rotation_component) as f64
    // };
    // let right_component = Component{
    //     x : (_xrpm) as f64,
    //     y : (_yrpm + rotation_component) as f64
    // };
    // let rear_left_component = Component{
    //     x : (_xrpm + (2_f64).sqrt()/2.0 * rotation_component) as f64,
    //     y : (_yrpm - (2_f64).sqrt()/2.0 * rotation_component) as f64
    // };
    // let rear_right_component = Component{
    //     x : (_xrpm + (2_f64).sqrt()/2.0 * rotation_component) as f64,
    //     y : (_yrpm + (2_f64).sqrt()/2.0 * rotation_component) as f64
    // };

    // //各ホイールの方向を出す
    // let front_left_direction: f64   = front_left_component.y.atan2(front_left_component.x);
    // let front_right_direction: f64  = front_right_component.y.atan2(front_right_component.x);
    // let rear_left_direction: f64    = rear_left_component.y.atan2(rear_left_component.x);
    // let rear_right_direction: f64   = rear_right_component.y.atan2(rear_right_component.x);


    // let front_left_speed: i64   = ((front_left_component.x.powf(2.0) + front_left_component.y.powf(2.0)).sqrt() / WHEEL_DIA) as i64;
    // let front_right_speed: i64  = ((front_right_component.x.powf(2.0) + front_right_component.y.powf(2.0)).sqrt() / WHEEL_DIA) as i64;
    // let rear_left_speed: i64    = ((rear_left_component.x.powf(2.0) + rear_left_component.y.powf(2.0)).sqrt() / WHEEL_DIA) as i64;
    // let rear_right_speed: i64   = ((rear_right_component.x.powf(2.0) + rear_right_component.y.powf(2.0)).sqrt() / WHEEL_DIA) as i64;

    let left_order = WheelOrder{
        phase : (left_speed < 0) as bool,
        speed : (abs(left_speed)  as f32 *3.0) as u16
    };

    let right_order = WheelOrder{
        phase : (right_speed < 0) as bool,
        speed : (abs(right_speed) as f32 * 3.0) as u16
    };

    // let rear_left_order = WheelOrder{
    //     direction : rear_left_component.y.atan2(rear_left_component.x) as f64,
    //     speed : ((rear_left_component.x.powf(2.0) + rear_left_component.y.powf(2.0)).sqrt() / WHEEL_DIA) as u16
    // };

    // let rear_right_order = WheelOrder{
    //     direction : rear_right_component.y.atan2(rear_right_component.x) as f64,
    //     speed : ((rear_right_component.x.powf(2.0) + rear_right_component.y.powf(2.0)).sqrt() / WHEEL_DIA) as u16
    // };

    (left_order, right_order)
}


// fn move_wheel(power_adress: WheelAdress, steering_adress : WheelAdress, wheel_order :WheelOrder, now_angle : &mut f64,publisher: &Publisher<drobo_interfaces::msg::MdLibMsg>){
//     //いい感じに制御しよう
//     let (power_phase, steering_speed, steering_angle) = calculate_steering(wheel_order.direction, now_angle);
    
//     send_pwm(power_adress as u8, 0, power_phase, wheel_order.speed, &publisher);

//     send_angle(steering_adress as u8, 0, steering_speed, steering_angle, 1000, &publisher);
// }


// fn calculate_steering(goal_angle:f64, now_angle : &mut f64) -> (bool, u16, i32) {
//     let mut power_phase  = false;
//     let mut target_angle = if((*now_angle - goal_angle) > PI/4.0) {
//         power_phase  = toggle_bool(power_phase);
//         (PI - (*now_angle - goal_angle))
//     }else {
//         (*now_angle - goal_angle)
//     };

//     target_angle *= STEERING_GEAR_RATIO as f64;
//     // if((*now_angle + target_angle as f64) > PI) {
//     //     target_angle = PI - target_angle;
//     // } else if((*now_angle + target_angle as f64) < -PI) {
//     //     target_angle = PI + target_angle;
//     // }
//     let steering_speed = (target_angle * 20.0)as u16;
//     *now_angle = target_angle;
//     (power_phase, steering_speed, target_angle as i32)
// }

// fn toggle_bool(mut target_bool:bool) -> bool{
//     if(target_bool){
//         target_bool = false;
//     } else {
//         target_bool = true;
//     }
//     return target_bool
// }

fn send_pwm(_adress: u8, _semi_id: u8, _phase: bool, _power: u16, _publisher:  &Publisher<drobo_interfaces::msg::MdLibMsg>){
    //dmotor_rosでsendPwmをするようにパブリッシュ
    let mut msg =  drobo_interfaces::msg::MdLibMsg::new().unwrap();
    msg.address = _adress as u8;
    msg.semi_id = _semi_id as u8;
    msg.mode = Mode::PWM as u8;
    msg.phase = _phase as bool;
    msg.power = _power as u16;

    let logger = Logger::new("differential_two_wheel_control");
    pr_info!(logger, "left_order:{:?}", msg);
    _publisher.send(&msg).unwrap()

}


fn send_speed(_adress: u8, _semi_id: u8, _phase: bool, _speed: u16, _angle: i32, _publisher:  &Publisher<drobo_interfaces::msg::MdLibMsg>){
    //dmotor_rosでsendSpeedをするようにパブリッシュ
    let mut msg =  drobo_interfaces::msg::MdLibMsg::new().unwrap();
    msg.address = _adress as u8;
    msg.semi_id = _semi_id as u8;
    msg.mode = Mode::SPEED as u8;
    msg.phase = _phase as bool;
    msg.power = _speed as u16;
    msg.angle = _angle as i32;

    _publisher.send(&msg).unwrap()

}

fn send_angle(_adress: u8, _semi_id: u8, _speed: u16, _angle: i32, _timeout: u16, _publisher:  &Publisher<drobo_interfaces::msg::MdLibMsg>){
    //dmotor_rosでsendSpeedをするようにパブリッシュ
    let mut msg =  drobo_interfaces::msg::MdLibMsg::new().unwrap();
    msg.address = _adress as u8;
    msg.semi_id = _semi_id as u8;
    msg.mode = Mode::ANGLE as u8;
    msg.power = _speed as u16;
    msg.angle = _angle as i32;
    msg.timeout = _timeout as u16;

    _publisher.send(&msg).unwrap()

}