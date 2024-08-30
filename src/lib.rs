use std::collections::HashMap;

pub struct DtwcSetting {
    pub l_id: usize,
    pub r_id: usize,
    pub robot_center_to_wheel_distance: f64,
}

impl DtwcSetting {
    pub fn move_chassis(
        &self,
        linear_x: f64,
        linear_y: f64,
        angular_z: f64,
    ) -> HashMap<usize, f64> {
        let [_xrpm, _yrpm, _yaw] = [-linear_x, linear_y, angular_z];
        //回転成分　θ・R _yawの０基準を前方にしてロボットからの距離をかける
        let mut rotation_component: f64 = _yaw * 0.04 * self.robot_center_to_wheel_distance;

        if _xrpm < 10.0 {
            rotation_component *= 1.5;
        }
        let left_speed: f64 = _yrpm - rotation_component;
        let right_speed: f64 = _yrpm + rotation_component;

        let mut re_motor_powar = HashMap::new();
        re_motor_powar.insert(self.l_id, -1. *left_speed * 3.5);
        re_motor_powar.insert(self.r_id,  right_speed * 3.5);

        re_motor_powar
    }
}
