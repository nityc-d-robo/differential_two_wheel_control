use std::collections::HashMap;

pub struct Tire {
    pub id: usize,
    pub raito: f64,
}

pub struct Chassis {
    pub l: Tire,
    pub r: Tire,
}
pub struct DtwcSetting {
    pub chassis: Chassis,
    pub robot_center_to_wheel_distance: f64,
    pub max_pawer_input: f64,
    pub max_pawer_output: f64,
    pub max_revolution: f64,
}

impl DtwcSetting {
    pub fn move_chassis(&self,linear_x: f64,linear_y: f64,angular_z: f64,) -> HashMap<usize, f64> {
        let power: f64 = (linear_x.powf(2.) + linear_y.powf(2.))
            .sqrt()
            .min(self.max_pawer_input);
        
        // 感
        let revolution = (angular_z/self.max_revolution)*self.robot_center_to_wheel_distance;
        let mut left_speed = self.chassis.r.raito * (power + revolution);
        let mut right_speed = self.chassis.l.raito * (power - revolution);


        // モーターの回転方向が逆だから
        left_speed *= -1.;

        // 範囲内に止める
        left_speed = left_speed.max(-self.max_pawer_output);
        left_speed = left_speed.min(self.max_pawer_output);

        right_speed = right_speed.max(-self.max_pawer_output);
        right_speed = right_speed.min(self.max_pawer_output);


        let mut re_motor_powar = HashMap::new();
        re_motor_powar.insert(self.chassis.l.id,left_speed);
        re_motor_powar.insert(self.chassis.r.id, right_speed);

        re_motor_powar
    }
}
