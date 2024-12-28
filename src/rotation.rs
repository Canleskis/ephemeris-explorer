use bevy::prelude::*;
use hifitime::Epoch;

#[derive(Component)]
pub struct Rotating {
    pub right_ascension: f64,
    pub declination: f64,
    pub reference_epoch: Epoch,
    pub reference_rotation: f64,
    pub rotation_rate: f64,
}

impl Rotating {
    pub fn at(&self, epoch: Epoch) -> bevy::math::DQuat {
        let axis = bevy::math::DVec3::new(
            self.declination.cos() * self.right_ascension.cos(),
            self.declination.cos() * self.right_ascension.sin(),
            self.declination.sin(),
        );

        let dt = (epoch - self.reference_epoch).to_unit(hifitime::Unit::Day);
        let angle = self.reference_rotation + dt * self.rotation_rate;

        bevy::math::DQuat::from_axis_angle(axis, angle)
    }
}
