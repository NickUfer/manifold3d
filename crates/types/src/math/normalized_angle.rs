use crate::math;
use std::ops::{Add, AddAssign, Sub, SubAssign};

/// Represents an angle, measured in degrees, constrained to the range [-360.0, 360.0].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct NormalizedAngle(f64);

impl NormalizedAngle {
    pub fn from_degrees<T>(value: T) -> Self
    where
        T: Into<f64>,
    {
        NormalizedAngle(Self::normalize(value))
    }

    pub fn as_degrees(&self) -> f64 {
        self.0
    }

    fn normalize(value: impl Into<f64>) -> f64 {
        let mut value = value.into() % 360.0;
        if value < 0.0 {
            value = 360.0 - value;
        }
        value
    }

    pub fn get(&self) -> f64 {
        self.0
    }
}

impl<T> From<math::PositiveNum<T>> for NormalizedAngle
where
    T: num_traits::Num + PartialOrd + Copy,
    f64: From<T>,
{
    fn from(value: math::PositiveNum<T>) -> Self {
        NormalizedAngle(f64::from(value.get()) % 360.0)
    }
}

impl Add for NormalizedAngle {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Self::from_degrees(self.0 + rhs.0)
    }
}

impl AddAssign for NormalizedAngle {
    fn add_assign(&mut self, rhs: Self) {
        self.0 = Self::normalize(self.0 + rhs.0);
    }
}

impl Sub for NormalizedAngle {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        Self::from_degrees(self.0 - rhs.0)
    }
}

impl SubAssign for NormalizedAngle {
    fn sub_assign(&mut self, rhs: Self) {
        self.0 = Self::normalize(self.0 - rhs.0);
    }
}

impl Into<f64> for NormalizedAngle {
    fn into(self) -> f64 {
        self.get()
    }
}
