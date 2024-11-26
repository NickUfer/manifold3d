use manifold3d_sys::ManifoldVec3;
use std::ops::{Add, Sub};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Vec3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Vec3 {
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }
}

impl From<ManifoldVec3> for Vec3 {
    fn from(value: ManifoldVec3) -> Self {
        Vec3 {
            x: value.x,
            y: value.y,
            z: value.z,
        }
    }
}

impl Add for Vec3 {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Vec3::new(self.x + rhs.x, self.y + rhs.y, self.z + rhs.z)
    }
}

impl<T> Add<T> for Vec3
where
    f64: From<T>,
    T: num_traits::ToPrimitive,
{
    type Output = Self;

    fn add(self, rhs: T) -> Self::Output {
        let value = f64::from(rhs);
        Vec3::new(self.x + value, self.y + value, self.z + value)
    }
}

impl Sub for Vec3 {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        Vec3::new(self.x - rhs.x, self.y - rhs.y, self.z - rhs.z)
    }
}

impl<T> Sub<T> for Vec3
where
    f64: From<T>,
    T: num_traits::ToPrimitive,
{
    type Output = Self;

    fn sub(self, rhs: T) -> Self::Output {
        let value = f64::from(rhs);
        Vec3::new(self.x - value, self.y - value, self.z - value)
    }
}

#[cfg(feature = "nalgebra_interop")]
impl From<nalgebra::Vector3<f64>> for Vec3 {
    fn from(value: nalgebra::Vector3<f64>) -> Self {
        Vec3 {
            x: value.x,
            y: value.y,
            z: value.z,
        }
    }
}
