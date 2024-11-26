use manifold3d_sys::ManifoldVec3;
use std::ops::{Add, Sub};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Point3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Point3 {
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }
}

impl From<ManifoldVec3> for Point3 {
    fn from(value: ManifoldVec3) -> Self {
        Point3 {
            x: value.x,
            y: value.y,
            z: value.z,
        }
    }
}

impl From<Point3> for ManifoldVec3 {
    fn from(value: Point3) -> Self {
        ManifoldVec3 {
            x: value.x,
            y: value.y,
            z: value.z,
        }
    }
}

impl Add for Point3 {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Point3::new(self.x + rhs.x, self.y + rhs.y, self.z + rhs.z)
    }
}

impl<T> Add<T> for Point3
where
    f64: From<T>,
    T: num_traits::ToPrimitive,
{
    type Output = Self;

    fn add(self, rhs: T) -> Self::Output {
        let value = f64::from(rhs);
        Point3::new(self.x + value, self.y + value, self.z + value)
    }
}

impl Sub for Point3 {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        Point3::new(self.x - rhs.x, self.y - rhs.y, self.z - rhs.z)
    }
}

impl<T> Sub<T> for Point3
where
    f64: From<T>,
    T: num_traits::ToPrimitive,
{
    type Output = Self;

    fn sub(self, rhs: T) -> Self::Output {
        let value = f64::from(rhs);
        Point3::new(self.x - value, self.y - value, self.z - value)
    }
}

#[cfg(feature = "nalgebra_interop")]
impl From<nalgebra::Point3<f64>> for Point3 {
    fn from(value: nalgebra::Point3<f64>) -> Self {
        Point3 {
            x: value.x,
            y: value.y,
            z: value.z,
        }
    }
}

#[cfg(feature = "nalgebra_interop")]
impl From<Point3> for nalgebra::Point3<f64> {
    fn from(value: Point3) -> Self {
        nalgebra::Point3::new(value.x, value.y, value.z)
    }
}
