use manifold3d_sys::ManifoldVec2;
use std::ops::{Add, Sub};

#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Vec2 {
    pub x: f64,
    pub y: f64,
}

impl Vec2 {
    pub fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }
}

impl From<ManifoldVec2> for Vec2 {
    fn from(value: ManifoldVec2) -> Self {
        Vec2 {
            x: value.x,
            y: value.y,
        }
    }
}

impl Add for Vec2 {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Vec2::new(self.x + rhs.x, self.y + rhs.y)
    }
}

impl<T> Add<T> for Vec2
where
    f64: From<T>,
    T: num_traits::ToPrimitive,
{
    type Output = Self;

    fn add(self, rhs: T) -> Self::Output {
        let value = f64::from(rhs);
        Vec2::new(self.x + value, self.y + value)
    }
}

impl Sub for Vec2 {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        Vec2::new(self.x - rhs.x, self.y - rhs.y)
    }
}

impl<T> Sub<T> for Vec2
where
    f64: From<T>,
    T: num_traits::ToPrimitive,
{
    type Output = Self;

    fn sub(self, rhs: T) -> Self::Output {
        let value = f64::from(rhs);
        Vec2::new(self.x - value, self.y - value)
    }
}

#[cfg(feature = "nalgebra_interop")]
impl From<nalgebra::Vector2<f64>> for Vec2 {
    fn from(value: nalgebra::Vector2<f64>) -> Self {
        Vec2 {
            x: value.x,
            y: value.y,
        }
    }
}
