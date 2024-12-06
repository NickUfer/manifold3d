use manifold3d_sys::ManifoldVec2;
use std::ops::{Add, Sub};

#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Point2 {
    pub x: f64,
    pub y: f64,
}

impl Point2 {
    pub fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }
}

impl From<ManifoldVec2> for Point2 {
    fn from(value: ManifoldVec2) -> Self {
        Point2 {
            x: value.x,
            y: value.y,
        }
    }
}

impl From<Point2> for ManifoldVec2 {
    fn from(value: Point2) -> Self {
        ManifoldVec2 {
            x: value.x,
            y: value.y,
        }
    }
}

impl Add for Point2 {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Point2::new(self.x + rhs.x, self.y + rhs.y)
    }
}

impl<T> Add<T> for Point2
where
    f64: From<T>,
    T: num_traits::ToPrimitive,
{
    type Output = Self;

    fn add(self, rhs: T) -> Self::Output {
        let value = f64::from(rhs);
        Point2::new(self.x + value, self.y + value)
    }
}

impl Sub for Point2 {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        Point2::new(self.x - rhs.x, self.y - rhs.y)
    }
}

impl<T> Sub<T> for Point2
where
    f64: From<T>,
    T: num_traits::ToPrimitive,
{
    type Output = Self;

    fn sub(self, rhs: T) -> Self::Output {
        let value = f64::from(rhs);
        Point2::new(self.x - value, self.y - value)
    }
}
