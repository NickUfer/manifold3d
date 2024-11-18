use manifold3d_sys::{ManifoldVec2, ManifoldVec3};
use std::ops::{Add, AddAssign, Sub, SubAssign};
use thiserror::Error;

#[derive(Error, Debug)]
pub enum PositiveNumError {
    #[error("the value is not positive")]
    NonPositiveValue,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Vec2 {
    pub x: f64,
    pub y: f64,
}

impl From<ManifoldVec2> for Vec2 {
    fn from(value: ManifoldVec2) -> Self {
        Vec2 {
            x: value.x,
            y: value.y,
        }
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

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Vec3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
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

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Point2 {
    pub x: f64,
    pub y: f64,
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

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Point3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
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

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Matrix4x3 {
    pub rows: [Vec3; 4],
}

#[cfg(feature = "nalgebra_interop")]
impl From<nalgebra::Matrix4x3<f64>> for Matrix4x3 {
    fn from(matrix: nalgebra::Matrix4x3<f64>) -> Self {
        Matrix4x3 {
            rows: [
                Vec3 {
                    x: matrix.m11,
                    y: matrix.m12,
                    z: matrix.m13,
                },
                Vec3 {
                    x: matrix.m21,
                    y: matrix.m22,
                    z: matrix.m23,
                },
                Vec3 {
                    x: matrix.m31,
                    y: matrix.m32,
                    z: matrix.m33,
                },
                Vec3 {
                    x: matrix.m41,
                    y: matrix.m42,
                    z: matrix.m43,
                },
            ],
        }
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct PositiveNum<T: num_traits::Num + Clone + Copy + PartialOrd>(T);

impl<T: num_traits::Num + Copy + PartialOrd> PositiveNum<T> {
    pub fn new(value: T) -> Result<Self, PositiveNumError> {
        if !Self::is_valid(value) {
            return Err(PositiveNumError::NonPositiveValue);
        }
        Ok(PositiveNum(value))
    }

    #[inline(always)]
    fn is_valid(value: T) -> bool {
        value > T::zero()
    }
}

impl<T: num_traits::Num + PartialOrd + Copy> PositiveNum<T> {
    pub fn get(&self) -> T {
        self.0
    }
}

macro_rules! impl_positive_num_from {
    ($ttype:ident, $underlying_primitive:ident, ($($from_type:ty),+)) => {
        $(
            impl From<$from_type> for $ttype {
                fn from(value: $from_type) -> Self {
                    PositiveNum($underlying_primitive::from(value))
                }
            }
        )+
    };
}

macro_rules! impl_positive_num_try_from {
    ($ttype:ident, $underlying_primitive:ident, ($($from_type:ty),+)) => {
        $(
            impl TryFrom<$from_type> for $ttype {
                type Error = PositiveNumError;

                fn try_from(value: $from_type) -> Result<Self, Self::Error> {
                    $ttype::new($underlying_primitive::from(value))
                }
            }
        )+
    };
}

macro_rules! impl_into_primitive {
    ($ttype:ident, $underlying_primitive:ident) => {
        #[allow(clippy::from_over_into)]
        impl Into<$underlying_primitive> for $ttype {
            fn into(self) -> $underlying_primitive {
                self.get()
            }
        }
    };
}

pub type PositiveI32 = PositiveNum<i32>;
impl_into_primitive!(PositiveI32, i32);
impl_positive_num_from!(PositiveI32, i32, (u8, u16));
impl_positive_num_try_from!(PositiveI32, i32, (i8, i16, i32));

pub type PositiveF64 = PositiveNum<f64>;
impl_into_primitive!(PositiveF64, f64);
impl_positive_num_from!(PositiveF64, f64, (u8, u16, u32));
impl_positive_num_try_from!(PositiveF64, f64, (i8, i16, i32, f32, f64));

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
}

impl<T> From<PositiveNum<T>> for NormalizedAngle
where
    T: num_traits::Num + PartialOrd + Copy,
    f64: From<T>,
{
    fn from(value: PositiveNum<T>) -> Self {
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
