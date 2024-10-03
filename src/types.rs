use thiserror::Error;

#[derive(Error, Debug)]
pub enum PositiveNumError {
    #[error("the value is not positive")]
    NonPositiveValue,
}

pub struct ManifoldVec3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl From<manifold_sys::ManifoldVec3> for ManifoldVec3 {
    fn from(value: manifold_sys::ManifoldVec3) -> Self {
        ManifoldVec3 {
            x: value.x,
            y: value.y,
            z: value.z,
        }
    }
}

#[cfg(feature = "cgmath_interop")]
impl From<cgmath::Vector3<f64>> for ManifoldVec3 {
    fn from(value: cgmath::Vector3<f64>) -> Self {
        ManifoldVec3 {
            x: value.x,
            y: value.y,
            z: value.y,
        }
    }
}

#[derive(Error, Debug, Copy, Clone, PartialEq, Eq)]
pub struct PositiveNum<T: num_traits::Num + Copy + PartialOrd>(T);

impl<T: num_traits::Num + Copy + PartialOrd> PositiveNum<T> {
    pub fn new(value: T) -> Result<Self, PositiveNumError> {
        if !Self::is_valid(value) {
            return Err(PositiveNumError::NonPositiveValue);
        }
        Ok(PositiveNum(value))
    }

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

pub type PositiveI32 = PositiveNum<i32>;
impl_positive_num_from!(PositiveI32, i32, (u8, u16));
impl_positive_num_try_from!(PositiveI32, i32, (i8, i16, i32));

pub type PositiveF64 = PositiveNum<f64>;
impl_positive_num_from!(PositiveF64, f64, (u8, u16, u32));
impl_positive_num_try_from!(PositiveF64, f64, (i8, i16, i32, f32, f64));

pub struct Angle(f64);

impl Angle {
    pub fn from_positive_num<T>(value: PositiveNum<T>) -> Self
    where
        T: num_traits::Num + PartialOrd + Copy,
        f64: From<T>,
    {
        Angle(f64::from(value.get()) % 360.0)
    }

    pub fn from_unchecked<T>(value: T) -> Self
    where
        f64: From<T>,
    {
        let mut value = f64::from(value) % 360.0;
        if value < 0.0 {
            value = 360.0 - value;
        }

        Angle(value)
    }

    pub fn as_degrees(&self) -> f64 {
        self.0
    }
}
