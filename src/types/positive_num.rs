use std::cmp::Ordering;
use thiserror::Error;

#[derive(Error, Debug)]
pub enum PositiveNumError {
    #[error("the value is not positive")]
    NonPositiveValue,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq, PartialOrd, Ord)]
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

macro_rules! impl_partial_eq {
    ($ttype:ident, $underlying_primitive:ident, ($($eq_type:ty),+)) => {
        $(
            impl PartialEq<$eq_type> for $ttype {
                fn eq(&self, other: &$eq_type) -> bool {
                    self.get().eq(&(*other as $underlying_primitive))
                }
            }
        )+
    };
}

macro_rules! impl_partial_ord {
    ($ttype:ident, $underlying_primitive:ident, ($($eq_type:ty),+)) => {
        $(
            impl PartialOrd<$eq_type> for $ttype {
                fn partial_cmp(&self, other: &$eq_type) -> Option<Ordering> {
                    self.get().partial_cmp(&(*other as $underlying_primitive))
                }
            }
        )+
    };
}

pub type PositiveI32 = PositiveNum<i32>;
impl_into_primitive!(PositiveI32, i32);
impl_positive_num_try_from!(PositiveI32, i32, (u8, u16, i8, i16, i32));
impl_partial_eq!(PositiveI32, i32, (u8, u16, i8, i16, i32));
impl_partial_ord!(PositiveI32, i32, (u8, u16, i8, i16, i32));

pub type PositiveF64 = PositiveNum<f64>;
impl_into_primitive!(PositiveF64, f64);
impl_positive_num_try_from!(PositiveF64, f64, (u8, u16, u32, i8, i16, i32, f32, f64));
impl_partial_eq!(PositiveF64, f64, (u8, u16, u32, i8, i16, i32, f32, f64));
impl_partial_ord!(PositiveF64, f64, (u8, u16, u32, i8, i16, i32, f32, f64));
