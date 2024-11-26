use thiserror::Error;

#[derive(Error, Debug)]
pub enum NonNegativeNumError {
    #[error("the value is not positive")]
    NonPositiveValue,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct NonNegativeNum<T: num_traits::Num + Clone + Copy + PartialOrd>(T);

impl<T: num_traits::Num + Copy + PartialOrd> NonNegativeNum<T> {
    pub fn new(value: T) -> Result<Self, NonNegativeNumError> {
        if !Self::is_valid(value) {
            return Err(NonNegativeNumError::NonPositiveValue);
        }
        Ok(NonNegativeNum(value))
    }

    #[inline(always)]
    fn is_valid(value: T) -> bool {
        value >= T::zero()
    }
}

impl<T: num_traits::Num + PartialOrd + Copy> NonNegativeNum<T> {
    pub fn get(&self) -> T {
        self.0
    }
}

macro_rules! impl_positive_num_from {
    ($ttype:ident, $underlying_primitive:ident, ($($from_type:ty),+)) => {
        $(
            impl From<$from_type> for $ttype {
                fn from(value: $from_type) -> Self {
                    NonNegativeNum($underlying_primitive::from(value))
                }
            }
        )+
    };
}

macro_rules! impl_positive_num_try_from {
    ($ttype:ident, $underlying_primitive:ident, ($($from_type:ty),+)) => {
        $(
            impl TryFrom<$from_type> for $ttype {
                type Error = NonNegativeNumError;

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

pub type NonNegativeI32 = NonNegativeNum<i32>;
impl_into_primitive!(NonNegativeI32, i32);
impl_positive_num_from!(NonNegativeI32, i32, (u8, u16));
impl_positive_num_try_from!(NonNegativeI32, i32, (i8, i16, i32));

pub type NonNegativeF64 = NonNegativeNum<f64>;
impl_into_primitive!(NonNegativeF64, f64);
impl_positive_num_from!(NonNegativeF64, f64, (u8, u16, u32));
impl_positive_num_try_from!(NonNegativeF64, f64, (i8, i16, i32, f32, f64));
