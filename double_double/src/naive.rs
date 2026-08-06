use std::ops::{Add, Div, Mul, Sub};

#[inline]
pub fn dw_add_dw<T>((ah, al): (T, T), (bh, bl): (T, T)) -> (T, T)
where
    T: Add<Output = T>,
{
    (ah + bh, al + bl)
}

#[inline]
pub fn dw_sub_dw<T>((ah, al): (T, T), (bh, bl): (T, T)) -> (T, T)
where
    T: Sub<Output = T>,
{
    (ah - bh, al - bl)
}

#[inline]
pub fn dw_mul<T, U>((ah, al): (T, T), b: U) -> (T, T)
where
    T: Mul<U, Output = T>,
    U: Copy,
{
    (ah * b, al * b)
}

#[inline]
pub fn dw_div<T, U>((ah, al): (T, T), b: U) -> (T, T)
where
    T: Div<U, Output = T>,
    U: Copy,
{
    (ah / b, al / b)
}
