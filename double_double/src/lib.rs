pub mod accurate;
pub mod naive;
pub mod sloppy;

use num_traits::MulAdd;
use std::ops::{Add, Mul, Neg, Sub};

pub const T: u32 = f64::MANTISSA_DIGITS;
pub const S: u32 = T.div_ceil(2);
pub const C: f64 = (2u32.pow(S) + 1) as f64;

#[inline]
pub fn two_sum<T>(x: T, y: T) -> (T, T)
where
    T: Add<Output = T> + Sub<Output = T> + Copy,
{
    let s = x + y;
    let aa = s - y;
    let bb = s - aa;
    let da = x - aa;
    let db = y - bb;

    (s, da + db)
}

#[inline]
pub fn fast_two_sum<T>(x: T, y: T) -> (T, T)
where
    T: Add<Output = T> + Sub<Output = T> + Copy,
{
    let s = x + y;
    let z = s - x;

    (s, y - z)
}

#[inline]
pub fn two_diff<T>(x: T, y: T) -> (T, T)
where
    T: Add<Output = T> + Sub<Output = T> + Copy,
{
    let s = x - y;
    let aa = s + y;
    let bb = s - aa;
    let da = x - aa;
    let db = y + bb;

    (s, da - db)
}

#[inline]
pub fn two_prod_fma<T, U>(x: T, y: U) -> (T, T)
where
    T: Mul<U, Output = T> + Neg<Output = T> + MulAdd<U, T, Output = T> + Copy,
    U: Copy,
{
    let p = x * y;

    (p, x.mul_add(y, -p))
}

#[inline]
pub fn split<T>(x: T, c: T) -> (T, T)
where
    T: Add<Output = T> + Sub<Output = T> + Mul<Output = T> + Copy,
{
    let p = x * c;
    let q = x - p;
    let x1 = p + q;
    let x2 = x - x1;

    (x1, x2)
}

#[inline]
pub fn two_prod<T, U>(x: T, y: U, cx: T, cy: U) -> (T, T)
where
    T: Add<Output = T> + Sub<Output = T> + Mul<U, Output = T> + Mul<Output = T> + Copy,
    U: Add<Output = U> + Sub<Output = U> + Mul<Output = U> + Copy,
{
    let p = x * y;
    let (a1h, a1l) = split(x, cx);
    let (b1h, b1l) = split(y, cy);

    (p, (((a1h * b1h - p) + a1h * b1l) + a1l * b1h) + a1l * b1l)
}
