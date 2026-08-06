use crate::{fast_two_sum, two_diff, two_prod, two_prod_fma, two_sum};
use num_traits::MulAdd;
use std::ops::{Add, Div, Mul, Neg, Sub};

#[inline]
pub fn dw_add_dw<T>((xh, xl): (T, T), (yh, yl): (T, T)) -> (T, T)
where
    T: Add<Output = T> + Sub<Output = T> + Copy,
{
    let (sh, sl) = two_sum(xh, yh);
    let (th, tl) = two_sum(xl, yl);
    let c = sl + th;
    let (vh, vl) = fast_two_sum(sh, c);
    let w = tl + vl;
    fast_two_sum(vh, w)
}

#[inline]
pub fn dw_add<T>((xh, xl): (T, T), y: T) -> (T, T)
where
    T: Add<Output = T> + Sub<Output = T> + Copy,
{
    let (sh, sl) = two_sum(xh, y);
    let v = xl + sl;
    fast_two_sum(sh, v)
}

#[inline]
pub fn dw_sub_dw<T>((xh, xl): (T, T), (yh, yl): (T, T)) -> (T, T)
where
    T: Add<Output = T> + Sub<Output = T> + Copy,
{
    let (sh, sl) = two_diff(xh, yh);
    let (th, tl) = two_diff(xl, yl);
    let c = sl + th;
    let (vh, vl) = fast_two_sum(sh, c);
    let w = tl + vl;
    fast_two_sum(vh, w)
}

#[inline]
pub fn dw_mul_dw_fma<T, U>((xh, xl): (T, T), (yh, yl): (U, U)) -> (T, T)
where
    T: Add<Output = T>
        + Sub<Output = T>
        + Mul<U, Output = T>
        + Neg<Output = T>
        + MulAdd<U, T, Output = T>
        + Copy,
    U: Copy,
{
    let (ch, cl) = two_prod_fma(xh, yh);
    let tl0 = xl * yl;
    let tl1 = xh.mul_add(yl, tl0);
    let cl2 = xl.mul_add(yh, tl1);
    let cl3 = cl + cl2;
    fast_two_sum(ch, cl3)
}

#[inline]
pub fn dw_mul_fma<T, U>((xh, xl): (T, T), y: U) -> (T, T)
where
    T: Add<Output = T>
        + Sub<Output = T>
        + Mul<U, Output = T>
        + Neg<Output = T>
        + MulAdd<U, T, Output = T>
        + Copy,
    U: Copy,
{
    let (ch, cl) = two_prod_fma(xh, y);
    let cl2 = xl * y;
    let (th, tl) = fast_two_sum(ch, cl2);
    let tl2 = tl + cl;
    fast_two_sum(th, tl2)
}

#[inline]
pub fn mul_dw_fma<T, U>(x: T, (yh, yl): (U, U)) -> (T, T)
where
    T: Add<Output = T>
        + Sub<Output = T>
        + Mul<U, Output = T>
        + Neg<Output = T>
        + MulAdd<U, T, Output = T>
        + Copy,
    U: Copy,
{
    let (ch, cl) = two_prod_fma(x, yh);
    let cl2 = x * yl;
    let (th, tl) = fast_two_sum(ch, cl2);
    let tl2 = tl + cl;
    fast_two_sum(th, tl2)
}

#[inline]
pub fn dw_mul_dw<T, U>((xh, xl): (T, T), (yh, yl): (U, U), cx: T, cy: U) -> (T, T)
where
    T: Add<Output = T> + Sub<Output = T> + Mul<U, Output = T> + Mul<Output = T> + Copy,
    U: Add<Output = U> + Sub<Output = U> + Mul<Output = U> + Copy,
{
    let (ch, cl) = two_prod(xh, yh, cx, cy);
    let tl1 = xh * yl;
    let tl2 = xl * yh;
    let cl2 = tl1 + tl2;
    let cl3 = cl + cl2;
    fast_two_sum(ch, cl3)
}

#[inline]
pub fn dw_mul<T, U>((xh, xl): (T, T), y: U, cx: T, cy: U) -> (T, T)
where
    T: Add<Output = T> + Sub<Output = T> + Mul<U, Output = T> + Mul<Output = T> + Copy,
    U: Add<Output = U> + Sub<Output = U> + Mul<Output = U> + Copy,
{
    let (ch, cl) = two_prod(xh, y, cx, cy);
    let cl2 = xl * y;
    let (th, tl) = fast_two_sum(ch, cl2);
    let tl2 = tl + cl;
    fast_two_sum(th, tl2)
}

#[inline]
pub fn mul_dw<T, U>(x: T, (yh, yl): (U, U), cx: T, cy: U) -> (T, T)
where
    T: Add<Output = T> + Sub<Output = T> + Mul<U, Output = T> + Mul<Output = T> + Copy,
    U: Add<Output = U> + Sub<Output = U> + Mul<Output = U> + Copy,
{
    let (ch, cl) = two_prod(x, yh, cx, cy);
    let cl2 = x * yl;
    let (th, tl) = fast_two_sum(ch, cl2);
    let tl2 = tl + cl;
    fast_two_sum(th, tl2)
}

#[inline]
pub fn dw_div_dw<T, U>((xh, xl): (T, T), (yh, yl): (U, U), cx: T, cy: U) -> (T, T)
where
    T: Add<Output = T>
        + Sub<Output = T>
        + Mul<U, Output = T>
        + Mul<Output = T>
        + Div<U, Output = T>
        + Copy,
    U: Add<Output = U> + Sub<Output = U> + Mul<Output = U> + Copy,
{
    let th = xh / yh;
    let (rh, rl) = mul_dw(th, (yh, yl), cx, cy);
    let pih = xh - rh;
    let deltal = xl - rl;
    let delta = pih + deltal;
    let tl = delta / yh;
    fast_two_sum(th, tl)
}

#[inline]
pub fn dw_div<T, U>((xh, xl): (T, T), y: U, cx: T, cy: U) -> (T, T)
where
    T: Add<Output = T>
        + Sub<Output = T>
        + Mul<U, Output = T>
        + Mul<Output = T>
        + Div<U, Output = T>
        + Copy,
    U: Add<Output = U> + Sub<Output = U> + Mul<Output = U> + Copy,
{
    let th = xh / y;
    let (pih, pil) = two_prod(th, y, cx, cy);
    let deltah = xh - pih;
    let deltat = deltah - pil;
    let delta = deltat + xl;
    let tl = delta / y;
    fast_two_sum(th, tl)
}
