use crate::{fast_two_sum, two_diff, two_sum};
use std::ops::{Add, Sub};

#[inline]
pub fn dw_add_dw<T>((ah, al): (T, T), (bh, bl): (T, T)) -> (T, T)
where
    T: Add<Output = T> + Sub<Output = T> + Copy,
{
    let (sh, sl) = two_sum(ah, bh);
    let v = al + bl;
    let w = sl + v;
    fast_two_sum(sh, w)
}

#[inline]
pub fn dw_sub_dw<T>((ah, al): (T, T), (bh, bl): (T, T)) -> (T, T)
where
    T: Add<Output = T> + Sub<Output = T> + Copy,
{
    let (sh, sl) = two_diff(ah, bh);
    let v = al - bl;
    let w = sl + v;
    fast_two_sum(sh, w)
}
