use std::ops::{Add, Sub};

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct Ratio<T = i128> {
    numer: T,
    denom: T,
}

impl<T> Ratio<T> {
    #[inline]
    pub const fn new_raw(numer: T, denom: T) -> Self {
        Self { numer, denom }
    }

    #[inline]
    pub const fn numerator(&self) -> T
    where
        T: Copy,
    {
        self.numer
    }

    #[inline]
    pub const fn denominator(&self) -> T
    where
        T: Copy,
    {
        self.denom
    }
}

impl Ratio {
    pub const ZERO: Ratio = Ratio { numer: 0, denom: 1 };

    pub const NAN: Ratio = Ratio { numer: 0, denom: 0 };

    pub const INFINITY: Ratio = Ratio { numer: 1, denom: 0 };

    pub const NEG_INFINITY: Ratio = Ratio {
        numer: -1,
        denom: 0,
    };

    #[inline]
    pub const fn const_new(numer: i128, denom: i128) -> Self {
        let mut result = Self { numer, denom };
        result.normalize();
        result
    }

    #[inline]
    pub fn new<T>(numer: T, denom: T) -> Self
    where
        T: Into<i128>,
    {
        Self::const_new(numer.into(), denom.into())
    }

    #[inline]
    pub const fn from_int(val: i128) -> Self {
        Ratio {
            numer: val,
            denom: 1,
        }
    }

    #[inline]
    pub fn from_recip(val: i128) -> Self {
        Ratio {
            numer: 1,
            denom: val,
        }
    }

    #[inline]
    pub const fn from_f64(val: f64) -> Option<Self> {
        if val.is_nan() {
            return Some(Ratio::NAN);
        };
        if val.is_infinite() {
            return Some(Ratio {
                numer: val.signum() as i128,
                denom: 0,
            });
        };

        let mut p: u32 = 0;
        let mut new_val = val;
        let ten: u128 = 10;
        loop {
            if (new_val.abs() as u64) as f64 == new_val.abs() {
                break;
            }

            p += 1;
            new_val = val * ten.pow(p) as f64;
            if new_val.is_infinite() {
                return None;
            }
        }

        Some(Self::const_new(new_val as i128, ten.pow(p) as i128))
    }

    #[inline]
    pub const fn const_add(self, rhs: Ratio) -> Ratio {
        if self.denom == rhs.denom {
            return Ratio {
                numer: self.numer + rhs.numer,
                denom: self.denom,
            };
        }

        let lcm = signed_lcm(self.denom, rhs.denom) as i128;
        let lhs_numer = self.numer * (lcm / self.denom);
        let rhs_numer = rhs.numer * (lcm / rhs.denom);

        Ratio {
            numer: lhs_numer + rhs_numer,
            denom: lcm,
        }
    }

    #[inline]
    pub const fn const_sub(self, rhs: Ratio) -> Ratio {
        if self.denom == rhs.denom {
            return Ratio {
                numer: self.numer - rhs.numer,
                denom: self.denom,
            };
        }

        let lcm = signed_lcm(self.denom, rhs.denom) as i128;
        let lhs_numer = self.numer * (lcm / self.denom);
        let rhs_numer = rhs.numer * (lcm / rhs.denom);

        Ratio {
            numer: lhs_numer - rhs_numer,
            denom: lcm,
        }
    }

    #[inline]
    pub const fn const_mul(self, rhs: Ratio) -> Ratio {
        let gcd_ad = signed_gcd(self.numer, rhs.denom) as i128;
        let gcd_bc = signed_gcd(self.denom, rhs.numer) as i128;
        Ratio {
            numer: self.numer / gcd_ad * rhs.numer / gcd_bc,
            denom: self.denom / gcd_bc * rhs.denom / gcd_ad,
        }
    }

    #[inline]
    pub const fn normalize(&mut self) {
        if self.denom == 0 {
            return;
        }
        if self.numer == 0 {
            self.denom = 1;
            return;
        }
        if self.numer == self.denom {
            self.numer = 1;
            self.denom = 1;
            return;
        }
        let g = signed_gcd(self.numer, self.denom) as i128;

        self.numer /= g;
        self.denom /= g;

        // keep denom positive!
        if self.denom < 0 {
            self.numer = -self.numer;
            self.denom = -self.denom;
        }
    }
}

impl Add for Ratio {
    type Output = Ratio;

    #[inline]
    fn add(self, rhs: Ratio) -> Self::Output {
        self.const_add(rhs)
    }
}

impl Sub for Ratio {
    type Output = Ratio;

    #[inline]
    fn sub(self, rhs: Ratio) -> Self::Output {
        self.const_sub(rhs)
    }
}

impl std::ops::Mul for Ratio {
    type Output = Ratio;

    #[inline]
    fn mul(self, rhs: Ratio) -> Self::Output {
        let gcd_ad = signed_gcd(self.numer, rhs.denom) as i128;
        let gcd_bc = signed_gcd(self.denom, rhs.numer) as i128;
        Ratio {
            numer: self.numer / gcd_ad * rhs.numer / gcd_bc,
            denom: self.denom / gcd_bc * rhs.denom / gcd_ad,
        }
    }
}

impl std::ops::Mul<Ratio> for f32 {
    type Output = f32;

    #[inline]
    fn mul(self, rhs: Ratio) -> Self::Output {
        self * (rhs.numer as f32 / rhs.denom as f32)
    }
}

impl std::ops::Mul<Ratio> for f64 {
    type Output = f64;

    #[inline]
    fn mul(self, rhs: Ratio) -> Self::Output {
        self * (rhs.numer as f64 / rhs.denom as f64)
    }
}

#[inline]
pub const fn signed_lcm(u: i128, v: i128) -> u128 {
    lcm(u.unsigned_abs(), v.unsigned_abs())
}

#[inline]
pub const fn signed_gcd(u: i128, v: i128) -> u128 {
    gcd(u.unsigned_abs(), v.unsigned_abs())
}

#[inline]
const fn lcm(a: u128, b: u128) -> u128 {
    a * b / gcd(a, b)
}

#[inline]
const fn gcd(a: u128, b: u128) -> u128 {
    // Use Stein's algorithm
    let mut m = a;
    let mut n = b;
    if m == 0 || n == 0 {
        return m | n;
    }

    // find common factors of 2
    let shift = (m | n).trailing_zeros();

    // divide n and m by 2 until odd
    m >>= m.trailing_zeros();
    n >>= n.trailing_zeros();

    while m != n {
        if m > n {
            m -= n;
            m >>= m.trailing_zeros();
        } else {
            n -= m;
            n >>= n.trailing_zeros();
        }
    }
    m << shift
}
