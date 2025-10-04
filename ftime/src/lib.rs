pub mod duration;
pub mod epoch;

pub use duration::*;
pub use epoch::*;

pub const SEC_PER_NANO: f64 = 1e-9;
pub const SEC_PER_MICRO: f64 = 1e-6;
pub const SEC_PER_MILLI: f64 = 1e-3;
pub const SEC_PER_MIN: f64 = 60.0;
pub const SEC_PER_HOUR: f64 = 60.0 * SEC_PER_MIN;
pub const SEC_PER_DAY: f64 = 24.0 * SEC_PER_HOUR;
pub const SEC_PER_YEAR: f64 = 365.25 * SEC_PER_DAY;

pub const MS_PER_SEC: f64 = 1_000.0;
pub const MS_PER_MIN: f64 = SEC_PER_MIN * MS_PER_SEC;
pub const MS_PER_HOUR: f64 = SEC_PER_HOUR * MS_PER_SEC;
pub const MS_PER_DAY: f64 = SEC_PER_DAY * MS_PER_SEC;
pub const MS_PER_YEAR: f64 = SEC_PER_YEAR * MS_PER_SEC;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Unit {
    Nanosecond,
    Microsecond,
    Millisecond,
    Second,
    Minute,
    Hour,
    Day,
    Year,
}

impl std::ops::Mul<f64> for Unit {
    type Output = Duration;

    #[inline]
    fn mul(self, rhs: f64) -> Self::Output {
        match self {
            Unit::Nanosecond => Duration::from_nanoseconds(rhs),
            Unit::Microsecond => Duration::from_microseconds(rhs),
            Unit::Millisecond => Duration::from_milliseconds(rhs),
            Unit::Second => Duration::from_seconds(rhs),
            Unit::Minute => Duration::from_minutes(rhs),
            Unit::Hour => Duration::from_hours(rhs),
            Unit::Day => Duration::from_days(rhs),
            Unit::Year => Duration::from_years(rhs),
        }
    }
}

impl std::ops::Mul<Unit> for f64 {
    type Output = Duration;

    #[inline]
    fn mul(self, rhs: Unit) -> Self::Output {
        rhs * self
    }
}
