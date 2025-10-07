use crate::{
    MS_PER_DAY, MS_PER_HOUR, MS_PER_MIN, MS_PER_SEC, MS_PER_YEAR, SEC_PER_DAY, SEC_PER_HOUR,
    SEC_PER_MICRO, SEC_PER_MILLI, SEC_PER_MIN, SEC_PER_NANO, SEC_PER_YEAR,
};

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Duration(f64);

impl Duration {
    pub const ZERO: Self = Duration(0.0);

    pub const MIN: Self = Duration(f64::MIN);

    pub const MAX: Self = Duration(f64::MAX);

    #[inline]
    pub const fn from_seconds(seconds: f64) -> Self {
        assert!(seconds.is_finite());
        Self(seconds)
    }

    #[inline]
    pub const fn from_nanoseconds(nanoseconds: f64) -> Self {
        Self::from_seconds(nanoseconds * SEC_PER_NANO)
    }

    #[inline]
    pub const fn from_microseconds(microseconds: f64) -> Self {
        Self::from_seconds(microseconds * SEC_PER_MICRO)
    }

    #[inline]
    pub const fn from_milliseconds(milliseconds: f64) -> Self {
        Self::from_seconds(milliseconds * SEC_PER_MILLI)
    }

    #[inline]
    pub const fn from_minutes(minutes: f64) -> Self {
        Self::from_seconds(minutes * SEC_PER_MIN)
    }

    #[inline]
    pub const fn from_hours(hours: f64) -> Self {
        Self::from_seconds(hours * SEC_PER_HOUR)
    }

    #[inline]
    pub const fn from_days(days: f64) -> Self {
        Self::from_seconds(days * SEC_PER_DAY)
    }

    #[inline]
    pub const fn from_years(years: f64) -> Self {
        Self::from_seconds(years * SEC_PER_YEAR)
    }

    #[inline]
    pub const fn as_seconds(self) -> f64 {
        self.0
    }

    #[inline]
    pub fn mut_seconds(&mut self) -> &mut f64 {
        &mut self.0
    }

    #[inline]
    pub const fn as_days(self) -> f64 {
        self.0 / SEC_PER_DAY as f64
    }

    #[inline]
    pub const fn abs(self) -> Self {
        Duration(self.0.abs())
    }

    #[inline]
    pub const fn is_negative(self) -> bool {
        self.0.is_sign_negative()
    }

    #[inline]
    pub fn floor(self, to: Duration) -> Self {
        Duration((self.0 / to.0).floor() * to.0)
    }

    #[inline]
    pub fn scaled(self, factor: f64) -> Self {
        Duration(self.0 * factor)
    }

    #[inline]
    pub fn round(self, to: Self) -> Self {
        Duration((self.0 / to.0).round() * to.0)
    }
}

impl std::ops::Add for Duration {
    type Output = Duration;

    #[inline]
    fn add(self, rhs: Self) -> Self::Output {
        Duration(self.0 + rhs.0)
    }
}

impl std::ops::AddAssign for Duration {
    #[inline]
    fn add_assign(&mut self, rhs: Self) {
        self.0 += rhs.0;
    }
}

impl std::ops::Sub for Duration {
    type Output = Duration;

    #[inline]
    fn sub(self, rhs: Self) -> Self::Output {
        Duration(self.0 - rhs.0)
    }
}

impl std::ops::SubAssign for Duration {
    #[inline]
    fn sub_assign(&mut self, rhs: Self) {
        self.0 -= rhs.0;
    }
}

impl std::ops::Mul<f64> for Duration {
    type Output = Duration;

    #[inline]
    fn mul(self, rhs: f64) -> Self::Output {
        Duration(self.0 * rhs)
    }
}

impl std::ops::Div<f64> for Duration {
    type Output = Duration;

    #[inline]
    fn div(self, rhs: f64) -> Self::Output {
        Duration(self.0 / rhs)
    }
}

impl std::ops::Neg for Duration {
    type Output = Duration;

    #[inline]
    fn neg(self) -> Self::Output {
        Duration(-self.0)
    }
}

impl PartialOrd for Duration {
    #[inline]
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for Duration {
    #[inline]
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        match (self.0 <= other.0, self.0 >= other.0) {
            (false, true) => std::cmp::Ordering::Greater,
            (true, false) => std::cmp::Ordering::Less,
            (true, true) => std::cmp::Ordering::Equal,
            (false, false) => unreachable!(),
        }
    }
}

impl Eq for Duration {}

impl std::hash::Hash for Duration {
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        let bits = self.0 + 0.0;
        bits.to_bits().hash(state)
    }
}

impl From<std::time::Duration> for Duration {
    #[inline]
    fn from(d: std::time::Duration) -> Self {
        Duration(d.as_secs_f64())
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum DurationParseError {
    Empty,
    InvalidNumber(String),
    UnknownUnit(String),
    Overflow,
}

impl std::fmt::Display for DurationParseError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            DurationParseError::Empty => write!(f, "empty duration string"),
            DurationParseError::InvalidNumber(s) => write!(f, "invalid number: {s}"),
            DurationParseError::UnknownUnit(u) => write!(f, "unknown unit: {u}"),
            DurationParseError::Overflow => write!(f, "duration overflow"),
        }
    }
}
impl std::error::Error for DurationParseError {}

impl std::fmt::Display for Duration {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let sign = if self.0.is_sign_negative() { "-" } else { "" };
        let t = self.0.abs();

        // Split into whole seconds and fractional seconds
        let secs_f = t.trunc();
        let frac = t.fract();

        // Convert fractional seconds to milliseconds (round once)
        let mut ms = (frac * 1e3).round() as u16;
        let mut secs_int = secs_f as u128;
        if ms == 1000 {
            ms = 0;
            secs_int = secs_int.saturating_add(1);
        }

        // unit sizes (in seconds)
        const SEC_PER_MIN: u128 = 60;
        const SEC_PER_HOUR: u128 = 60 * SEC_PER_MIN;
        const SEC_PER_DAY: u128 = 24 * SEC_PER_HOUR; // 86,400 s
        const SEC_PER_YEAR: u128 = 31_557_600; // 365.25 d (Julian year)

        // Decompose whole seconds
        let y = secs_int / SEC_PER_YEAR;
        secs_int %= SEC_PER_YEAR;
        let days = secs_int / SEC_PER_DAY;
        secs_int %= SEC_PER_DAY;
        let hours = secs_int / SEC_PER_HOUR;
        secs_int %= SEC_PER_HOUR;
        let mins = secs_int / SEC_PER_MIN;
        secs_int %= SEC_PER_MIN;
        let secs = secs_int;

        // Build output (skip zero units, show "0 ms" if everything is zero)
        let mut parts = Vec::with_capacity(6);
        if y > 0 {
            parts.push(format!("{} y", y));
        }
        if days > 0 {
            parts.push(format!("{} d", days));
        }
        if hours > 0 {
            parts.push(format!("{} h", hours));
        }
        if mins > 0 {
            parts.push(format!("{} m", mins));
        }
        if secs > 0 {
            parts.push(format!("{} s", secs));
        }
        if ms > 0 {
            parts.push(format!("{} ms", ms));
        }
        if parts.is_empty() {
            parts.push("0 s".to_string());
        }

        write!(f, "{}{}", sign, parts.join(" "))
    }
}

impl std::str::FromStr for Duration {
    type Err = DurationParseError;

    fn from_str(input: &str) -> Result<Self, Self::Err> {
        let mut s = input.trim();
        if s.is_empty() {
            return Err(DurationParseError::Empty);
        }

        // Leading sign applies to the whole value
        let mut sign = 1.0f64;
        if let Some(rest) = s.strip_prefix('+') {
            s = rest.trim_start();
        } else if let Some(rest) = s.strip_prefix('-') {
            sign = -1.0;
            s = rest.trim_start();
        }

        fn unit_to_ms(unit_raw: &str) -> Option<u128> {
            // Normalize case and μ->µ
            let mut u = unit_raw.trim().to_lowercase();
            u = u.replace('μ', "µ"); // Greek mu -> MICRO SIGN (U+00B5)
            match u.as_str() {
                // years
                "y" | "yr" | "yrs" | "year" | "years" => Some(MS_PER_YEAR as u128),
                // days
                "d" | "day" | "days" => Some(MS_PER_DAY as u128),
                // hours
                "h" | "hr" | "hrs" | "hour" | "hours" => Some(MS_PER_HOUR as u128),
                // minutes
                "m" | "min" | "mins" | "minute" | "minutes" => Some(MS_PER_MIN as u128),
                // seconds
                "s" | "sec" | "secs" | "second" | "seconds" => Some(MS_PER_SEC as u128),
                // milliseconds
                "ms" | "msec" | "msecs" | "millisecond" | "milliseconds" => Some(1),
                _ => None,
            }
        }

        let mut total_ms: u128 = 0;
        for (num, unit) in s
            .split_whitespace()
            .zip(s.split_whitespace().skip(1))
            .step_by(2)
        {
            // integers only; adjust if decimal per-unit magnitudes are needed
            let value: u128 = num
                .parse::<u128>()
                .map_err(|_| DurationParseError::InvalidNumber(num.to_string()))?;

            let scale_ms = unit_to_ms(unit)
                .ok_or_else(|| DurationParseError::UnknownUnit(unit.to_string()))?;

            let add = value
                .checked_mul(scale_ms)
                .ok_or(DurationParseError::Overflow)?;
            total_ms = total_ms
                .checked_add(add)
                .ok_or(DurationParseError::Overflow)?;
        }

        // Single conversion to f64 seconds at the end
        let seconds = (total_ms as f64) * 1e-3;
        // let seconds = 52944172.426;
        Ok(Self(sign * seconds))
    }
}
