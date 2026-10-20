use crate::{SEC_PER_DAY, SEC_PER_HOUR, SEC_PER_MIN, SEC_PER_YEAR, Unit};
use std::{
    cmp::Ordering,
    fmt,
    hash::{Hash, Hasher},
};

#[derive(Clone, Copy, Debug, Default)]
pub struct Duration(f64);

impl Duration {
    pub const ZERO: Self = Duration(0.0);

    pub const MIN: Self = Duration(f64::MIN);

    pub const MAX: Self = Duration(f64::MAX);

    #[inline]
    pub const fn from_seconds(seconds: f64) -> Self {
        Self(seconds)
    }

    #[inline]
    pub const fn from_nanoseconds(nanoseconds: f64) -> Self {
        Self::from_seconds(nanoseconds / Unit::Nanosecond.per_second())
    }

    #[inline]
    pub const fn from_microseconds(microseconds: f64) -> Self {
        Self::from_seconds(microseconds / Unit::Microsecond.per_second())
    }

    #[inline]
    pub const fn from_milliseconds(milliseconds: f64) -> Self {
        Self::from_seconds(milliseconds / Unit::Millisecond.per_second())
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
        self.0 / SEC_PER_DAY
    }

    #[inline]
    pub const fn abs(self) -> Self {
        Duration(self.0.abs())
    }

    #[inline]
    pub const fn is_positive(self) -> bool {
        self.0.is_sign_positive()
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

    #[inline]
    fn key(self) -> f64 {
        self.0 + 0.0
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

impl PartialEq for Duration {
    #[inline]
    fn eq(&self, other: &Self) -> bool {
        self.cmp(other).is_eq()
    }
}

impl Eq for Duration {}

impl PartialOrd for Duration {
    #[inline]
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for Duration {
    #[inline]
    fn cmp(&self, other: &Self) -> Ordering {
        self.key().total_cmp(&other.key())
    }
}

impl Hash for Duration {
    #[inline]
    fn hash<H: Hasher>(&self, state: &mut H) {
        self.key().to_bits().hash(state)
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
    MissingUnit(String),
    Overflow,
}

impl fmt::Display for DurationParseError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            DurationParseError::Empty => write!(f, "empty duration string"),
            DurationParseError::InvalidNumber(s) => write!(f, "invalid number: {s}"),
            DurationParseError::UnknownUnit(u) => write!(f, "unknown unit: {u}"),
            DurationParseError::MissingUnit(n) => write!(f, "missing unit after {n}"),
            DurationParseError::Overflow => write!(f, "duration overflow"),
        }
    }
}

impl std::error::Error for DurationParseError {}

#[inline]
fn split_whole_seconds(seconds: u64) -> [u64; 5] {
    const YEAR: u64 = SEC_PER_YEAR as u64;
    const DAY: u64 = SEC_PER_DAY as u64;
    const HOUR: u64 = SEC_PER_HOUR as u64;
    const MINUTE: u64 = SEC_PER_MIN as u64;
    let (years, seconds) = (seconds / YEAR, seconds % YEAR);
    let (days, seconds) = (seconds / DAY, seconds % DAY);
    let (hours, seconds) = (seconds / HOUR, seconds % HOUR);
    [years, days, hours, seconds / MINUTE, seconds % MINUTE]
}

fn write_parts(f: &mut fmt::Formatter<'_>, parts: &[(u64, &str)]) -> fmt::Result {
    let mut separator = "";
    for &(value, unit) in parts {
        if value > 0 {
            write!(f, "{separator}{value} {unit}")?;
            separator = " ";
        }
    }
    Ok(())
}

impl Duration {
    fn fmt_exact(self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let sign = if self.0 < 0.0 { "-" } else { "" };
        let seconds = self.0.abs();
        let whole = seconds.trunc();
        let Some(whole_seconds) = (whole < u64::MAX as f64).then_some(whole as u64) else {
            return write!(f, "{sign}{seconds} s");
        };
        let [years, days, hours, minutes, rest] = split_whole_seconds(whole_seconds);
        let parts = [(years, "y"), (days, "d"), (hours, "h"), (minutes, "m")];
        let prefix = format!("{sign}{}", fmt::from_fn(|f| write_parts(f, &parts)));
        let separator = if years + days + hours + minutes > 0 {
            " "
        } else {
            ""
        };
        // Exact: less than a minute, with the resolution of `seconds`.
        let rest = rest as f64 + (seconds - whole);
        if rest == 0.0 && !separator.is_empty() {
            return f.write_str(&prefix);
        }
        // The fewest decimals of the rest that parse back to this duration once added to the larger
        // units. The shortest representation of the rest alone always does, as the rest is exact.
        let with_rest = |rest: &str| format!("{prefix}{separator}{rest} s");
        let text = (0..=17)
            .map(|decimals| with_rest(&format!("{rest:.decimals$}")))
            .find(|text| text.parse() == Ok(self))
            .unwrap_or_else(|| with_rest(&rest.to_string()));
        f.write_str(&text)
    }

    #[inline]
    pub fn expanded(self) -> impl fmt::Display {
        fmt::from_fn(move |f| self.fmt_rounded(f, false))
    }

    fn fmt_rounded(self, f: &mut fmt::Formatter<'_>, compact: bool) -> fmt::Result {
        let milliseconds = (self.0.abs() * 1e3).round();
        let sign = if self.0 < 0.0 && milliseconds > 0.0 {
            "-"
        } else {
            ""
        };
        let Some(milliseconds) = (milliseconds < u64::MAX as f64).then_some(milliseconds as u64)
        else {
            return write!(f, "{sign}{:.3e} s", self.0.abs());
        };
        const DAY: u64 = SEC_PER_DAY as u64 * 1000;
        const HOUR: u64 = SEC_PER_HOUR as u64 * 1000;
        const MINUTE: u64 = SEC_PER_MIN as u64 * 1000;
        let (days, rest) = (milliseconds / DAY, milliseconds % DAY);
        let (hours, rest) = (rest / HOUR, rest % HOUR);
        let (minutes, rest) = (rest / MINUTE, rest % MINUTE);
        let (seconds, milliseconds) = (rest / 1000, rest % 1000);

        f.write_str(sign)?;
        let mut separator = None;
        for (value, unit) in [(days, "d"), (hours, "h"), (minutes, "m")] {
            match separator {
                None if value > 0 => write!(f, "{value}{unit}")?,
                Some(separator) if value > 0 || !compact => {
                    write!(f, "{separator}{value:02}{unit}")?
                }
                _ => continue,
            }
            separator = Some(" ");
        }
        let separator = separator.unwrap_or("");
        match (compact, seconds, milliseconds) {
            (false, ..) => write!(f, "{separator}{seconds:02}.{milliseconds:03}s"),
            (true, 0, 0) if !separator.is_empty() => Ok(()),
            (true, _, 0) if separator.is_empty() => write!(f, "{seconds}s"),
            (true, _, 0) => write!(f, " {seconds:02}s"),
            (true, ..) if separator.is_empty() => write!(f, "{seconds}.{milliseconds:03}s"),
            (true, ..) => write!(f, " {seconds:02}.{milliseconds:03}s"),
        }
    }
}

impl fmt::Display for Duration {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if f.alternate() {
            self.fmt_exact(f)
        } else {
            self.fmt_rounded(f, true)
        }
    }
}

fn parse_unit(unit: &str) -> Option<Unit> {
    Some(match unit.to_lowercase().as_str() {
        "y" | "yr" | "yrs" | "year" | "years" => Unit::Year,
        "d" | "day" | "days" => Unit::Day,
        "h" | "hr" | "hrs" | "hour" | "hours" => Unit::Hour,
        "m" | "min" | "mins" | "minute" | "minutes" => Unit::Minute,
        "s" | "sec" | "secs" | "second" | "seconds" => Unit::Second,
        "ms" | "msec" | "msecs" | "millisecond" | "milliseconds" => Unit::Millisecond,
        "us" | "µs" | "μs" | "usec" | "usecs" | "microsecond" | "microseconds" => {
            Unit::Microsecond
        }
        "ns" | "nsec" | "nsecs" | "nanosecond" | "nanoseconds" => Unit::Nanosecond,
        _ => return None,
    })
}

impl std::str::FromStr for Duration {
    type Err = DurationParseError;

    fn from_str(input: &str) -> Result<Self, Self::Err> {
        let input = input.trim();
        let (negative, mut rest) = match input.strip_prefix('-') {
            Some(rest) => (true, rest.trim_start()),
            None => (false, input.strip_prefix('+').unwrap_or(input).trim_start()),
        };
        if rest.is_empty() {
            return Err(DurationParseError::Empty);
        }

        let mut whole: u128 = 0;
        // The decimals of the seconds, parsed with the whole seconds as one number, so that a duration
        // split into larger units (`1 m 0.3 s`) is the same as in seconds (`60.3 s`).
        let mut decimals = None;
        let mut fraction = 0.0;
        let mut first = true;
        while !rest.is_empty() {
            let component = rest;
            let token = || {
                let token = component.split_whitespace().next().unwrap_or(component);
                DurationParseError::InvalidNumber(token.to_string())
            };

            let number_end = rest
                .find(|c: char| !(c.is_ascii_digit() || c == '.'))
                .unwrap_or(rest.len());
            let (number, tail) = rest.split_at(number_end);
            let tail = tail.trim_start();
            let unit_end = tail
                .find(|c: char| !c.is_alphabetic())
                .unwrap_or(tail.len());
            let (unit, tail) = tail.split_at(unit_end);
            rest = tail.trim_start();

            if !number.bytes().any(|b| b.is_ascii_digit()) || number.matches('.').count() > 1 {
                return Err(token());
            }
            let unit = match unit {
                "" if first && rest.is_empty() => Unit::Second,
                "" => return Err(DurationParseError::MissingUnit(number.to_string())),
                unit => parse_unit(unit)
                    .ok_or_else(|| DurationParseError::UnknownUnit(unit.to_string()))?,
            };
            first = false;

            if let (Unit::Second, None, Some((integer, digits))) =
                (unit, decimals, number.split_once('.'))
                && let Ok(integer) = match integer {
                    "" => Ok(0),
                    integer => integer.parse::<u128>(),
                }
            {
                whole = whole
                    .checked_add(integer)
                    .ok_or(DurationParseError::Overflow)?;
                decimals = Some(digits);
                continue;
            }
            match (unit.whole_seconds(), number.parse::<u128>()) {
                (Some(scale), Ok(value)) => {
                    whole = value
                        .checked_mul(scale as u128)
                        .and_then(|value| whole.checked_add(value))
                        .ok_or(DurationParseError::Overflow)?;
                }
                (scale, _) => {
                    let value: f64 = number.parse().map_err(|_| token())?;
                    fraction += match scale {
                        Some(scale) => value * scale as f64,
                        None => value / unit.per_second(),
                    };
                }
            }
        }

        let seconds = format!("{whole}.{}", decimals.unwrap_or_default())
            .parse::<f64>()
            .map_or(f64::INFINITY, |seconds| seconds + fraction);
        if !seconds.is_finite() {
            return Err(DurationParseError::Overflow);
        }
        Ok(Duration(if negative { -seconds } else { seconds }))
    }
}

#[cfg(feature = "serde")]
impl serde::Serialize for Duration {
    #[inline]
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        serializer.collect_str(&format_args!("{self:#}"))
    }
}

#[cfg(feature = "serde")]
impl<'de> serde::Deserialize<'de> for Duration {
    #[inline]
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        String::deserialize(deserializer)?
            .parse()
            .map_err(serde::de::Error::custom)
    }
}
