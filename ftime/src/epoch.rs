use crate::{SEC_PER_DAY, duration::Duration};
use std::{
    cmp::Ordering,
    fmt,
    hash::{Hash, Hasher},
};

#[derive(Clone, Copy, Debug)]
pub struct Epoch {
    seconds: i64,
    fraction: f64,
}

const SATURATION: f64 = 9_223_372_036_854_775_808.0;

const SECONDS_PER_DAY: i64 = SEC_PER_DAY as i64;

const DAYS_1958_TO_1970: i64 = 4_383;

impl Epoch {
    pub const ZERO: Self = Epoch {
        seconds: 0,
        fraction: 0.0,
    };

    pub const MIN: Self = Epoch {
        seconds: i64::MIN,
        fraction: 0.0,
    };

    pub const MAX: Self = Epoch {
        seconds: i64::MAX,
        fraction: 1.0 - f64::EPSILON / 2.0,
    };

    #[inline]
    pub const fn new(seconds: i64, fraction: f64) -> Self {
        Epoch {
            seconds,
            fraction: 0.0,
        }
        .add_seconds(fraction)
    }

    pub fn from_datetime(
        year: i64,
        month: u32,
        day: u32,
        hour: u32,
        minute: u32,
        second: u32,
        fraction: f64,
    ) -> Result<Self, EpochParseError> {
        const MAX_YEAR: i64 = 1_000_000_000_000;
        if !(-MAX_YEAR..=MAX_YEAR).contains(&year)
            || !(1..=12).contains(&month)
            || hour > 23
            || minute > 59
            || second > 59
            || !(0.0..=1.0).contains(&fraction)
        {
            return Err(EpochParseError::OutOfRange);
        }
        let days = days_from_civil(year, month, day);
        if civil_from_days(days) != (year, month, day) {
            return Err(EpochParseError::InvalidDate);
        }
        let seconds_of_day = (hour * 3600 + minute * 60 + second) as i64;
        let seconds = (days + DAYS_1958_TO_1970)
            .checked_mul(SECONDS_PER_DAY)
            .and_then(|seconds| seconds.checked_add(seconds_of_day))
            .ok_or(EpochParseError::OutOfRange)?;
        Ok(Epoch::new(seconds, fraction))
    }

    #[inline]
    pub const fn from_offset(offset: Duration) -> Self {
        Epoch::from_offset_seconds(offset.as_seconds())
    }

    #[inline]
    pub const fn from_offset_seconds(seconds: f64) -> Self {
        Epoch::ZERO.add_seconds(seconds)
    }

    #[inline]
    pub const fn as_offset(self) -> Duration {
        Duration::from_seconds(self.as_offset_seconds())
    }

    #[inline]
    pub const fn as_offset_seconds(self) -> f64 {
        self.seconds as f64 + self.fraction
    }

    #[inline]
    pub const fn whole_seconds(self) -> i64 {
        self.seconds
    }

    #[inline]
    pub const fn fraction(self) -> f64 {
        self.fraction
    }

    #[inline]
    pub fn floor(self, to: Duration) -> Self {
        self.round_to(to, false)
    }

    #[inline]
    pub fn round(self, to: Duration) -> Self {
        self.round_to(to, true)
    }

    pub fn add_months(self, months: i64) -> Self {
        const MAX_YEAR: i64 = 1_000_000_000_000;
        let saturated = if months > 0 { Epoch::MAX } else { Epoch::MIN };

        let (days, seconds_of_day) = floor_div_mod(self.seconds, SECONDS_PER_DAY);
        let (year, month, day) = civil_from_days(days - DAYS_1958_TO_1970);
        let Some(months) = (year * 12 + month as i64 - 1).checked_add(months) else {
            return saturated;
        };
        let (year, month) = (months.div_euclid(12), months.rem_euclid(12) as u32 + 1);
        if !(-MAX_YEAR..=MAX_YEAR).contains(&year) {
            return saturated;
        }
        let day = day.min(days_in_month(year, month));

        match (days_from_civil(year, month, day) + DAYS_1958_TO_1970)
            .checked_mul(SECONDS_PER_DAY)
            .and_then(|seconds| seconds.checked_add(seconds_of_day))
        {
            Some(seconds) => Epoch {
                seconds,
                fraction: self.fraction,
            },
            None => saturated,
        }
    }

    fn round_to(self, to: Duration, nearest: bool) -> Self {
        let step = to.as_seconds();
        if step.is_nan() || step <= 0.0 {
            return self;
        }
        let round = |x: f64| {
            if nearest {
                (x + 0.5).floor()
            } else {
                x.floor()
            }
        };
        if (1.0..SATURATION).contains(&step) && step.fract() == 0.0 {
            let step = step as i64;
            let (multiples, rest) = floor_div_mod(self.seconds, step);
            let up = nearest && 2.0 * (rest as f64 + self.fraction) >= step as f64;
            return match multiples
                .checked_add(up as i64)
                .and_then(|multiples| multiples.checked_mul(step))
            {
                Some(seconds) => Epoch {
                    seconds,
                    fraction: 0.0,
                },
                None if up => Epoch::MAX,
                None => Epoch::MIN,
            };
        }
        let per_second = (1.0 / step).round();
        if step > 0.0 && step < 1.0 && per_second * step == 1.0 {
            return Epoch::new(self.seconds, round(self.fraction * per_second) / per_second);
        }
        Epoch::from_offset_seconds(round(self.as_offset_seconds() / step) * step)
    }

    #[inline]
    const fn add_seconds(self, seconds: f64) -> Self {
        if seconds.is_nan() || seconds.abs() >= SATURATION {
            return if seconds > 0.0 {
                Epoch::MAX
            } else {
                Epoch::MIN
            };
        }
        let whole = seconds as i64;
        let rest = seconds - whole as f64;
        match self.seconds.checked_add(whole) {
            Some(whole) => Epoch::normalised(whole, self.fraction + rest),
            None if whole > 0 => Epoch::MAX,
            None => Epoch::MIN,
        }
    }

    #[inline]
    const fn normalised(seconds: i64, s: f64) -> Self {
        let whole = s as i64;
        let fraction = s - whole as f64;
        let (whole, fraction) = if fraction >= 0.0 {
            (whole, fraction + 0.0)
        } else if fraction + 1.0 < 1.0 {
            (whole - 1, fraction + 1.0)
        } else {
            (whole, 0.0)
        };
        match seconds.checked_add(whole) {
            Some(seconds) => Epoch { seconds, fraction },
            None if whole > 0 => Epoch::MAX,
            None => Epoch::MIN,
        }
    }
}

impl Default for Epoch {
    #[inline]
    fn default() -> Self {
        Epoch::ZERO
    }
}

impl std::ops::Add<Duration> for Epoch {
    type Output = Epoch;

    #[inline]
    fn add(self, rhs: Duration) -> Self::Output {
        self.add_seconds(rhs.as_seconds())
    }
}

impl std::ops::AddAssign<Duration> for Epoch {
    #[inline]
    fn add_assign(&mut self, rhs: Duration) {
        *self = *self + rhs;
    }
}

impl std::ops::Sub<Duration> for Epoch {
    type Output = Epoch;

    #[inline]
    fn sub(self, rhs: Duration) -> Self::Output {
        self.add_seconds(-rhs.as_seconds())
    }
}

impl std::ops::SubAssign<Duration> for Epoch {
    #[inline]
    fn sub_assign(&mut self, rhs: Duration) {
        *self = *self - rhs;
    }
}

impl std::ops::Sub for Epoch {
    type Output = Duration;

    #[inline]
    fn sub(self, rhs: Self) -> Self::Output {
        let seconds = match self.seconds.checked_sub(rhs.seconds) {
            Some(seconds) => seconds as f64,
            None => self.seconds as f64 - rhs.seconds as f64,
        };
        Duration::from_seconds(seconds + (self.fraction - rhs.fraction))
    }
}

impl PartialEq for Epoch {
    #[inline]
    fn eq(&self, other: &Self) -> bool {
        self.seconds == other.seconds && self.fraction == other.fraction
    }
}

impl Eq for Epoch {}

impl PartialOrd for Epoch {
    #[inline]
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for Epoch {
    #[inline]
    fn cmp(&self, other: &Self) -> Ordering {
        self.seconds
            .cmp(&other.seconds)
            .then(self.fraction.total_cmp(&other.fraction))
    }
}

impl Hash for Epoch {
    #[inline]
    fn hash<H: Hasher>(&self, state: &mut H) {
        (self.as_offset_seconds() + 0.0).to_bits().hash(state)
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum EpochParseError {
    MissingSeparator,
    BadDate,
    BadTime,
    BadNumber,
    OutOfRange,
    InvalidDate,
}

impl fmt::Display for EpochParseError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            EpochParseError::MissingSeparator => write!(f, "missing space between date and time"),
            EpochParseError::BadDate => write!(f, "bad date format"),
            EpochParseError::BadTime => write!(f, "bad time format"),
            EpochParseError::BadNumber => write!(f, "invalid number"),
            EpochParseError::OutOfRange => write!(f, "date or time component out of range"),
            EpochParseError::InvalidDate => write!(f, "invalid date"),
        }
    }
}

impl std::error::Error for EpochParseError {}

fn component<T: std::str::FromStr>(
    part: Option<&str>,
    missing: EpochParseError,
) -> Result<T, EpochParseError> {
    part.ok_or(missing)?
        .parse()
        .map_err(|_| EpochParseError::BadNumber)
}

impl std::str::FromStr for Epoch {
    type Err = EpochParseError;

    #[inline]
    fn from_str(s: &str) -> Result<Self, Self::Err> {
        let (date, time) = s
            .trim()
            .split_once([' ', 'T'])
            .ok_or(EpochParseError::MissingSeparator)?;

        let mut date = date.splitn(3, '-');
        let year = component(date.next(), EpochParseError::BadDate)?;
        let month = component(date.next(), EpochParseError::BadDate)?;
        let day = component(date.next(), EpochParseError::BadDate)?;

        let (time, fraction) = match time.split_once('.') {
            Some((time, fraction)) => (time, Some(fraction)),
            None => (time, None),
        };
        let mut time = time.splitn(3, ':');
        let hour = component(time.next(), EpochParseError::BadTime)?;
        let minute = component(time.next(), EpochParseError::BadTime)?;
        let second = component(time.next(), EpochParseError::BadTime)?;

        let fraction = match fraction {
            None => 0.0,
            Some(digits) if !digits.is_empty() && digits.bytes().all(|b| b.is_ascii_digit()) => {
                format!("0.{digits}")
                    .parse()
                    .map_err(|_| EpochParseError::BadNumber)?
            }
            Some(_) => return Err(EpochParseError::BadNumber),
        };

        Epoch::from_datetime(year, month, day, hour, minute, second, fraction)
    }
}

fn fmt_date_time(seconds: i64, f: &mut fmt::Formatter<'_>) -> fmt::Result {
    let (days, seconds_of_day) = floor_div_mod(seconds, SECONDS_PER_DAY);
    let (year, month, day) = civil_from_days(days - DAYS_1958_TO_1970);
    let hour = seconds_of_day / 3600;
    let minute = seconds_of_day % 3600 / 60;
    let second = seconds_of_day % 60;
    write!(
        f,
        "{year:04}-{month:02}-{day:02} {hour:02}:{minute:02}:{second:02}"
    )
}

impl fmt::Display for Epoch {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if f.alternate() {
            fmt_date_time(self.seconds, f)?;
            if self.fraction > 0.0 {
                f.write_str(&self.fraction.to_string()[1..])?;
            }
            return Ok(());
        }
        let fraction = format!("{:.*}", f.precision().unwrap_or(3), self.fraction);
        match fraction.strip_prefix('1') {
            Some(zeros) => {
                fmt_date_time(self.seconds.saturating_add(1), f)?;
                f.write_str(zeros)
            }
            None => {
                fmt_date_time(self.seconds, f)?;
                f.write_str(&fraction[1..])
            }
        }
    }
}

#[inline]
fn floor_div_mod(a: i64, b: i64) -> (i64, i64) {
    let mut q = a / b;
    let mut r = a % b;
    if r < 0 {
        q -= 1;
        r += b;
    }
    (q, r)
}

#[inline]
fn days_from_civil(y: i64, m: u32, d: u32) -> i64 {
    let y = y - if m <= 2 { 1 } else { 0 };
    let era = if y >= 0 { y } else { y - 399 } / 400;
    let yoe = y - era * 400;
    let mp = if m > 2 { m as i64 - 3 } else { m as i64 + 9 };
    let doy = (153 * mp + 2) / 5 + d as i64 - 1;
    let doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    era * 146_097 + doe - 719_468
}

#[inline]
fn days_in_month(year: i64, month: u32) -> u32 {
    match month {
        2 if year % 4 == 0 && (year % 100 != 0 || year % 400 == 0) => 29,
        2 => 28,
        4 | 6 | 9 | 11 => 30,
        _ => 31,
    }
}

#[inline]
fn civil_from_days(mut z: i64) -> (i64, u32, u32) {
    z += 719_468;
    let era = if z >= 0 { z } else { z - 146_096 } / 146_097;
    let doe = z - era * 146_097;
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let y = yoe + era * 400;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = (doy - (153 * mp + 2) / 5 + 1) as u32;
    let m = (if mp < 10 { mp + 3 } else { mp - 9 }) as u32;
    let year = y + if m <= 2 { 1 } else { 0 };
    (year, m, d)
}

#[cfg(feature = "serde")]
impl serde::Serialize for Epoch {
    #[inline]
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        serializer.collect_str(&format_args!("{self:#}"))
    }
}

#[cfg(feature = "serde")]
impl<'de> serde::Deserialize<'de> for Epoch {
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
