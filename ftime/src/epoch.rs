use crate::{MS_PER_SEC, SEC_PER_DAY, duration::Duration};

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Epoch {
    /// Seconds since TAI epoch (1958-01-01T00:00:00).
    offset: Duration,
}

impl Epoch {
    /// Construct from civil date-time components, interpreted on the TAI scale.
    /// Fractional seconds are provided as nanoseconds (0..=999_999_999).
    #[inline]
    pub fn from_datetime(
        year: i64,
        month: u32,
        day: u32,
        hour: u32,
        minute: u32,
        second: u32,
        millis: u32,
    ) -> Result<Self, EpochParseError> {
        if !(1..=12).contains(&month) || hour > 23 || minute > 59 || second > 59 || millis > 999 {
            return Err(EpochParseError::OutOfRange);
        }
        let z = days_from_civil(year, month, day);
        let (vy, vm, vd) = civil_from_days(z);
        if vy != year || vm != month || vd != day {
            return Err(EpochParseError::InvalidDate);
        }
        let base_1958 = days_from_civil(1958, 1, 1);
        let days_since_1958 = z - base_1958;
        let sod = (hour as i64) * 3600 + (minute as i64) * 60 + (second as i64);
        let secs = days_since_1958 * SEC_PER_DAY as i64 + sod;
        let total = secs as f64 + (millis as f64) / MS_PER_SEC as f64;
        Ok(Epoch {
            offset: Duration::from_seconds(total),
        })
    }

    #[inline]
    pub const fn from_offset(offset: Duration) -> Self {
        Epoch { offset }
    }

    #[inline]
    pub fn as_offset(self) -> Duration {
        self.offset
    }

    #[inline]
    pub fn as_offset_seconds(&self) -> f64 {
        self.offset.as_seconds()
    }

    #[inline]
    pub fn mut_offset(&mut self) -> &mut Duration {
        &mut self.offset
    }

    #[inline]
    pub fn floor(self, to: Duration) -> Self {
        Epoch {
            offset: self.offset.floor(to),
        }
    }
}

impl std::ops::Add<Duration> for Epoch {
    type Output = Epoch;

    #[inline]
    fn add(self, rhs: Duration) -> Self::Output {
        Epoch {
            offset: self.offset + rhs,
        }
    }
}

impl std::ops::AddAssign<Duration> for Epoch {
    #[inline]
    fn add_assign(&mut self, rhs: Duration) {
        self.offset += rhs;
    }
}

impl std::ops::Sub<Duration> for Epoch {
    type Output = Epoch;

    #[inline]
    fn sub(self, rhs: Duration) -> Self::Output {
        Epoch {
            offset: self.offset - rhs,
        }
    }
}

impl std::ops::SubAssign<Duration> for Epoch {
    #[inline]
    fn sub_assign(&mut self, rhs: Duration) {
        self.offset -= rhs;
    }
}

impl std::ops::Sub for Epoch {
    type Output = Duration;

    #[inline]
    fn sub(self, rhs: Self) -> Self::Output {
        self.offset - rhs.offset
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
impl std::fmt::Display for EpochParseError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
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

// Parse "YYYY-MM-DD HH:MM:SS[.frac]" (TAI scale).
impl std::str::FromStr for Epoch {
    type Err = EpochParseError;

    #[inline]
    fn from_str(s: &str) -> Result<Self, Self::Err> {
        // Split date/time
        let (date_str, time_str) = s.split_once(' ').ok_or(EpochParseError::MissingSeparator)?;

        // Date
        let mut dparts = date_str.splitn(3, '-');
        let year: i64 = dparts
            .next()
            .ok_or(EpochParseError::BadDate)?
            .parse()
            .map_err(|_| EpochParseError::BadNumber)?;
        let month: u32 = dparts
            .next()
            .ok_or(EpochParseError::BadDate)?
            .parse()
            .map_err(|_| EpochParseError::BadNumber)?;
        let day: u32 = dparts
            .next()
            .ok_or(EpochParseError::BadDate)?
            .parse()
            .map_err(|_| EpochParseError::BadNumber)?;

        // Time and optional fractional part
        let (hms_str, frac_opt) = match time_str.split_once('.') {
            Some((hms, frac)) => (hms, Some(frac)),
            None => (time_str, None),
        };
        let mut tparts = hms_str.splitn(3, ':');
        let hour: u32 = tparts
            .next()
            .ok_or(EpochParseError::BadTime)?
            .parse()
            .map_err(|_| EpochParseError::BadNumber)?;
        let minute: u32 = tparts
            .next()
            .ok_or(EpochParseError::BadTime)?
            .parse()
            .map_err(|_| EpochParseError::BadNumber)?;
        let second: u32 = tparts
            .next()
            .ok_or(EpochParseError::BadTime)?
            .parse()
            .map_err(|_| EpochParseError::BadNumber)?;

        let millis: u32 = if let Some(frac) = frac_opt {
            // We only support milliseconds precision and truncate extra digits.
            frac[..3].parse().map_err(|_| EpochParseError::BadNumber)?
        } else {
            0
        };

        Epoch::from_datetime(year, month, day, hour, minute, second, millis)
    }
}

// Format as "YYYY-MM-DD HH:MM:SS.nnnnnnnnn (TAI)"
impl std::fmt::Display for Epoch {
    #[inline]
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let mut secs = self.offset.as_seconds().floor() as i64;
        let mut millis =
            ((self.offset.as_seconds() - secs as f64) * MS_PER_SEC as f64).round() as i64;
        if millis == MS_PER_SEC as i64 {
            secs += 1;
            millis = 0;
        }

        // Days since 1958-01-01 and seconds-of-day
        let (days_since_1958, sod) = floor_div_mod(secs, SEC_PER_DAY as i64);

        // Days between 1958-01-01 and 1970-01-01 in the proleptic Gregorian calendar.
        const DAYS_1958_TO_1970: i64 = 4_383;

        // Convert to days since 1970-01-01 for civil_from_days
        let z_1970 = days_since_1958 - DAYS_1958_TO_1970;
        let (year, month, day) = civil_from_days(z_1970);

        let hour = sod / 3600;
        let minute = (sod % 3600) / 60;
        let second = sod % 60;

        write!(
            f,
            "{year:04}-{month:02}-{day:02} {hour:02}:{minute:02}:{second:02}.{millis:03}"
        )
    }
}

// Floor div/mod that yields r in [0, b-1] even for negative a.
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

// Howard Hinnant's public-domain algorithms.
// days_from_civil: days since 1970-01-01 (Unix epoch), proleptic Gregorian.
#[inline]
fn days_from_civil(y: i64, m: u32, d: u32) -> i64 {
    let y = y - if m <= 2 { 1 } else { 0 };
    let era = if y >= 0 { y } else { y - 399 } / 400;
    let yoe = y - era * 400; // [0,399]
    let mp = if m > 2 { m as i64 - 3 } else { m as i64 + 9 }; // Mar=0..Jan=10,Feb=11
    let doy = (153 * mp + 2) / 5 + d as i64 - 1; // [0,365]
    let doe = yoe * 365 + yoe / 4 - yoe / 100 + doy; // [0,146096]
    era * 146_097 + doe - 719_468
}

// civil_from_days: inverse of days_from_civil (z is days since 1970-01-01)
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
impl serde::Serialize for Duration {
    #[inline]
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        serializer.serialize_str(&self.to_string())
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

#[cfg(feature = "serde")]
impl serde::Serialize for Epoch {
    #[inline]
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        serializer.serialize_str(&self.to_string())
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
