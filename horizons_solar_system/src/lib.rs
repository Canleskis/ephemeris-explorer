mod data;
pub use data::*;

use ftime::{Duration, Epoch};
use glam::DVec3;
use serde::{Deserialize, Serialize};
use std::str::FromStr;

fn from_horizons_date(date_str: &str) -> Option<Epoch> {
    let date_str = date_str.split_once(' ')?.1;
    let month_idx = date_str.find('-')? + 1;
    let month_num = match &date_str[month_idx..month_idx + 3] {
        "Jan" => "01",
        "Feb" => "02",
        "Mar" => "03",
        "Apr" => "04",
        "May" => "05",
        "Jun" => "06",
        "Jul" => "07",
        "Aug" => "08",
        "Sep" => "09",
        "Oct" => "10",
        "Nov" => "11",
        "Dec" => "12",
        _ => return None,
    };
    let mut result = date_str.to_string();
    result.replace_range(month_idx..month_idx + 3, month_num);
    result.parse().ok()
}

fn to_iso(date_str: Epoch) -> String {
    date_str.to_string().replace(' ', "T")
}

const TT_OFFSET: Duration = Duration::from_seconds(32.184);

fn to_tai(epoch: Epoch) -> Epoch {
    epoch - TT_OFFSET
}

fn to_tt(epoch: Epoch) -> Epoch {
    epoch + TT_OFFSET
}

#[inline]
fn parse_line(line: &str) -> Option<(Epoch, DVec3, DVec3)> {
    let mut data = line.split(',').map(str::trim).skip(1);
    let epoch = from_horizons_date(data.next()?)?;
    let mut data = data.flat_map(f64::from_str);
    Some((
        to_tai(epoch),
        DVec3::new(data.next()?, data.next()?, data.next()?),
        DVec3::new(data.next()?, data.next()?, data.next()?),
    ))
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct ParseLineError;
impl std::fmt::Display for ParseLineError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "failed to parse line")
    }
}
impl std::error::Error for ParseLineError {}

#[inline]
pub fn fetch_body(
    object: SolarSystemObject,
    start: Epoch,
    end: Epoch,
    step: Duration,
) -> Result<Vec<(Epoch, Body)>, Box<dyn std::error::Error>> {
    const HORIZONS_URL: &str = "https://ssd.jpl.nasa.gov/api/horizons.api";

    let id = object as i32;
    let url = reqwest::Url::parse_with_params(
        HORIZONS_URL,
        [
            ("format", "text"),
            ("MAKE_EPHEM", "YES"),
            ("COMMAND", &id.to_string()),
            ("EPHEM_TYPE", "VECTORS"),
            ("OBJ_DATA", "NO"),
            ("CENTER", "500@0"),
            ("TIME_TYPE", "TT"),
            ("START_TIME", &to_iso(to_tt(start))),
            ("STOP_TIME", &to_iso(to_tt(end))),
            ("STEP_SIZE", &format!("'{}'", step.as_seconds())),
            ("REF_SYSTEM", "ICRF"),
            ("REF_PLANE", "FRAME"),
            ("VEC_TABLE", "2"),
            ("OUT_UNITS", "KM-S"),
            ("VEC_CORR", "NONE"),
            ("CSV_FORMAT", "YES"),
        ],
    )?;

    let resp = reqwest::blocking::get(url)?.text()?;
    let start = resp.find("$$SOE").ok_or_else(|| resp.clone())? + 6;
    let end = resp.find("$$EOE").ok_or_else(|| resp.clone())? - 1;

    resp[start..end]
        .lines()
        .map(|line| {
            parse_line(line)
                .map(|(epoch, pos, vel)| {
                    (
                        epoch,
                        Body {
                            name: object.to_string(),
                            mu: GRAVITATIONAL_PARAMETERS[&id],
                            position: pos,
                            velocity: vel,
                        },
                    )
                })
                .ok_or(Box::new(ParseLineError) as _)
        })
        .collect()
}

#[inline]
pub fn fetch_solar_system(
    objects: &[SolarSystemObject],
    start: Epoch,
    end: Epoch,
    step: Duration,
) -> Result<Vec<SolarSystem>, Box<dyn std::error::Error>> {
    objects
        .iter()
        .try_fold(Vec::new(), |mut acc: Vec<SolarSystem>, &object| {
            let body_data = fetch_body(object, start, end, step)?;
            for (i, (epoch, data)) in body_data.into_iter().enumerate() {
                match acc.get_mut(i) {
                    Some(solar_system) => solar_system.bodies.push(data),
                    None => acc.push(SolarSystem {
                        epoch,
                        bodies: vec![data],
                    }),
                }
            }
            Ok(acc)
        })
}

#[derive(Serialize, Deserialize, Debug)]
pub struct Body {
    pub name: String,
    pub mu: f64,
    pub position: DVec3,
    pub velocity: DVec3,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct SolarSystem {
    pub epoch: Epoch,
    pub bodies: Vec<Body>,
}
