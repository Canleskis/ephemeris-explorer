mod data;
pub use data::*;

use bevy_math::DVec3;
use hifitime::{Duration, Epoch, MonthName, TimeScale};
use serde::{Deserialize, Serialize};
use std::str::FromStr;

#[inline]
fn convert_date(date_str: &str) -> Option<String> {
    let date_str = date_str.split_once(' ')?.1;
    let mut result = date_str.to_string();
    result.push_str(" TT");

    if let Some(month_pos) = date_str.find(char::is_alphabetic) {
        if let Ok(month) = MonthName::from_str(&date_str[month_pos..month_pos + 3]) {
            let month_num = match month {
                MonthName::January => "01",
                MonthName::February => "02",
                MonthName::March => "03",
                MonthName::April => "04",
                MonthName::May => "05",
                MonthName::June => "06",
                MonthName::July => "07",
                MonthName::August => "08",
                MonthName::September => "09",
                MonthName::October => "10",
                MonthName::November => "11",
                MonthName::December => "12",
            };
            result.replace_range(month_pos..month_pos + 3, month_num);
            return Some(result);
        }
    }

    None
}

#[inline]
fn parse_line(line: &str) -> Option<(Epoch, DVec3, DVec3)> {
    let mut data = line.split(',').map(str::trim).skip(1);
    let epoch = Epoch::from_str(&convert_date(data.next()?)?).ok()?;
    let mut data = data.flat_map(f64::from_str);
    Some((
        epoch.to_time_scale(TimeScale::TAI),
        DVec3::new(data.next()?, data.next()?, data.next()?),
        DVec3::new(data.next()?, data.next()?, data.next()?),
    ))
}

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
            (
                "START_TIME",
                &start.to_time_scale(TimeScale::TT).to_isoformat(),
            ),
            (
                "STOP_TIME",
                &end.to_time_scale(TimeScale::TT).to_isoformat(),
            ),
            ("STEP_SIZE", &format!("'{step}'")),
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

    Ok(resp[start..end]
        .lines()
        .flat_map(parse_line)
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
        .collect())
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
