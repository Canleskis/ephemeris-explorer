use ftime::{Duration, Epoch, SEC_PER_DAY};
use horizons_solar_system::SolarSystemObject;

#[rustfmt::skip]
const BODIES: &[SolarSystemObject] = {
    use SolarSystemObject::*;
    &[
        Sun,
            Mercury,
            Venus,
            Earth,
                Moon,
            Mars,
            JupiterBarycenter,
            SaturnBarycenter,
            UranusBarycenter,
            NeptuneBarycenter,
    ]
};

// #[rustfmt::skip]
// const BODIES: &[SolarSystemObject] = {
//     use SolarSystemObject::*;
//     &[
//         Sun,
//             Mercury,
//             Venus,
//             Earth,
//             Moon,
//             Mars,
//                 Phobos,
//                 Deimos,
//             Ceres,
//             Jupiter,
//                 Io,
//                 Europa,
//                 Ganymede,
//                 Callisto,
//             Saturn,
//                 Mimas,
//                 Enceladus,
//                 Tethys,
//                 Dione,
//                 Rhea,
//                 Titan,
//                 Iapetus,
//             Uranus,
//                 Ariel,
//                 Umbriel,
//                 Titania,
//                 Oberon,
//                 Miranda,
//             Neptune,
//                 Triton,
//             Pluto,
//                 Charon,
//     ]
// };

const EPOCH: &str = "1950-01-01 00:00:00";

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let epoch: Epoch = EPOCH.parse()?;
    let system = horizons_solar_system::fetch_solar_system(
        BODIES,
        epoch,
        epoch + Duration::from_seconds(1.0),
        Duration::from_days(1.0),
    )?
    .remove(0);

    let path = format!("solar_system_{}.json", to_jde_tai_days(epoch));
    let writer = std::io::BufWriter::new(std::fs::File::create(path)?);
    serde_json::to_writer_pretty(writer, &system)?;

    Ok(())
}

pub const JD_TAI_EPOCH: f64 = 2436204.5;

fn to_jde_tai_days(epoch: Epoch) -> f64 {
    epoch.as_offset_seconds() / SEC_PER_DAY + JD_TAI_EPOCH
}
