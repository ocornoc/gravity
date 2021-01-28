use bevy::prelude::{Transform, Color, Vec3};
use bevy::reflect::TypeUuid;
use serde::{Deserialize, Serialize};
use ultraviolet::DVec3;
use crate::body;

#[allow(non_camel_case_types)]
#[derive(Serialize, Deserialize, Clone, Copy)]
pub enum Distance {
    mm(f64),
    cm(f64),
    r#in(f64),
    inches(f64),
    ft(f64),
    feet(f64),
    yd(f64),
    yards(f64),
    m(f64),
    meters(f64),
    km(f64),
    miles(f64),
    lunar_radii(f64),
    earth_radii(f64),
    jupiter_radii(f64),
    jovian_radii(f64),
    light_seconds(f64),
    ld(f64),
    lunar_distances(f64),
    solar_radii(f64),
    light_minutes(f64),
    au(f64),
    light_hours(f64),
    light_days(f64),
    light_years(f64),
    pc(f64),
    parsecs(f64),
}

impl Distance {
    pub fn to_meters(self) -> f64 {
        use Distance::*;

        match self {
            mm(x) => x * 0.001,
            cm(x) => x * 0.01,
            r#in(x) | inches(x) => x * 0.0254,
            ft(x) | feet(x) => x * 0.3048,
            yd(x) | yards(x) => x * 0.9144,
            m(x) | meters(x) => x,
            km(x) => x * 1000.0,
            miles(x) => x * 1609.344,
            lunar_radii(x) => x * 1_737_400.0,
            earth_radii(x) => x * 6.371e6,
            jupiter_radii(x) | jovian_radii(x) => x * 7.1492e7,
            light_seconds(x) => x * 2.99792458e8,
            ld(x) | lunar_distances(x) => x * 397126e3,
            solar_radii(x) => x * 6.957e8,
            light_minutes(x) => x * 1.798754748e10,
            au(x) => x * 1.495978707e11,
            light_hours(x) => x * 1.0792528488e12,
            light_days(x) => x * 2.59020683712e13,
            light_years(x) => x * 9.4607304725808e15,
            pc(x) | parsecs(x) => x * 3.085677581491367278913937957796472e16,
        }
    }
}

#[allow(non_camel_case_types)]
#[derive(Serialize, Deserialize, Clone, Copy)]
pub enum Time {
    ms(f64),
    s(f64),
    seconds(f64),
    min(f64),
    minutes(f64),
    hr(f64),
    hours(f64),
    days(f64),
    weeks(f64),
    years(f64),
}

impl Default for Time {
    fn default() -> Self {
        Time::seconds(1.0)
    }
}

impl Time {
    pub fn to_seconds(self) -> f64 {
        use Time::*;

        match self {
            ms(x) => x * 0.001,
            s(x) | seconds(x) => x,
            min(x) | minutes(x) => x * 60.0,
            hr(x) | hours(x) => x * 360.0,
            days(x) => x * 86400.0,
            weeks(x) => x * 604800.0,
            years(x) => x * 3.154e7,
        }
    }
}

#[allow(non_camel_case_types)]
#[derive(Serialize, Deserialize, Clone, Copy)]
pub enum Mass {
    g(f64),
    grams(f64),
    oz(f64),
    ounces(f64),
    lb(f64),
    pounds(f64),
    kg(f64),
    kilograms(f64),
    t(f64),
    ton(f64),
    tons(f64),
    metric_tons(f64),
    lunar_masses(f64),
    earth_masses(f64),
    jupiter_masses(f64),
    jovian_masses(f64),
    solar_masses(f64),
}

impl Mass {
    pub fn to_kg(self) -> f64 {
        use Mass::*;

        match self {
            g(x) | grams(x) => x * 0.001,
            oz(x) | ounces(x) => x * 0.0283495,
            lb(x) | pounds(x) => x * 0.453592,
            kg(x) | kilograms(x) => x,
            t(x) | ton(x) | tons(x) | metric_tons(x) => x * 1000.0,
            lunar_masses(x) => x * 7.342e22,
            earth_masses(x) => x *  5.9722e24,
            jupiter_masses(x) | jovian_masses(x) => x * 1.89813e27,
            solar_masses(x) => x * 1.98847e30,
        }
    }
}

#[derive(Serialize, Deserialize, Clone, Copy)]
pub struct Position {
    pub x: Distance,
    pub y: Distance,
    pub z: Distance,
}

impl Default for Position {
    fn default() -> Self {
        Position {
            x: Distance::m(0.0),
            y: Distance::m(0.0),
            z: Distance::m(0.0),
        }
    }
}

impl Position {
    pub fn to_meters(self) -> DVec3 {
        DVec3::new(self.x.to_meters(), self.y.to_meters(), self.z.to_meters())
    }

    const fn nonzero() -> Position {
        Position {
            x: Distance::mm(1.0),
            y: Distance::mm(1.0),
            z: Distance::mm(1.0),
        }
    }
}

#[derive(Serialize, Deserialize, Clone, Copy)]
pub struct Velocity {
    pub change: Position,
    #[serde(default)]
    pub per: Time,
}

impl Default for Velocity {
    fn default() -> Self {
        Velocity {
            change: Position::nonzero(),
            per: Default::default(),
        }
    }
}

impl Velocity {
    pub fn to_mps(self) -> DVec3 {
        self.change.to_meters() / self.per.to_seconds()
    }
}

#[derive(Serialize, Deserialize, Clone, Copy)]
pub struct Body {
    #[serde(default)]
    pub position: Position,
    pub mass: Mass,
    #[serde(default)]
    pub velocity: Velocity,
    pub radius: Distance,
    #[serde(default = "super::new_color")]
    pub color: [f32; 3],
}

type Decomposition = (
    body::Mass,
    body::LinAccel,
    body::NewLinAccel,
    body::Position,
    body::Velocity,
);

impl Body {
    pub fn decompose(&self, tfs: f64) -> (Decomposition, (f64, Color, Transform)) {
        let position = self.position.to_meters();

        ((
            body::Mass(self.mass.to_kg()),
            body::LinAccel(DVec3::zero()),
            body::NewLinAccel(DVec3::zero()),
            body::Position {
                current: position,
                previous: position,
            },
            body::Velocity(self.velocity.to_mps()),
        ), (
            self.radius.to_meters(),
            Color::rgb(self.color[0], self.color[1], self.color[2]),
            Transform::from_translation(Vec3::new(
                (position.x * tfs) as f32,
                (position.y * tfs) as f32,
                (position.z * tfs) as f32,
            ))
        ))
    }
}

#[derive(Serialize, Deserialize, TypeUuid)]
#[uuid = "99652da1-11d6-44fe-94cb-91b4271b5cc8"]
pub struct Scene(pub Vec<Body>);
