use bevy::prelude::*;
use ultraviolet::DVec3;

/// The scaling for transforms.
///
/// This scales all the [`Transform`]s related to bodies so that reasonable
/// clipping planes and continue to be used. It just scales the universe down.
pub struct TransformScale(pub f64);

impl Default for TransformScale {
    fn default() -> Self {
        TransformScale(1e-8)
    }
}

/// Mass in kg.
#[derive(Clone, Copy, PartialEq, Debug, Default, PartialOrd)]
pub struct Mass(pub f64);

/// Barycentric position as specified in meters.
#[derive(Clone, Copy, PartialEq, Debug, Default)]
pub struct Position {
    /// The current position of the body.
    pub current: DVec3,
    /// The previous position of the body.
    pub previous: DVec3,
}

impl Position {
    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        let pos = DVec3::new(x, y, z);

        Position { current: pos, previous: pos }
    }
}

/// Velocity as specified in meters.
#[derive(Clone, Copy, PartialEq, Debug, Default)]
pub struct Velocity(pub DVec3);

/// Linear acceleration as specified in m/s^2.
#[derive(Clone, Copy, PartialEq, Debug, Default)]
pub struct LinAccel(pub DVec3);

/// The new linear acceleration as calculated by the simulation.
#[derive(Clone, Copy, PartialEq, Debug, Default)]
pub struct NewLinAccel(pub DVec3);

/// The timestep information for the simulation.
#[derive(Clone, Copy, PartialEq, Debug)]
pub struct Timestep {
    /// The target sim seconds to elapse every real life second.
    pub current: f64,
    /// The number of substeps to perform per simulation update.
    pub substeps: usize,
    /// The current time taken to render this frame.
    current_frame_time: f64,
    /// Set whether or not the simulation is paused.
    pub paused: bool,
}

impl Default for Timestep {
    fn default() -> Self {
        Self::new(Timestep::DAY_PER_SECOND, true)
    }
}

impl Timestep {
    /// Have the simulation execute one sim second per real life second.
    pub const REALTIME: f64 = 1.0;
    /// Have the simulation execute one sim minute per real life second.
    pub const MINUTE_PER_SECOND: f64 = 60.0 * Timestep::REALTIME;
    /// Have the simulation execute one sim hour per real life second.
    pub const HOUR_PER_SECOND: f64 = 60.0 * Timestep::MINUTE_PER_SECOND;
    /// Have the simulation execute one sim day per real life second.
    pub const DAY_PER_SECOND: f64 = Timestep::HOUR_PER_SECOND * 24.0;

    pub fn new(rate: f64, paused: bool) -> Self {
        Self::new_with_substeps(rate, paused, 1)
    }

    pub fn new_with_substeps(rate: f64, paused: bool, substeps: usize) -> Self {
        assert_ne!(substeps, 0, "must be a positive amount of substeps");

        Timestep {
            current: rate,
            substeps,
            current_frame_time: 0.00001,
            paused,
        }
    }

    #[allow(dead_code)]
    pub const fn real_time(paused: bool) -> Self {
        Timestep {
            current: Self::REALTIME,
            substeps: 5,
            current_frame_time: 0.00001,
            paused,
        }
    }
}

// Some simple, precalculated constants used in the simulation.

/// The speed of light in meters per second.
const SPEED_OF_LIGHT: f64 = 299792458.0;
/// The square of the speed of light.
const SQUARE_SOL: f64 = SPEED_OF_LIGHT * SPEED_OF_LIGHT;
/// The reciprocal of the square of the speed of light.
const SQUARE_SOL_RECIP: f64 = 1.0 / SQUARE_SOL;
/// The gravitational constant.
const GRAV: f64 = 6.6743015e-11;

pub struct GravityPlugin;

impl Plugin for GravityPlugin {
    fn build(&self, app: &mut AppBuilder) {
        app
            .add_resource(Timestep::default())
            .add_resource(TransformScale::default())
            .add_system(eih_integrate_position.system());
    }
}

/// The query type used by the simulation.
type Q<'a, 'b> = Query<'b, (
    Entity,
    &'a mut NewLinAccel,
    &'a Mass,
    &'a mut Position,
    &'a mut Velocity,
    &'a mut LinAccel,
    &'a mut Transform,
)>;

/// Perform a step of the simulation and integrate the positions of bodies.
fn eih_integrate_position(
    mut timestep: ResMut<Timestep>,
    mut time: ResMut<Time>,
    tfs: Res<TransformScale>,
    mut q: Q,
) {
    // just leave if paused
    if timestep.paused {
        return;
    }
    // perform some convenience copies and conversions to mut refs
    let substeps = timestep.substeps;
    let time = &mut *time;
    let timestep = &mut *timestep;
    // the "total" target sim rate; what's passed after all substeps
    let real_rate = timestep.current;
    // set the target sim rate to be divided evenly up by the substeps
    timestep.current /= substeps as f64;
    // perform an initial universe scaling and centering
    center_and_set_positions(tfs.0, &mut q);

    for _ in 0..substeps {
        // calculate the new acceleration due to EIH
        calculate_newaccel_eih(&q);
        // integrate the acceleration
        integrate_accel(&mut q, time, timestep);
        // scale and center the universe
        center_and_set_positions(tfs.0, &mut q);
    }
    // put the timestep back to the "real" rate
    timestep.current = real_rate;
}

/// Calculate the new accleration due to the [Einstein-Infeld-Hoffmann equations].
///
/// [Einstein-Infeld-Hoffmann equations]: https://en.wikipedia.org/wiki/Einstein%E2%80%93Infeld%E2%80%93Hoffmann_equations
fn calculate_newaccel_eih(q: &Q) {
    // SAFE:
    // we only mutate the new acceleration, which isn't mutated by any other
    // query, and all other query are read-only
    for (
        id0,
        mut newaccel,
        _,
        pos0,
        vel0,
        _,
        _,
    ) in unsafe { q.iter_unsafe() } {
        // set up the new sums
        let [mut sum0, mut sum1, mut sum2, mut sum3] = [DVec3::new(0.0, 0.0, 0.0); 4];
        // copy body 0's position and velocity
        let (pos0, vel0) = (pos0.current, vel0.0);
        // SAFE:
        // the query is only read and these components aren't modified
        for (
            id1,
            _,
            &Mass(mass1),
            pos1,
            vel1,
            accel1,
            _,
        ) in unsafe { q.iter_unsafe() } {
            // we skip if body 0 == body 1
            if id0 == id1 {
                continue;
            }
            // we used FMA throughout for increased performance and accuracy
            // copy the position, velocity, and acceleration of body 1
            let (pos1, vel1, accel1) = (pos1.current, vel1.0, accel1.0);
            // save the difference between body positions
            let pos0spos1 = pos0 - pos1;
            // save the *square* of the distance between body 0 and body 1
            let distsq01 = pos0spos1.mag_sq();
            // save the reciprocal of the square of the distance between body 0
            // and body 1
            let distsq01rec = distsq01.recip();
            // save the distance between body 0 and body 1
            let dist01 = distsq01.sqrt();
            // save the reciprocal of the distance between body 0 and body 1
            let dist01rec = dist01.recip();
            // save the normal vector pointing from body 0 to body 1
            let norm01 = pos0spos1 * dist01rec;
            // save the normal vector pointing from body 1 to body 0
            let norm10: DVec3 = -norm01;
            // save GRAV * the mass of body 1
            let grav_mass1 = GRAV * mass1;
            // save (GRAV * the mass of body 1) / (the square of the distance
            // between body 0 and body 1)
            let grm1divdistsq01 = grav_mass1 * distsq01rec;
            // save (GRAV * the mass of body 1) / (the distance between body 0
            // and body 1)
            let grm1divdist01 = grav_mass1 * dist01rec;
            // this term of sum0 is the norm from body 1 to body 0 times
            // `grm1divdistsq01`
            sum0 = norm10.mul_add(DVec3::broadcast(grm1divdistsq01), sum0);
            // see the third summation in the EIH equations
            sum2 = (vel0 - vel1).mul_add(DVec3::broadcast(
                grm1divdistsq01 * norm01.dot(vel0.mul_add(DVec3::broadcast(4.0), -3.0 * vel1))),
                sum2,
            );
            // see the fourth summation in the EIH equations
            sum3 = accel1.mul_add(DVec3::broadcast(grm1divdist01), sum3);
            let mut temp_sum1_0: f64 = 0.0;
            let mut temp_sum1_1: f64 = 0.0;
            // SAFE:
            // the query is only read and these components aren't modified
            for (id2, _, &Mass(mass2), pos2, _, _, _) in unsafe { q.iter_unsafe() } {
                // here we do two sums at once: we sum GRAV * body 2 mass /
                // distance between body 0 and body 2 for the first sum when
                // body 0 =/= body 2, and the same between body 1 and body 2
                // when they're not equal.
                // copy the position of body 2.
                let pos2 = pos2.current;
                // if body 2 =/= body 1
                if id2 != id0 {
                    // add to temp_sum1_0: body 2 mass / the distance between
                    // body 2 and body 0
                    temp_sum1_0 = mass2.mul_add((pos2 - pos0).mag().recip(), temp_sum1_0);
                }

                if id2 != id1 {
                    // add to temp_sum1_1: body 2 mass / the distance between
                    // body 2 and body 1
                    temp_sum1_1 = mass2.mul_add((pos2 - pos1).mag().recip(), temp_sum1_1);
                }
            }
            // see the second sum in the EIH equations
            sum1 = norm10.mul_add(
                DVec3::broadcast(grm1divdistsq01 * (-pos0spos1).dot(accel1).mul_add(
                    0.5,
                    temp_sum1_1.mul_add(-GRAV, temp_sum1_0.mul_add(
                        -4.0 * GRAV,
                        norm01.dot(vel0).powi(2).mul_add(-1.5, vel0.dot(vel1).mul_add(
                            -4.0,
                            vel1.dot(vel1).mul_add(2.0, vel0.dot(vel0)),
                        ))
                    ))
                )),
                sum1,
            );
        }

        // the new acceleration is the sum of sum0, sum1 / c^2, sum2 / c^2,
        // and 3.5 * sum3 / c^2
        newaccel.0 =
            sum0
            + sum1 * SQUARE_SOL_RECIP
            + sum2 * SQUARE_SOL_RECIP
            + sum3 * (3.5 * SQUARE_SOL_RECIP);
    }
}

/// Perform a modified [Verlet integration] over the simulation.
///
/// [Verlet integration]: https://en.wikipedia.org/wiki/Verlet_integration#Non-constant_time_differences
fn integrate_accel(q: &mut Q, time: &Time, timestep: &mut Timestep) {
    // save the old time to render
    let old_time = timestep.current_frame_time;
    // set the current time to render insofar
    timestep.current_frame_time = time.delta_seconds_f64();
    // set sim dt: the irl -> sim rate * the frame time insofar
    let dt = timestep.current * timestep.current_frame_time;
    // a vector of all dt
    let br0 = DVec3::broadcast(dt);
    // a vector of all dt^2
    let br1 = DVec3::broadcast(dt * dt);
    // reciprocal of irl -> sim rate * old time to render
    let br2 = (timestep.current * old_time).recip();

    for (_, newaccel, _, mut pos, mut vel, mut accel, _) in q.iter_mut() {
        // if the body has never been updated before and has either nonzero
        // velocity or nonzero acceleration,
        if pos.current == pos.previous && (vel.0 != DVec3::zero() || accel.0 != DVec3::zero()) {
            // perform a "naive" integration
            vel.0 = accel.0.mul_add(br0, vel.0);
            pos.previous = pos.current;
            pos.current = vel.0.mul_add(br0, pos.current);
        } else {
            // perform verlet integration
            let diff = pos.current - pos.previous;
            vel.0 = diff * br2;
            pos.previous = pos.current;
            pos.current = vel.0.mul_add(br0, accel.0.mul_add(br1, pos.current));
        }
        // update the acceleration
        accel.0 = newaccel.0;
    }
}

/// Set [`Transform`]s and center the universe.
fn center_and_set_positions(tfs: f64, q: &mut Q) {
    // this lets the function complain if it took too many iters to center
    #[cold]
    fn fail() {
        println!("Warning: failed to center and set positions within {} iters", MAX_INIT);
    }
    // declare the NEGATIVE of the delta by which to change all positions
    let mut delta;
    // declare total mass
    let mut total_mass = 0.0;
    // get the most massive body
    let max = q
        .iter_mut()
        .max_by(|l, r| l.2.partial_cmp(&r.2).unwrap_or(std::cmp::Ordering::Equal));

    // if there are any bodies in the universe
    if let Some((_, _, _, base_pos, ..)) = max {
        // set the delta to its position. this way, its position is zeroed.
        delta = base_pos.current;
    } else {
        // just return, there's nothing to do with an empty universe
        return;
    }

    // make all positions relative to the most massive body. this helps the
    // iterative barycentric coordinates converge much faster
    for (_, _, &Mass(mass), mut pos, .., mut tf) in q.iter_mut() {
        pos.previous -= delta;
        pos.current -= delta;
        tf.translation = Vec3::new(
            (pos.current.x * tfs) as f32,
            (pos.current.y * tfs) as f32,
            (pos.current.z * tfs) as f32,
        );
        total_mass += mass;
    }

    // set up the maximum number of iterations for centering the universe by
    // mass
    const MAX_INIT: u32 = 500;
    let mut iters_left = MAX_INIT;
    // while there's still iterations left
    while iters_left > 0 {
        // set delta to the weighted average of all bodies. the averaged values
        // are the current positions while the weights are the masses.
        for (_, _, &Mass(mass), pos, ..) in q.iter_mut() {
            delta = pos.current.mul_add(DVec3::broadcast(mass), delta);
        }

        delta /= total_mass;
        // if delta is significant
        if delta.mag_sq() >= 1e-3 {
            for (_, _, _, mut pos, .., mut tf) in q.iter_mut() {
                pos.previous -= delta;
                pos.current -= delta;
                tf.translation = Vec3::new(
                    (pos.current.x * tfs) as f32,
                    (pos.current.y * tfs) as f32,
                    (pos.current.z * tfs) as f32,
                );
            }

            iters_left -= 1;
        } else {
            // we've converged enough to be finished
            return;
        }
    }
    // if we surpassed the maximum number of iterations, log it and move on
    fail()
}
