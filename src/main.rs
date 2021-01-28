use bevy::prelude::*;
use ultraviolet::DVec3;

mod body;
mod fps;
mod scene;
use body::*;
use fps::*;
use scene::*;

#[bevy_main]
fn main() {
    App::build()
        .add_plugins(DefaultPlugins)
        .add_plugin(GravityPlugin)
        .add_plugin(FPSPlugin)
        .add_plugin(GravScenePlugin)
        //.add_resource(Msaa { samples: 4 })
        .add_startup_system(add_bodies.system())
        .add_system(unpause_after_3s.system())
        //.add_system(print_positions.system())
        .run()
}

fn add_bodies(
    commands: &mut Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut time: ResMut<Timestep>,
    scale: ResMut<TransformScale>,
    asset_server: Res<AssetServer>,
) {
    const EARTH_MASS: f64 = 5.972e24;
    const EARTH_RADIUS: f64 = 6_371_009.0;
    const LUNAR_MASS: f64 = 7.3459e22;
    const LUNAR_RADIUS: f64 = 1_737_400.0;
    const LUNAR_APOGEE: f64 = 405_700_000.0;

    asset_server.load::<GravScene, _>("scenes/many.grav");

    *time = Timestep::new_with_substeps(Timestep::DAY_PER_SECOND, false, 500);
    let scale = scale.0;
    commands
        .spawn((
            Mass(EARTH_MASS),
            Position::new(0.0, 0.0, 0.0),
            Velocity::default(),
            LinAccel::default(),
            NewLinAccel::default(),
        ))
        .with_bundle(PbrBundle {
            mesh: meshes.add(Mesh::from(shape::Icosphere { radius: (EARTH_RADIUS * scale) as f32, subdivisions: 30 })),
            material: materials.add(Color::BLUE.into()),
            transform: Transform::from_translation(Vec3::zero()),
            ..Default::default()
        })
        .with_children(|cb| { cb
            .spawn(Camera3dBundle {
                transform: Transform::from_translation(Vec3::new(6.0, 2.0, 6.0))
                    .looking_at(Vec3::zero(), Vec3::unit_y()),
                ..Default::default()
            });
        })
        .spawn((
            Mass(LUNAR_MASS),
            Position::new(LUNAR_APOGEE, 0.0, 0.0),
            Velocity(DVec3::new(0.0, 0.0, 970.0)),
            LinAccel::default(),
            NewLinAccel::default(),
        ))
        .with_bundle(PbrBundle {
            mesh: meshes.add(Mesh::from(shape::Icosphere { radius: (LUNAR_RADIUS * scale) as f32, subdivisions: 30 })),
            material: materials.add(Color::WHITE.into()),
            transform: Transform::from_translation(Vec3::new((LUNAR_APOGEE * scale) as f32, 0.0, 0.0)),
            ..Default::default()
        });
}

fn unpause_after_3s(time: Res<Time>, mut step: ResMut<Timestep>) {
    if step.paused && time.seconds_since_startup() >= 3.0 {
        println!("Unpaused!");
        step.paused = false;
    }
}
