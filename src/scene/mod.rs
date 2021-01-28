use bevy::prelude::*;
use bevy::ecs::Stage;
use bevy::app::startup_stage::POST_STARTUP;
use bevy::asset::{LoadContext, AssetLoader, LoadedAsset, Assets};
use bevy::utils::BoxedFuture;

mod units;
mod newcolor;
use newcolor::new_color;
use units::{*, Scene};
use crate::body::TransformScale;
pub use units::Scene as GravScene;

/// Asset loader for `.grav` scenes.
#[derive(Default)]
struct SceneLoader;

impl AssetLoader for SceneLoader {
    fn load<'a>(&'a self, bytes: &'a [u8], load_context: &'a mut LoadContext)
        -> BoxedFuture<'a, Result<(), anyhow::Error>>
    {
        Box::pin(async move {
            load_context.set_default_asset(LoadedAsset::new(
                ron::from_str::<'_, Scene>(std::str::from_utf8(bytes)?)?
            ));
            Ok(())
        })
    }

    fn extensions(&self) -> &[&str] {
        &["grav"]
    }
}

/// Stage to automatically `.grav` scene assets.
struct ImportScenes;

impl Stage for ImportScenes {
    fn initialize(&mut self, world: &mut World, resources: &mut Resources) {
        // get the meshes, for the body spheres
        let mut meshes = resources.get_mut::<Assets<Mesh>>().unwrap();
        // get the materials resources, for colors
        let mut materials = resources.get_mut::<Assets<StandardMaterial>>().unwrap();
        // get the transform scale
        let scale = resources.get::<TransformScale>().unwrap().0;
        // load all scenes from the asset server
        let scenes = resources.get::<Assets<Scene>>().unwrap();
        // an iterator over all decomposed bodies in all scenes
        let iter = scenes
            .iter()
            .flat_map(|(_, scene)| scene.0.iter())
            .map(|b| Body::decompose(b, scale));
        
        for (body_data, (radius, color, transform)) in iter {
            // add a new color material
            let material = materials.add(color.into());
            world
                .build()
                // add the body bundle
                .spawn(body_data)
                // add a mesh bundle for the sphere
                .with_bundle(PbrBundle {
                    mesh: meshes.add(Mesh::from(shape::Icosphere {
                        radius: (radius * scale) as f32,
                        subdivisions: 10,
                    })),
                    material,
                    transform,
                    ..Default::default()
                });
        }
    }

    fn run(&mut self, _world: &mut World, _resources: &mut Resources) {}
}

/// Plugin that loads and imports all `.grav` assets into the universe.
#[derive(Default)]
pub struct GravScenePlugin;

impl Plugin for GravScenePlugin {
    fn build(&self, app: &mut bevy::prelude::AppBuilder) {
        app
            .add_asset::<Scene>()
            .add_asset_loader(SceneLoader)
            .add_startup_stage_before(POST_STARTUP, IMPORT_SCENES_STAGE, ImportScenes);
    }
}

pub const IMPORT_SCENES_STAGE: &str = "import scene stage";
