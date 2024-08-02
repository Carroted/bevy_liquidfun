use bevy::{
    color::palettes::css::{DARK_GRAY, GREEN, MIDNIGHT_BLUE, ORANGE},
    prelude::*,
};

#[derive(Component, Clone, Debug, Reflect)]
#[reflect(Component)]
#[type_path = "bevy_liquidfun"]
pub struct DebugDrawFixtures {
    pub awake_color: Color,
    pub asleep_color: Color,

    pub pivot_scale: f32,
    pub draw_pivot: bool,
    pub vector_scale: f32,
    pub draw_up_vector: bool,
    pub draw_right_vector: bool,
}

impl Default for DebugDrawFixtures {
    fn default() -> Self {
        Self {
            awake_color: GREEN.into(),
            asleep_color: DARK_GRAY.into(),
            pivot_scale: 0.1,
            draw_pivot: false,
            vector_scale: 1.,
            draw_up_vector: false,
            draw_right_vector: false,
        }
    }
}

impl DebugDrawFixtures {
    pub fn splat(color: Color) -> Self {
        Self {
            awake_color: color,
            asleep_color: color,
            ..default()
        }
    }

    pub fn default_static() -> Self {
        Self::splat(MIDNIGHT_BLUE.into())
    }
    pub fn default_dynamic() -> Self {
        Self {
            awake_color: ORANGE.into(),
            draw_up_vector: true,
            draw_right_vector: true,
            ..default()
        }
    }
}

#[derive(Component, Debug)]
pub struct DebugDrawParticleSystem {}
