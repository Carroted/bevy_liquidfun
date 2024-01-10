pub mod collision;
pub mod plugins;
pub mod utils;

pub(crate) mod internal;

pub mod dynamics {
    mod body;
    mod joints {
        mod joint;
        pub use joint::*;
        mod revolute_joint;
        pub use revolute_joint::*;

        mod prismatic_joint;
        pub use prismatic_joint::*;

        mod distance_joint;
        pub use distance_joint::*;
    }
    mod fixture;
    mod ray_cast;
    mod world;

    pub use body::*;
    pub use fixture::*;
    pub use joints::*;
    pub use ray_cast::*;
    pub use world::*;
}

pub mod particles {
    mod particle;
    pub use particle::*;
    mod particle_group;
    pub use particle_group::*;
    mod particle_system;
    pub use particle_system::*;
}
#[cfg(test)]
mod tests {}
