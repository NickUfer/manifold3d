#![allow(dead_code)]

mod bounding_box;
mod error;
mod mesh_gl;
mod polygons;
mod quality;
mod simple_polygon;

pub mod manifold;
pub mod types;

pub use bounding_box::*;
pub use error::*;
pub use manifold::Manifold;

#[doc(inline)]
pub use manifold3d_sys as sys;
pub use mesh_gl::*;
pub use polygons::*;
pub use quality::*;
pub use simple_polygon::*;

pub mod macros {
    pub mod manifold {
        #[doc(inline)]
        pub use manifold3d_macros::manifold_manage_vertex_properties as manage_vertex_properties;
        #[doc(inline)]
        pub use manifold3d_macros::manifold_warp as warp;
    }
}
