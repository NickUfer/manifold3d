#![allow(dead_code)]

mod bounding_box;
mod cross_section;
mod error;
mod fill_rule;
mod manifold_vec;
mod mesh_gl;
mod polygons;
mod quality;
mod simple_polygon;

pub mod manifold;
pub mod types;

pub type HalfEdgeIndex = usize;
pub type Smoothness = f64;

pub use bounding_box::*;
pub use cross_section::*;
pub use error::*;
pub use fill_rule::*;
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
