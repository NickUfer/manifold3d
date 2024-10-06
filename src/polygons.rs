use crate::manifold::Manifold;
use crate::simple_polygon::SimplePolygon;
use crate::types::{PositiveF64, PositiveI32, Vec2};
use crate::{manifold, simple_polygon};
use manifold_sys::{
    manifold_alloc_manifold, manifold_alloc_polygons, manifold_alloc_simple_polygon,
    manifold_extrude, manifold_polygons, manifold_polygons_get_simple, manifold_polygons_length,
    ManifoldPolygons,
};
use std::os::raw::c_void;

pub struct Polygons(*mut ManifoldPolygons);

pub fn new_from_simple_polygons(simple_polygons: Vec<SimplePolygon>) -> Polygons {
    let simple_polygons_length = simple_polygons.len();
    let mut simple_polygons = simple_polygons
        .into_iter()
        .map(|p| p.ptr())
        .collect::<Vec<_>>();

    let polygons_ptr = unsafe { manifold_alloc_polygons() };
    unsafe {
        manifold_polygons(
            polygons_ptr as *mut c_void,
            simple_polygons.as_mut_ptr(),
            simple_polygons_length,
        )
    };
    Polygons(polygons_ptr)
}

impl Polygons {
    pub fn simple_polygons_count(&self) -> usize {
        unsafe { manifold_polygons_length(self.0) }
    }

    pub fn get_simple_polygon(&self, index: impl Into<PositiveI32>) -> Option<SimplePolygon> {
        let index: i32 = index.into().get();
        if index as i64 >= self.simple_polygons_count() as i64 {
            return None;
        }

        let simple_polygon_ptr = unsafe { manifold_alloc_simple_polygon() };
        unsafe { manifold_polygons_get_simple(simple_polygon_ptr as *mut c_void, self.0, index) };
        Some(simple_polygon::from_ptr(simple_polygon_ptr))
    }

    pub fn extrude(
        &self,
        height: impl Into<PositiveF64>,
        division_count: Option<impl Into<PositiveI32>>,
        twist_degrees: Option<f64>,
        top_scaling: Option<impl Into<Vec2>>,
    ) -> Manifold {
        let height = height.into().get();
        let division_count = division_count.map(|d| d.into().get()).unwrap_or(0);
        let twist_degrees = twist_degrees.unwrap_or(0.0);
        let top_scaling = top_scaling
            .map(|v| v.into())
            .unwrap_or(Vec2 { x: 1.0, y: 1.0 });

        let manifold_ptr = unsafe { manifold_alloc_manifold() };
        unsafe {
            manifold_extrude(
                manifold_ptr as *mut c_void,
                self.0,
                height,
                division_count,
                twist_degrees,
                top_scaling.x,
                top_scaling.y,
            )
        };
        manifold::from_ptr(manifold_ptr)
    }
}
