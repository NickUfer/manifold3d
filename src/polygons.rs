use crate::error::{check_error, Error};
use crate::manifold::Manifold;
use crate::simple_polygon::SimplePolygon;
use crate::types::{NormalizedAngle, PositiveF64, PositiveI32, Vec2};
use manifold3d_sys::{
    manifold_alloc_manifold, manifold_alloc_polygons, manifold_alloc_simple_polygon,
    manifold_delete_polygons, manifold_extrude, manifold_polygons, manifold_polygons_get_simple,
    manifold_polygons_length, manifold_revolve, ManifoldPolygons,
};
use std::os::raw::c_void;

pub struct Polygons(*mut ManifoldPolygons);

pub fn new_from_simple_polygons(simple_polygons: Vec<SimplePolygon>) -> Polygons {
    let simple_polygons_length = simple_polygons.len();
    let mut simple_polygons = simple_polygons
        .into_iter()
        .map(|p| p.ptr())
        .collect::<Vec<_>>();

    let polygons_ptr = unsafe {
        manifold_polygons(
            manifold_alloc_polygons() as *mut c_void,
            simple_polygons.as_mut_ptr(),
            simple_polygons_length,
        )
    };
    Polygons(polygons_ptr)
}

impl Polygons {
    pub fn count(&self) -> usize {
        unsafe { manifold_polygons_length(self.0) }
    }

    pub fn get(&self, index: impl Into<PositiveI32>) -> Option<SimplePolygon> {
        let index: i32 = index.into().get();
        if index as i64 >= self.count() as i64 {
            return None;
        }

        let simple_polygon_ptr = unsafe {
            manifold_polygons_get_simple(
                manifold_alloc_simple_polygon() as *mut c_void,
                self.0,
                index,
            )
        };
        Some(SimplePolygon::from_ptr(simple_polygon_ptr))
    }

    pub fn extrude(
        &self,
        height: impl Into<PositiveF64>,
        division_count: impl Into<PositiveI32>,
        twist_degrees: f64,
        top_scaling: Option<impl Into<Vec2>>,
    ) -> Result<Manifold, Error> {
        let height = height.into().get();
        let division_count: i32 = division_count.into().into();
        let top_scaling = top_scaling
            .map(|v| v.into())
            .unwrap_or(Vec2 { x: 1.0, y: 1.0 });

        let manifold_ptr = unsafe { manifold_alloc_manifold() };
        let manifold_ptr = unsafe {
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
        check_error(Manifold::from_ptr(manifold_ptr))
    }

    pub fn revolve(
        &self,
        circular_segments: Option<impl Into<PositiveI32>>,
        revolve_degrees: Option<impl Into<NormalizedAngle>>,
    ) -> Result<Manifold, Error> {
        // 0 segments triggers use of static quality defaults
        let circular_segments = circular_segments.map(|c| c.into().get()).unwrap_or(0);
        let revolve_degrees = revolve_degrees
            .map(|a| a.into().as_degrees())
            .unwrap_or(360.0);
        let manifold_ptr = unsafe {
            manifold_revolve(
                manifold_alloc_manifold() as *mut c_void,
                self.0,
                circular_segments,
                revolve_degrees,
            )
        };
        check_error(Manifold::from_ptr(manifold_ptr))
    }
}

impl Drop for Polygons {
    fn drop(&mut self) {
        unsafe { manifold_delete_polygons(self.0) }
    }
}
