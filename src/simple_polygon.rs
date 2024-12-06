use crate::types::Point2;
use manifold3d_sys::{
    manifold_alloc_simple_polygon, manifold_delete_simple_polygon, manifold_simple_polygon,
    manifold_simple_polygon_get_point, manifold_simple_polygon_length, ManifoldSimplePolygon,
    ManifoldVec2,
};
use std::os::raw::c_void;

pub struct SimplePolygon(*mut ManifoldSimplePolygon);

impl SimplePolygon {
    pub fn new_from_points(points: Vec<impl Into<ManifoldVec2>>) -> SimplePolygon {
        let mut points = points.into_iter().map(|x| x.into()).collect::<Vec<_>>();
        let points_length = points.len();

        let polygon_ptr = unsafe { manifold_alloc_simple_polygon() };
        unsafe {
            manifold_simple_polygon(
                polygon_ptr as *mut c_void,
                points.as_mut_ptr(),
                points_length,
            )
        };
        SimplePolygon(polygon_ptr)
    }

    pub(crate) fn from_ptr(ptr: *mut ManifoldSimplePolygon) -> SimplePolygon {
        SimplePolygon(ptr)
    }

    pub fn point_count(&self) -> usize {
        unsafe { manifold_simple_polygon_length(self.0) }
    }

    pub fn get_point(&self, index: usize) -> Option<Point2> {
        if index >= self.point_count() {
            return None;
        }
        let vec = unsafe { manifold_simple_polygon_get_point(self.0, index) };
        Some(Point2::from(vec))
    }

    pub(crate) fn ptr(&self) -> *mut ManifoldSimplePolygon {
        self.0
    }
}

impl Drop for SimplePolygon {
    fn drop(&mut self) {
        unsafe { manifold_delete_simple_polygon(self.0) }
    }
}
