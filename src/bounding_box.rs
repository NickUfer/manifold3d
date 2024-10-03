use crate::types::{Matrix4x3, Point3, Vec3};
use manifold_sys::{
    manifold_alloc_box, manifold_box, manifold_box_center, manifold_box_contains_box,
    manifold_box_contains_pt, manifold_box_dimensions, manifold_box_does_overlap_box,
    manifold_box_does_overlap_pt, manifold_box_include_pt, manifold_box_is_finite,
    manifold_box_max, manifold_box_min, manifold_box_mul, manifold_box_scale,
    manifold_box_transform, manifold_box_translate, manifold_box_union, manifold_delete_box,
    ManifoldBox,
};

pub struct BoundingBox(*mut ManifoldBox);

pub fn new(min_point: Point3, max_point: Point3) -> BoundingBox {
    let manifold_box_ptr = unsafe { manifold_alloc_box() };
    unsafe {
        manifold_box(
            manifold_box_ptr as *mut std::os::raw::c_void,
            min_point.x,
            min_point.y,
            min_point.z,
            max_point.x,
            max_point.y,
            max_point.z,
        )
    };
    BoundingBox(manifold_box_ptr)
}

impl BoundingBox {
    pub(crate) fn from_ptr(ptr: *mut ManifoldBox) -> BoundingBox {
        BoundingBox(ptr)
    }

    pub fn min_point(&self) -> Point3 {
        unsafe { Point3::from(manifold_box_min(self.0)) }
    }

    pub fn max_point(&self) -> Point3 {
        unsafe { Point3::from(manifold_box_max(self.0)) }
    }

    pub fn dimensions(&self) -> Vec3 {
        unsafe { Vec3::from(manifold_box_dimensions(self.0)) }
    }

    pub fn center(&self) -> Point3 {
        unsafe { Point3::from(manifold_box_center(self.0)) }
    }

    pub fn scale(&self) -> f64 {
        unsafe { manifold_box_scale(self.0) }
    }

    pub fn contains_point(&self, point: impl Into<Point3>) -> bool {
        let point = point.into();
        unsafe { manifold_box_contains_pt(self.0, point.x, point.y, point.z) == 1 }
    }

    pub fn contains_bounding_box(&self, bounding_box: &BoundingBox) -> bool {
        unsafe { manifold_box_contains_box(self.0, bounding_box.0) == 1 }
    }

    pub fn expand_to_include_point(&mut self, point: impl Into<Vec3>) {
        let point = point.into();
        unsafe { manifold_box_include_pt(self.0, point.x, point.y, point.z) }
    }

    pub fn union(&self, other: &Self) -> Self {
        let manifold_box_ptr = unsafe { manifold_alloc_box() };
        unsafe {
            manifold_box_union(
                manifold_box_ptr as *mut std::os::raw::c_void,
                self.0,
                other.0,
            )
        };
        BoundingBox(manifold_box_ptr)
    }

    pub fn transform(&self, matrix: impl Into<Matrix4x3>) -> BoundingBox {
        let matrix = matrix.into();
        let manifold_box_ptr = unsafe { manifold_alloc_box() };
        unsafe {
            manifold_box_transform(
                manifold_box_ptr as *mut std::os::raw::c_void,
                self.0,
                matrix.rows[0].x,
                matrix.rows[0].y,
                matrix.rows[0].z,
                matrix.rows[1].x,
                matrix.rows[1].y,
                matrix.rows[1].z,
                matrix.rows[2].x,
                matrix.rows[2].y,
                matrix.rows[2].z,
                matrix.rows[3].x,
                matrix.rows[3].y,
                matrix.rows[3].z,
            )
        };
        BoundingBox(manifold_box_ptr)
    }

    pub fn translate(&self, translation: impl Into<Vec3>) -> BoundingBox {
        let translation = translation.into();
        let manifold_box_ptr = unsafe { manifold_alloc_box() };
        unsafe {
            manifold_box_translate(
                manifold_box_ptr as *mut std::os::raw::c_void,
                self.0,
                translation.x,
                translation.y,
                translation.z,
            )
        };
        BoundingBox(manifold_box_ptr)
    }

    pub fn multiply(&self, scale_factor: impl Into<Vec3>) -> BoundingBox {
        let scale = scale_factor.into();
        let manifold_box_ptr = unsafe { manifold_alloc_box() };
        unsafe {
            manifold_box_mul(
                manifold_box_ptr as *mut std::os::raw::c_void,
                self.0,
                scale.x,
                scale.y,
                scale.z,
            )
        };
        BoundingBox(manifold_box_ptr)
    }

    pub fn overlaps_point(&self, point: impl Into<Point3>) -> bool {
        let point = point.into();
        let manifold_box_ptr = unsafe { manifold_alloc_box() };
        unsafe { manifold_box_does_overlap_pt(manifold_box_ptr, point.x, point.y, point.z) == 1 }
    }

    pub fn overlaps_bounding_box(&self, other: &BoundingBox) -> bool {
        unsafe { manifold_box_does_overlap_box(self.0, other.0) == 1 }
    }

    pub fn is_finite(&self) -> bool {
        unsafe { manifold_box_is_finite(self.0) == 1 }
    }
}

impl Drop for BoundingBox {
    fn drop(&mut self) {
        unsafe {
            manifold_delete_box(self.0);
        }
    }
}
