use crate::bounding_box::BoundingBox;
use crate::error::{check_error, Error};
use crate::mesh_gl::MeshGL;
use crate::types::manifold::vertex;
use crate::types::math::{PositiveF64, PositiveI32, Vec3};
use manifold3d_sys::{
    manifold_alloc_box, manifold_alloc_manifold, manifold_alloc_manifold_vec,
    manifold_alloc_meshgl, manifold_batch_boolean, manifold_boolean, manifold_bounding_box,
    manifold_copy, manifold_cube, manifold_cylinder, manifold_delete_manifold, manifold_difference,
    manifold_empty, manifold_get_meshgl, manifold_intersection, manifold_is_empty,
    manifold_manifold_vec, manifold_manifold_vec_set, manifold_mirror, manifold_num_edge,
    manifold_num_tri, manifold_num_vert, manifold_of_meshgl, manifold_refine,
    manifold_refine_to_length, manifold_refine_to_tolerance, manifold_scale,
    manifold_smooth_by_normals, manifold_smooth_out, manifold_sphere, manifold_split,
    manifold_split_by_plane, manifold_tetrahedron, manifold_translate, manifold_trim_by_plane,
    manifold_union, manifold_warp, ManifoldManifold, ManifoldOpType,
};
use manifold3d_types::math::{NonNegativeF64, NonNegativeI32, NormalizedAngle};
use std::os::raw::{c_int, c_void};
use std::pin::Pin;
use thiserror::Error;

pub struct Manifold(*mut ManifoldManifold);

impl Manifold {
    pub fn new_tetrahedron() -> Manifold {
        let manifold_ptr = unsafe { manifold_alloc_manifold() };
        unsafe { manifold_tetrahedron(manifold_ptr as *mut c_void) };
        Manifold(manifold_ptr)
    }

    /// Constructs a 3D cuboid with the specified dimensions in the first octant of 3D space.
    ///
    /// By default, the cuboid's origin will be at the corner touching the coordinate system's origin
    /// (i.e., the point (0, 0, 0)). If [origin_at_center] is set to `true`, the cuboid will be centered
    /// at the origin, with its edges extending equally in all directions.
    ///
    /// # Returns
    /// - A guaranteed non-empty `Manifold` representing a cuboid with the specified dimensions.
    ///
    /// # Examples
    /// ```
    /// use manifold3d::types::math::PositiveF64;
    /// use manifold3d::Manifold;
    ///
    /// // A cuboid of size 1x2x3, touching the origin in the first octant.
    /// let cuboid = Manifold::new_cuboid(1u8, 2u16, 3u32, false);
    ///
    /// // A cube of size 1.5x1.5x1.5 with its center at (0, 0, 0).
    /// let cube_edge_length: PositiveF64 = 1.5.try_into().unwrap();
    /// let cube = Manifold::new_cuboid(cube_edge_length, cube_edge_length, cube_edge_length, true);
    /// ```
    pub fn new_cuboid(
        x_size: impl Into<PositiveF64>,
        y_size: impl Into<PositiveF64>,
        z_size: impl Into<PositiveF64>,
        origin_at_center: bool,
    ) -> Manifold {
        unsafe {
            Self::new_cuboid_unchecked(
                x_size.into(),
                y_size.into(),
                z_size.into(),
                origin_at_center,
            )
        }
    }

    /// Constructs a 3D cuboid with the specified dimensions in the first octant of 3D space.
    ///
    /// By default, the cuboid's origin will be at the corner touching the coordinate system's origin
    /// (i.e., the point (0, 0, 0)). If [origin_at_center] is set to `true`, the cuboid will be centered
    /// at the origin, with its edges extending equally in all directions.
    ///
    /// # Returns
    /// - If any dimension (`x_size`, `y_size`, or `z_size`) is negative, or if all dimensions are zero,
    ///   an empty `Manifold` will be returned.
    /// - Otherwise, a `Manifold` representing a cuboid with the specified dimensions will be created.
    ///
    /// # Examples
    /// ```
    /// use manifold3d::Manifold;
    ///
    /// // A cuboid of size 1x2x3, touching the origin in the first octant.
    /// let cuboid = unsafe { Manifold::new_cuboid_unchecked(1u8, 2u16, 3u32, false) };
    ///
    /// // A cube of size 1.5x1.5x1.5 with its center at the coordinate system's origin.
    /// let cube_edge_length = 1.5;
    /// let cube = unsafe {
    ///     Manifold::new_cuboid_unchecked(cube_edge_length, cube_edge_length, cube_edge_length, true)
    /// };
    /// ```
    pub unsafe fn new_cuboid_unchecked(
        x_size: impl Into<f64>,
        y_size: impl Into<f64>,
        z_size: impl Into<f64>,
        origin_at_center: bool,
    ) -> Manifold {
        let manifold_ptr = unsafe { manifold_alloc_manifold() };
        unsafe {
            manifold_cube(
                manifold_ptr as *mut c_void,
                x_size.into(),
                y_size.into(),
                z_size.into(),
                origin_at_center as c_int,
            )
        };

        Manifold(manifold_ptr)
    }

    /// Constructs a 3D cuboid with the specified dimensions in the first octant of 3D space.
    ///
    /// By default, the cuboid's origin will be at the corner touching the coordinate system's origin
    /// (i.e., the point (0, 0, 0)). If [origin_at_center] is set to `true`, the cuboid will be centered
    /// at the origin, with its edges extending equally in all directions.
    ///
    /// # Returns
    /// - If any dimension (`x_size`, `y_size`, or `z_size`) is negative, or if all dimensions are zero,
    ///   an empty `Manifold` will be returned.
    /// - Otherwise, a `Manifold` representing a cuboid with the specified dimensions will be created.
    ///
    /// # Examples
    /// ```
    /// use manifold3d::types::math::Vec3;
    /// use manifold3d::Manifold;
    ///
    /// // A cuboid of size 1x2x3, touching the origin in the first octant.
    /// let cuboid = unsafe {
    ///     Manifold::new_cuboid_from_vec_unchecked(
    ///         Vec3 {
    ///             x: 1.0,
    ///             y: 2.0,
    ///             z: 3.0,
    ///         },
    ///         false,
    ///     )
    /// };
    ///
    /// // A cube of size 1.5x1.5x1.5 with its center at (0, 0, 0).
    /// let cube_edge_length = 1.5;
    /// let cube = unsafe {
    ///     Manifold::new_cuboid_from_vec_unchecked(
    ///         Vec3 {
    ///             x: cube_edge_length,
    ///             y: cube_edge_length,
    ///             z: cube_edge_length,
    ///         },
    ///         true,
    ///     )
    /// };
    /// ```
    pub unsafe fn new_cuboid_from_vec_unchecked(
        size: impl Into<Vec3>,
        origin_at_center: bool,
    ) -> Manifold {
        let size = size.into();
        Self::new_cuboid_unchecked(size.x, size.y, size.z, origin_at_center)
    }

    pub fn new_cylinder(
        height: impl Into<PositiveF64>,
        bottom_radius: impl Into<PositiveF64>,
        top_radius: Option<impl Into<PositiveF64>>,
        circular_segments: Option<impl Into<PositiveI32>>,
        origin_at_center: bool,
    ) -> Manifold {
        let bottom_radius = bottom_radius.into();
        // Set top radius = bottom radius if none is set
        let top_radius = top_radius.map_or(bottom_radius, |t| t.into());
        // 0 segments triggers use of static quality defaults
        let circular_segments = circular_segments.map_or(0, |c| c.into().get());
        unsafe {
            Self::new_cylinder_unchecked(
                height.into(),
                bottom_radius,
                top_radius,
                circular_segments,
                origin_at_center,
            )
        }
    }

    pub unsafe fn new_cylinder_unchecked(
        height: impl Into<f64>,
        bottom_radius: impl Into<f64>,
        top_radius: impl Into<f64>,
        circular_segments: impl Into<i32>,
        origin_at_center: bool,
    ) -> Manifold {
        let manifold_ptr = unsafe {
            manifold_cylinder(
                manifold_alloc_manifold() as *mut c_void,
                height.into(),
                bottom_radius.into(),
                top_radius.into(),
                circular_segments.into(),
                origin_at_center as c_int,
            )
        };
        check_error(Manifold::from_ptr(manifold_ptr)).unwrap()
    }

    pub fn new_sphere(
        radius: impl Into<PositiveF64>,
        circular_segments: Option<impl Into<PositiveI32>>,
    ) -> Manifold {
        // 0 segments triggers use of static quality defaults
        let circular_segments = circular_segments.map_or(0, |c| c.into().get());
        unsafe { Self::new_sphere_unchecked(radius.into(), circular_segments) }.unwrap()
    }

    pub unsafe fn new_sphere_unchecked(
        radius: impl Into<f64>,
        circular_segments: impl Into<f64>,
    ) -> Result<Manifold, Error> {
        let manifold_ptr = unsafe {
            manifold_sphere(
                manifold_alloc_manifold() as *mut c_void,
                radius.into(),
                circular_segments.into() as c_int,
            )
        };
        check_error(Manifold::from_ptr(manifold_ptr))
    }

    pub fn new_empty() -> Manifold {
        Manifold::from_ptr(unsafe { manifold_empty(manifold_alloc_manifold() as *mut c_void) })
    }

    pub fn from_mesh_gl(mesh_gl: &MeshGL) -> Result<Manifold, Error> {
        Manifold::try_from(mesh_gl)
    }

    pub(crate) fn from_ptr(ptr: *mut ManifoldManifold) -> Manifold {
        Manifold(ptr)
    }

    pub(crate) fn ptr(&self) -> *mut ManifoldManifold {
        self.0
    }

    // Boolean Operations
    pub fn boolean(&self, other: &Manifold, operation: BooleanOperation) -> Manifold {
        let manifold_ptr = unsafe {
            manifold_boolean(
                manifold_alloc_manifold() as *mut c_void,
                self.0,
                other.0,
                operation.into(),
            )
        };
        Manifold::from_ptr(manifold_ptr)
    }

    pub fn batch_boolean(&self, others: &[Manifold], operation: BooleanOperation) -> Manifold {
        if others.is_empty() {
            return self.clone();
        }
        // Check includes self in vec
        if others.len() >= usize::MAX {
            panic!("Batch operation exceeds maximum allowed count of elements")
        }

        let batch_vec_ptr = unsafe {
            manifold_manifold_vec(
                manifold_alloc_manifold_vec() as *mut c_void,
                others.len() + 1,
            )
        };
        let mut batch_vec_index = 0;
        unsafe { manifold_manifold_vec_set(batch_vec_ptr, batch_vec_index, self.0) };
        batch_vec_index += 1;

        for other in others {
            unsafe { manifold_manifold_vec_set(batch_vec_ptr, batch_vec_index, other.0) };
            batch_vec_index += 1;
        }
        let manifold_ptr = unsafe {
            manifold_batch_boolean(
                manifold_alloc_manifold() as *mut c_void,
                batch_vec_ptr,
                operation.into(),
            )
        };
        Manifold::from_ptr(manifold_ptr)
    }

    pub fn union(&self, other: &Manifold) -> Manifold {
        let manifold_ptr =
            unsafe { manifold_union(manifold_alloc_manifold() as *mut c_void, self.0, other.0) };
        Manifold::from_ptr(manifold_ptr)
    }

    pub fn difference(&self, other: &Manifold) -> Manifold {
        let manifold_ptr = unsafe {
            manifold_difference(manifold_alloc_manifold() as *mut c_void, self.0, other.0)
        };
        Manifold::from_ptr(manifold_ptr)
    }

    pub fn intersection(&self, other: &Manifold) -> Manifold {
        let manifold_ptr = unsafe {
            manifold_intersection(manifold_alloc_manifold() as *mut c_void, self.0, other.0)
        };
        Manifold::from_ptr(manifold_ptr)
    }

    pub fn split(&self, other: &Manifold) -> (Manifold, Manifold) {
        let manifold_pair = unsafe {
            manifold_split(
                manifold_alloc_manifold() as *mut c_void,
                manifold_alloc_manifold() as *mut c_void,
                self.0,
                other.0,
            )
        };
        (
            Manifold::from_ptr(manifold_pair.first),
            Manifold::from_ptr(manifold_pair.second),
        )
    }

    pub fn split_by_plane(&self, plane: Plane) -> (Manifold, Manifold) {
        let manifold_pair = unsafe {
            manifold_split_by_plane(
                manifold_alloc_manifold() as *mut c_void,
                manifold_alloc_manifold() as *mut c_void,
                self.0,
                plane.x_normal,
                plane.y_normal,
                plane.z_normal,
                plane.offset,
            )
        };
        (
            Manifold::from_ptr(manifold_pair.first),
            Manifold::from_ptr(manifold_pair.second),
        )
    }

    pub fn trim_by_plane(&self, plane: Plane) -> Manifold {
        let manifold_ptr = unsafe {
            manifold_trim_by_plane(
                manifold_alloc_manifold() as *mut c_void,
                self.0,
                plane.x_normal,
                plane.y_normal,
                plane.z_normal,
                plane.offset,
            )
        };
        Manifold::from_ptr(manifold_ptr)
    }

    // Transformations

    pub fn translate(&self, translation: impl Into<Vec3>) -> Manifold {
        let translation = translation.into();
        let manifold_ptr = unsafe {
            manifold_translate(
                manifold_alloc_manifold() as *mut c_void,
                self.0,
                translation.x,
                translation.y,
                translation.z,
            )
        };
        Manifold::from_ptr(manifold_ptr)
    }

    pub fn rotate(&self, rotation: impl Into<Vec3>) -> Manifold {
        let rotation = rotation.into();
        let manifold_ptr = unsafe {
            manifold_translate(
                manifold_alloc_manifold() as *mut c_void,
                self.0,
                rotation.x,
                rotation.y,
                rotation.z,
            )
        };
        Manifold::from_ptr(manifold_ptr)
    }

    pub fn scale(&self, scale: impl Into<Vec3>) -> Manifold {
        let scale = scale.into();
        let manifold_ptr = unsafe {
            manifold_scale(
                manifold_alloc_manifold() as *mut c_void,
                self.0,
                scale.x,
                scale.y,
                scale.z,
            )
        };
        Manifold::from_ptr(manifold_ptr)
    }

    pub fn mirror(&self, scale: impl Into<Vec3>) -> Manifold {
        let scale = scale.into();
        let manifold_ptr = unsafe {
            manifold_mirror(
                manifold_alloc_manifold() as *mut c_void,
                self.0,
                scale.x,
                scale.y,
                scale.z,
            )
        };
        Manifold::from_ptr(manifold_ptr)
    }

    pub fn warp(&self, warp: Pin<&impl vertex::Warp>) -> Manifold {
        let warp_ptr = &raw const *warp;
        let manifold_ptr = unsafe {
            manifold_warp(
                manifold_alloc_manifold() as *mut c_void,
                self.0,
                Some(warp.extern_c_warp_fn()),
                warp_ptr as *mut c_void,
            )
        };
        let _ = warp;
        Manifold::from_ptr(manifold_ptr)
    }

    pub fn smooth_by_normals(&self, vertex_normal_property_index: NonNegativeI32) -> Manifold {
        let manifold_ptr = unsafe {
            manifold_smooth_by_normals(
                manifold_alloc_manifold() as *mut c_void,
                self.0,
                vertex_normal_property_index.into(),
            )
        };
        Manifold::from_ptr(manifold_ptr)
    }

    pub fn smooth_out(
        &self,
        min_sharp_angle: NormalizedAngle,
        min_smoothness: MinimumSmoothness,
    ) -> Manifold {
        let manifold_ptr = unsafe {
            manifold_smooth_out(
                manifold_alloc_manifold() as *mut c_void,
                self.0,
                min_sharp_angle.into(),
                min_smoothness.into(),
            )
        };
        Manifold::from_ptr(manifold_ptr)
    }

    fn refine_via_edge_splits(&self, edge_split_count: EdgeSplitCount) -> Manifold {
        let manifold_ptr = unsafe {
            manifold_refine(
                manifold_alloc_manifold() as *mut c_void,
                self.0,
                edge_split_count.into(),
            )
        };
        Manifold::from_ptr(manifold_ptr)
    }

    pub fn refine_to_edge_length(&self, edge_length: NonNegativeF64) -> Manifold {
        let manifold_ptr = unsafe {
            manifold_refine_to_length(
                manifold_alloc_manifold() as *mut c_void,
                self.0,
                edge_length.into(),
            )
        };
        Manifold::from_ptr(manifold_ptr)
    }

    pub fn refine_to_tolerance(&self, tolerance: NonNegativeF64) -> Manifold {
        let manifold_ptr = unsafe {
            manifold_refine_to_tolerance(
                manifold_alloc_manifold() as *mut c_void,
                self.0,
                tolerance.into(),
            )
        };
        Manifold::from_ptr(manifold_ptr)
    }

    pub fn is_empty(&self) -> bool {
        unsafe { manifold_is_empty(self.0) == 1 }
    }

    pub fn vertex_count(&self) -> usize {
        unsafe { manifold_num_vert(self.0) }
    }

    pub fn edge_count(&self) -> usize {
        unsafe { manifold_num_edge(self.0) }
    }

    pub fn triangle_count(&self) -> usize {
        unsafe { manifold_num_tri(self.0) }
    }

    pub fn mesh(&self) -> MeshGL {
        let mesh_gl_ptr =
            unsafe { manifold_get_meshgl(manifold_alloc_meshgl() as *mut c_void, self.0) };
        MeshGL::from_ptr(mesh_gl_ptr)
    }

    pub fn bounding_box(&self) -> BoundingBox {
        let bounding_box_ptr =
            unsafe { manifold_bounding_box(manifold_alloc_box() as *mut c_void, self.0) };
        BoundingBox::from_ptr(bounding_box_ptr)
    }
}

impl TryFrom<&'_ MeshGL> for Manifold {
    type Error = Error;

    fn try_from(value: &'_ MeshGL) -> Result<Self, Self::Error> {
        let manifold_ptr =
            unsafe { manifold_of_meshgl(manifold_alloc_manifold() as *mut c_void, value.ptr()) };
        check_error(Manifold::from_ptr(manifold_ptr))
    }
}

impl Clone for Manifold {
    fn clone(&self) -> Self {
        let manifold_ptr =
            unsafe { manifold_copy(manifold_alloc_manifold() as *mut c_void, self.0) };
        Manifold::from_ptr(manifold_ptr)
    }
}

impl Drop for Manifold {
    fn drop(&mut self) {
        unsafe {
            manifold_delete_manifold(self.0);
        }
    }
}

pub enum BooleanOperation {
    Add,
    Subtract,
    Intersect,
}

impl From<BooleanOperation> for ManifoldOpType {
    fn from(val: BooleanOperation) -> Self {
        match val {
            BooleanOperation::Add => manifold3d_sys::ManifoldOpType_MANIFOLD_ADD,
            BooleanOperation::Subtract => manifold3d_sys::ManifoldOpType_MANIFOLD_SUBTRACT,
            BooleanOperation::Intersect => manifold3d_sys::ManifoldOpType_MANIFOLD_INTERSECT,
        }
    }
}

#[derive(Error, Debug)]
pub enum MinimumSmoothnessError {
    #[error(
        "Minimum smoothness value must be between {minimum} and {maximum}. {actual} was provided"
    )]
    OutOfBounds {
        minimum: f64,
        maximum: f64,
        actual: f64,
    },
}

pub struct MinimumSmoothness(f64);

impl MinimumSmoothness {
    const MINIMUM: f64 = 0.0;
    const MAXIMUM: f64 = 1.0;

    pub fn new(smoothness: impl Into<f64>) -> Result<Self, MinimumSmoothnessError> {
        let smoothness = smoothness.into();
        if !(MinimumSmoothness::MINIMUM..=MinimumSmoothness::MAXIMUM).contains(&smoothness) {
            return Err(MinimumSmoothnessError::OutOfBounds {
                minimum: MinimumSmoothness::MINIMUM,
                maximum: MinimumSmoothness::MAXIMUM,
                actual: smoothness,
            });
        }
        Ok(Self(smoothness))
    }

    pub fn get(&self) -> f64 {
        self.0
    }
}

impl From<MinimumSmoothness> for f64 {
    fn from(val: MinimumSmoothness) -> Self {
        val.get()
    }
}

#[derive(Error, Debug)]
pub enum EdgeSplitCountError {
    #[error("Edge split count must be at least {minimum}. {actual} was provided")]
    TooSmall { minimum: i32, actual: i32 },
}

pub struct EdgeSplitCount(PositiveI32);

impl EdgeSplitCount {
    const MINIMUM_SPLIT_COUNT: i32 = 2;

    pub fn new(num: impl Into<PositiveI32>) -> Result<Self, EdgeSplitCountError> {
        let num = num.into();
        if num < EdgeSplitCount::MINIMUM_SPLIT_COUNT {
            return Err(EdgeSplitCountError::TooSmall {
                minimum: EdgeSplitCount::MINIMUM_SPLIT_COUNT,
                actual: num.get(),
            });
        }
        Ok(Self(num))
    }

    pub fn get(&self) -> i32 {
        self.0.get()
    }
}

impl From<EdgeSplitCount> for i32 {
    fn from(val: EdgeSplitCount) -> Self {
        val.get()
    }
}

pub struct Plane {
    pub x_normal: f64,
    pub y_normal: f64,
    pub z_normal: f64,
    pub offset: f64,
}

impl Plane {
    #[must_use]
    pub fn new(x_normal: f64, y_normal: f64, z_normal: f64, offset: f64) -> Self {
        Self {
            x_normal,
            y_normal,
            z_normal,
            offset,
        }
    }
}
