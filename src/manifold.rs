use crate::bounding_box::BoundingBox;
use crate::meshgl::MeshGL;
use crate::types::{PositiveF64, PositiveI32, Vec3};
use manifold_sys::{
    manifold_alloc_box, manifold_alloc_manifold, manifold_alloc_meshgl, manifold_bounding_box,
    manifold_copy, manifold_cube, manifold_cylinder, manifold_delete_manifold, manifold_empty,
    manifold_get_meshgl, manifold_is_empty, manifold_num_edge, manifold_num_tri, manifold_num_vert,
    manifold_tetrahedron, ManifoldManifold,
};
use std::os::raw::c_int;

pub struct Manifold(*mut ManifoldManifold);

pub fn new_tetrahedron() -> Manifold {
    let manifold_ptr = unsafe { manifold_alloc_manifold() };
    unsafe { manifold_tetrahedron(manifold_ptr as *mut std::os::raw::c_void) };
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
/// use manifold3d::manifold::new_cuboid;
/// use manifold3d::types::PositiveF64;
///
/// // A cuboid of size 1x2x3, touching the origin in the first octant.
/// let cuboid = new_cuboid(1u8, 2u16, 3u32, false);
///
/// // A cube of size 1.5x1.5x1.5 with its center at (0, 0, 0).
/// let cube_edge_length: PositiveF64 = 1.5.try_into().unwrap();
/// let cube = new_cuboid(cube_edge_length, cube_edge_length, cube_edge_length, true);
/// ```
pub fn new_cuboid(
    x_size: impl Into<PositiveF64>,
    y_size: impl Into<PositiveF64>,
    z_size: impl Into<PositiveF64>,
    origin_at_center: bool,
) -> Manifold {
    new_cuboid_unchecked(
        x_size.into(),
        y_size.into(),
        z_size.into(),
        origin_at_center,
    )
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
/// use manifold3d::manifold::new_cuboid_unchecked;
///
/// // A cuboid of size 1x2x3, touching the origin in the first octant.
/// let cuboid = new_cuboid_unchecked(1u8, 2u16, 3u32, false);
///
/// // A cube of size 1.5x1.5x1.5 with its center at the coordinate system's origin.
/// let cube_edge_length = 1.5;
/// let cube = new_cuboid_unchecked(cube_edge_length, cube_edge_length, cube_edge_length, true);
/// ```
pub fn new_cuboid_unchecked(
    x_size: impl Into<f64>,
    y_size: impl Into<f64>,
    z_size: impl Into<f64>,
    origin_at_center: bool,
) -> Manifold {
    let manifold_ptr = unsafe { manifold_alloc_manifold() };
    unsafe {
        manifold_cube(
            manifold_ptr as *mut std::os::raw::c_void,
            x_size.into(),
            y_size.into(),
            z_size.into(),
            c_int::from(origin_at_center),
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
/// use manifold3d::manifold::{new_cuboid_from_vec_unchecked, new_cuboid_unchecked};
/// use manifold3d::types::Vec3;
///
/// // A cuboid of size 1x2x3, touching the origin in the first octant.
/// let cuboid = new_cuboid_from_vec_unchecked(
///     Vec3 {
///         x: 1.0,
///         y: 2.0,
///         z: 3.0,
///     },
///     false,
/// );
///
/// // A cube of size 1.5x1.5x1.5 with its center at (0, 0, 0).
/// let cube_edge_length = 1.5;
/// let cube = new_cuboid_from_vec_unchecked(
///     Vec3 {
///         x: cube_edge_length,
///         y: cube_edge_length,
///         z: cube_edge_length,
///     },
///     true,
/// );
/// ```
pub fn new_cuboid_from_vec_unchecked(size: impl Into<Vec3>, origin_at_center: bool) -> Manifold {
    let size = size.into();
    new_cuboid_unchecked(size.x, size.y, size.z, origin_at_center)
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
    // 0 segments = usage of static quality defaults
    let circular_segments = circular_segments.map_or(0, |c| c.into().get());
    new_cylinder_unchecked(
        height.into(),
        bottom_radius,
        top_radius,
        circular_segments,
        origin_at_center,
    )
}

pub fn new_cylinder_unchecked(
    height: impl Into<f64>,
    bottom_radius: impl Into<f64>,
    top_radius: impl Into<f64>,
    circular_segments: impl Into<i32>,
    origin_at_center: bool,
) -> Manifold {
    let manifold_ptr = unsafe { manifold_alloc_manifold() };
    unsafe {
        manifold_cylinder(
            manifold_ptr as *mut std::os::raw::c_void,
            height.into(),
            bottom_radius.into(),
            top_radius.into(),
            circular_segments.into(),
            c_int::from(origin_at_center),
        )
    };
    Manifold(manifold_ptr)
}

pub fn new_empty() -> Manifold {
    let manifold_ptr = unsafe { manifold_alloc_manifold() };
    unsafe { manifold_empty(manifold_ptr as *mut std::os::raw::c_void) };
    Manifold(manifold_ptr)
}

impl Manifold {
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

    pub fn get_mesh(&self) -> MeshGL {
        let mesh_gl_ptr = unsafe { manifold_alloc_meshgl() };
        unsafe { manifold_get_meshgl(mesh_gl_ptr as *mut std::os::raw::c_void, self.0) };
        MeshGL::from_ptr(mesh_gl_ptr)
    }

    pub fn bounding_box(&self) -> BoundingBox {
        let bounding_box_ptr = unsafe { manifold_alloc_box() };
        unsafe { manifold_bounding_box(bounding_box_ptr as *mut std::os::raw::c_void, self.0) };
        BoundingBox::from_ptr(bounding_box_ptr)
    }
}

impl Clone for Manifold {
    fn clone(&self) -> Self {
        let manifold_ptr = unsafe { manifold_alloc_manifold() };
        unsafe { manifold_copy(manifold_ptr as *mut std::os::raw::c_void, self.0) };
        Manifold(manifold_ptr)
    }
}

impl Drop for Manifold {
    fn drop(&mut self) {
        unsafe {
            manifold_delete_manifold(self.0);
        }
    }
}
