use crate::bounding_box::BoundingBox;
use crate::error::{check_error, Error};
use crate::mesh_gl::MeshGL;
use crate::types::{PositiveF64, PositiveI32, Vec3};
use manifold3d_sys::{
    manifold_alloc_box, manifold_alloc_manifold, manifold_alloc_meshgl, manifold_bounding_box,
    manifold_copy, manifold_cube, manifold_cylinder, manifold_delete_manifold, manifold_empty,
    manifold_get_meshgl, manifold_is_empty, manifold_num_edge, manifold_num_tri, manifold_num_vert,
    manifold_of_meshgl, manifold_sphere, manifold_tetrahedron, ManifoldManifold,
};
use std::os::raw::{c_int, c_void};

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
    /// use manifold3d::Manifold;
    /// use manifold3d::PositiveF64;
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
    /// use manifold3d::Manifold;
    /// use manifold3d::Vec3;
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
                bottom_radius.into().into(),
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
