use crate::bounding_box::BoundingBox;
use crate::error::{check_error, Error};
use crate::mesh_gl::MeshGL;
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
use std::os::raw::{c_int, c_void};
use std::pin::Pin;
use thiserror::Error;

use crate::types::{
    NonNegativeF64, NonNegativeI32, NormalizedAngle, PositiveF64, PositiveI32, Vec3,
};

pub use warp::*;

/// Represents a manifold.
pub struct Manifold(*mut ManifoldManifold);

impl Manifold {
    /// Creates a new tetrahedron manifold.
    ///
    /// # Returns
    /// A [Manifold] representing a tetrahedron.
    pub fn new_tetrahedron() -> Manifold {
        let manifold_ptr = unsafe { manifold_alloc_manifold() };
        unsafe { manifold_tetrahedron(manifold_ptr as *mut c_void) };
        Manifold(manifold_ptr)
    }

    /// Constructs a 3D cuboid with the specified dimensions in the first octant of 3D space.
    ///
    /// By default, the cuboid's origin will be at the corner touching the coordinate system's origin
    /// (i.e., the point (0, 0, 0)). If `origin_at_center` is set to `true`, the cuboid will be centered
    /// at the origin, with its edges extending equally in all directions.
    ///
    /// # Returns
    /// - A guaranteed non-empty `Manifold` representing a cuboid with the specified dimensions.
    ///
    /// # Examples
    /// ```
    /// use manifold3d::types::PositiveF64;
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
    /// (i.e., the point (0, 0, 0)). If `origin_at_center` is set to `true`, the cuboid will be centered
    /// at the origin, with its edges extending equally in all directions.
    ///
    /// # Returns
    /// - If any dimension (`x_size`, `y_size`, or `z_size`) is negative, or if all dimensions are zero,
    ///   an empty `Manifold` will be returned.
    /// - Otherwise, a `Manifold` representing a cuboid with the specified dimensions will be created.
    ///
    /// # Safety
    /// This function is unsafe because it does not check if the input is valid.
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
    /// (i.e., the point (0, 0, 0)). If `origin_at_center` is set to `true`, the cuboid will be centered
    /// at the origin, with its edges extending equally in all directions.
    ///
    /// # Returns
    /// - If any dimension (`x_size`, `y_size`, or `z_size`) is negative, or if all dimensions are zero,
    ///   an empty `Manifold` will be returned.
    /// - Otherwise, a `Manifold` representing a cuboid with the specified dimensions will be created.
    ///
    /// # Safety
    /// This function is unsafe because it does not check if the input is valid.
    ///
    /// # Examples
    /// ```
    /// use manifold3d::types::Vec3;
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

    /// Creates a new cylinder with the specified attributes.
    ///
    /// # Arguments
    /// * `height`: The height of the cylinder. Must be a value that can be converted to [PositiveF64].
    /// * `bottom_radius`: The radius at the bottom of the cylinder. Must be a value that can be converted to [PositiveF64].
    /// * `top_radius`: An optional radius at the top of the cylinder. If not provided, it defaults to the bottom radius.
    /// * `circular_segments`: An optional number of circular segments used to approximate the shape. If not provided, the default global quality setting is used.
    /// * `origin_at_center`: A boolean indicating whether the origin (0, 0, 0) should be at the center of the cylinder.
    ///
    /// # Returns
    /// A new manifold representing a cylinder.
    ///
    /// # Examples
    /// ```rust
    /// use manifold3d::types::{PositiveF64, PositiveI32};
    /// use manifold3d::Manifold;
    ///
    /// let height = PositiveF64::new(10.0).unwrap();
    /// let bottom_radius = PositiveF64::new(5.0).unwrap();
    /// let top_radius = PositiveF64::new(3.0).unwrap();
    /// let circular_segments = PositiveI32::new(32).unwrap();
    ///
    /// let cylinder = Manifold::new_cylinder(
    ///     height,
    ///     bottom_radius,
    ///     Some(top_radius),
    ///     Some(circular_segments),
    ///     true,
    /// );
    /// ```
    ///
    /// To create a cylinder with the same top and bottom radius:
    ///
    /// ```rust
    /// use manifold3d::types::{PositiveF64, PositiveI32};
    /// use manifold3d::Manifold;
    ///
    /// let height = PositiveF64::new(10.0).unwrap();
    /// let bottom_radius = PositiveF64::new(5.0).unwrap();
    /// let top_radius = PositiveF64::new(3.0).unwrap();
    /// let circular_segments = 32u16;
    ///
    /// let cylinder = Manifold::new_cylinder(
    ///     height,
    ///     bottom_radius,
    ///     None::<PositiveF64>,
    ///     Some(circular_segments),
    ///     false,
    /// );
    /// ```
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

    /// Creates a new cylinder manifold with the given height, bottom radius, top radius, number of circular segments, and origin.
    ///
    /// # Arguments
    /// * `height`: The height of the cylinder.
    /// * `bottom_radius`: The radius of the bottom circle of the cylinder.
    /// * `top_radius`: The radius of the top circle of the cylinder.
    /// * `circular_segments`: The number of circular segments to use when constructing the cylinder.
    /// * `origin_at_center`: If true, the origin of the cylinder will be at its center. Otherwise, the origin will be at the bottom center.
    ///
    /// # Returns
    /// A new manifold representing the cylinder.
    ///
    /// # Safety
    /// This function is unsafe because it does not check if the input is valid.
    ///
    /// # Examples
    /// ```
    /// use manifold3d::Manifold;
    ///
    /// let cylinder = unsafe {
    ///     Manifold::new_cylinder_unchecked(1.0, 0.5, 0.5, 32, true)
    /// };
    /// ```
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

    /// Creates a new sphere manifold with the given radius and number of circular segments.
    ///
    /// # Arguments
    /// * `radius`: The radius of the sphere.
    /// * `circular_segments`: The number of circular segments used to approximate the sphere.
    ///
    /// # Returns
    /// A [Result] containing the new manifold if successful, and an [Error] if not.
    ///
    /// # Safety
    /// This function is unsafe because it does not check if the input is valid.
    ///
    /// # Examples
    /// ```
    /// use manifold3d::Manifold;
    ///
    /// let sphere = unsafe { Manifold::new_sphere_unchecked(1.0, 30) }.unwrap();
    /// ```
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
        if others.len() == usize::MAX {
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

    pub fn split_by_offset_plane(&self, offset_plane: OffsetPlane) -> (Manifold, Manifold) {
        let manifold_pair = unsafe {
            manifold_split_by_plane(
                manifold_alloc_manifold() as *mut c_void,
                manifold_alloc_manifold() as *mut c_void,
                self.0,
                offset_plane.plane.normal.x,
                offset_plane.plane.normal.y,
                offset_plane.plane.normal.z,
                offset_plane.offset,
            )
        };
        (
            Manifold::from_ptr(manifold_pair.first),
            Manifold::from_ptr(manifold_pair.second),
        )
    }

    pub fn trim_by_offset_plane(&self, offset_plane: OffsetPlane) -> Manifold {
        let manifold_ptr = unsafe {
            manifold_trim_by_plane(
                manifold_alloc_manifold() as *mut c_void,
                self.0,
                offset_plane.plane.normal.x,
                offset_plane.plane.normal.y,
                offset_plane.plane.normal.z,
                offset_plane.offset,
            )
        };
        Manifold::from_ptr(manifold_ptr)
    }

    // Transformations

    /// Moves the manifold in space. This operation can be chained. Transforms are
    /// combined and applied lazily.
    ///
    /// # Arguments
    /// * `translation`: The vector to add to every vertex.
    ///
    /// # Returns
    /// [Manifold]: The translated manifold.
    ///
    /// # Examples
    /// ```
    /// use manifold3d::types::Vec3;
    /// use manifold3d::Manifold;
    ///
    /// let manifold = Manifold::new_cuboid(1u8, 1u8, 1u8, true);
    /// let translated_manifold = manifold.translate(Vec3::new(1.0, 2.0, 3.0));
    /// ```
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

    /// Rotates the manifold using the given rotation vector.
    ///
    /// # Arguments
    /// * `rotation`: The rotation vector.
    ///
    /// # Returns
    /// A new rotated manifold object.
    ///
    /// # Examples
    /// ```
    /// use manifold3d::types::Vec3;
    /// use manifold3d::Manifold;
    ///
    /// let manifold = Manifold::new_cuboid(1u8, 1u8, 1u8, true);
    /// let rotated_manifold = manifold.rotate(Vec3::new(0.0, 45.0, 45.0));
    /// ```
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

    /// Scales the manifold in space. This operation can be chained. Transforms are
    /// combined and applied lazily.
    ///
    /// # Arguments
    /// * `scale`: The vector to multiply every vertex by per component.
    ///
    /// # Returns
    /// A new scaled manifold object.
    ///
    /// # Examples
    /// ```
    /// use manifold3d::types::Vec3;
    /// use manifold3d::Manifold;
    ///
    /// let manifold = Manifold::new_cuboid(1u8, 1u8, 1u8, true);
    /// let scaled_manifold = manifold.scale(Vec3::new(2.0, 2.0, 2.0));
    /// ```
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

    /// Mirrors the manifold over the plane described by the given [Plane].
    ///
    /// # Arguments
    /// * `plane`: The plane to be mirrored over.
    ///
    /// # Returns
    /// A new manifold object that is the result of mirroring this manifold over the given plane.
    ///
    /// # Examples
    /// ```
    /// use manifold3d::manifold::Plane;
    /// use manifold3d::types::Vec3;
    /// use manifold3d::Manifold;
    ///
    /// let manifold = Manifold::new_cuboid(1u8, 1u8, 1u8, true);
    /// let plane = Plane::new(Vec3::new(1.0, 0.0, 0.0));
    /// let mirrored_manifold = manifold.mirror(plane);
    /// ```
    pub fn mirror(&self, plane: Plane) -> Manifold {
        let manifold_ptr = unsafe {
            manifold_mirror(
                manifold_alloc_manifold() as *mut c_void,
                self.0,
                plane.normal.x,
                plane.normal.y,
                plane.normal.z,
            )
        };
        Manifold::from_ptr(manifold_ptr)
    }

    /// Warps the manifold by applying a transformation function to each vertex.
    ///
    /// The topology of the manifold remains unchanged.
    ///
    /// # Arguments
    /// * `warp`: A pinned reference to a type implementing the [Warp] trait.
    ///   This warp object defines the transformation logic.
    ///
    /// # Returns
    /// A new manifold object representing the warped manifold.
    ///
    /// # Examples
    /// ```
    /// use manifold3d::macros::manifold_warp;
    /// use manifold3d::manifold::WarpVertex;
    /// use manifold3d::types::Point3;
    /// use manifold3d::Manifold;
    /// use std::pin::Pin;
    ///
    /// // Users are advised to use the manifold_warp macro to automatically implement
    /// // Warp and WarpExternCFn and only implement the WarpVertex trait themselves
    /// #[manifold_warp]
    /// struct MyWarp;
    ///
    /// impl WarpVertex for MyWarp {
    ///     fn warp_vertex(&self, vertex: Point3) -> Point3 {
    ///         // Example: Translate the vertex by (1.0, 2.0, 3.0)
    ///         Point3::new(vertex.x + 1.0, vertex.y + 2.0, vertex.z + 3.0)
    ///     }
    /// }
    ///
    /// let manifold = Manifold::new_tetrahedron();
    /// let warp = MyWarp;
    /// let warped_manifold = manifold.warp(Pin::new(&warp));
    /// ```
    pub fn warp(&self, warp: Pin<&impl Warp>) -> Manifold {
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

    /// Smooths the manifold by filling in the halfedge tangent vectors.
    ///
    /// The geometry remains unchanged until [Manifold#refine_via_edge_splits](Manifold#method.refine_via_edge_splits)
    /// or [Manifold#refine_to_tolerance](Manifold#method.refine_to_tolerance) is called to interpolate the surface.
    ///
    /// This function uses the vertex normal properties of the manifold to define the tangent vectors.
    ///
    /// Faces of two coplanar triangles will be marked as quads, while faces with three or more coplanar triangles will be flat.
    ///
    /// # Parameters
    /// * `vertex_normal_property_index`: The first property channel of the normals.
    ///   Any vertex where multiple normals exist and don't agree will result in a sharp edge.
    ///
    /// # Returns
    /// A new manifold object with filled halfedge tangent vectors.
    ///
    /// # Examples
    ///
    /// ```
    /// use manifold3d::types::NonNegativeI32;
    /// use manifold3d::Manifold;
    ///
    /// let manifold = Manifold::new_tetrahedron();
    /// // TODO check if normals are always present
    ///
    /// // In this example the first property of the normal vertex is in the 4th channel.
    /// // 1-3th channel = position xyz
    /// // 4-6th channel = normal xyz
    /// let vertex_normal_property_index = NonNegativeI32::new(4).unwrap();
    ///
    /// let smoothed_manifold = manifold.smooth_by_normals(vertex_normal_property_index);
    /// ```
    pub fn smooth_by_normals(
        &self,
        vertex_normal_property_index: impl Into<NonNegativeI32>,
    ) -> Manifold {
        let manifold_ptr = unsafe {
            manifold_smooth_by_normals(
                manifold_alloc_manifold() as *mut c_void,
                self.0,
                vertex_normal_property_index.into().into(),
            )
        };
        Manifold::from_ptr(manifold_ptr)
    }

    /// Smooths the manifold by filling in the halfedge tangent vectors.
    ///
    /// The geometry remains unchanged until [Manifold#refine_via_edge_splits](Manifold#method.refine_via_edge_splits)
    /// or [Manifold#refine_to_tolerance](Manifold#method.refine_to_tolerance) is called to interpolate the surface.
    /// This function uses the geometry of the triangles and pseudo-normals to define the tangent vectors.
    /// Faces of two coplanar triangles will be marked as quads.
    ///
    /// # Arguments
    /// * `minimum_sharpness_angle`: Angle in degrees. Any edges with angles greater than this value will remain sharp.
    ///   The rest will be smoothed to G1 continuity, with the caveat that flat faces of three or more triangles will always remain flat.
    ///   With a value of zero, the model is faceted.
    /// * `minimum_smoothness`: The smoothness applied to sharp angles. The default gives a hard edge,
    ///   while values > 0 will give a small fillet on these sharp edges.
    ///   A value of 1 is equivalent to a `minimum_sharpness_angle` of 180 - all edges will be smooth.
    ///
    /// # Returns
    /// A new manifold object with filled halfedge tangent vectors.
    ///
    /// # Examples
    /// ```
    /// use manifold3d::manifold::MinimumSmoothness;
    /// use manifold3d::types::NormalizedAngle;
    /// use manifold3d::Manifold;
    ///
    /// let manifold = Manifold::new_cuboid(1u8, 1u8, 1u8, true);
    ///
    /// let angle = NormalizedAngle::from_degrees(60.0);
    /// let minimum_smoothness = MinimumSmoothness::new(MinimumSmoothness::MINIMUM).unwrap();
    /// let smoothed_manifold = manifold.smooth_out(angle, minimum_smoothness);
    /// ```
    pub fn smooth_out(
        &self,
        minimum_sharpness_angle: NormalizedAngle,
        minimum_smoothness: MinimumSmoothness,
    ) -> Manifold {
        let manifold_ptr = unsafe {
            manifold_smooth_out(
                manifold_alloc_manifold() as *mut c_void,
                self.0,
                minimum_sharpness_angle.into(),
                minimum_smoothness.into(),
            )
        };
        Manifold::from_ptr(manifold_ptr)
    }

    /// Increases the density of the mesh by splitting every edge into n pieces.
    ///
    /// For instance, with n = 2, each triangle will be split into 4 triangles. Quads
    /// will ignore their interior triangle bisector.
    ///
    /// These will all be coplanar (and will not be immediately collapsed), unless the
    /// [MeshGL]/[Manifold] has halfedge tangents specified (e.g. from [Manifold#smooth_out](Manifold#method.smooth_out))
    /// in which case the new vertices will be moved to the interpolated surface according to
    /// their barycentric coordinates.
    ///
    /// # Arguments
    /// * `edge_split_count`: The number of pieces to split every edge into.
    ///
    /// # Returns
    /// A new manifold object with increased density.
    ///
    /// # Examples
    /// ```
    /// use manifold3d::manifold::EdgeSplitCount;
    /// use manifold3d::types::PositiveI32;
    /// use manifold3d::Manifold;
    ///
    /// let manifold = Manifold::new_tetrahedron();
    ///
    /// let edge_split_count = EdgeSplitCount::new(PositiveI32::new(2).unwrap()).unwrap();
    /// let refined_manifold = manifold.refine_via_edge_splits(edge_split_count);
    /// ```
    pub fn refine_via_edge_splits(&self, edge_split_count: EdgeSplitCount) -> Manifold {
        let manifold_ptr = unsafe {
            manifold_refine(
                manifold_alloc_manifold() as *mut c_void,
                self.0,
                edge_split_count.into(),
            )
        };
        Manifold::from_ptr(manifold_ptr)
    }

    /// Increase the density of the mesh by splitting each edge into pieces of
    /// roughly the input length.
    ///
    /// Interior verts are added to keep the rest of the triangulation edges also of roughly the same length.
    ///
    /// If halfedge tangents are present (e.g. from the [Manifold#smooth_out](Manifold#method.smooth_out)), the new
    /// vertices will be moved to the interpolated surface according to their barycentric coordinates.
    /// Quads will ignore their interior triangle bisector.
    ///
    /// # Arguments
    /// * `edge_length`: The length that edges will be broken down to.
    ///
    /// # Returns
    /// A new manifold object with increased density.
    ///
    /// # Examples
    /// ```
    /// use manifold3d::types::{NonNegativeF64, PositiveF64};
    /// use manifold3d::Manifold;
    ///
    /// // Create a manifold
    /// let manifold = Manifold::new_cuboid(
    ///     PositiveF64::new(1.0).unwrap(),
    ///     PositiveF64::new(1.0).unwrap(),
    ///     PositiveF64::new(1.0).unwrap(),
    ///     true,
    /// );
    ///
    /// // Refine the manifold to an edge length of 0.5
    /// let refined_manifold = manifold.refine_to_edge_length(NonNegativeF64::new(0.5).unwrap());
    /// ```
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

    /// Increases the density of the mesh by splitting each edge into pieces such that
    /// any point on the resulting triangles is roughly within tolerance of the
    /// smoothly curved surface defined by the tangent vectors.
    ///
    /// This means tightly curving regions will be divided more finely than smoother regions. If
    /// halfedgeTangents are not present, the result will simply be a copy of the
    /// original. Quads will ignore their interior triangle bisector.
    ///
    /// # Arguments
    /// * `tolerance`: The desired maximum distance between the faceted mesh
    ///   produced and the exact smoothly curving surface. All vertices are exactly on
    ///   the surface, within rounding error.
    ///
    /// # Returns
    /// A new manifold object with increased density.
    ///
    /// # Examples
    /// ```
    /// use manifold3d::types::NonNegativeF64;
    /// use manifold3d::Manifold;
    ///
    /// // Create a manifold
    /// let manifold = Manifold::new_tetrahedron();
    ///
    /// // Refine the manifold to a tolerance of 0.1
    /// let refined_manifold = manifold.refine_to_tolerance(NonNegativeF64::new(0.1).unwrap());
    /// ```
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

    /// Returns `true` if the manifold is empty.
    ///
    /// # Examples
    ///
    /// ```
    /// use manifold3d::Manifold;
    ///
    /// let empty_manifold = Manifold::new_empty();
    /// assert!(empty_manifold.is_empty());
    ///
    /// let tetrahedron_manifold = Manifold::new_tetrahedron();
    /// assert!(!tetrahedron_manifold.is_empty());
    /// ```
    pub fn is_empty(&self) -> bool {
        unsafe { manifold_is_empty(self.0) == 1 }
    }

    /// Returns the number of vertices in the manifold.
    ///
    /// # Examples
    ///
    /// ```
    /// use manifold3d::Manifold;
    ///
    /// let manifold = Manifold::new_cuboid(1u8, 1u8, 1u8, true);
    /// assert_eq!(manifold.vertex_count(), 8);
    /// ```
    pub fn vertex_count(&self) -> usize {
        unsafe { manifold_num_vert(self.0) }
    }

    /// Returns the number of edges in the manifold.
    ///
    /// # Examples
    ///
    /// ```
    /// use manifold3d::Manifold;
    ///
    /// let manifold = Manifold::new_cuboid(1u8, 1u8, 1u8, true);
    /// assert_eq!(manifold.edge_count(), 18);
    /// ```
    pub fn edge_count(&self) -> usize {
        unsafe { manifold_num_edge(self.0) }
    }

    /// Returns the number of triangles in the manifold.
    ///
    /// # Examples
    ///
    /// ```
    /// use manifold3d::Manifold;
    ///
    /// let manifold = Manifold::new_cuboid(1u8, 1u8, 1u8, true);
    /// assert_eq!(manifold.triangle_count(), 12);
    /// ```
    pub fn triangle_count(&self) -> usize {
        unsafe { manifold_num_tri(self.0) }
    }

    /// Returns a [MeshGL] representation of the manifold.
    ///
    /// # Examples
    ///
    /// ```
    /// use manifold3d::{Manifold, MeshGL};
    ///
    /// let manifold = Manifold::new_cuboid(1u8, 1u8, 1u8, true);
    /// let mesh = manifold.mesh();
    /// ```
    pub fn mesh(&self) -> MeshGL {
        let mesh_gl_ptr =
            unsafe { manifold_get_meshgl(manifold_alloc_meshgl() as *mut c_void, self.0) };
        MeshGL::from_ptr(mesh_gl_ptr)
    }

    /// Returns the [BoundingBox] representing the bounds of the manifold.
    ///
    /// # Examples
    ///
    /// ```
    /// use manifold3d::{BoundingBox, Manifold};
    ///
    /// let manifold = Manifold::new_cuboid(1u8, 1u8, 1u8, true);
    /// let bounding_box = manifold.bounding_box();
    /// ```
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

/// Represents a boolean operation that can be performed on a [Manifold].
pub enum BooleanOperation {
    /// Represents a union or addition operation.
    Add,
    /// Represents a subtraction operation.
    Subtract,
    /// Represents an intersection operation.
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

/// Defines custom error types for handling minimum smoothness constraints.
#[derive(Error, Debug)]
pub enum MinimumSmoothnessError {
    /// Error indicating that the provided smoothness value is out of bounds.
    ///
    /// # Arguments
    /// - `minimum`: The minimum allowed value for smoothness. It is defined by [MinimumSmoothness::MINIMUM]
    /// - `maximum`: The maximum allowed value for smoothness. It is defined by [MinimumSmoothness::MAXIMUM].
    /// - `actual`: The actual value provided that is out of the allowed range.
    ///
    /// # Examples
    /// ```
    /// use manifold3d::manifold::{MinimumSmoothness, MinimumSmoothnessError};
    ///
    /// match MinimumSmoothness::new(1.5) {
    ///     Ok(_) => println!("Smoothness created successfully!"),
    ///     Err(e) => println!("Error: {:?}", e),
    /// }
    /// ```
    #[error(
        "Minimum smoothness value must be between {minimum} and {maximum}. {actual} was provided"
    )]
    OutOfBounds {
        minimum: f64,
        maximum: f64,
        actual: f64,
    },
}

/// A struct representing a minimum smoothness value constrained within a specific range. The range
/// is defined by minimum value [MinimumSmoothness::MINIMUM] and maximum value [MinimumSmoothness::MAXIMUM].
pub struct MinimumSmoothness(f64);

impl MinimumSmoothness {
    /// The minimum allowed value for smoothness.
    pub const MINIMUM: f64 = 0.0;

    /// The maximum allowed value for smoothness.
    pub const MAXIMUM: f64 = 1.0;

    /// Constructs a new `MinimumSmoothness` instance.
    ///
    /// # Arguments
    /// - `smoothness`: The desired smoothness value.
    ///
    /// # Returns
    /// - `Ok(MinimumSmoothness)`: If the provided smoothness value is within the allowed range.
    /// - `Err(MinimumSmoothnessError)`: If the provided smoothness value is out of the allowed range.
    ///
    /// # Examples
    /// ```
    /// use manifold3d::manifold::{MinimumSmoothness, MinimumSmoothnessError};
    ///
    /// fn create_smoothness(value: f64) -> Result<MinimumSmoothness, MinimumSmoothnessError> {
    ///     MinimumSmoothness::new(value)
    /// }
    ///
    /// let valid_smoothness = create_smoothness(0.5);
    /// assert!(valid_smoothness.is_ok());
    ///
    /// let invalid_smoothness = create_smoothness(1.5);
    /// assert!(invalid_smoothness.is_err());
    /// ```
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

    /// Returns the smoothness value.
    pub fn get(&self) -> f64 {
        self.0
    }
}

impl Default for MinimumSmoothness {
    fn default() -> Self {
        Self::new(Self::MINIMUM).unwrap()
    }
}

impl From<MinimumSmoothness> for f64 {
    fn from(val: MinimumSmoothness) -> Self {
        val.get()
    }
}

/// An error type representing possible errors when creating an [EdgeSplitCount].
#[derive(Error, Debug)]
pub enum EdgeSplitCountError {
    /// Error variant indicating the provided edge split count is too small.
    ///
    /// The minimum required value is specified by `minimum` and the provided value
    /// is specified by `actual`. The minimum is defined in [EdgeSplitCount::MINIMUM].
    #[error("Edge split count must be at least {minimum}. {actual} was provided")]
    TooSmall { minimum: i32, actual: i32 },
}

/// A wrapper around a positive integer representing the edge split count.
///
/// The edge split count must be at least the value specified by [EdgeSplitCount::MINIMUM].
pub struct EdgeSplitCount(PositiveI32);

impl EdgeSplitCount {
    pub const MINIMUM: i32 = 2;

    /// Creates a new [EdgeSplitCount] if the provided number meets the minimum split count requirement.
    ///
    /// # Arguments
    /// - `num`: A value that can be converted into a [PositiveI32].
    ///
    /// # Returns
    /// This function returns a [Result], where the `Ok` variant contains the [EdgeSplitCount]
    /// if the provided number is valid, and the `Err` variant contains an [EdgeSplitCountError]
    /// if the number is too small.
    ///
    /// # Examples
    /// ```
    /// // Successful creation
    /// use manifold3d::manifold::EdgeSplitCount;
    /// use manifold3d::types::PositiveI32;
    ///
    /// let split_count = EdgeSplitCount::new(PositiveI32::new(3).unwrap());
    /// assert!(split_count.is_ok());
    ///
    /// // Error due to too small a value
    /// let split_count = EdgeSplitCount::new(PositiveI32::new(1).unwrap());
    /// assert!(split_count.is_err());
    /// ```
    pub fn new(num: impl Into<PositiveI32>) -> Result<Self, EdgeSplitCountError> {
        let num = num.into();
        if num < EdgeSplitCount::MINIMUM {
            return Err(EdgeSplitCountError::TooSmall {
                minimum: EdgeSplitCount::MINIMUM,
                actual: num.get(),
            });
        }
        Ok(Self(num))
    }

    /// Returns the internal [PositiveI32] value.
    pub fn get(&self) -> PositiveI32 {
        self.0
    }
}

impl From<EdgeSplitCount> for i32 {
    fn from(val: EdgeSplitCount) -> Self {
        val.get().get()
    }
}

/// Represents a plane in 3D space.
pub struct Plane {
    /// The normal vector of the plane.
    pub normal: Vec3,
}

impl Plane {
    /// Creates a new [Plane] from a given normal vector.
    ///
    /// # Arguments
    /// * `normal`: The normal vector of the plane.
    ///
    /// # Returns
    /// A new [Plane] object.
    ///
    /// # Examples
    /// ```
    /// use manifold3d::manifold::Plane;
    /// use manifold3d::types::Vec3;
    ///
    /// let normal = Vec3::new(0.0, 1.0, 0.0);
    /// let plane = Plane::new(normal);
    /// ```
    #[must_use]
    pub fn new(normal: Vec3) -> Self {
        Self { normal }
    }
}

pub struct OffsetPlane {
    /// The underlying plane.
    pub plane: Plane,
    /// The offset of the plane.
    pub offset: f64,
}

/// Represents a plane with an offset in 3D space.
impl OffsetPlane {
    /// Creates a new [OffsetPlane] with the given plane and offset.
    ///
    /// # Arguments
    /// * `plane`: The underlying plane: [Plane].
    /// * `offset`: The offset of the plane.
    ///
    /// # Returns
    /// A new [OffsetPlane] object.
    ///
    /// # Examples
    /// ```
    /// use manifold3d::manifold::{OffsetPlane, Plane};
    /// use manifold3d::types::Vec3;
    ///
    /// let plane = Plane::new(Vec3::new(0.0, 1.0, 0.0));
    /// let offset_plane = OffsetPlane::new(plane, 1.0);
    /// ```
    #[must_use]
    pub fn new(plane: Plane, offset: f64) -> Self {
        Self { plane, offset }
    }
}

mod warp {
    use crate::types::Point3;

    /// A trait that combines the functionality of [WarpVertex] and [ExternCWarpFn].
    ///
    /// This trait is automatically implemented by the [manifold3d::manifold_warp](crate::macros::manifold_warp)
    /// macro, which ensures that both [WarpVertex] and [ExternCWarpFn] are implemented
    /// for the annotated struct.
    ///
    /// # Context
    /// [Warp] is used in conjunction with the [Manifold#warp](struct.Manifold.html#method.warp)
    /// method to apply a transformation or deformation to a 3D manifold. The user needs to
    /// implement the [WarpVertex] trait to define the specific transformation logic.
    pub trait Warp: WarpVertex + ExternCWarpFn {}

    /// A trait for defining vertex transformations.
    ///
    /// Implementing this trait allows you to define how individual vertices in a 3D
    /// space are transformed. This is the core functionality that you need to implement
    /// when using the [manifold3d::manifold_warp](crate::macros::manifold_warp) macro.
    ///
    /// # Example
    /// ```
    /// use manifold3d::macros::manifold_warp;
    /// use manifold3d::manifold::WarpVertex;
    /// use manifold3d::types::Point3;
    ///
    /// #[manifold_warp]
    /// struct MyWarp;
    ///
    /// impl WarpVertex for MyWarp {
    ///     fn warp_vertex(&self, vertex: Point3) -> Point3 {
    ///         // Example: Translate the vertex by (1.0, 2.0, 3.0)
    ///         Point3::new(vertex.x + 1.0, vertex.y + 2.0, vertex.z + 3.0)
    ///     }
    /// }
    /// ```
    pub trait WarpVertex {
        /// Transforms a single vertex.
        ///
        /// # Arguments
        /// - `vertex`: A point in 3D space to be transformed.
        ///
        /// # Returns
        /// A new `Point3` representing the transformed vertex.
        fn warp_vertex(&self, vertex: Point3) -> Point3;
    }

    /// A trait for providing an `extern "C"` function pointer for vertex transformations.
    ///
    /// This trait is automatically implemented by the
    /// [manifold3d::manifold_warp](crate::macros::manifold_warp) macro. It provides
    /// a function pointer that can be used in contexts requiring an `extern "C"` interface,
    /// such as the [Manifold#warp](crate::manifold::Manifold#method.warp) function.
    ///
    /// Users typically do not need to implement this trait manually; instead, it is
    /// derived by the macro.
    ///
    /// # Safety
    /// The function pointer returned by this trait must be used correctly, adhering to
    /// C-style calling conventions. Improper use can lead to undefined behavior.
    pub trait ExternCWarpFn {
        /// Returns a function pointer to an `extern "C"` function implementing the
        /// vertex transformation logic.
        ///
        /// # Safety
        /// - The caller must ensure that the `ctx` pointer passed to the function
        ///   points to a valid instance of the struct implementing the trait.
        ///
        /// # Returns
        /// An unsafe `extern "C"` function pointer that can be used to transform vertices.
        fn extern_c_warp_fn(
            &self,
        ) -> unsafe extern "C" fn(
            arg1: f64,
            arg2: f64,
            arg3: f64,
            arg4: *mut ::std::os::raw::c_void,
        ) -> manifold3d_sys::ManifoldVec3;
    }
}
