use crate::manifold::Manifold;
use crate::{check_error, Error, HalfEdgeIndex};
use manifold3d_sys::{
    manifold_alloc_manifold, manifold_alloc_meshgl, manifold_delete_meshgl, manifold_meshgl_copy,
    manifold_meshgl_face_id_length, manifold_meshgl_merge, manifold_meshgl_merge_length,
    manifold_meshgl_num_prop, manifold_meshgl_num_tri, manifold_meshgl_num_vert,
    manifold_meshgl_run_index_length, manifold_meshgl_run_original_id_length,
    manifold_meshgl_run_transform_length, manifold_meshgl_tangent_length,
    manifold_meshgl_tri_length, manifold_meshgl_vert_properties,
    manifold_meshgl_vert_properties_length, manifold_smooth, ManifoldMeshGL,
};
use std::alloc::{alloc, Layout};
use std::os::raw::c_void;

pub struct MeshGL(*mut ManifoldMeshGL);

impl MeshGL {
    pub(crate) fn from_ptr(ptr: *mut ManifoldMeshGL) -> MeshGL {
        MeshGL(ptr)
    }

    pub(crate) fn ptr(&self) -> *mut ManifoldMeshGL {
        self.0
    }

    pub fn merge(&self) -> Option<MeshGL> {
        let duplicate_ptr = unsafe { manifold_alloc_meshgl() };
        let returned_ptr = unsafe { manifold_meshgl_merge(duplicate_ptr as *mut c_void, self.0) };

        // If the pointer to the duplicate_ptr was returned it means the operation was successful
        if duplicate_ptr == returned_ptr {
            return Some(MeshGL(duplicate_ptr));
        }
        None
    }

    /// Constructs a smooth version of the input [`MeshGL`] mesh by creating tangents.
    ///
    /// The actual triangle resolution remains unchanged; use [`Manifold::refine_via_edge_splits`]
    /// to further interpolate to a higher-resolution curve.
    ///
    /// By default, each edge is assessed for maximum smoothness, aiming to minimize the
    /// maximum mean curvature magnitude. Higher-order derivatives are not considered,
    /// as interpolation is carried out independently per triangle, with constraints shared
    /// only along their boundaries.
    ///
    /// # Arguments
    ///
    /// * `half_edge_smoothness`: Optionally, provide a vector of sharpened halfedges, typically a small subset
    ///   of all halfedges. The order of entries is irrelevant, as each specifies the desired smoothness
    ///   (ranging from zero to one, with one being the default for all unspecified halfedges) alongside the
    ///   halfedge index (calculated as 3 * triangle index + 0, 1, 2, where 0 is the edge between triVert 0 and 1, etc).
    ///   A smoothness of zero results in a sharp crease. Smoothness is averaged along each edge; when
    ///   two sharpened edges meet at a vertex, their tangents are aligned to be colinear, allowing continuity
    ///   of the sharpened edge. Vertices with only one sharpened edge are completely smooth, enabling
    ///   sharpened edges to smoothly disappear at their ends. To sharpen a single vertex, sharpen all
    ///   incident edges, which facilitates forming cones.
    ///
    /// # Returns
    ///
    /// A new manifold that represents a smoothed version of the original mesh.
    ///
    /// # Examples
    /// ```rust
    /// use manifold3d::types::PositiveF64;
    /// use manifold3d::{Manifold, MeshGL};
    ///
    /// let manifold = Manifold::new_cuboid(
    ///     PositiveF64::new(1.0).unwrap(),
    ///     PositiveF64::new(1.0).unwrap(),
    ///     PositiveF64::new(1.0).unwrap(),
    ///     true,
    /// );
    /// let mesh_gl = manifold.as_mesh();
    ///
    /// let half_edge_smoothness = vec![(0, 0.5), (1, 1.0)];
    /// let smoothed_manifold = mesh_gl.smooth(Some(half_edge_smoothness));
    /// ```
    pub fn smooth(
        &self,
        half_edge_smoothness: Option<Vec<(HalfEdgeIndex, f64)>>,
    ) -> Result<Manifold, Error> {
        let (half_edge_indices_ptr, half_edge_smoothness_ptr, length) = match half_edge_smoothness {
            None => (
                std::ptr::null::<HalfEdgeIndex>() as *mut HalfEdgeIndex,
                std::ptr::null::<f64>() as *mut f64,
                0usize,
            ),
            Some(vec) => {
                let (half_edge_indices, half_edge_smoothness): (Vec<_>, Vec<_>) =
                    vec.into_iter().unzip();

                let half_edge_indices_ptr = half_edge_indices.as_ptr() as *mut HalfEdgeIndex;
                let half_edge_smoothness_ptr = half_edge_smoothness.as_ptr() as *mut f64;
                (
                    half_edge_indices_ptr,
                    half_edge_smoothness_ptr,
                    half_edge_indices.len(),
                )
            }
        };

        let manifold_ptr = unsafe {
            manifold_smooth(
                manifold_alloc_manifold() as *mut c_void,
                self.ptr(),
                half_edge_indices_ptr,
                half_edge_smoothness_ptr,
                length,
            )
        };
        check_error(Manifold::from_ptr(manifold_ptr))
    }

    pub fn properties_per_vertex_count(&self) -> i32 {
        unsafe { manifold_meshgl_num_prop(self.0) }
    }

    pub fn vertex_count(&self) -> i32 {
        unsafe { manifold_meshgl_num_vert(self.0) }
    }

    pub fn triangle_count(&self) -> i32 {
        unsafe { manifold_meshgl_num_tri(self.0) }
    }

    /// Returns the length of the flat GL-style interleaved list of all vertex properties.
    pub fn vertex_property_count(&self) -> usize {
        unsafe { manifold_meshgl_vert_properties_length(self.0) }
    }

    pub fn vertex_index_count(&self) -> usize {
        unsafe { manifold_meshgl_tri_length(self.0) }
    }

    pub fn mesh_merge_count(&self) -> usize {
        unsafe { manifold_meshgl_merge_length(self.0) }
    }

    pub fn run_index_count(&self) -> usize {
        unsafe { manifold_meshgl_run_index_length(self.0) }
    }

    pub fn run_original_id_count(&self) -> usize {
        unsafe { manifold_meshgl_run_original_id_length(self.0) }
    }

    pub fn run_transform_count(&self) -> usize {
        unsafe { manifold_meshgl_run_transform_length(self.0) }
    }

    pub fn face_id_count(&self) -> usize {
        unsafe { manifold_meshgl_face_id_length(self.0) }
    }

    pub fn tangent_count(&self) -> usize {
        unsafe { manifold_meshgl_tangent_length(self.0) }
    }

    /// Returns a copy of the original data
    pub fn vertex_properties(&self) -> Vec<f32> {
        let element_count = self.vertex_property_count();
        let layout = Layout::array::<f32>(element_count).unwrap();
        let array_start_ptr = unsafe { alloc(layout) } as *mut f32;
        unsafe { manifold_meshgl_vert_properties(array_start_ptr as *mut c_void, self.0) };

        unsafe { Vec::from_raw_parts(array_start_ptr, element_count, element_count) }
    }
}

impl Clone for MeshGL {
    fn clone(&self) -> Self {
        let mesh_gl_ptr =
            unsafe { manifold_meshgl_copy(manifold_alloc_meshgl() as *mut c_void, self.0) };
        MeshGL(mesh_gl_ptr)
    }
}

impl Drop for MeshGL {
    fn drop(&mut self) {
        unsafe { manifold_delete_meshgl(self.0) }
    }
}
