use manifold_sys::{
    manifold_alloc_meshgl, manifold_delete_meshgl, manifold_meshgl_copy,
    manifold_meshgl_face_id_length, manifold_meshgl_merge, manifold_meshgl_merge_length,
    manifold_meshgl_num_prop, manifold_meshgl_num_tri, manifold_meshgl_num_vert,
    manifold_meshgl_run_index_length, manifold_meshgl_run_original_id_length,
    manifold_meshgl_run_transform_length, manifold_meshgl_tangent_length,
    manifold_meshgl_tri_length, manifold_meshgl_vert_properties,
    manifold_meshgl_vert_properties_length, ManifoldMeshGL,
};
use std::alloc::{alloc, Layout};

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
        let returned_ptr =
            unsafe { manifold_meshgl_merge(duplicate_ptr as *mut std::os::raw::c_void, self.0) };

        // If the pointer to the duplicate_ptr was returned it means the operation was successful
        if duplicate_ptr == returned_ptr {
            return Some(MeshGL(duplicate_ptr));
        }
        None
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
        unsafe {
            manifold_meshgl_vert_properties(array_start_ptr as *mut std::os::raw::c_void, self.0)
        };

        unsafe { Vec::from_raw_parts(array_start_ptr, element_count, element_count) }
    }
}

impl Drop for MeshGL {
    fn drop(&mut self) {
        unsafe { manifold_delete_meshgl(self.0) }
    }
}

impl Clone for MeshGL {
    fn clone(&self) -> Self {
        let mesh_gl_ptr = unsafe { manifold_alloc_meshgl() };
        unsafe { manifold_meshgl_copy(mesh_gl_ptr as *mut std::os::raw::c_void, self.0) };
        MeshGL(mesh_gl_ptr)
    }
}
