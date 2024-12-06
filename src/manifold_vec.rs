use crate::Manifold;
use manifold3d_sys::{
    manifold_alloc_manifold, manifold_compose, manifold_manifold_vec_get,
    manifold_manifold_vec_length, ManifoldManifoldVec,
};
use std::os::raw::c_void;

pub struct ManifoldVec(*mut ManifoldManifoldVec);

impl ManifoldVec {
    pub(crate) fn from_ptr(ptr: *mut ManifoldManifoldVec) -> ManifoldVec {
        ManifoldVec(ptr)
    }

    pub fn compose(&self) -> Manifold {
        let manifold_ptr =
            unsafe { manifold_compose(manifold_alloc_manifold() as *mut c_void, self.0) };
        Manifold::from_ptr(manifold_ptr)
    }

    pub fn count(&self) -> usize {
        unsafe { manifold_manifold_vec_length(self.0) }
    }

    pub fn get(&self, index: usize) -> Option<Manifold> {
        if index >= self.count() {
            return None;
        }

        let simple_polygon_ptr = unsafe {
            manifold_manifold_vec_get(manifold_alloc_manifold() as *mut c_void, self.0, index)
        };
        Some(Manifold::from_ptr(simple_polygon_ptr))
    }

    pub fn as_vec(&self) -> Vec<Manifold> {
        let mut manifolds = Vec::<Manifold>::with_capacity(self.count());

        for i in 0..self.count() {
            manifolds.push(self.get(i).unwrap());
        }

        manifolds
    }
}
