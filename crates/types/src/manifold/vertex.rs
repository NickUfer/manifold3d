use crate::math;
use manifold3d_sys::ManifoldVec3;

pub trait Warp: WarpImpl + ExternCWarpFn {}

pub trait WarpImpl {
    fn warp_vertex(&self, vertex: math::Point3) -> math::Point3;
}

pub trait ExternCWarpFn {
    fn extern_c_warp_fn(
        &self,
    ) -> unsafe extern "C" fn(
        arg1: f64,
        arg2: f64,
        arg3: f64,
        arg4: *mut ::std::os::raw::c_void,
    ) -> ManifoldVec3;
}
