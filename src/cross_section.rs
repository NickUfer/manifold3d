use manifold3d_sys::ManifoldCrossSection;

pub struct CrossSection(*mut ManifoldCrossSection);

impl CrossSection {
    pub(crate) fn from_ptr(ptr: *mut ManifoldCrossSection) -> CrossSection {
        CrossSection(ptr)
    }
}
