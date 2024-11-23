use crate::manifold::Manifold;
use manifold3d_sys::{manifold_status, ManifoldError};

#[derive(Debug, Clone, Copy, Ord, PartialOrd, Eq, PartialEq)]
#[non_exhaustive]
pub enum Error {
    NoError,
    NonFiniteVertex,
    NotManifold,
    VertexIndexOutOfBounds,
    PropertiesWrongLength,
    MissingPositionProperties,
    MergeVectorsDifferentLengths,
    MergeIndexOutOfBounds,
    TransformWrongLength,
    RunIndexWrongLength,
    FaceIdWrongLength,
    InvalidConstruction,
    Unknown(u32),
}

impl From<ManifoldError> for Error {
    fn from(value: ManifoldError) -> Self {
        // #[allow(non_upper_case_globals)]
        match value {
            manifold3d_sys::ManifoldError_MANIFOLD_NO_ERROR => Error::NoError,
            manifold3d_sys::ManifoldError_MANIFOLD_NON_FINITE_VERTEX => Error::NonFiniteVertex,
            manifold3d_sys::ManifoldError_MANIFOLD_NOT_MANIFOLD => Error::NotManifold,
            manifold3d_sys::ManifoldError_MANIFOLD_VERTEX_INDEX_OUT_OF_BOUNDS => {
                Error::VertexIndexOutOfBounds
            }
            manifold3d_sys::ManifoldError_MANIFOLD_PROPERTIES_WRONG_LENGTH => {
                Error::PropertiesWrongLength
            }
            manifold3d_sys::ManifoldError_MANIFOLD_MISSING_POSITION_PROPERTIES => {
                Error::MissingPositionProperties
            }
            manifold3d_sys::ManifoldError_MANIFOLD_MERGE_VECTORS_DIFFERENT_LENGTHS => {
                Error::MergeVectorsDifferentLengths
            }
            manifold3d_sys::ManifoldError_MANIFOLD_MERGE_INDEX_OUT_OF_BOUNDS => {
                Error::MergeIndexOutOfBounds
            }
            manifold3d_sys::ManifoldError_MANIFOLD_TRANSFORM_WRONG_LENGTH => {
                Error::TransformWrongLength
            }
            manifold3d_sys::ManifoldError_MANIFOLD_RUN_INDEX_WRONG_LENGTH => {
                Error::RunIndexWrongLength
            }
            manifold3d_sys::ManifoldError_MANIFOLD_FACE_ID_WRONG_LENGTH => Error::FaceIdWrongLength,
            manifold3d_sys::ManifoldError_MANIFOLD_INVALID_CONSTRUCTION => {
                Error::InvalidConstruction
            }
            value => Error::Unknown(value),
        }
    }
}

pub trait ManifoldErrorExt {
    fn is_error(&self) -> bool;
}

pub fn check_error(manifold: Manifold) -> Result<Manifold, Error> {
    match Error::from(unsafe { manifold_status(manifold.ptr()) }) {
        Error::NoError => Ok(manifold),
        e => Err(e),
    }
}

impl ManifoldErrorExt for ManifoldError {
    fn is_error(&self) -> bool {
        *self != 0
    }
}

mod tests {

    #[test]
    fn test_error_from_u32() {
        // Checks whether the error discrimination works at all
        assert_eq!(crate::Error::from(manifold3d_sys::ManifoldError_MANIFOLD_NO_ERROR), crate::Error::NoError);
        assert_eq!(
            crate::Error::from(manifold3d_sys::ManifoldError_MANIFOLD_NON_FINITE_VERTEX),
            crate::Error::NonFiniteVertex
        );
    }
}
