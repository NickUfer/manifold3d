use crate::manifold::Manifold;
use manifold3d_sys::{
    manifold_status, ManifoldError, ManifoldError_MANIFOLD_FACE_ID_WRONG_LENGTH,
    ManifoldError_MANIFOLD_INVALID_CONSTRUCTION, ManifoldError_MANIFOLD_MERGE_INDEX_OUT_OF_BOUNDS,
    ManifoldError_MANIFOLD_MERGE_VECTORS_DIFFERENT_LENGTHS,
    ManifoldError_MANIFOLD_MISSING_POSITION_PROPERTIES, ManifoldError_MANIFOLD_NON_FINITE_VERTEX,
    ManifoldError_MANIFOLD_NOT_MANIFOLD, ManifoldError_MANIFOLD_NO_ERROR,
    ManifoldError_MANIFOLD_PROPERTIES_WRONG_LENGTH, ManifoldError_MANIFOLD_RUN_INDEX_WRONG_LENGTH,
    ManifoldError_MANIFOLD_TRANSFORM_WRONG_LENGTH,
    ManifoldError_MANIFOLD_VERTEX_INDEX_OUT_OF_BOUNDS,
};

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

impl From<u32> for Error {
    fn from(value: u32) -> Self {
        #[allow(non_upper_case_globals)]
        match value {
            ManifoldError_MANIFOLD_NO_ERROR => Error::NoError,
            ManifoldError_MANIFOLD_NON_FINITE_VERTEX => Error::NonFiniteVertex,
            ManifoldError_MANIFOLD_NOT_MANIFOLD => Error::NotManifold,
            ManifoldError_MANIFOLD_VERTEX_INDEX_OUT_OF_BOUNDS => Error::VertexIndexOutOfBounds,
            ManifoldError_MANIFOLD_PROPERTIES_WRONG_LENGTH => Error::PropertiesWrongLength,
            ManifoldError_MANIFOLD_MISSING_POSITION_PROPERTIES => Error::MissingPositionProperties,
            ManifoldError_MANIFOLD_MERGE_VECTORS_DIFFERENT_LENGTHS => {
                Error::MergeVectorsDifferentLengths
            }
            ManifoldError_MANIFOLD_MERGE_INDEX_OUT_OF_BOUNDS => Error::MergeIndexOutOfBounds,
            ManifoldError_MANIFOLD_TRANSFORM_WRONG_LENGTH => Error::TransformWrongLength,
            ManifoldError_MANIFOLD_RUN_INDEX_WRONG_LENGTH => Error::RunIndexWrongLength,
            ManifoldError_MANIFOLD_FACE_ID_WRONG_LENGTH => Error::FaceIdWrongLength,
            ManifoldError_MANIFOLD_INVALID_CONSTRUCTION => Error::InvalidConstruction,
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
    use crate::error::Error;
    use manifold3d_sys::{
        ManifoldError_MANIFOLD_NON_FINITE_VERTEX, ManifoldError_MANIFOLD_NO_ERROR,
    };

    #[test]
    fn test_error_from_u32() {
        // Checks whether the error discrimination works at all
        assert_eq!(Error::from(ManifoldError_MANIFOLD_NO_ERROR), Error::NoError);
        assert_eq!(
            Error::from(ManifoldError_MANIFOLD_NON_FINITE_VERTEX),
            Error::NonFiniteVertex
        );
    }
}
