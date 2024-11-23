use crate::math;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Matrix4x3 {
    pub rows: [math::Vec3; 4],
}

impl Matrix4x3 {
    pub fn new(rows: [math::Vec3; 4]) -> Self {
        Self { rows }
    }
}

#[cfg(feature = "nalgebra_interop")]
impl From<nalgebra::Matrix4x3<f64>> for Matrix4x3 {
    fn from(matrix: nalgebra::Matrix4x3<f64>) -> Self {
        Matrix4x3 {
            rows: [
                math::Vec3 {
                    x: matrix.m11,
                    y: matrix.m12,
                    z: matrix.m13,
                },
                math::Vec3 {
                    x: matrix.m21,
                    y: matrix.m22,
                    z: matrix.m23,
                },
                math::Vec3 {
                    x: matrix.m31,
                    y: matrix.m32,
                    z: matrix.m33,
                },
                math::Vec3 {
                    x: matrix.m41,
                    y: matrix.m42,
                    z: matrix.m43,
                },
            ],
        }
    }
}
