#[non_exhaustive]
pub enum FillRule {
    EvenOdd,
    NonZero,
    Positive,
    Negative,
    Unknown(u32),
}

impl From<u32> for FillRule {
    fn from(value: u32) -> Self {
        match value {
            manifold3d_sys::ManifoldFillRule_MANIFOLD_FILL_RULE_EVEN_ODD => FillRule::EvenOdd,
            manifold3d_sys::ManifoldFillRule_MANIFOLD_FILL_RULE_NON_ZERO => FillRule::NonZero,
            manifold3d_sys::ManifoldFillRule_MANIFOLD_FILL_RULE_POSITIVE => FillRule::Positive,
            manifold3d_sys::ManifoldFillRule_MANIFOLD_FILL_RULE_NEGATIVE => FillRule::Negative,
            _ => FillRule::Unknown(value),
        }
    }
}

impl From<FillRule> for u32 {
    fn from(value: FillRule) -> Self {
        match value {
            FillRule::EvenOdd => manifold3d_sys::ManifoldFillRule_MANIFOLD_FILL_RULE_EVEN_ODD,
            FillRule::NonZero => manifold3d_sys::ManifoldFillRule_MANIFOLD_FILL_RULE_NON_ZERO,
            FillRule::Positive => manifold3d_sys::ManifoldFillRule_MANIFOLD_FILL_RULE_POSITIVE,
            FillRule::Negative => manifold3d_sys::ManifoldFillRule_MANIFOLD_FILL_RULE_NEGATIVE,
            FillRule::Unknown(v) => v,
        }
    }
}
