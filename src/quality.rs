use crate::types::{Angle, PositiveF64, PositiveI32};
use manifold_sys::{
    manifold_get_circular_segments, manifold_reset_to_circular_defaults,
    manifold_set_circular_segments, manifold_set_min_circular_angle,
    manifold_set_min_circular_edge_length,
};
use std::num::NonZeroI32;

pub fn set_min_circular_angle(angle: Angle) {
    unsafe { manifold_set_min_circular_angle(angle.as_degrees()) }
}

pub fn set_min_circular_angle_unchecked(angle: f64) {
    unsafe { manifold_set_min_circular_angle(angle) }
}

pub fn set_min_circular_edge_length(length: PositiveF64) {
    unsafe { manifold_set_min_circular_edge_length(length.get()) }
}

pub fn set_min_circular_edge_length_unchecked(length: f64) {
    unsafe { manifold_set_min_circular_edge_length(length) }
}

pub fn set_circular_segments(segments: PositiveI32) {
    unsafe { manifold_set_circular_segments(segments.get()) }
}

pub fn set_circular_segments_unchecked(segments: i32) {
    unsafe { manifold_set_circular_segments(segments) }
}

pub fn get_circular_segments(radius: PositiveF64) -> NonZeroI32 {
    unsafe { NonZeroI32::new_unchecked(manifold_get_circular_segments(radius.get())) }
}

pub fn get_circular_segments_unchecked(radius: f64) -> i32 {
    unsafe { manifold_get_circular_segments(radius) }
}

pub fn reset_to_circular_defaults() {
    unsafe { manifold_reset_to_circular_defaults() }
}

#[cfg(test)]
mod tests {
    use crate::quality::*;
    use crate::types::{Angle, PositiveF64, PositiveI32};

    #[test]
    fn test_set_and_get_circular_segments() {
        reset_to_circular_defaults();
        let circular_segments = 42;
        set_circular_segments(PositiveI32::new(circular_segments).unwrap());
        let result = get_circular_segments(PositiveF64::new(1.0).unwrap());
        assert_eq!(result.get(), circular_segments);
    }

    #[test]
    fn test_set_min_circular_angle_and_edge_length_and_get_circular_segments() {
        reset_to_circular_defaults();
        set_min_circular_angle(Angle::from_positive_num(PositiveI32::new(365).unwrap()));
        set_min_circular_edge_length(PositiveF64::new(1.0).unwrap());
        let result = get_circular_segments(PositiveF64::new(10.0).unwrap());
        assert_eq!(result.get(), 64);
    }
}
