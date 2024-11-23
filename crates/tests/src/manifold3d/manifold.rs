use manifold3d::macros::manifold_warp;
use manifold3d::types::manifold::vertex;
use manifold3d::{types, BooleanOperation, Manifold};
use std::pin::Pin;

#[test]
fn test_translation() {
    let original = Manifold::new_cuboid(1u8, 1u8, 1u8, true);
    let translated = original.translate(types::math::Vec3::new(1.0, -1.0, 3.0));

    assert_eq!(
        translated.bounding_box().min_point(),
        types::math::Point3::new(0.5, -1.5, 2.5)
    );
    assert_eq!(
        translated.bounding_box().max_point(),
        types::math::Point3::new(1.5, -0.5, 3.5)
    );
}

#[test]
fn test_boolean_subtraction() {
    let manifold = Manifold::new_cuboid(1u8, 1u8, 1u8, true);
    let expected_bounding_box = manifold.bounding_box();

    let other =
        Manifold::new_cuboid(1u8, 1u8, 1u8, true).translate(types::math::Vec3::new(0.0, 0.5, 0.0));
    let result = manifold.boolean(&other, BooleanOperation::Subtract);
    let result_bounding_box = result.bounding_box();

    assert_eq!(
        result_bounding_box.min_point(),
        expected_bounding_box.min_point()
    );
    assert_eq!(result_bounding_box.max_point().x, 0.5);
    assert_eq!(result_bounding_box.max_point().y, 0.0);
    assert_eq!(result_bounding_box.max_point().z, 0.5);
}

#[test]
fn test_batch_boolean_subtraction() {
    let manifold = Manifold::new_cuboid(1u8, 1u8, 1u8, true);

    // Removes all edges to form a cross
    let others = vec![
        Manifold::new_cuboid(1u8, 1u8, 1u8, true)
            .translate(types::math::Vec3::new(0.75, 0.75, 0.0)),
        Manifold::new_cuboid(1u8, 1u8, 1u8, true)
            .translate(types::math::Vec3::new(-0.75, -0.75, 0.0)),
        Manifold::new_cuboid(1u8, 1u8, 1u8, true)
            .translate(types::math::Vec3::new(0.75, -0.75, 0.0)),
        Manifold::new_cuboid(1u8, 1u8, 1u8, true)
            .translate(types::math::Vec3::new(-0.75, 0.75, 0.0)),
    ];
    let result = manifold.batch_boolean(&others, BooleanOperation::Subtract);

    assert_eq!(result.vertex_count(), 24);
}

#[test]
fn test_linear_warping() {
    #[manifold_warp]
    pub struct TranslationWarp {
        translation: types::math::Vec3,
    }

    impl vertex::WarpImpl for TranslationWarp {
        fn warp_vertex(&self, vertex: types::math::Point3) -> types::math::Point3 {
            let result = types::math::Point3::new(
                vertex.x + self.translation.x,
                vertex.y + self.translation.y,
                vertex.z + self.translation.z,
            );
            result
        }
    }

    let manifold = Manifold::new_cuboid(1u8, 1u8, 1u8, true);
    let expected_bounding_box = manifold.bounding_box();

    let translation_warp = TranslationWarp {
        translation: types::math::Vec3::new(1.0, 1.0, 1.0),
    };

    let translation_warp = Pin::new(&translation_warp);
    let result = manifold.warp(translation_warp);
    let result_bounding_box = result.bounding_box();
    assert_eq!(
        result_bounding_box.min_point(),
        expected_bounding_box.min_point() + 1.0
    );
    assert_eq!(
        result_bounding_box.max_point(),
        expected_bounding_box.max_point() + 1.0
    );
}
