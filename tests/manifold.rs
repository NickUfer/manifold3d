use manifold3d::macros::manifold;
use manifold3d::manifold::{BooleanOperation, ReplaceVertexProperties};
use manifold3d::types::{Point3, PositiveF64};
use manifold3d::{types, Manifold};
use std::pin::Pin;

#[test]
fn test_translation() {
    let original = Manifold::new_cuboid(
        PositiveF64::new(1.0).unwrap(),
        PositiveF64::new(1.0).unwrap(),
        PositiveF64::new(1.0).unwrap(),
        true,
    );
    let translated = original.translate(types::Vec3::new(1.0, -1.0, 3.0));

    assert_eq!(
        translated.bounding_box().min_point(),
        types::Point3::new(0.5, -1.5, 2.5)
    );
    assert_eq!(
        translated.bounding_box().max_point(),
        types::Point3::new(1.5, -0.5, 3.5)
    );
}

#[test]
fn test_boolean_subtraction() {
    let manifold = Manifold::new_cuboid(
        PositiveF64::new(1.0).unwrap(),
        PositiveF64::new(1.0).unwrap(),
        PositiveF64::new(1.0).unwrap(),
        true,
    );
    let expected_bounding_box = manifold.bounding_box();

    let other = Manifold::new_cuboid(
        PositiveF64::new(1.0).unwrap(),
        PositiveF64::new(1.0).unwrap(),
        PositiveF64::new(1.0).unwrap(),
        true,
    )
    .translate(types::Vec3::new(0.0, 0.5, 0.0));
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
    let manifold = Manifold::new_cuboid(
        PositiveF64::new(1.0).unwrap(),
        PositiveF64::new(1.0).unwrap(),
        PositiveF64::new(1.0).unwrap(),
        true,
    );

    // Removes all edges to form a cross
    let others = vec![
        Manifold::new_cuboid(
            PositiveF64::new(1.0).unwrap(),
            PositiveF64::new(1.0).unwrap(),
            PositiveF64::new(1.0).unwrap(),
            true,
        )
        .translate(types::Vec3::new(0.75, 0.75, 0.0)),
        Manifold::new_cuboid(
            PositiveF64::new(1.0).unwrap(),
            PositiveF64::new(1.0).unwrap(),
            PositiveF64::new(1.0).unwrap(),
            true,
        )
        .translate(types::Vec3::new(-0.75, -0.75, 0.0)),
        Manifold::new_cuboid(
            PositiveF64::new(1.0).unwrap(),
            PositiveF64::new(1.0).unwrap(),
            PositiveF64::new(1.0).unwrap(),
            true,
        )
        .translate(types::Vec3::new(0.75, -0.75, 0.0)),
        Manifold::new_cuboid(
            PositiveF64::new(1.0).unwrap(),
            PositiveF64::new(1.0).unwrap(),
            PositiveF64::new(1.0).unwrap(),
            true,
        )
        .translate(types::Vec3::new(-0.75, 0.75, 0.0)),
    ];
    let result = manifold.batch_boolean(&others, BooleanOperation::Subtract);

    assert_eq!(result.vertex_count(), 24);
}

#[test]
fn test_linear_warping() {
    #[manifold::warp]
    pub struct TranslationWarp {
        translation: types::Vec3,
    }

    impl manifold3d::manifold::WarpVertex for TranslationWarp {
        fn warp_vertex(&self, vertex: types::Point3) -> types::Point3 {
            types::Point3::new(
                vertex.x + self.translation.x,
                vertex.y + self.translation.y,
                vertex.z + self.translation.z,
            )
        }
    }

    let manifold = Manifold::new_cuboid(
        PositiveF64::new(1.0).unwrap(),
        PositiveF64::new(1.0).unwrap(),
        PositiveF64::new(1.0).unwrap(),
        true,
    );
    let expected_bounding_box = manifold.bounding_box();

    let translation_warp = TranslationWarp {
        translation: types::Vec3::new(1.0, 1.0, 1.0),
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

#[test]
fn test_replace_vertex_properties() {
    #[manifold::manage_vertex_properties]
    pub struct MyPropertyReplacer {}

    pub struct MyPropertyReplacerCtx {
        vertex_count: usize,
    }

    impl ReplaceVertexProperties for MyPropertyReplacer {
        type CTX = MyPropertyReplacerCtx;

        fn new_ctx(&self) -> Self::CTX {
            MyPropertyReplacerCtx { vertex_count: 0 }
        }

        fn new_vertex_properties_count(&self, target: &Manifold) -> usize {
            // We add 3 more channels per vertex
            target.properties_per_vertex_count() + 3
        }

        fn replace_vertex_properties(
            &self,
            ctx: &mut Self::CTX,
            _vertex_position: Point3,
            old_properties: &[f64],
            new_properties: &mut [f64],
        ) {
            ctx.vertex_count += 1;
            println!("{}", ctx.vertex_count);
            new_properties[..old_properties.len()].copy_from_slice(old_properties);

            let new_data_index = old_properties.len();
            new_properties[new_data_index] = (ctx.vertex_count + 1) as f64;
            new_properties[new_data_index + 1] = (ctx.vertex_count + 2) as f64;
            new_properties[new_data_index + 2] = (ctx.vertex_count + 3) as f64;
        }
    }

    let manifold = manifold3d::Manifold::new_cuboid(
        PositiveF64::new(1.0).unwrap(),
        PositiveF64::new(1.0).unwrap(),
        PositiveF64::new(1.0).unwrap(),
        true,
    );
    let replacer = MyPropertyReplacer {};

    let new_manifold = manifold.replace_vertex_properties(Pin::new(&replacer));
    println!("{}", new_manifold.properties_per_vertex_count());
}
