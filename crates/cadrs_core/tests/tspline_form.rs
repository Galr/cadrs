//! The T-spline Form feature: TForm's models rebuilt as exact NURBS parts.

use cadrs_core::FeatureId;
use cadrs_core::document::{Feature, FeatureKind};
use cadrs_core::parts::PartKind;
use cadrs_core::rebuild::Rebuilder;
use cadrs_core::tspline_form::TSplineFormFeature;
use cadrs_tspline::{Doc, create_cylinder, create_plane, create_torus};

fn form(x: TSplineFormFeature) -> Feature {
    Feature::new(FeatureId::new(), "Form 1", FeatureKind::TSplineForm(x))
}

fn tform(bodies: Vec<(&str, cadrs_tspline::TSpline)>) -> String {
    Doc {
        bodies: bodies
            .into_iter()
            .map(|(n, t)| (n.to_string(), true, t))
            .collect(),
    }
    .to_json()
}

#[test]
fn a_torus_is_a_solid_of_the_t_splines_volume() {
    let x = TSplineFormFeature::from_tform(&tform(vec![("Ring", create_torus(40.0, 14.0, 12, 8))]))
        .unwrap();
    let build = Rebuilder::new().rebuild(&[form(x)]);
    assert!(build.errors.is_empty(), "{:?}", build.errors);
    assert_eq!(build.parts.len(), 1);
    let part = &build.parts[0];
    assert_eq!(part.name, "Ring");
    assert_eq!(part.kind, PartKind::Solid);
    // TForm's torus is a T-spline close to a torus of radii 40 and 14 (2π²Rr² ≈ 154 750 mm³)
    let v = part.mass.unwrap().volume;
    let torus = 2.0 * std::f64::consts::PI.powi(2) * 40.0 * 14.0 * 14.0;
    assert!((v - torus).abs() < 0.05 * torus, "{v} vs {torus}");
    // Z up: the ring lies in the XY plane, 2 × 14 mm thick
    let zs = part.solid.positions.iter().map(|p| p[2]);
    let (lo, hi) = (
        zs.clone().fold(f64::MAX, f64::min),
        zs.fold(f64::MIN, f64::max),
    );
    assert!((hi - lo - 28.0).abs() < 1.5, "{lo} … {hi}");
}

#[test]
fn a_plane_and_a_tube_are_surfaces() {
    let x = TSplineFormFeature::from_tform(&tform(vec![
        ("Sheet", create_plane(100.0, 100.0, 4, 4)),
        ("Tube", create_cylinder(30.0, 80.0, 8, 4)),
    ]))
    .unwrap();
    let build = Rebuilder::new().rebuild(&[form(x)]);
    assert!(build.errors.is_empty(), "{:?}", build.errors);
    assert!(build.warnings.is_empty(), "{:?}", build.warnings);
    let names: Vec<&str> = build.parts.iter().map(|p| p.name.as_str()).collect();
    assert_eq!(names, ["Sheet", "Tube"]);
    assert!(build.parts.iter().all(|p| p.kind == PartKind::Surface));
    // the tube runs up Z from 0 to 80 (TForm's Y)
    let zs = build.parts[1].solid.positions.iter().map(|p| p[2]);
    let (lo, hi) = (
        zs.clone().fold(f64::MAX, f64::min),
        zs.fold(f64::MIN, f64::max),
    );
    assert!(
        lo > -1.0 && lo < 25.0 && hi > 55.0 && hi < 81.0,
        "{lo} … {hi}"
    );
}

#[test]
fn a_refined_and_edited_t_spline_rebuilds() {
    let mut ts = create_plane(100.0, 100.0, 8, 8);
    let f = ts.face_at(3.5, 3.5).unwrap();
    ts.subdivide_face(f, true).unwrap();
    // as in TForm: the split's new edges stop inside the mesh
    assert_eq!(ts.t_junction_count(), 4);
    assert_eq!(ts.verts.len(), 90);
    // pull a point of the refined face up
    let i = ts.vert_at(3.5, 3.5).unwrap();
    ts.verts[i].p[1] = 30.0;
    let x = TSplineFormFeature::from_tform(&tform(vec![("Bump", ts.clone())])).unwrap();
    let build = Rebuilder::new().rebuild(&[form(x)]);
    assert!(build.errors.is_empty(), "{:?}", build.errors);
    let zs = build.parts[0].solid.positions.iter().map(|p| p[2]);
    let hi = zs.fold(f64::MIN, f64::max);
    // the surface rises towards the pulled point (TForm's Y is cadrs's Z)
    let top = ts.eval(3.5, 3.5)[1];
    assert!((hi - top).abs() < 0.5, "{hi} vs {top}");
}

#[test]
fn a_bad_form_fails_with_a_reason() {
    let build = Rebuilder::new().rebuild(&[form(TSplineFormFeature::default())]);
    assert_eq!(build.errors.len(), 1);
    assert!(build.parts.is_empty());
}
