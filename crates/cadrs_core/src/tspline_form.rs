//! The **T-spline Form** feature: freeform bodies modelled as cubic T-splines, the models of
//! TForm (`.tform.json`). Its rebuild is in `rebuild/kernel_ops/linked.rs`.
//!
//! - The feature keeps each body's T-mesh ([`cadrs_tspline`]): control points, weights and
//!   edges, in cadrs's frame (Z up, mm). TForm is Y up, so reading and writing its files turns
//!   them ([`TSplineFormFeature::from_tform`], [`TSplineFormFeature::to_tform`]).
//! - On rebuild each body becomes one exact NURBS face ([`cadrs_tspline::TSpline::to_nurbs`]):
//!   a surface part, or a solid when the T-spline closes in both directions (a torus).
//! - Face names: face `i` of body `k` is `FaceOrigin::Imported { body: k, face: i }` under the
//!   feature's id, as Import names its faces, so later features can refer to them.

use serde::{Deserialize, Serialize};

use cadrs_tspline::{Doc, TSpline, TSplineFile};

use crate::command::{Command, CommandError, Scope};
use crate::document::{Document, FeatureKind};
use crate::ids::{ElementId, FeatureId};

/// One body of a T-spline Form: its name and T-spline.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FormBody {
    pub name: String,
    /// The T-spline, Z up, mm.
    pub tspline: TSplineFile,
}

/// A T-spline Form: one or more freeform bodies.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
pub struct TSplineFormFeature {
    pub bodies: Vec<FormBody>,
}

impl TSplineFormFeature {
    /// Why it can't rebuild as it stands.
    pub fn problem(&self) -> Option<&'static str> {
        if self.bodies.is_empty() {
            return Some("The form has no bodies");
        }
        if self.splines().is_err() {
            return Some("A body of the form is not a valid T-spline");
        }
        None
    }

    /// Every body's T-spline.
    pub fn splines(&self) -> Result<Vec<(String, TSpline)>, String> {
        self.bodies
            .iter()
            .map(|b| {
                Ok((
                    b.name.clone(),
                    TSpline::try_from(&b.tspline).map_err(|e| e.to_string())?,
                ))
            })
            .collect()
    }

    /// A form from the visible bodies of a TForm document (or a bare T-spline): turned from
    /// TForm's Y up to Z up.
    pub fn from_tform(text: &str) -> Result<Self, String> {
        let doc = Doc::parse(text).map_err(|e| e.to_string())?;
        let mut bodies: Vec<FormBody> = doc
            .bodies
            .iter()
            .filter(|(_, visible, _)| *visible)
            .map(|(name, _, ts)| FormBody {
                name: name.clone(),
                tspline: TSplineFile::from(&ts.map_points(cadrs_tspline::tform_to_z_up)),
            })
            .collect();
        if bodies.is_empty() {
            // every body hidden: take them all rather than nothing
            bodies = doc
                .bodies
                .iter()
                .map(|(name, _, ts)| FormBody {
                    name: name.clone(),
                    tspline: TSplineFile::from(&ts.map_points(cadrs_tspline::tform_to_z_up)),
                })
                .collect();
        }
        Ok(TSplineFormFeature { bodies })
    }

    /// The form as a TForm document (Y up), to keep editing it there.
    pub fn to_tform(&self) -> Result<String, String> {
        let bodies = self
            .splines()?
            .into_iter()
            .map(|(name, ts)| (name, true, ts.map_points(cadrs_tspline::z_up_to_tform)))
            .collect();
        Ok(Doc { bodies }.to_json())
    }

    /// A form of one of TForm's primitives (`"plane"`, `"cylinder"`, `"torus"`) at its default
    /// size, Z up.
    pub fn primitive(kind: &str) -> Option<Self> {
        let (name, ts) = match kind {
            "plane" => ("Plane", cadrs_tspline::create_plane(100.0, 100.0, 4, 4)),
            "cylinder" => ("Cylinder", cadrs_tspline::create_cylinder(30.0, 80.0, 8, 4)),
            "torus" => ("Torus", cadrs_tspline::create_torus(40.0, 14.0, 12, 8)),
            _ => return None,
        };
        Some(TSplineFormFeature {
            bodies: vec![FormBody {
                name: name.into(),
                tspline: TSplineFile::from(&ts.map_points(cadrs_tspline::tform_to_z_up)),
            }],
        })
    }
}

/// The file names a T-spline Form reads (TForm's documents).
pub fn is_tform_file(name: &str) -> bool {
    let lower = name.to_ascii_lowercase();
    lower.ends_with(".tform.json") || lower.ends_with(".tform")
}

/// Inserts a T-spline Form made from a TForm document's text.
#[derive(Debug, Clone)]
pub struct AddTSplineForm {
    pub element: ElementId,
    pub feature: FeatureId,
    /// A `.tform.json` file's text.
    pub text: String,
}

impl Command for AddTSplineForm {
    fn label(&self) -> String {
        "Insert T-spline form".into()
    }
    fn scope(&self) -> Scope {
        Scope::Element(self.element)
    }
    fn apply(&self, doc: &mut Document) -> Result<(), CommandError> {
        let x = TSplineFormFeature::from_tform(&self.text).map_err(CommandError::Invalid)?;
        crate::commands::AddFeature {
            element: self.element,
            feature: self.feature,
            base_name: "Form".into(),
            kind: FeatureKind::TSplineForm(x),
        }
        .apply(doc)
    }
}

/// The kernel's B-spline surface of a T-spline's NURBS conversion (rows along `u`).
#[cfg(feature = "brep")]
pub fn kernel_surface(n: &cadrs_tspline::Nurbs) -> cadrs_kernel::BsplineSurface {
    use nalgebra::Point3;
    let poles = (0..n.nu)
        .map(|i| {
            (0..n.nv)
                .map(|j| {
                    let c = n.cp[j * n.nu + i];
                    Point3::new(c[0], c[1], c[2])
                })
                .collect()
        })
        .collect();
    let weights = if n.is_polynomial() {
        Vec::new()
    } else {
        (0..n.nu)
            .map(|i| (0..n.nv).map(|j| n.cp[j * n.nu + i][3]).collect())
            .collect()
    };
    cadrs_kernel::BsplineSurface {
        degree_u: cadrs_tspline::DEGREE,
        degree_v: cadrs_tspline::DEGREE,
        knots_u: n.u.clone(),
        knots_v: n.v.clone(),
        poles,
        weights,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_tform_document_turns_z_up_and_back() {
        let ts = cadrs_tspline::create_cylinder(30.0, 80.0, 8, 4);
        let doc = Doc {
            bodies: vec![("Tube".into(), true, ts.clone())],
        }
        .to_json();
        let x = TSplineFormFeature::from_tform(&doc).unwrap();
        assert!(x.problem().is_none());
        let (_, z_up) = &x.splines().unwrap()[0];
        // TForm's cylinder runs up +Y: here it runs up +Z
        let top = z_up.verts.iter().map(|v| v.p[2]).fold(f64::MIN, f64::max);
        assert!((top - 80.0).abs() < 1e-9);
        let back = Doc::parse(&x.to_tform().unwrap()).unwrap();
        for (a, b) in back.bodies[0].2.verts.iter().zip(&ts.verts) {
            for k in 0..3 {
                assert!((a.p[k] - b.p[k]).abs() < 1e-12);
            }
        }
        assert!(
            TSplineFormFeature::primitive("torus")
                .unwrap()
                .problem()
                .is_none()
        );
        assert!(TSplineFormFeature::default().problem().is_some());
    }
}
