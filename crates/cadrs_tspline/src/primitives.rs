//! TForm's primitives, in TForm's Y-up frame: a plane, a cylinder (closed in `s`) and a torus
//! (closed in both directions). Their control points sit so the surface matches the named size.

use std::f64::consts::PI;

use crate::{HSeg, Kind, TSpline, VSeg, Vert};

fn grid(ts: &mut TSpline, ns: usize, nt: usize) {
    ts.s0 = 0.0;
    ts.s1 = ns as f64;
    ts.t0 = 0.0;
    ts.t1 = nt as f64;
    let i_max = if ts.per_s { ns - 1 } else { ns };
    let j_max = if ts.per_t { nt - 1 } else { nt };
    for j in 0..=j_max {
        for i in 0..=i_max {
            ts.verts.push(Vert {
                s: i as f64,
                t: j as f64,
                p: [0.0; 3],
                w: 1.0,
            });
        }
    }
    for i in 0..=i_max {
        ts.vsegs.push(VSeg {
            s: i as f64,
            a: 0.0,
            b: nt as f64,
        });
    }
    for j in 0..=j_max {
        ts.hsegs.push(HSeg {
            t: j as f64,
            a: 0.0,
            b: ns as f64,
        });
    }
}

/// How far a periodic cubic B-spline's control polygon must reach out for the curve to pass
/// through a circle of radius 1, with `n` control points.
fn shrink(n: usize) -> f64 {
    3.0 / (2.0 + (2.0 * PI / n as f64).cos())
}

/// A flat sheet `width × depth` in the XZ plane, `nu × nv` faces.
pub fn create_plane(width: f64, depth: f64, nu: usize, nv: usize) -> TSpline {
    let mut ts = TSpline {
        kind: Kind::Plane,
        ..TSpline::default()
    };
    grid(&mut ts, nu.max(2), nv.max(2));
    let (nu, nv) = (ts.s1, ts.t1);
    for v in &mut ts.verts {
        v.p = [(v.s / nu - 0.5) * width, 0.0, (v.t / nv - 0.5) * depth];
    }
    ts
}

/// An open tube of `radius` and `height` along Y, `nu` faces round and `nv` up.
pub fn create_cylinder(radius: f64, height: f64, nu: usize, nv: usize) -> TSpline {
    let mut ts = TSpline {
        kind: Kind::Cylinder,
        per_s: true,
        ..TSpline::default()
    };
    let nu = nu.max(3);
    grid(&mut ts, nu, nv.max(1));
    let r = radius * shrink(nu);
    let nv = ts.t1;
    for v in &mut ts.verts {
        let a = 2.0 * PI * v.s / nu as f64;
        v.p = [r * a.cos(), (v.t / nv) * height, -r * a.sin()];
    }
    ts
}

/// A torus round Y with `major` and `minor` radii, `nu × nv` faces.
pub fn create_torus(major: f64, minor: f64, nu: usize, nv: usize) -> TSpline {
    let mut ts = TSpline {
        kind: Kind::Torus,
        per_s: true,
        per_t: true,
        ..TSpline::default()
    };
    let (nu, nv) = (nu.max(3), nv.max(3));
    grid(&mut ts, nu, nv);
    let r = minor * shrink(nv);
    let k = shrink(nu);
    for v in &mut ts.verts {
        let a = 2.0 * PI * v.s / nu as f64;
        let b = 2.0 * PI * v.t / nv as f64;
        let rr = (major + r * b.cos()) * k;
        v.p = [rr * a.cos(), r * b.sin(), -rr * a.sin()];
    }
    ts
}
