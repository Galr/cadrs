//! The port against TForm's own engine: `reference.json` holds models, surface points and NURBS
//! conversions that TForm's JavaScript engine produced (see `tests/README.md`).

use cadrs_tspline::edit::Dir;
use cadrs_tspline::{Doc, TSpline, create_cylinder, create_plane, create_torus};
use serde_json::Value;

fn cases() -> Vec<Value> {
    serde_json::from_str(include_str!("reference.json")).unwrap()
}

fn case(name: &str) -> Value {
    cases()
        .into_iter()
        .find(|c| c["name"] == name)
        .unwrap_or_else(|| panic!("no case {name}"))
}

fn model(c: &Value) -> TSpline {
    TSpline::from_json(&c["model"].to_string()).unwrap()
}

fn close(a: f64, b: f64, tol: f64) -> bool {
    (a - b).abs() <= tol * (1.0 + a.abs().max(b.abs()))
}

#[test]
fn surface_points_match_tform() {
    for c in cases() {
        let mut ts = model(&c);
        for p in c["pts"].as_array().unwrap() {
            let p: Vec<f64> = p
                .as_array()
                .unwrap()
                .iter()
                .map(|x| x.as_f64().unwrap())
                .collect();
            let q = ts.eval(p[0], p[1]);
            for k in 0..3 {
                assert!(
                    close(q[k], p[2 + k], 1e-9),
                    "{}: at ({}, {}) {q:?} vs {:?}",
                    c["name"],
                    p[0],
                    p[1],
                    &p[2..]
                );
            }
        }
    }
}

#[test]
fn nurbs_conversion_matches_tform() {
    for c in cases() {
        let mut ts = model(&c);
        let n = ts.to_nurbs().unwrap();
        let r = &c["nurbs"];
        assert_eq!(n.nu as u64, r["nu"].as_u64().unwrap(), "{}", c["name"]);
        assert_eq!(n.nv as u64, r["nv"].as_u64().unwrap(), "{}", c["name"]);
        assert_eq!(n.closed_u, r["closedU"].as_bool().unwrap());
        assert_eq!(n.closed_v, r["closedV"].as_bool().unwrap());
        for (k, rk) in n.u.iter().zip(r["U"].as_array().unwrap()) {
            assert!(close(*k, rk.as_f64().unwrap(), 1e-12), "{} U", c["name"]);
        }
        for (k, rk) in n.v.iter().zip(r["V"].as_array().unwrap()) {
            assert!(close(*k, rk.as_f64().unwrap(), 1e-12), "{} V", c["name"]);
        }
        for (cp, rc) in n.cp.iter().zip(r["cp"].as_array().unwrap()) {
            for k in 0..4 {
                let want = rc[k].as_f64().unwrap();
                assert!(close(cp[k], want, 1e-8), "{}: cp {cp:?} vs {rc}", c["name"]);
            }
        }
    }
}

#[test]
fn the_nurbs_is_the_same_surface() {
    for c in cases() {
        let mut ts = model(&c);
        let n = ts.to_nurbs().unwrap();
        let (sa, sb) = ts.domain_s();
        let (ta, tb) = ts.domain_t();
        let scale = ts
            .verts
            .iter()
            .flat_map(|v| v.p)
            .fold(1.0f64, |m, x| m.max(x.abs()));
        for i in 0..=20 {
            for j in 0..=20 {
                // inside the seam: the NURBS's last column is the T-spline's limit from inside
                let s = sa + (sb - sa) * (i as f64 / 20.0).min(1.0 - 1e-9);
                let t = ta + (tb - ta) * (j as f64 / 20.0).min(1.0 - 1e-9);
                let a = ts.eval(s, t);
                let b = n.eval(s, t);
                let d =
                    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt();
                assert!(d < 1e-9 * scale, "{}: {d} at ({s}, {t})", c["name"]);
            }
        }
    }
}

fn same_topology(a: &TSpline, b: &TSpline, name: &str) {
    let key = |ts: &TSpline| {
        let mut v: Vec<(i64, i64)> = ts
            .verts
            .iter()
            .map(|v| ((v.s * 1e6).round() as i64, (v.t * 1e6).round() as i64))
            .collect();
        v.sort_unstable();
        let mut vs: Vec<[i64; 3]> = ts
            .vsegs
            .iter()
            .map(|e| [e.s, e.a, e.b].map(|x| (x * 1e6).round() as i64))
            .collect();
        vs.sort_unstable();
        let mut hs: Vec<[i64; 3]> = ts
            .hsegs
            .iter()
            .map(|e| [e.t, e.a, e.b].map(|x| (x * 1e6).round() as i64))
            .collect();
        hs.sort_unstable();
        (v, vs, hs)
    };
    assert_eq!(key(a), key(b), "{name}: the T-mesh differs");
}

fn same_points(a: &TSpline, b: &TSpline, tol: f64, name: &str) {
    for v in &a.verts {
        let w = b
            .verts
            .iter()
            .find(|w| (w.s - v.s).abs() < 1e-9 && (w.t - v.t).abs() < 1e-9)
            .unwrap();
        for k in 0..3 {
            assert!(
                close(v.p[k], w.p[k], tol),
                "{name}: point at ({}, {}) {:?} vs {:?}",
                v.s,
                v.t,
                v.p,
                w.p
            );
        }
        assert!(close(v.w, w.w, tol), "{name}: weight");
    }
}

#[test]
fn primitives_match_tform() {
    for (name, ts) in [
        ("plane", create_plane(100.0, 100.0, 4, 4)),
        ("cylinder", create_cylinder(30.0, 80.0, 8, 4)),
        ("torus", create_torus(40.0, 14.0, 12, 8)),
    ] {
        let r = model(&case(name));
        same_topology(&ts, &r, name);
        same_points(&ts, &r, 1e-12, name);
        assert_eq!(ts.per_s, r.per_s);
        assert_eq!(ts.per_t, r.per_t);
    }
}

#[test]
fn subdivide_matches_tform() {
    let mut p = create_plane(100.0, 100.0, 4, 4);
    let f = p.face_at(1.5, 1.5).unwrap();
    p.subdivide_face(f, true).unwrap();
    let r = model(&case("plane-subdiv-raw"));
    same_topology(&p, &r, "plane-subdiv");
    same_points(&p, &r, 1e-7, "plane-subdiv");

    let mut t = create_torus(40.0, 14.0, 12, 8);
    let f = t.face_at(0.5, 7.5).unwrap();
    t.subdivide_face(f, true).unwrap();
    let r = model(&case("torus-subdiv-seam-raw"));
    same_topology(&t, &r, "torus-subdiv-seam");
    same_points(&t, &r, 1e-7, "torus-subdiv-seam");
}

#[test]
fn insert_loop_matches_tform() {
    let mut c = create_cylinder(30.0, 80.0, 8, 4);
    let f = c.face_at(2.5, 1.5).unwrap();
    c.insert_loop(f, Dir::S, true).unwrap();
    let r = model(&case("cylinder-loop-raw"));
    same_topology(&c, &r, "cylinder-loop");
    same_points(&c, &r, 1e-7, "cylinder-loop");

    let mut w = model(&case("plane-weight"));
    let f = w.face_at(1.5, 1.5).unwrap();
    w.insert_loop(f, Dir::T, true).unwrap();
    let r = model(&case("plane-weight-loop-raw"));
    same_topology(&w, &r, "plane-weight-loop");
    same_points(&w, &r, 1e-7, "plane-weight-loop");
}

#[test]
fn refinement_keeps_the_shape() {
    let mut a = model(&case("plane-subdiv"));
    let before = a.clone();
    let f = a.face_at(2.5, 2.5).unwrap();
    a.subdivide_face(f, true).unwrap();
    let mut before = before;
    for i in 0..=10 {
        for j in 0..=10 {
            let (s, t) = (1.0 + 2.0 * i as f64 / 10.0, 1.0 + 2.0 * j as f64 / 10.0);
            let (p, q) = (before.eval(s, t), a.eval(s, t));
            let d = ((p[0] - q[0]).powi(2) + (p[1] - q[1]).powi(2) + (p[2] - q[2]).powi(2)).sqrt();
            assert!(d < 1e-6, "moved {d} at ({s}, {t})");
        }
    }
}

#[test]
fn documents_round_trip() {
    let ts = create_torus(40.0, 14.0, 12, 8);
    let doc = Doc {
        bodies: vec![("Ring".into(), true, ts.clone())],
    };
    let back = Doc::parse(&doc.to_json()).unwrap();
    assert_eq!(back.bodies.len(), 1);
    assert_eq!(back.bodies[0].0, "Ring");
    same_topology(&back.bodies[0].2, &ts, "round trip");
    // a bare T-spline reads as a document of one body
    assert_eq!(Doc::parse(&ts.to_json()).unwrap().bodies.len(), 1);
    assert!(Doc::parse("{\"format\":\"something\"}").is_err());
    assert!(TSpline::from_json("{\"format\":\"tform-tspline\",\"s0\":0,\"s1\":1,\"t0\":0,\"t1\":1,\"verts\":[],\"vsegs\":[],\"hsegs\":[]}").is_err());
}
