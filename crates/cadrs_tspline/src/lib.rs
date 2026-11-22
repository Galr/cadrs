//! Cubic T-spline surfaces on a T-mesh (Sederberg, Zheng, Bakenov and Nasri 2003; Sederberg,
//! Cardon, Finnigan, North, Zheng and Lyche 2004), for the Form feature.
//!
//! - A [`TSpline`] is a T-mesh in parameter space: control points ([`Vert`]) at `(s, t)` with a
//!   position and a weight, and axis-aligned edges, vertical ([`VSeg`], constant `s`) and
//!   horizontal ([`HSeg`], constant `t`). A row or column may stop at a T-junction.
//! - Each control point's local knot vectors come from casting rays in the T-mesh
//!   ([`TSpline::prepare`]); the surface is `Σ w B P / Σ w B`.
//! - The outer ring of a direction that is not periodic has a zero knot interval, which clamps
//!   the surface at its boundary like a NURBS edge.
//! - [`TSpline::to_nurbs`] converts it exactly to one cubic NURBS surface whose knot vectors hold
//!   every local knot.
//! - The file format is TForm's (`tform-tspline` inside a `tform-doc`, `.tform.json`), so a
//!   model made in TForm opens here unchanged ([`Doc`]).
//!
//! The parameter space is TForm's "index space": a plane made with `nu` columns spans
//! `s0 = 0 .. s1 = nu`, and the surface is defined on `[s0 + 1, s1 - 1]` (all of `[s0, s1)` when
//! periodic).

use serde::{Deserialize, Serialize};

pub mod edit;
pub mod primitives;

pub use primitives::{create_cylinder, create_plane, create_torus};

/// Tolerance for comparing parameter values.
pub const EPS: f64 = 1e-9;

fn near(a: f64, b: f64) -> bool {
    (a - b).abs() < EPS
}

/// Errors reading or converting a model.
#[derive(Debug, Clone, PartialEq)]
pub enum Error {
    /// Not a TForm document or T-spline.
    Format(String),
    /// The model has no control points, or its blending functions are degenerate.
    Degenerate(String),
}

impl std::fmt::Display for Error {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Error::Format(m) => write!(f, "not a TForm model: {m}"),
            Error::Degenerate(m) => write!(f, "the T-spline cannot be evaluated: {m}"),
        }
    }
}

impl std::error::Error for Error {}

/// A control point: its place in the T-mesh, position and weight.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Vert {
    pub s: f64,
    pub t: f64,
    pub p: [f64; 3],
    pub w: f64,
}

/// A vertical edge: constant `s`, from `t = a` to `t = b`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct VSeg {
    pub s: f64,
    pub a: f64,
    pub b: f64,
}

/// A horizontal edge: constant `t`, from `s = a` to `s = b`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct HSeg {
    pub t: f64,
    pub a: f64,
    pub b: f64,
}

/// What a T-spline started as (TForm's primitives), kept for display and naming.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Kind {
    #[default]
    Plane,
    Cylinder,
    Torus,
    Box,
    Sphere,
    #[serde(other)]
    Other,
}

/// The two local knot vectors of a control point (five knots each).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LocalKnots {
    pub ks: [f64; 5],
    pub kt: [f64; 5],
}

/// A cubic T-spline surface.
#[derive(Debug, Clone, PartialEq)]
pub struct TSpline {
    pub verts: Vec<Vert>,
    pub vsegs: Vec<VSeg>,
    pub hsegs: Vec<HSeg>,
    pub s0: f64,
    pub s1: f64,
    pub t0: f64,
    pub t1: f64,
    /// Periodic (closed) in `s`.
    pub per_s: bool,
    /// Periodic (closed) in `t`.
    pub per_t: bool,
    pub kind: Kind,
    knots: Option<Vec<LocalKnots>>,
}

/// A cubic rational B-spline surface, `u` along `s` and `v` along `t`.
#[derive(Debug, Clone, PartialEq)]
pub struct Nurbs {
    /// Clamped knot vector in `u` (`nu + 4` values).
    pub u: Vec<f64>,
    /// Clamped knot vector in `v` (`nv + 4` values).
    pub v: Vec<f64>,
    pub nu: usize,
    pub nv: usize,
    /// Control points `[x, y, z, w]` (Euclidean position and weight), `u` fastest.
    pub cp: Vec<[f64; 4]>,
    /// The surface closes on itself in `u` (its first and last columns meet).
    pub closed_u: bool,
    /// The surface closes on itself in `v`.
    pub closed_v: bool,
}

/// The degree of every surface here.
pub const DEGREE: usize = 3;

impl Default for TSpline {
    fn default() -> Self {
        TSpline {
            verts: Vec::new(),
            vsegs: Vec::new(),
            hsegs: Vec::new(),
            s0: 0.0,
            s1: 1.0,
            t0: 0.0,
            t1: 1.0,
            per_s: false,
            per_t: false,
            kind: Kind::Plane,
            knots: None,
        }
    }
}

// ---------------------------------------------------------------------------------------------
// B-spline helpers

/// The cubic B-spline basis function with the five local knots `k`, at `x` (Cox–de Boor). The
/// last knot is included, so a clamped end evaluates.
pub fn basis5(k: &[f64; 5], x: f64) -> f64 {
    if x < k[0] || x > k[4] {
        return 0.0;
    }
    let mut n = [0.0; 4];
    for i in 0..4 {
        n[i] = if x >= k[i] && x < k[i + 1] { 1.0 } else { 0.0 };
    }
    if x == k[4] {
        for i in (0..4).rev() {
            if k[i] < k[i + 1] {
                n[i] = 1.0;
                break;
            }
        }
    }
    for p in 1..=3 {
        for i in 0..4 - p {
            let d1 = k[i + p] - k[i];
            let d2 = k[i + p + 1] - k[i + 1];
            let mut v = 0.0;
            if d1 > 0.0 {
                v += (x - k[i]) / d1 * n[i];
            }
            if d2 > 0.0 {
                v += (k[i + p + 1] - x) / d2 * n[i + 1];
            }
            n[i] = v;
        }
    }
    n[0]
}

/// The knot span of `x` in the clamped cubic knot vector `u` of `n` control points.
pub fn find_span(u: &[f64], n: usize, x: f64) -> usize {
    if x >= u[n] {
        return n - 1;
    }
    if x <= u[3] {
        return 3;
    }
    let (mut lo, mut hi) = (3, n);
    while hi - lo > 1 {
        let mid = (lo + hi) / 2;
        if x < u[mid] {
            hi = mid;
        } else {
            lo = mid;
        }
    }
    lo
}

/// The four non-zero cubic basis values at `x` in span `span` of the knot vector `u`.
pub fn basis_funs(u: &[f64], span: usize, x: f64) -> [f64; 4] {
    let mut n = [1.0, 0.0, 0.0, 0.0];
    let mut left = [0.0; 4];
    let mut right = [0.0; 4];
    for j in 1..=3 {
        left[j] = x - u[span + 1 - j];
        right[j] = u[span + j] - x;
        let mut saved = 0.0;
        for r in 0..j {
            let tmp = n[r] / (right[r + 1] + left[j - r]);
            n[r] = saved + right[r + 1] * tmp;
            saved = left[j - r] * tmp;
        }
        n[j] = saved;
    }
    n
}

/// Solves `A X = B` with partial pivoting; `B` has four columns (homogeneous points).
fn lu_solve(a: &[Vec<f64>], b: &[[f64; 4]]) -> Result<Vec<[f64; 4]>, Error> {
    let n = a.len();
    let mut a: Vec<Vec<f64>> = a.to_vec();
    let mut b: Vec<[f64; 4]> = b.to_vec();
    for c in 0..n {
        let mut p = c;
        let mut best = a[c][c].abs();
        for (r, row) in a.iter().enumerate().skip(c + 1) {
            let v = row[c].abs();
            if v > best {
                best = v;
                p = r;
            }
        }
        if best < 1e-300 {
            return Err(Error::Degenerate("singular collocation matrix".into()));
        }
        if p != c {
            a.swap(p, c);
            b.swap(p, c);
        }
        for r in c + 1..n {
            let f = a[r][c] / a[c][c];
            if f == 0.0 {
                continue;
            }
            for k in c..n {
                a[r][k] -= f * a[c][k];
            }
            for k in 0..4 {
                b[r][k] -= f * b[c][k];
            }
        }
    }
    for r in (0..n).rev() {
        for k in 0..4 {
            let mut v = b[r][k];
            for c in r + 1..n {
                v -= a[r][c] * b[c][k];
            }
            b[r][k] = v / a[r][r];
        }
    }
    Ok(b)
}

fn uniq_sorted(mut a: Vec<f64>) -> Vec<f64> {
    a.sort_by(f64::total_cmp);
    let mut out: Vec<f64> = Vec::with_capacity(a.len());
    for x in a {
        if out.last().is_none_or(|&l| (l - x).abs() > EPS) {
            out.push(x);
        }
    }
    out
}

fn wrap(x: f64, lo: f64, per: f64) -> f64 {
    ((x - lo) % per + per) % per + lo
}

// ---------------------------------------------------------------------------------------------
// The T-spline

impl TSpline {
    /// The period in `s` (`s1 - s0`).
    pub fn period_s(&self) -> f64 {
        self.s1 - self.s0
    }

    /// The period in `t` (`t1 - t0`).
    pub fn period_t(&self) -> f64 {
        self.t1 - self.t0
    }

    /// The surface's domain in `s`.
    pub fn domain_s(&self) -> (f64, f64) {
        if self.per_s {
            (self.s0, self.s1)
        } else {
            (self.s0 + 1.0, self.s1 - 1.0)
        }
    }

    /// The surface's domain in `t`.
    pub fn domain_t(&self) -> (f64, f64) {
        if self.per_t {
            (self.t0, self.t1)
        } else {
            (self.t0 + 1.0, self.t1 - 1.0)
        }
    }

    /// Marks the local knots as stale after the T-mesh changed (positions and weights alone
    /// don't need it).
    pub fn touch(&mut self) {
        self.knots = None;
    }

    /// `s` values of the vertical edges crossing height `t`.
    pub fn v_lines_at(&self, t: f64) -> Vec<f64> {
        self.vsegs
            .iter()
            .filter(|e| t >= e.a - EPS && t <= e.b + EPS)
            .map(|e| e.s)
            .collect()
    }

    /// `t` values of the horizontal edges crossing `s`.
    pub fn h_lines_at(&self, s: f64) -> Vec<f64> {
        self.hsegs
            .iter()
            .filter(|e| s >= e.a - EPS && s <= e.b + EPS)
            .map(|e| e.t)
            .collect()
    }

    /// A control point's local knot vector in one direction: `lines` are the crossing edges,
    /// `x` its own position.
    fn ray_knots(lines: &[f64], x: f64, lo: f64, hi: f64, periodic: bool) -> [f64; 5] {
        let per = hi - lo;
        let vals: Vec<f64> = lines
            .iter()
            .map(|&v| if periodic { wrap(v, lo, per) } else { v })
            .collect();
        let mut sorted = vals;
        sorted.sort_by(f64::total_cmp);
        sorted.dedup();
        let mut uniq: Vec<f64> = Vec::new();
        for v in sorted {
            if uniq.last().is_none_or(|&l| !near(l, v)) {
                uniq.push(v);
            }
        }
        if !periodic {
            let ups: Vec<f64> = uniq.iter().copied().filter(|&v| v > x + EPS).collect();
            let dns: Vec<f64> = uniq
                .iter()
                .rev()
                .copied()
                .filter(|&v| v < x - EPS)
                .collect();
            let u1 = ups.first().copied().unwrap_or(hi);
            let u2 = ups.get(1).copied().unwrap_or(hi);
            let d1 = dns.first().copied().unwrap_or(lo);
            let d2 = dns.get(1).copied().unwrap_or(lo);
            let m = |v: f64| v.max(lo + 1.0).min(hi - 1.0);
            return [m(d2), m(d1), m(x), m(u1), m(u2)];
        }
        let mut ext: Vec<f64> = Vec::with_capacity(uniq.len() * 5);
        for k in -2..=2 {
            for &v in &uniq {
                ext.push(v + k as f64 * per);
            }
        }
        ext.sort_by(f64::total_cmp);
        let ups: Vec<f64> = ext.iter().copied().filter(|&v| v > x + EPS).collect();
        let dns: Vec<f64> = ext.iter().rev().copied().filter(|&v| v < x - EPS).collect();
        let g = |v: &[f64], i: usize| v.get(i).copied().unwrap_or(x);
        [g(&dns, 1), g(&dns, 0), x, g(&ups, 0), g(&ups, 1)]
    }

    /// Computes every control point's local knots (cached until [`TSpline::touch`]).
    pub fn prepare(&mut self) {
        if self.knots.is_some() {
            return;
        }
        let k = self
            .verts
            .iter()
            .map(|v| LocalKnots {
                ks: Self::ray_knots(&self.v_lines_at(v.t), v.s, self.s0, self.s1, self.per_s),
                kt: Self::ray_knots(&self.h_lines_at(v.s), v.t, self.t0, self.t1, self.per_t),
            })
            .collect();
        self.knots = Some(k);
    }

    /// The local knots (after [`TSpline::prepare`]).
    pub fn local_knots(&mut self) -> &[LocalKnots] {
        self.prepare();
        self.knots.as_deref().unwrap_or(&[])
    }

    fn bs(k: &[f64; 5], x: f64, per: f64, periodic: bool) -> f64 {
        if !periodic {
            return basis5(k, x);
        }
        let mut best: f64 = 0.0;
        for m in -1..=1 {
            let xx = x + m as f64 * per;
            if xx >= k[0] && xx <= k[4] {
                best = best.max(basis5(k, xx));
            }
        }
        best
    }

    fn wrap_s(&self, s: f64) -> f64 {
        if self.per_s {
            wrap(s, self.s0, self.period_s())
        } else {
            s
        }
    }

    fn wrap_t(&self, t: f64) -> f64 {
        if self.per_t {
            wrap(t, self.t0, self.period_t())
        } else {
            t
        }
    }

    fn clamp_s(&self, s: f64) -> f64 {
        if self.per_s {
            return s;
        }
        let (a, b) = self.domain_s();
        s.max(a).min(b)
    }

    fn clamp_t(&self, t: f64) -> f64 {
        if self.per_t {
            return t;
        }
        let (a, b) = self.domain_t();
        t.max(a).min(b)
    }

    /// The homogeneous point `[X, Y, Z, W] = Σ w B (P, 1)` at `(s, t)`.
    pub fn eval_h(&mut self, s: f64, t: f64) -> [f64; 4] {
        self.prepare();
        let s = self.clamp_s(self.wrap_s(s));
        let t = self.clamp_t(self.wrap_t(t));
        let (ps, pt) = (self.period_s(), self.period_t());
        let mut h = [0.0; 4];
        let knots = self.knots.as_ref().expect("prepared");
        for (v, k) in self.verts.iter().zip(knots) {
            let bs = Self::bs(&k.ks, s, ps, self.per_s);
            if bs == 0.0 {
                continue;
            }
            let bt = Self::bs(&k.kt, t, pt, self.per_t);
            if bt == 0.0 {
                continue;
            }
            let c = v.w * bs * bt;
            h[0] += c * v.p[0];
            h[1] += c * v.p[1];
            h[2] += c * v.p[2];
            h[3] += c;
        }
        h
    }

    /// The surface point at `(s, t)`.
    pub fn eval(&mut self, s: f64, t: f64) -> [f64; 3] {
        let h = self.eval_h(s, t);
        [h[0] / h[3], h[1] / h[3], h[2] / h[3]]
    }

    /// The distinct parameter lines in `s` (edges and local knots) inside the domain.
    pub fn param_lines_s(&mut self) -> Vec<f64> {
        self.prepare();
        let (sa, sb) = self.domain_s();
        let mut set = vec![sa, sb];
        set.extend(self.vsegs.iter().map(|e| e.s));
        for k in self.knots.as_ref().expect("prepared") {
            set.extend(k.ks.iter().map(|&v| self.wrap_s(v)));
        }
        uniq_sorted(
            set.into_iter()
                .filter(|&v| v >= sa - EPS && v <= sb + EPS)
                .collect(),
        )
    }

    /// The distinct parameter lines in `t` inside the domain.
    pub fn param_lines_t(&mut self) -> Vec<f64> {
        self.prepare();
        let (ta, tb) = self.domain_t();
        let mut set = vec![ta, tb];
        set.extend(self.hsegs.iter().map(|e| e.t));
        for k in self.knots.as_ref().expect("prepared") {
            set.extend(k.kt.iter().map(|&v| self.wrap_t(v)));
        }
        uniq_sorted(
            set.into_iter()
                .filter(|&v| v >= ta - EPS && v <= tb + EPS)
                .collect(),
        )
    }

    /// Number of T-junctions inside the T-mesh (control points with three edges).
    pub fn t_junction_count(&self) -> usize {
        let edges = self.cage_edges();
        let mut deg = vec![0usize; self.verts.len()];
        for &(a, b) in &edges {
            deg[a] += 1;
            deg[b] += 1;
        }
        let on_boundary = |v: &Vert| {
            (!self.per_s && (near(v.s, self.s0) || near(v.s, self.s1)))
                || (!self.per_t && (near(v.t, self.t0) || near(v.t, self.t1)))
        };
        self.verts
            .iter()
            .enumerate()
            .filter(|(i, v)| !on_boundary(v) && deg[*i] == 3)
            .count()
    }

    /// The control cage's edges, as pairs of control point indices (each once).
    pub fn cage_edges(&self) -> Vec<(usize, usize)> {
        let mut pairs: Vec<(usize, usize)> = Vec::new();
        for e in &self.hsegs {
            let mut ids: Vec<usize> = (0..self.verts.len())
                .filter(|&i| {
                    let v = self.verts[i];
                    near(v.t, e.t) && v.s >= e.a - EPS && v.s <= e.b + EPS
                })
                .collect();
            ids.sort_by(|&x, &y| self.verts[x].s.total_cmp(&self.verts[y].s));
            if self.per_s
                && near(e.b, self.s1)
                && let Some(z) = self.vert_at(self.s0, e.t)
            {
                ids.push(z);
            }
            pairs.extend(ids.windows(2).map(|w| (w[0], w[1])));
        }
        for e in &self.vsegs {
            let mut ids: Vec<usize> = (0..self.verts.len())
                .filter(|&i| {
                    let v = self.verts[i];
                    near(v.s, e.s) && v.t >= e.a - EPS && v.t <= e.b + EPS
                })
                .collect();
            ids.sort_by(|&x, &y| self.verts[x].t.total_cmp(&self.verts[y].t));
            if self.per_t
                && near(e.b, self.t1)
                && let Some(z) = self.vert_at(e.s, self.t0)
            {
                ids.push(z);
            }
            pairs.extend(ids.windows(2).map(|w| (w[0], w[1])));
        }
        let mut seen = std::collections::HashSet::new();
        pairs
            .into_iter()
            .filter(|&(a, b)| a != b && seen.insert((a.min(b), a.max(b))))
            .collect()
    }

    /// Converts the T-spline exactly to one cubic NURBS surface.
    ///
    /// Every local knot belongs to the global knot vectors, so each blending function lies in
    /// the tensor-product spline space; the NURBS's homogeneous control points come from
    /// interpolating the surface at the Greville points (one collocation solve per direction).
    pub fn to_nurbs(&mut self) -> Result<Nurbs, Error> {
        if self.verts.is_empty() {
            return Err(Error::Degenerate("no control points".into()));
        }
        self.prepare();
        let (sa, sb) = self.domain_s();
        let (ta, tb) = self.domain_t();
        if sb - sa < EPS || tb - ta < EPS {
            return Err(Error::Degenerate("an empty parameter domain".into()));
        }
        let knots_from = |lines: &[f64], lo: f64, hi: f64| {
            let mut k = vec![lo; 4];
            k.extend(
                lines
                    .iter()
                    .copied()
                    .filter(|&v| v > lo + EPS && v < hi - EPS),
            );
            k.extend([hi; 4]);
            k
        };
        let u = knots_from(&self.param_lines_s(), sa, sb);
        let v = knots_from(&self.param_lines_t(), ta, tb);
        let (nu, nv) = (u.len() - 4, v.len() - 4);
        let greville = |k: &[f64], n: usize| -> Vec<f64> {
            (0..n)
                .map(|i| (k[i + 1] + k[i + 2] + k[i + 3]) / 3.0)
                .collect()
        };
        let (gu, gv) = (greville(&u, nu), greville(&v, nv));
        let colloc = |k: &[f64], n: usize, g: &[f64]| -> Vec<Vec<f64>> {
            g.iter()
                .map(|&x| {
                    let mut row = vec![0.0; n];
                    let sp = find_span(k, n, x);
                    let b = basis_funs(k, sp, x);
                    for (j, bj) in b.iter().enumerate() {
                        row[sp - 3 + j] = *bj;
                    }
                    row
                })
                .collect()
        };
        let (au, av) = (colloc(&u, nu, &gu), colloc(&v, nv, &gv));
        // limits from inside at a periodic seam
        let (per_s, per_t, s1, t1) = (self.per_s, self.per_t, self.s1, self.t1);
        let in_s = |x: f64| {
            if per_s && x >= s1 - EPS {
                s1 - 1e-12
            } else {
                x
            }
        };
        let in_t = |x: f64| {
            if per_t && x >= t1 - EPS {
                t1 - 1e-12
            } else {
                x
            }
        };
        let mut c1: Vec<Vec<[f64; 4]>> = Vec::with_capacity(nv);
        for &y in &gv {
            let row: Vec<[f64; 4]> = gu.iter().map(|&x| self.eval_h(in_s(x), in_t(y))).collect();
            c1.push(lu_solve(&au, &row)?);
        }
        let mut c = vec![[0.0; 4]; nu * nv];
        for i in 0..nu {
            let column: Vec<[f64; 4]> = c1.iter().map(|r| r[i]).collect();
            let col = lu_solve(&av, &column)?;
            for (j, h) in col.into_iter().enumerate() {
                c[j * nu + i] = h;
            }
        }
        let mut cp = Vec::with_capacity(c.len());
        for h in c {
            if h[3].abs() < 1e-300 {
                return Err(Error::Degenerate("a zero weight".into()));
            }
            cp.push([h[0] / h[3], h[1] / h[3], h[2] / h[3], h[3]]);
        }
        Ok(Nurbs {
            u,
            v,
            nu,
            nv,
            cp,
            closed_u: self.per_s,
            closed_v: self.per_t,
        })
    }

    /// The same surface with every control point moved by `f` (positions only).
    pub fn map_points(&self, f: impl Fn([f64; 3]) -> [f64; 3]) -> TSpline {
        let mut out = self.clone();
        for v in &mut out.verts {
            v.p = f(v.p);
        }
        out
    }
}

impl Nurbs {
    /// The surface point at `(u, v)`.
    pub fn eval(&self, u: f64, v: f64) -> [f64; 3] {
        let su = find_span(&self.u, self.nu, u);
        let sv = find_span(&self.v, self.nv, v);
        let bu = basis_funs(&self.u, su, u);
        let bv = basis_funs(&self.v, sv, v);
        let mut h = [0.0; 4];
        for (b, nvb) in bv.iter().enumerate() {
            for (a, nua) in bu.iter().enumerate() {
                let c = self.cp[(sv - 3 + b) * self.nu + (su - 3 + a)];
                let f = nua * nvb * c[3];
                h[0] += f * c[0];
                h[1] += f * c[1];
                h[2] += f * c[2];
                h[3] += f;
            }
        }
        [h[0] / h[3], h[1] / h[3], h[2] / h[3]]
    }

    /// All weights are 1 (within 1e-12): a polynomial B-spline.
    pub fn is_polynomial(&self) -> bool {
        self.cp.iter().all(|c| (c[3] - 1.0).abs() < 1e-12)
    }
}

// ---------------------------------------------------------------------------------------------
// TForm's file format

/// A T-spline as TForm writes it (`format: "tform-tspline"`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct TSplineFile {
    pub format: String,
    #[serde(default = "one")]
    pub version: u32,
    #[serde(default)]
    pub kind: Kind,
    pub s0: f64,
    pub s1: f64,
    pub t0: f64,
    pub t1: f64,
    #[serde(default)]
    pub per_s: bool,
    #[serde(default)]
    pub per_t: bool,
    /// `[s, t, x, y, z, w]` each.
    pub verts: Vec<[f64; 6]>,
    /// `[s, a, b]` each.
    pub vsegs: Vec<[f64; 3]>,
    /// `[t, a, b]` each.
    pub hsegs: Vec<[f64; 3]>,
}

fn one() -> u32 {
    1
}

/// The name of a T-spline in TForm's files.
pub const TSPLINE_FORMAT: &str = "tform-tspline";
/// The name of a TForm document.
pub const DOC_FORMAT: &str = "tform-doc";

impl From<&TSpline> for TSplineFile {
    fn from(ts: &TSpline) -> Self {
        TSplineFile {
            format: TSPLINE_FORMAT.into(),
            version: 1,
            kind: ts.kind,
            s0: ts.s0,
            s1: ts.s1,
            t0: ts.t0,
            t1: ts.t1,
            per_s: ts.per_s,
            per_t: ts.per_t,
            verts: ts
                .verts
                .iter()
                .map(|v| [v.s, v.t, v.p[0], v.p[1], v.p[2], v.w])
                .collect(),
            vsegs: ts.vsegs.iter().map(|e| [e.s, e.a, e.b]).collect(),
            hsegs: ts.hsegs.iter().map(|e| [e.t, e.a, e.b]).collect(),
        }
    }
}

impl TryFrom<&TSplineFile> for TSpline {
    type Error = Error;
    fn try_from(f: &TSplineFile) -> Result<Self, Error> {
        if f.format != TSPLINE_FORMAT {
            return Err(Error::Format(format!("format {:?}", f.format)));
        }
        let ts = TSpline {
            verts: f
                .verts
                .iter()
                .map(|a| Vert {
                    s: a[0],
                    t: a[1],
                    p: [a[2], a[3], a[4]],
                    w: a[5],
                })
                .collect(),
            vsegs: f
                .vsegs
                .iter()
                .map(|a| VSeg {
                    s: a[0],
                    a: a[1],
                    b: a[2],
                })
                .collect(),
            hsegs: f
                .hsegs
                .iter()
                .map(|a| HSeg {
                    t: a[0],
                    a: a[1],
                    b: a[2],
                })
                .collect(),
            s0: f.s0,
            s1: f.s1,
            t0: f.t0,
            t1: f.t1,
            per_s: f.per_s,
            per_t: f.per_t,
            kind: f.kind,
            knots: None,
        };
        ts.validate()?;
        Ok(ts)
    }
}

impl TSpline {
    /// Checks the numbers a file can get wrong.
    pub fn validate(&self) -> Result<(), Error> {
        let finite = |x: f64| x.is_finite();
        if self.verts.is_empty() {
            return Err(Error::Format("no control points".into()));
        }
        if ![self.s0, self.s1, self.t0, self.t1].into_iter().all(finite)
            || self.s1 <= self.s0
            || self.t1 <= self.t0
        {
            return Err(Error::Format("a bad parameter range".into()));
        }
        for v in &self.verts {
            if !(finite(v.s) && finite(v.t) && v.p.iter().all(|&x| finite(x)) && v.w.is_finite())
                || v.w <= 0.0
            {
                return Err(Error::Format(
                    "a control point that isn't a finite point with a positive weight".into(),
                ));
            }
        }
        if self.verts.len() > 200_000 {
            return Err(Error::Format("too many control points".into()));
        }
        Ok(())
    }

    /// Reads a `tform-tspline` JSON value.
    pub fn from_json(text: &str) -> Result<TSpline, Error> {
        let f: TSplineFile =
            serde_json::from_str(text).map_err(|e| Error::Format(e.to_string()))?;
        TSpline::try_from(&f)
    }

    /// Writes a `tform-tspline` JSON value.
    pub fn to_json(&self) -> String {
        serde_json::to_string(&TSplineFile::from(self)).expect("serializable")
    }
}

/// One body of a TForm document.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct DocBody {
    #[serde(default)]
    pub name: String,
    #[serde(default = "yes")]
    pub visible: bool,
    pub ts: TSplineFile,
}

fn yes() -> bool {
    true
}

/// A TForm document (`.tform.json`, `format: "tform-doc"`): its T-spline bodies. A bare
/// `tform-tspline` reads as a document of one body.
#[derive(Debug, Clone, PartialEq)]
pub struct Doc {
    pub bodies: Vec<(String, bool, TSpline)>,
}

impl Doc {
    /// Reads a `.tform.json` file's text.
    pub fn parse(text: &str) -> Result<Doc, Error> {
        let value: serde_json::Value =
            serde_json::from_str(text).map_err(|e| Error::Format(e.to_string()))?;
        match value.get("format").and_then(|f| f.as_str()) {
            Some(DOC_FORMAT) => {
                let bodies = value
                    .get("bodies")
                    .and_then(|b| b.as_array())
                    .ok_or_else(|| Error::Format("a document without bodies".into()))?;
                let mut out = Vec::new();
                for (i, b) in bodies.iter().enumerate() {
                    // bodies TForm converted to plain NURBS keep their T-spline too; skip any
                    // body without one
                    if b.get("ts").is_none() {
                        continue;
                    }
                    let body: DocBody = serde_json::from_value(b.clone())
                        .map_err(|e| Error::Format(format!("body {}: {e}", i + 1)))?;
                    let name = if body.name.is_empty() {
                        format!("Form {}", i + 1)
                    } else {
                        body.name
                    };
                    out.push((name, body.visible, TSpline::try_from(&body.ts)?));
                }
                if out.is_empty() {
                    return Err(Error::Format("a document without T-spline bodies".into()));
                }
                Ok(Doc { bodies: out })
            }
            Some(TSPLINE_FORMAT) => Ok(Doc {
                bodies: vec![("Form 1".into(), true, TSpline::from_json(text)?)],
            }),
            other => Err(Error::Format(format!("format {other:?}"))),
        }
    }

    /// Writes a `tform-doc` that TForm opens.
    pub fn to_json(&self) -> String {
        let bodies: Vec<serde_json::Value> = self
            .bodies
            .iter()
            .map(|(name, visible, ts)| {
                serde_json::json!({ "name": name, "visible": visible, "ts": TSplineFile::from(ts) })
            })
            .collect();
        serde_json::json!({ "format": DOC_FORMAT, "version": 1, "active": 0, "bodies": bodies })
            .to_string()
    }
}

/// TForm is Y-up and cadrs is Z-up: TForm `(x, y, z)` is cadrs `(x, -z, y)`.
pub fn tform_to_z_up(p: [f64; 3]) -> [f64; 3] {
    [p[0], -p[2], p[1]]
}

/// cadrs `(x, y, z)` is TForm `(x, z, -y)`.
pub fn z_up_to_tform(p: [f64; 3]) -> [f64; 3] {
    [p[0], p[2], -p[1]]
}
