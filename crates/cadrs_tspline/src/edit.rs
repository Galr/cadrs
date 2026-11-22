//! Editing the T-mesh: faces, edge loops, and local refinement (Subdivide, Insert Edge) that
//! keeps the surface's shape. A port of TForm's editing operations.

use crate::{EPS, Error, Kind, TSpline, Vert, lu_solve, near, uniq_sorted, wrap};

/// A face of the T-mesh: the parameter rectangle `[l, r] × [b, t]`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Face {
    pub l: f64,
    pub r: f64,
    pub b: f64,
    pub t: f64,
}

/// Which way a new line runs: `S` adds a vertical line (constant `s`), `T` a horizontal one.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Dir {
    S,
    T,
}

impl TSpline {
    pub(crate) fn wrap_s_pub(&self, s: f64) -> f64 {
        if self.per_s {
            wrap(s, self.s0, self.period_s())
        } else {
            s
        }
    }

    pub(crate) fn wrap_t_pub(&self, t: f64) -> f64 {
        if self.per_t {
            wrap(t, self.t0, self.period_t())
        } else {
            t
        }
    }

    /// The control point at `(s, t)` (TForm's `findVert`: a periodic seam's far side is its
    /// near side).
    pub fn vert_at(&self, s: f64, t: f64) -> Option<usize> {
        let mut s = self.wrap_s_pub(s);
        let mut t = self.wrap_t_pub(t);
        if self.per_s && near(s, self.s1) {
            s = self.s0;
        }
        if self.per_t && near(t, self.t1) {
            t = self.t0;
        }
        self.verts.iter().position(|v| near(v.s, s) && near(v.t, t))
    }

    /// The face of the T-mesh containing `(s, t)`.
    pub fn face_at(&self, s: f64, t: f64) -> Option<Face> {
        let s = self.wrap_s_pub(s);
        let t = self.wrap_t_pub(t);
        let (mut l, mut r, mut b, mut tt) = (
            f64::NEG_INFINITY,
            f64::INFINITY,
            f64::NEG_INFINITY,
            f64::INFINITY,
        );
        for v in self.v_lines_at(t) {
            if v <= s + EPS && v > l {
                l = v;
            }
            if v > s + EPS && v < r {
                r = v;
            }
        }
        for v in self.h_lines_at(s) {
            if v <= t + EPS && v > b {
                b = v;
            }
            if v > t + EPS && v < tt {
                tt = v;
            }
        }
        if self.per_s && r == f64::INFINITY {
            r = self.s1;
        }
        if self.per_t && tt == f64::INFINITY {
            tt = self.t1;
        }
        if !(l.is_finite() && r.is_finite() && b.is_finite() && tt.is_finite()) || near(l, r) {
            return None;
        }
        Some(Face { l, r, b, t: tt })
    }

    /// A face of the zero-width outer ring (no area on the surface).
    pub fn is_ring_face(&self, f: &Face) -> bool {
        let (sa, sb) = self.domain_s();
        let (ta, tb) = self.domain_t();
        (!self.per_s && (f.r <= sa + EPS || f.l >= sb - EPS))
            || (!self.per_t && (f.t <= ta + EPS || f.b >= tb - EPS))
    }

    /// The control points on a face's boundary.
    pub fn face_verts(&self, f: &Face) -> Vec<usize> {
        let mut out = Vec::new();
        for (i, v) in self.verts.iter().enumerate() {
            let s = if self.per_s && near(v.s, self.s0) && near(f.r, self.s1) {
                self.s1
            } else {
                v.s
            };
            let t = if self.per_t && near(v.t, self.t0) && near(f.t, self.t1) {
                self.t1
            } else {
                v.t
            };
            let on_v = (near(s, f.l) || near(s, f.r)) && t >= f.b - EPS && t <= f.t + EPS;
            let on_h = (near(t, f.b) || near(t, f.t)) && s >= f.l - EPS && s <= f.r + EPS;
            if on_v || on_h {
                out.push(i);
            }
        }
        out
    }

    /// Every control point on the same connected parameter line as point `i` (an edge loop):
    /// `Dir::S` along `s` at its `t`, `Dir::T` along `t` at its `s`.
    pub fn line_verts(&self, i: usize, dir: Dir) -> Vec<usize> {
        let v = self.verts[i];
        match dir {
            Dir::S => {
                let segs: Vec<_> = self.hsegs.iter().filter(|e| near(e.t, v.t)).collect();
                (0..self.verts.len())
                    .filter(|&j| {
                        let u = self.verts[j];
                        near(u.t, v.t) && segs.iter().any(|e| u.s >= e.a - EPS && u.s <= e.b + EPS)
                    })
                    .collect()
            }
            Dir::T => {
                let segs: Vec<_> = self.vsegs.iter().filter(|e| near(e.s, v.s)).collect();
                (0..self.verts.len())
                    .filter(|&j| {
                        let u = self.verts[j];
                        near(u.s, v.s) && segs.iter().any(|e| u.t >= e.a - EPS && u.t <= e.b + EPS)
                    })
                    .collect()
            }
        }
    }

    /// Normalised blending coefficients at `(s, t)`: `(point, c)` with `Σ c = 1`.
    pub fn coeffs_at(&mut self, s: f64, t: f64) -> Vec<(usize, f64)> {
        let mut out = self.basis_at(s, t);
        let mut den = 0.0;
        for o in &mut out {
            o.1 *= self.verts[o.0].w;
            den += o.1;
        }
        for o in &mut out {
            o.1 /= den;
        }
        out
    }

    /// Raw blending functions (unweighted, not normalised) at `(s, t)`.
    pub fn basis_at(&mut self, s: f64, t: f64) -> Vec<(usize, f64)> {
        self.prepare();
        let (sa, sb) = self.domain_s();
        let (ta, tb) = self.domain_t();
        let mut s = self.wrap_s_pub(s);
        let mut t = self.wrap_t_pub(t);
        if !self.per_s {
            s = s.max(sa).min(sb);
        }
        if !self.per_t {
            t = t.max(ta).min(tb);
        }
        let (ps, pt) = (self.period_s(), self.period_t());
        let knots = self.local_knots().to_vec();
        let mut out = Vec::new();
        for (i, k) in knots.iter().enumerate() {
            let bs = Self::bs_pub(&k.ks, s, ps, self.per_s);
            if bs == 0.0 {
                continue;
            }
            let bt = Self::bs_pub(&k.kt, t, pt, self.per_t);
            if bt == 0.0 {
                continue;
            }
            out.push((i, bs * bt));
        }
        out
    }

    fn bs_pub(k: &[f64; 5], x: f64, per: f64, periodic: bool) -> f64 {
        if !periodic {
            return crate::basis5(k, x);
        }
        let mut best: f64 = 0.0;
        for m in -1..=1 {
            let xx = x + m as f64 * per;
            if xx >= k[0] && xx <= k[4] {
                best = best.max(crate::basis5(k, xx));
            }
        }
        best
    }

    fn guess_on_h_line(&self, s: f64, t: f64) -> Option<[f64; 3]> {
        let mut left: Option<(f64, [f64; 3])> = None;
        let mut right: Option<(f64, [f64; 3])> = None;
        for v in self.verts.iter().filter(|v| near(v.t, t)) {
            if v.s < s - EPS && left.is_none_or(|l| v.s > l.0) {
                left = Some((v.s, v.p));
            }
            if v.s > s + EPS && right.is_none_or(|r| v.s < r.0) {
                right = Some((v.s, v.p));
            }
        }
        if self.per_s {
            let mut row: Vec<&Vert> = self.verts.iter().filter(|v| near(v.t, t)).collect();
            row.sort_by(|a, b| a.s.total_cmp(&b.s));
            if right.is_none()
                && let Some(v0) = row.first()
            {
                right = Some((v0.s + self.period_s(), v0.p));
            }
            if left.is_none()
                && let Some(vl) = row.last()
            {
                left = Some((vl.s - self.period_s(), vl.p));
            }
        }
        let (l, r) = (left?, right?);
        let u = (s - l.0) / (r.0 - l.0);
        Some([0, 1, 2].map(|k| l.1[k] + u * (r.1[k] - l.1[k])))
    }

    fn guess_on_v_line(&self, s: f64, t: f64) -> Option<[f64; 3]> {
        let mut below: Option<(f64, [f64; 3])> = None;
        let mut above: Option<(f64, [f64; 3])> = None;
        for v in self.verts.iter().filter(|v| near(v.s, s)) {
            if v.t < t - EPS && below.is_none_or(|b| v.t > b.0) {
                below = Some((v.t, v.p));
            }
            if v.t > t + EPS && above.is_none_or(|a| v.t < a.0) {
                above = Some((v.t, v.p));
            }
        }
        if self.per_t {
            let mut col: Vec<&Vert> = self.verts.iter().filter(|v| near(v.s, s)).collect();
            col.sort_by(|a, b| a.t.total_cmp(&b.t));
            if above.is_none()
                && let Some(v0) = col.first()
            {
                above = Some((v0.t + self.period_t(), v0.p));
            }
            if below.is_none()
                && let Some(vl) = col.last()
            {
                below = Some((vl.t - self.period_t(), vl.p));
            }
        }
        let (b, a) = (below?, above?);
        let u = (t - b.0) / (a.0 - b.0);
        Some([0, 1, 2].map(|k| b.1[k] + u * (a.1[k] - b.1[k])))
    }

    fn add_vert(&mut self, old: &mut TSpline, s: f64, t: f64, guess: Option<[f64; 3]>) {
        if self.vert_at(s, t).is_some() {
            return;
        }
        let ws = self.wrap_s_pub(s);
        let wt = self.wrap_t_pub(t);
        let s = if self.per_s && near(ws, self.s1) {
            self.s0
        } else {
            ws
        };
        let t = if self.per_t && near(wt, self.t1) {
            self.t0
        } else {
            wt
        };
        let p = guess.unwrap_or_else(|| old.eval(s, t));
        self.verts.push(Vert { s, t, p, w: 1.0 });
    }

    fn split_range(a: f64, b: f64, lo: f64, hi: f64, periodic: bool) -> Vec<(f64, f64)> {
        if !periodic {
            return vec![(a.max(lo), b.min(hi))];
        }
        let per = hi - lo;
        if b - a >= per - EPS {
            return vec![(lo, hi)];
        }
        if a < lo - EPS {
            return vec![(a + per, hi), (lo, b)];
        }
        if b > hi + EPS {
            return vec![(a, hi), (lo, b - per)];
        }
        vec![(a, b)]
    }

    fn beyond(lines: &[f64], v: f64, dir: i32, lo: f64, hi: f64, periodic: bool) -> f64 {
        let per = hi - lo;
        let ext: Vec<f64> = if periodic {
            lines.iter().flat_map(|&x| [x - per, x, x + per]).collect()
        } else {
            lines.to_vec()
        };
        if dir < 0 {
            ext.into_iter()
                .filter(|&x| x < v - EPS)
                .fold(None, |m: Option<f64>, x| Some(m.map_or(x, |m| m.max(x))))
                .unwrap_or(lo)
        } else {
            ext.into_iter()
                .filter(|&x| x > v + EPS)
                .fold(None, |m: Option<f64>, x| Some(m.map_or(x, |m| m.min(x))))
                .unwrap_or(hi)
        }
    }

    /// Adds a vertical (`Dir::S`, at `s = x`, `t` from `a` to `b`) or horizontal line, with
    /// control points at every crossing.
    fn add_line(&mut self, old: &mut TSpline, dir: Dir, x: f64, a: f64, b: f64) {
        let (mut a, mut b) = (a, b);
        // an edge that stops on the first interior line lies on the boundary in knot space:
        // carry it through the zero-width ring
        match dir {
            Dir::S if !self.per_t => {
                if (a - (self.t0 + 1.0)).abs() < 1e-9 {
                    a = self.t0;
                }
                if (b - (self.t1 - 1.0)).abs() < 1e-9 {
                    b = self.t1;
                }
            }
            Dir::T if !self.per_s => {
                if (a - (self.s0 + 1.0)).abs() < 1e-9 {
                    a = self.s0;
                }
                if (b - (self.s1 - 1.0)).abs() < 1e-9 {
                    b = self.s1;
                }
            }
            _ => {}
        }
        match dir {
            Dir::S => {
                for (p, q) in Self::split_range(a, b, self.t0, self.t1, self.per_t) {
                    for t in uniq_sorted(self.h_lines_at(x)) {
                        if t >= p - 1e-9 && t <= q + 1e-9 {
                            let g = self.guess_on_h_line(x, t);
                            self.add_vert(old, x, t, g);
                        }
                    }
                    self.vsegs.push(crate::VSeg { s: x, a: p, b: q });
                }
            }
            Dir::T => {
                for (p, q) in Self::split_range(a, b, self.s0, self.s1, self.per_s) {
                    for s in uniq_sorted(self.v_lines_at(x)) {
                        if s >= p - 1e-9 && s <= q + 1e-9 {
                            let g = self.guess_on_v_line(s, x);
                            self.add_vert(old, s, x, g);
                        }
                    }
                    self.hsegs.push(crate::HSeg { t: x, a: p, b: q });
                }
            }
        }
    }

    /// Applies a structural edit, then refits the control points whose knots changed so the
    /// surface keeps its old shape (least squares on a fine sample of the affected region).
    /// Returns the affected control points.
    fn refine(
        &mut self,
        edit: impl FnOnce(&mut TSpline, &mut TSpline),
        exact: bool,
    ) -> Result<Vec<usize>, Error> {
        let mut old = self.clone();
        old.prepare();
        let old_knots: Vec<_> = self.local_knots().to_vec();
        let n_old = self.verts.len();
        edit(self, &mut old);
        self.touch();
        let knots = self.local_knots().to_vec();
        let affected: Vec<usize> = (0..self.verts.len())
            .filter(|&i| i >= n_old || knots[i] != old_knots[i])
            .collect();
        if !exact || affected.is_empty() {
            return Ok(affected);
        }
        let (mut sa, mut sb, mut ta, mut tb) = (
            f64::INFINITY,
            f64::NEG_INFINITY,
            f64::INFINITY,
            f64::NEG_INFINITY,
        );
        for &i in &affected {
            let k = knots[i];
            sa = sa.min(k.ks[0]);
            sb = sb.max(k.ks[4]);
            ta = ta.min(k.kt[0]);
            tb = tb.max(k.kt[4]);
        }
        let (da, db) = self.domain_s();
        let (ea, eb) = self.domain_t();
        if !self.per_s {
            sa = sa.max(da);
            sb = sb.min(db);
        } else if sb - sa > self.period_s() {
            sa = self.s0;
            sb = self.s1;
        }
        if !self.per_t {
            ta = ta.max(ea);
            tb = tb.min(eb);
        } else if tb - ta > self.period_t() {
            ta = self.t0;
            tb = self.t1;
        }
        let ls = self.param_lines_s();
        let lt = self.param_lines_t();
        let fine = |lines: &[f64], a: f64, b: f64, per: f64, periodic: bool| -> Vec<f64> {
            let ext: Vec<f64> = if periodic {
                lines.iter().flat_map(|&v| [v - per, v, v + per]).collect()
            } else {
                lines.to_vec()
            };
            let mut inside = uniq_sorted(
                ext.into_iter()
                    .filter(|&v| v >= a - EPS && v <= b + EPS)
                    .collect(),
            );
            if inside.first().is_none_or(|&f| f > a + EPS) {
                inside.insert(0, a);
            }
            if inside.last().is_some_and(|&l| l < b - EPS) {
                inside.push(b);
            }
            let mut pts = Vec::new();
            for w in inside.windows(2) {
                for k in 0..4 {
                    pts.push(w[0] + (w[1] - w[0]) * (k as f64 + 0.5) / 4.0);
                }
            }
            pts
        };
        let ss = fine(&ls, sa, sb, self.period_s(), self.per_s);
        let tt = fine(&lt, ta, tb, self.period_t(), self.per_t);
        let mut col = vec![usize::MAX; self.verts.len()];
        for (c, &i) in affected.iter().enumerate() {
            col[i] = c;
        }
        let n = affected.len();
        let rational = old.verts.iter().any(|v| (v.w - 1.0).abs() > 1e-12);
        let dims = if rational { 4 } else { 3 };
        let mut ata = vec![vec![0.0; n]; n];
        let mut atr = vec![[0.0; 4]; n];
        for &t in &tt {
            for &s in &ss {
                let (r, terms) = if rational {
                    (old.eval_h(s, t), self.basis_at(s, t))
                } else {
                    let p = old.eval(s, t);
                    ([p[0], p[1], p[2], 0.0], self.coeffs_at(s, t))
                };
                let mut r = r;
                let mut row: Vec<(usize, f64)> = Vec::new();
                for (i, c) in terms {
                    if col[i] != usize::MAX {
                        row.push((col[i], c));
                    } else {
                        let v = self.verts[i];
                        let wf = if rational { v.w } else { 1.0 };
                        for k in 0..3 {
                            r[k] -= c * wf * v.p[k];
                        }
                        if rational {
                            r[3] -= c * v.w;
                        }
                    }
                }
                for &(a, ca) in &row {
                    for &(b, cb) in &row {
                        ata[a][b] += ca * cb;
                    }
                    for k in 0..dims {
                        atr[a][k] += ca * r[k];
                    }
                }
            }
        }
        let tr: f64 = (0..n).map(|i| ata[i][i]).sum();
        let lam = 1e-9 * tr / n as f64 + 1e-14;
        for (c, &i) in affected.iter().enumerate() {
            ata[c][c] += lam;
            let v = self.verts[i];
            let wf = if rational { v.w } else { 1.0 };
            for k in 0..3 {
                atr[c][k] += lam * wf * v.p[k];
            }
            if rational {
                atr[c][3] += lam * v.w;
            }
        }
        let x = lu_solve(&ata, &atr)?;
        for (c, &i) in affected.iter().enumerate() {
            let v = &mut self.verts[i];
            if rational {
                let w = x[c][3];
                if w > 1e-9 {
                    v.w = w;
                    v.p = [x[c][0] / w, x[c][1] / w, x[c][2] / w];
                }
            } else {
                v.p = [x[c][0], x[c][1], x[c][2]];
            }
        }
        self.touch();
        Ok(affected)
    }

    /// Splits a face into four. `exact`: the new edges run one face past the split
    /// (analysis-suitable T-junction extension), so the refined surface is the old one exactly.
    pub fn subdivide_face(&mut self, f: Face, exact: bool) -> Result<Vec<usize>, Error> {
        let m = (f.l + f.r) / 2.0;
        let n = (f.b + f.t) / 2.0;
        self.refine(
            |ts, old| {
                let (mut b, mut t, mut l, mut r) = (f.b, f.t, f.l, f.r);
                if exact {
                    let hl = ts.h_lines_at(m);
                    let vl = ts.v_lines_at(n);
                    b = Self::beyond(&hl, f.b, -1, ts.t0, ts.t1, ts.per_t);
                    t = Self::beyond(&hl, f.t, 1, ts.t0, ts.t1, ts.per_t);
                    l = Self::beyond(&vl, f.l, -1, ts.s0, ts.s1, ts.per_s);
                    r = Self::beyond(&vl, f.r, 1, ts.s0, ts.s1, ts.per_s);
                }
                ts.add_line(old, Dir::S, m, b, t);
                ts.add_line(old, Dir::T, n, l, r);
            },
            true,
        )
    }

    /// Splits a face in one direction (`Dir::S`: a new vertical line).
    pub fn split_face(&mut self, f: Face, dir: Dir, exact: bool) -> Result<Vec<usize>, Error> {
        self.refine(
            |ts, old| match dir {
                Dir::S => ts.add_line(old, Dir::S, (f.l + f.r) / 2.0, f.b, f.t),
                Dir::T => ts.add_line(old, Dir::T, (f.b + f.t) / 2.0, f.l, f.r),
            },
            exact,
        )
    }

    /// Inserts a full edge loop through the middle of a face, across the whole body.
    pub fn insert_loop(&mut self, f: Face, dir: Dir, exact: bool) -> Result<Vec<usize>, Error> {
        self.refine(
            |ts, old| match dir {
                Dir::S => {
                    let m = (f.l + f.r) / 2.0;
                    for t in uniq_sorted(ts.h_lines_at(m)) {
                        let g = ts.guess_on_h_line(m, t);
                        ts.add_vert(old, m, t, g);
                    }
                    ts.vsegs.push(crate::VSeg {
                        s: m,
                        a: ts.t0,
                        b: ts.t1,
                    });
                }
                Dir::T => {
                    let n = (f.b + f.t) / 2.0;
                    for s in uniq_sorted(ts.v_lines_at(n)) {
                        let g = ts.guess_on_v_line(s, n);
                        ts.add_vert(old, s, n, g);
                    }
                    ts.hsegs.push(crate::HSeg {
                        t: n,
                        a: ts.s0,
                        b: ts.s1,
                    });
                }
            },
            exact,
        )
    }

    /// The mirror image of a parameter point across the symmetry plane (TForm's YZ symmetry).
    pub fn mirror_param(&self, s: f64, t: f64) -> (f64, f64) {
        if self.kind == Kind::Plane {
            return (self.s0 + self.s1 - s, t);
        }
        let p = self.period_s();
        (self.wrap_s_pub(self.s0 + p / 2.0 - (s - self.s0)), t)
    }

    /// For each control point, its mirror image's index (`None` without one).
    pub fn mirror_map(&self) -> Vec<Option<usize>> {
        self.verts
            .iter()
            .map(|v| {
                let (s, t) = self.mirror_param(v.s, v.t);
                self.vert_at(s, t)
            })
            .collect()
    }
}
