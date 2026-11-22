// Reference data for the Rust port: models, evaluations and NURBS conversions from TForm's engine.
const TF = require(process.argv[2]);
const out = [];
function rec(name, ts) {
  const pts = [];
  const sa = ts.sa, sb = ts.sb, ta = ts.ta, tb = ts.tb;
  for (let i = 0; i <= 6; i++) for (let j = 0; j <= 5; j++) {
    const s = sa + (sb - sa) * i / 6, t = ta + (tb - ta) * j / 5;
    pts.push([s, t, ...ts.eval(s, t)]);
  }
  const N = ts.toNurbs();
  out.push({ name, model: ts.toJSON(), pts, nurbs: N });
}
rec('plane', TF.createPlane());
rec('cylinder', TF.createCylinder());
rec('torus', TF.createTorus());
// T-junctions: a subdivided face and an inserted loop, with points moved
const p = TF.createPlane();
let f = p.faceAt(1.5, 1.5); p.subdivideFace(f); p.touch();
p.verts.forEach((v, i) => { v.p[1] = Math.sin(i * 0.7) * 6; });
p.touch(); rec('plane-subdiv', p);
const c = TF.createCylinder();
f = c.faceAt(2.5, 1.5); c.insertLoop(f, 's'); c.touch();
c.verts.forEach((v, i) => { v.p[0] *= 1 + 0.1 * Math.cos(i); }); c.touch(); rec('cylinder-loop', c);
const w = TF.createPlane({ nu: 3, nv: 3 }); w.verts[5].w = 2.5; w.verts[5].p[1] = 20; w.touch(); rec('plane-weight', w);
const p2 = TF.createPlane(); p2.subdivideFace(p2.faceAt(1.5, 1.5)); p2.touch(); rec('plane-subdiv-raw', p2);
const c2 = TF.createCylinder(); c2.insertLoop(c2.faceAt(2.5, 1.5), 's'); c2.touch(); rec('cylinder-loop-raw', c2);
const t2 = TF.createTorus(); t2.subdivideFace(t2.faceAt(0.5, 7.5)); t2.touch(); rec('torus-subdiv-seam-raw', t2);
const w2 = TF.createPlane({ nu: 3, nv: 3 }); w2.verts[5].w = 2.5; w2.verts[5].p[1] = 20; w2.touch(); w2.insertLoop(w2.faceAt(1.5, 1.5), 't'); w2.touch(); rec('plane-weight-loop-raw', w2);
console.log(JSON.stringify(out));
