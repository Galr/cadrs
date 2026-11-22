# Reference data

`reference.json` comes from TForm's own JavaScript T-spline engine, so the Rust port is checked
against the implementation it was ported from: models (primitives, a subdivided face, an inserted
edge loop, a weighted point), surface points on a grid, and each model's exact NURBS conversion.

To regenerate it, save TForm's engine script (`<script id="tf-engine">`) as `engine.js` and run

    node tests/gen_reference.js ./engine.js > tests/reference.json
