//! The **T-spline Form** feature's dialog and editing environment ([`cadrs_core::tspline_form`],
//! TForm's form modelling): a freeform body's control cage, edited in the view.
//!
//! - **Toolbar**: Form (beside Thicken) opens a menu of TForm's primitives (Plane, Cylinder,
//!   Torus) and *Open TForm file…*; each inserts a Form feature and opens its dialog. A
//!   `.tform.json` dropped on a Part Studio is inserted the same way ([`crate::drop_import`]).
//!   Double-clicking a Form in the feature list opens it again.
//! - **Edit form**: the active body's control cage is drawn over it, its control points as
//!   small squares. Click a point, an edge or a face of the cage to select its points (Shift
//!   adds or removes); the *Select* row limits picks to points, edges or faces. Drag a selected
//!   point to move the selection in the plane facing the view; the surface follows live, and
//!   the release is one change. *Move X/Y/Z* moves the selection by an exact distance.
//! - **Insert edge**: click a face to add an edge loop through it, across the body, along the
//!   face's edge nearest the pointer (TForm's rule); the shape is kept exactly.
//! - **Subdivide**: click a face to split it in four. *Keep shape* (on) runs the new edges one
//!   face further so the surface doesn't change; off, the T-junctions stay local and the shape
//!   is kept as closely as it can be.
//! - **Symmetry**: moves and refinements are mirrored across the YZ plane (x → −x), TForm's
//!   symmetry; points on the plane stay on it.
//! - Each change is a command, shown at once; ✓ keeps them as one step ("Edit Form 1"), ✕ puts
//!   the form back (and removes a form the dialog inserted).
//!
//! Names: `form-dialog`, `form-body` (a body's segment `form-body-<i>`), `form-mode`,
//! `form-filter`, `form-symmetry`, `form-keep-shape`, `form-move-x|y|z`, `form-info`, the
//! toolbar's `form` and its menu `form-menu-plane|cylinder|torus|open`.

use bevy::prelude::*;
use cadrs_core::commands::{ReplaceFeature, SetFeature};
use cadrs_core::document::ElementKind;
use cadrs_core::tspline_form::{AddTSplineForm, TSplineFormFeature};
use cadrs_core::{ElementId, Feature, FeatureId, FeatureKind};
use cadrs_sketch::units::Quantity;
use cadrs_tspline::edit::{Dir, Face};
use cadrs_tspline::{TSpline, TSplineFile};
use cadrs_ui::prelude::*;
use cadrs_ui::{
    CheckboxChange, FeatureDialogAccept, FeatureDialogCancel, FeatureDialogState, NumberField,
    NumberFieldCommit, OptionRow, Segmented, SegmentedSelect,
};

use crate::GizmoGroups;
use crate::parts::PartCache;
use crate::viewport::ViewportArea;
use crate::{ActiveDocument, AppState};

pub struct TSplineUiPlugin;

impl Plugin for TSplineUiPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<FormPointer>()
            .init_resource::<FormSamples>()
            .gizmo_group::<FormGizmos>()
            .add_systems(Startup, configure_gizmos)
            .add_systems(Update, on_file_picked.run_if(in_state(AppState::Document)))
            .add_systems(
                Update,
                form_pointer
                    .after(crate::viewport::apply_view_to_camera)
                    .before(crate::parts::PartsSet)
                    .run_if(in_state(AppState::Document)),
            )
            .add_systems(
                Update,
                (sync_form_dialog, draw_cage)
                    .chain()
                    .after(crate::parts::PartsSet)
                    .run_if(in_state(AppState::Document)),
            )
            .add_systems(OnExit(AppState::Document), |world: &mut World| {
                if world.contains_resource::<FormSession>() {
                    finish(world);
                }
            })
            .add_observer(on_accept)
            .add_observer(on_cancel)
            .add_observer(on_segment)
            .add_observer(on_option)
            .add_observer(on_number);
    }
}

// ---------------------------------------------------------------------------------------------
// The session

/// What a click in the view does.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Mode {
    #[default]
    Edit,
    InsertEdge,
    Subdivide,
}

/// What Edit form's clicks pick.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Filter {
    #[default]
    All,
    Point,
    Edge,
    Face,
}

/// The Form whose dialog is open.
#[derive(Resource, Debug, Clone)]
pub struct FormSession {
    pub element: ElementId,
    pub feature: FeatureId,
    pub mark: usize,
    /// The feature as it was (none: the dialog inserted it).
    pub before: Option<Feature>,
    /// The body being edited.
    pub body: usize,
    pub mode: Mode,
    pub filter: Filter,
    /// The selected control points of the body.
    pub selected: Vec<usize>,
    pub symmetry: bool,
    pub keep_shape: bool,
}

#[derive(Component)]
struct FormDialog;

/// What the dialog shows (it is rebuilt when it changes).
#[derive(Component, Debug, Clone, PartialEq)]
struct BuiltFor {
    bodies: Vec<String>,
    body: usize,
    mode: Mode,
    filter: Filter,
    symmetry: bool,
    keep_shape: bool,
    info: String,
}

fn form_of(world: &World) -> Option<TSplineFormFeature> {
    let s = world.get_resource::<FormSession>()?;
    match &world
        .get_resource::<ActiveDocument>()?
        .doc
        .element(s.element)?
        .feature(s.feature)?
        .kind
    {
        FeatureKind::TSplineForm(x) => Some(x.clone()),
        _ => None,
    }
}

fn set(world: &mut World, x: TSplineFormFeature, label: &str) {
    let Some(s) = world.get_resource::<FormSession>().cloned() else {
        return;
    };
    let Some(mut doc) = world.get_resource_mut::<ActiveDocument>() else {
        return;
    };
    if let Err(e) = doc.execute(&SetFeature {
        element: s.element,
        feature: s.feature,
        kind: FeatureKind::TSplineForm(x),
        label: label.into(),
    }) {
        warn!("cannot change the form: {e}");
    }
}

fn close_forms(world: &mut World) {
    if world.contains_resource::<FormSession>() {
        finish(world);
    }
    if world.contains_resource::<crate::image_dialog::ImageSession>() {
        crate::image_dialog::finish(world);
    }
    crate::import_dialog::close_other_sessions(world);
}

fn open(
    world: &mut World,
    element: ElementId,
    feature: FeatureId,
    mark: usize,
    before: Option<Feature>,
) {
    world.insert_resource(FormSession {
        element,
        feature,
        mark,
        before,
        body: 0,
        mode: Mode::Edit,
        filter: Filter::All,
        selected: Vec::new(),
        symmetry: false,
        keep_shape: true,
    });
    *world.resource_mut::<FormPointer>() = FormPointer::default();
}

/// The active Part Studio, if one is open.
fn studio(world: &World) -> Option<ElementId> {
    world
        .get_resource::<ActiveDocument>()?
        .active_element()
        .filter(|e| matches!(e.kind, ElementKind::PartStudio { .. }))
        .map(|e| e.id)
}

fn insert(world: &mut World, x: TSplineFormFeature) -> Result<(), String> {
    close_forms(world);
    let element = studio(world).ok_or("open a Part Studio first")?;
    let mut doc = world
        .get_resource_mut::<ActiveDocument>()
        .ok_or("no document")?;
    let feature = FeatureId::new();
    let mark = doc.history.undo_len();
    doc.execute(&cadrs_core::commands::AddFeature {
        element,
        feature,
        base_name: "Form".into(),
        kind: FeatureKind::TSplineForm(x),
    })
    .map_err(|e| e.to_string())?;
    open(world, element, feature, mark, None);
    Ok(())
}

/// The toolbar's Form menu: a primitive (`"plane"`, `"cylinder"`, `"torus"`) as a new Form,
/// with its dialog open.
pub fn begin_form(world: &mut World, primitive: &str) {
    if crate::linked_session::refuse(world) {
        return;
    }
    let Some(x) = TSplineFormFeature::primitive(primitive) else {
        return;
    };
    if let Err(e) = insert(world, x) {
        warn!("cannot insert a form: {e}");
    }
}

/// A TForm document (`.tform.json`) as a new Form, with its dialog open.
pub fn insert_file(world: &mut World, path: &std::path::Path) -> Result<(), String> {
    let bytes = cadrs_fs::read(path).map_err(|e| e.to_string())?;
    let text = String::from_utf8(bytes).map_err(|_| "not a TForm document".to_string())?;
    close_forms(world);
    let element = studio(world).ok_or("open a Part Studio first")?;
    let mut doc = world
        .get_resource_mut::<ActiveDocument>()
        .ok_or("no document")?;
    let feature = FeatureId::new();
    let mark = doc.history.undo_len();
    doc.execute(&AddTSplineForm {
        element,
        feature,
        text,
    })
    .map_err(|e| e.to_string())?;
    open(world, element, feature, mark, None);
    Ok(())
}

/// The file picker's tag for *Open TForm file…*.
pub const PICK_TAG: &str = "tform-open";

/// *Open TForm file…*: the file picker; the file is inserted when it is picked.
pub fn open_picker(world: &mut World) {
    let theme = world.resource::<Theme>().clone();
    let mut commands = world.commands();
    cadrs_ui::open_file_picker(
        &mut commands,
        &theme,
        "tform-picker",
        "Open TForm file",
        PICK_TAG,
        crate::import_dialog::start_dir(),
        &["json", "tform"],
    );
    world.flush();
}

/// A TForm document picked: inserted.
fn on_file_picked(mut msgs: MessageReader<cadrs_ui::FilePicked>, mut commands: Commands) {
    for m in msgs.read() {
        if m.tag != PICK_TAG {
            continue;
        }
        let path = m.path.clone();
        commands.queue(move |world: &mut World| {
            if let Err(e) = insert_file(world, &path) {
                let theme = world.resource::<Theme>().clone();
                let mut commands = world.commands();
                cadrs_ui::show_notification(
                    &mut commands,
                    &theme,
                    cadrs_ui::Notification::warning(format!("{}: {e}", path.display())),
                );
                world.flush();
            }
        });
    }
}

/// Opens a Form for editing.
pub fn edit_form(world: &mut World, feature: FeatureId) {
    if world
        .get_resource::<FormSession>()
        .is_some_and(|s| s.feature == feature)
    {
        return;
    }
    close_forms(world);
    let Some(doc) = world.get_resource::<ActiveDocument>() else {
        return;
    };
    let Some(el) = doc.active_element() else {
        return;
    };
    let element = el.id;
    let Some(before) = el
        .feature(feature)
        .filter(|f| matches!(f.kind, FeatureKind::TSplineForm(_)))
        .cloned()
    else {
        return;
    };
    let mark = doc.history.undo_len();
    open(world, element, feature, mark, Some(before));
}

/// ✓: keeps the changes as one step.
pub fn accept(world: &mut World) {
    let Some(s) = world.remove_resource::<FormSession>() else {
        return;
    };
    world.resource_mut::<crate::parts::PartOverride>().applied = None;
    let Some(mut doc) = world.get_resource_mut::<ActiveDocument>() else {
        return;
    };
    let name = doc
        .doc
        .element(s.element)
        .and_then(|e| e.feature(s.feature))
        .map(|f| f.name.clone())
        .unwrap_or_else(|| "Form".into());
    let label = if s.before.is_some() {
        format!("Edit {name}")
    } else {
        format!("Insert {name}")
    };
    doc.squash_element_since(s.mark, s.element, label);
}

/// ✕: puts the form back as it was (removes a form the dialog inserted).
pub fn cancel(world: &mut World) {
    let Some(s) = world.remove_resource::<FormSession>() else {
        return;
    };
    world.resource_mut::<crate::parts::PartOverride>().applied = None;
    let Some(mut doc) = world.get_resource_mut::<ActiveDocument>() else {
        return;
    };
    match s.before {
        None => {
            // everything since the insert, the insert included
            while doc.history.undo_len() > s.mark {
                if doc.undo().is_none() {
                    break;
                }
            }
        }
        Some(before) => {
            let name = before.name.clone();
            doc.squash_element_since(s.mark, s.element, format!("Edit {name}"));
            let now = doc
                .doc
                .element(s.element)
                .and_then(|e| e.feature(s.feature))
                .cloned();
            if now.is_some_and(|f| f != before) {
                let _ = doc.execute(&ReplaceFeature {
                    element: s.element,
                    feature: before,
                    label: format!("Cancel {name}"),
                });
            }
        }
    }
}

/// Keeps what changed (the dialog closes when something else opens).
pub fn finish(world: &mut World) {
    accept(world);
}

fn on_accept(ev: On<FeatureDialogAccept>, q: Query<(), With<FormDialog>>, mut commands: Commands) {
    if q.contains(ev.entity) {
        commands.queue(accept);
    }
}

fn on_cancel(ev: On<FeatureDialogCancel>, q: Query<(), With<FormDialog>>, mut commands: Commands) {
    if q.contains(ev.entity) {
        commands.queue(cancel);
    }
}

fn on_segment(ev: On<SegmentedSelect>, q: Query<&Name>, mut session: Option<ResMut<FormSession>>) {
    let (Ok(name), Some(s)) = (q.get(ev.entity), session.as_mut()) else {
        return;
    };
    match name.as_str() {
        "form-mode" => {
            s.mode = [Mode::Edit, Mode::InsertEdge, Mode::Subdivide][ev.index.min(2)];
        }
        "form-filter" => {
            s.filter = [Filter::All, Filter::Point, Filter::Edge, Filter::Face][ev.index.min(3)];
        }
        "form-body" => {
            if s.body != ev.index {
                s.body = ev.index;
                s.selected.clear();
            }
        }
        _ => {}
    }
}

fn on_option(ev: On<CheckboxChange>, q: Query<&Name>, mut session: Option<ResMut<FormSession>>) {
    let (Ok(name), Some(s)) = (q.get(ev.entity), session.as_mut()) else {
        return;
    };
    match name.as_str() {
        "form-symmetry-checkbox" => s.symmetry = ev.checked,
        "form-keep-shape-checkbox" => s.keep_shape = ev.checked,
        _ => {}
    }
}

/// *Move X/Y/Z*: the selection moved by an exact distance.
fn on_number(
    ev: On<NumberFieldCommit>,
    q: Query<&Name>,
    units: Res<crate::WorkspaceUnits>,
    mut commands: Commands,
) {
    let Ok(name) = q.get(ev.entity) else { return };
    let axis = match name.as_str() {
        "form-move-x" => 0,
        "form-move-y" => 1,
        "form-move-z" => 2,
        _ => return,
    };
    let Ok(v) = units.0.eval(ev.text.trim(), Quantity::Length) else {
        return;
    };
    if !v.is_finite() || v == 0.0 {
        return;
    }
    commands.queue(move |world: &mut World| {
        let Some(s) = world.get_resource::<FormSession>().cloned() else {
            return;
        };
        let Some(mut x) = form_of(world) else { return };
        let mut d = [0.0; 3];
        d[axis] = v;
        if moved(&mut x, s.body, &s.selected, d, s.symmetry) {
            set(world, x, "Move points");
        }
    });
}

// ---------------------------------------------------------------------------------------------
// Editing the T-spline

fn spline(x: &TSplineFormFeature, body: usize) -> Option<TSpline> {
    TSpline::try_from(&x.bodies.get(body)?.tspline).ok()
}

fn put(x: &mut TSplineFormFeature, body: usize, ts: &TSpline) {
    if let Some(b) = x.bodies.get_mut(body) {
        b.tspline = TSplineFile::from(ts);
    }
}

/// The points `selected` of `body` moved by `d` (and their mirror images, with symmetry).
/// Returns whether anything moved.
fn moved(
    x: &mut TSplineFormFeature,
    body: usize,
    selected: &[usize],
    d: [f64; 3],
    symmetry: bool,
) -> bool {
    let Some(mut ts) = spline(x, body) else {
        return false;
    };
    let before = ts.clone();
    let mirror = symmetry.then(|| ts.mirror_map());
    for &i in selected {
        let Some(v) = before.verts.get(i) else {
            continue;
        };
        let p = [v.p[0] + d[0], v.p[1] + d[1], v.p[2] + d[2]];
        ts.verts[i].p = p;
        if let Some(m) = mirror.as_ref().and_then(|m| m[i]) {
            if m == i {
                ts.verts[i].p[0] = 0.0;
            } else if !selected.contains(&m) {
                ts.verts[m].p = [-p[0], p[1], p[2]];
            }
        }
    }
    let changed = ts != before;
    put(x, body, &ts);
    changed
}

/// A face refined: an edge loop inserted or the face subdivided (and its mirror image with
/// symmetry).
fn refined(x: &mut TSplineFormFeature, s: &FormSession, f: Face, dir: Dir) -> Result<(), String> {
    let mut ts = spline(x, s.body).ok_or("the body is not a T-spline")?;
    let (cs, ct) = ((f.l + f.r) / 2.0, (f.b + f.t) / 2.0);
    let run = |ts: &mut TSpline, f: Face| -> Result<(), String> {
        match s.mode {
            Mode::InsertEdge => ts.insert_loop(f, dir, true),
            _ => ts.subdivide_face(f, s.keep_shape),
        }
        .map(|_| ())
        .map_err(|e| e.to_string())
    };
    run(&mut ts, f)?;
    if s.symmetry && (s.mode == Mode::Subdivide || dir == Dir::S) {
        let (ms, mt) = ts.mirror_param(cs, ct);
        if (ms - cs).abs() > 1e-6
            && let Some(g) = ts.face_at(ms, mt)
        {
            run(&mut ts, g)?;
        }
    }
    put(x, s.body, &ts);
    Ok(())
}

/// Which way Insert edge's loop runs: along the face's edge nearest the pointer (TForm's rule).
fn insert_dir(f: &Face, s: f64, t: f64) -> Dir {
    let ds = (s - f.l).min(f.r - s) / (f.r - f.l);
    let dt = (t - f.b).min(f.t - t) / (f.t - f.b);
    if ds < dt { Dir::S } else { Dir::T }
}

// ---------------------------------------------------------------------------------------------
// The dialog

#[allow(clippy::too_many_arguments)]
fn sync_form_dialog(
    doc: Option<Res<ActiveDocument>>,
    session: Option<Res<FormSession>>,
    cache: Res<PartCache>,
    theme: Res<Theme>,
    q_area: Query<Entity, With<ViewportArea>>,
    mut q_dialog: Query<(Entity, &BuiltFor, &mut FeatureDialogState), With<FormDialog>>,
    mut commands: Commands,
) {
    let (Some(doc), Some(s)) = (doc, session) else {
        for (e, ..) in &q_dialog {
            commands.entity(e).try_despawn();
        }
        return;
    };
    let Some(f) = doc
        .doc
        .element(s.element)
        .and_then(|e| e.feature(s.feature))
    else {
        return;
    };
    let FeatureKind::TSplineForm(x) = &f.kind else {
        return;
    };
    let error = cache.errors.get(&f.id).cloned();
    let valid = f.is_valid() && error.is_none();
    let info = spline(x, s.body)
        .map(|ts| {
            let sel = if s.selected.is_empty() {
                String::new()
            } else {
                format!("\n{} selected", s.selected.len())
            };
            format!(
                "{} control points · {} T-junctions{sel}",
                ts.verts.len(),
                ts.t_junction_count()
            )
        })
        .unwrap_or_default();
    let layout = BuiltFor {
        bodies: x.bodies.iter().map(|b| b.name.clone()).collect(),
        body: s.body,
        mode: s.mode,
        filter: s.filter,
        symmetry: s.symmetry,
        keep_shape: s.keep_shape,
        info,
    };
    let built = q_dialog.iter().next().map(|(e, k, _)| (e, k.clone()));
    if built.as_ref().is_none_or(|(_, k)| *k != layout) {
        if let Some((e, _)) = built {
            commands.entity(e).try_despawn();
        }
        let Some(area) = q_area.iter().next() else {
            return;
        };
        let t = theme.clone();
        let l = layout.clone();
        let dialog = commands
            .spawn((
                FormDialog,
                layout,
                DespawnOnExit(AppState::Document),
                FeatureDialog::new("form-dialog")
                    .title(f.name.clone())
                    .valid(valid)
                    .body(move |b| {
                        b.spawn(Node {
                            flex_direction: FlexDirection::Column,
                            row_gap: Val::Px(6.0),
                            ..default()
                        })
                        .with_children(|b| {
                            if l.bodies.len() > 1 {
                                let mut seg = Segmented::new("form-body").selected(l.body);
                                for name in &l.bodies {
                                    seg = seg.option(name.clone(), None, name.clone());
                                }
                                b.spawn(seg.build(&t));
                            }
                            b.spawn(
                                Segmented::new("form-mode")
                                    .option(
                                        "Edit form",
                                        None,
                                        "Select and move points, edges and faces",
                                    )
                                    .option(
                                        "Insert edge",
                                        None,
                                        "Click a face: an edge loop through it",
                                    )
                                    .option("Subdivide", None, "Click a face: split it in four")
                                    .selected(match l.mode {
                                        Mode::Edit => 0,
                                        Mode::InsertEdge => 1,
                                        Mode::Subdivide => 2,
                                    })
                                    .build(&t),
                            );
                            if l.mode == Mode::Edit {
                                b.spawn(
                                    Segmented::new("form-filter")
                                        .option("All", None, "Pick points, edges and faces")
                                        .option("Point", None, "Pick control points")
                                        .option("Edge", None, "Pick cage edges")
                                        .option("Face", None, "Pick cage faces")
                                        .selected(match l.filter {
                                            Filter::All => 0,
                                            Filter::Point => 1,
                                            Filter::Edge => 2,
                                            Filter::Face => 3,
                                        })
                                        .build(&t),
                                );
                                for (name, label) in [
                                    ("form-move-x", "Move X"),
                                    ("form-move-y", "Move Y"),
                                    ("form-move-z", "Move Z"),
                                ] {
                                    b.spawn(
                                        NumberField::new(name, label)
                                            .text("0 mm")
                                            .label_width(64.0)
                                            .build(&t),
                                    );
                                }
                            }
                            if l.mode == Mode::Subdivide {
                                b.spawn(
                                    OptionRow::new("form-keep-shape", "Keep shape")
                                        .checked(l.keep_shape)
                                        .build(&t),
                                );
                            }
                            b.spawn(
                                OptionRow::new("form-symmetry", "Symmetry (mirror in X)")
                                    .checked(l.symmetry)
                                    .build(&t),
                            );
                            for line in l.info.lines() {
                                b.spawn((
                                    Name::new("form-info"),
                                    t.text(
                                        line.to_string(),
                                        t.font_sm,
                                        FontWeight::NORMAL,
                                        t.muted_foreground,
                                    ),
                                ));
                            }
                        });
                    })
                    .build(&theme),
            ))
            .id();
        commands.entity(area).add_child(dialog);
        return;
    }
    for (_, _, mut st) in &mut q_dialog {
        let want = FeatureDialogState {
            title: f.name.clone(),
            valid,
            error: error.is_some(),
        };
        if *st != want {
            *st = want;
        }
    }
}

// ---------------------------------------------------------------------------------------------
// The view: hovering, picking and dragging the cage

/// Something of the cage under the pointer.
#[derive(Debug, Clone, Copy, PartialEq)]
enum Hover {
    Point(usize),
    Edge(usize, usize),
    /// A face, and the parameter point under the pointer.
    Face(Face, f64, f64),
}

#[derive(Resource, Debug, Default)]
pub struct FormPointer {
    hovered: Option<Hover>,
    drag: Option<Drag>,
}

#[derive(Debug, Clone)]
struct Drag {
    /// The press on the plane facing the view through the grabbed point.
    origin: Vec3,
    normal: Vec3,
    before: TSplineFormFeature,
    now: TSplineFormFeature,
}

/// Surface samples of the body being edited, for picking faces: `(s, t, point)`.
#[derive(Resource, Debug, Default)]
struct FormSamples {
    key: Option<(FeatureId, usize, TSplineFile)>,
    samples: Vec<(f64, f64, Vec3)>,
}

fn v3(p: [f64; 3]) -> Vec3 {
    Vec3::new(p[0] as f32, p[1] as f32, p[2] as f32)
}

/// Samples of the surface on a grid, four a span each way.
fn sample(ts: &mut TSpline) -> Vec<(f64, f64, Vec3)> {
    let fine = |lines: Vec<f64>| -> Vec<f64> {
        let mut out = Vec::new();
        for w in lines.windows(2) {
            for k in 0..4 {
                out.push(w[0] + (w[1] - w[0]) * (k as f64 + 0.5) / 4.0);
            }
        }
        out
    };
    let ss = fine(ts.param_lines_s());
    let tt = fine(ts.param_lines_t());
    let mut out = Vec::with_capacity(ss.len() * tt.len());
    for &t in &tt {
        for &s in &ss {
            out.push((s, t, v3(ts.eval(s, t))));
        }
    }
    out
}

const POINT_PX: f32 = 4.0;
const GRAB_PX: f32 = 8.0;
const FACE_PX: f32 = 18.0;

fn seg_dist(p: Vec2, a: Vec2, b: Vec2) -> f32 {
    let ab = b - a;
    let t = ((p - a).dot(ab) / ab.length_squared().max(1e-6)).clamp(0.0, 1.0);
    p.distance(a + ab * t)
}

/// What of the cage is under `at` (screen), by the filter.
fn hover_at(
    ts: &TSpline,
    samples: &[(f64, f64, Vec3)],
    view: &crate::camera::ViewState,
    rect: &crate::viewport::ViewportRect,
    at: Vec2,
    filter: Filter,
    faces_only: bool,
) -> Option<Hover> {
    let screen = |p: Vec3| rect.to_screen(view.project(p));
    let depth = |p: Vec3| p.dot(view.back());
    if !faces_only && matches!(filter, Filter::All | Filter::Point) {
        let best = ts
            .verts
            .iter()
            .enumerate()
            .map(|(i, v)| (i, v3(v.p)))
            .filter(|(_, p)| screen(*p).distance(at) <= GRAB_PX)
            .max_by(|a, b| depth(a.1).total_cmp(&depth(b.1)));
        if let Some((i, _)) = best {
            return Some(Hover::Point(i));
        }
    }
    if !faces_only && matches!(filter, Filter::All | Filter::Edge) {
        let best = ts
            .cage_edges()
            .into_iter()
            .map(|(a, b)| {
                (
                    a,
                    b,
                    seg_dist(at, screen(v3(ts.verts[a].p)), screen(v3(ts.verts[b].p))),
                )
            })
            .filter(|e| e.2 <= GRAB_PX * 0.75)
            .min_by(|a, b| a.2.total_cmp(&b.2));
        if let Some((a, b, _)) = best {
            return Some(Hover::Edge(a, b));
        }
    }
    if faces_only || matches!(filter, Filter::All | Filter::Face) {
        let best = samples
            .iter()
            .filter(|(_, _, p)| screen(*p).distance(at) <= FACE_PX)
            .max_by(|a, b| {
                // nearest the eye among those close on screen; ties: nearest on screen
                let (da, db) = (depth(a.2), depth(b.2));
                if (da - db).abs() > 1e-3 {
                    da.total_cmp(&db)
                } else {
                    screen(b.2)
                        .distance(at)
                        .total_cmp(&screen(a.2).distance(at))
                }
            });
        if let Some(&(s, t, _)) = best
            && let Some(f) = ts.face_at(s, t)
            && !ts.is_ring_face(&f)
        {
            return Some(Hover::Face(f, s, t));
        }
    }
    None
}

fn picked_points(ts: &TSpline, h: Hover) -> Vec<usize> {
    match h {
        Hover::Point(i) => vec![i],
        Hover::Edge(a, b) => vec![a, b],
        Hover::Face(f, ..) => ts.face_verts(&f),
    }
}

fn ray_plane(o: Vec3, d: Vec3, p0: Vec3, n: Vec3) -> Option<Vec3> {
    let den = d.dot(n);
    if den.abs() < 1e-9 {
        return None;
    }
    Some(o + d * ((p0 - o).dot(n) / den))
}

/// Hovers, picks, drags and refines the cage.
#[allow(clippy::too_many_arguments)]
fn form_pointer(
    mut inputs: MessageReader<bevy::picking::pointer::PointerInput>,
    session: Option<ResMut<FormSession>>,
    mut pointer: ResMut<FormPointer>,
    mut samples: ResMut<FormSamples>,
    doc: Option<Res<ActiveDocument>>,
    view: Res<crate::viewport::ViewportView>,
    rect: Res<crate::viewport::ViewportRect>,
    drag: Res<crate::viewport::ViewportDrag>,
    keys: Res<ButtonInput<KeyCode>>,
    mut over: ResMut<crate::parts::PartOverride>,
    mut grab: ResMut<crate::assembly::ViewportGrab>,
    mut commands: Commands,
) {
    use bevy::picking::pointer::{PointerAction, PointerButton};
    let current = session.as_ref().and_then(|s| {
        match &doc
            .as_ref()?
            .doc
            .element(s.element)?
            .feature(s.feature)?
            .kind
        {
            FeatureKind::TSplineForm(x) => Some(x.clone()),
            _ => None,
        }
    });
    let (Some(mut s), Some(current)) = (session, current) else {
        inputs.clear();
        pointer.hovered = None;
        pointer.drag = None;
        return;
    };
    if s.body >= current.bodies.len() {
        s.body = 0;
        s.selected.clear();
    }
    let shown = pointer
        .drag
        .as_ref()
        .map_or(current.clone(), |d| d.now.clone());
    let Some(mut ts) = spline(&shown, s.body) else {
        inputs.clear();
        return;
    };
    s.selected.retain(|&i| i < ts.verts.len());
    let key = (s.feature, s.body, shown.bodies[s.body].tspline.clone());
    if samples.key.as_ref() != Some(&key) {
        samples.samples = sample(&mut ts);
        samples.key = Some(key);
    }
    let faces_only = s.mode != Mode::Edit;
    let hovered = if pointer.drag.is_some() {
        pointer.hovered
    } else {
        let at = drag.pointer();
        rect.0
            .contains(at)
            .then(|| {
                hover_at(
                    &ts,
                    &samples.samples,
                    &view.view,
                    &rect,
                    at,
                    s.filter,
                    faces_only,
                )
            })
            .flatten()
    };
    if pointer.hovered != hovered {
        pointer.hovered = hovered;
    }
    let shift = keys.any_pressed([KeyCode::ShiftLeft, KeyCode::ShiftRight]);
    for input in inputs.read() {
        let pos = input.location.position;
        match input.action {
            PointerAction::Press(PointerButton::Primary) if rect.0.contains(pos) => {
                let Some(h) = hover_at(
                    &ts,
                    &samples.samples,
                    &view.view,
                    &rect,
                    pos,
                    s.filter,
                    faces_only,
                ) else {
                    continue;
                };
                // not a click that picks parts
                grab.0 = true;
                if s.mode != Mode::Edit {
                    let Hover::Face(f, hs, ht) = h else { continue };
                    let dir = insert_dir(&f, hs, ht);
                    let snapshot = s.clone();
                    commands.queue(move |world: &mut World| {
                        let Some(mut x) = form_of(world) else { return };
                        match refined(&mut x, &snapshot, f, dir) {
                            Ok(()) => {
                                let label = if snapshot.mode == Mode::InsertEdge {
                                    "Insert edge"
                                } else {
                                    "Subdivide"
                                };
                                set(world, x, label);
                            }
                            Err(e) => {
                                let theme = world.resource::<Theme>().clone();
                                let mut commands = world.commands();
                                cadrs_ui::show_notification(
                                    &mut commands,
                                    &theme,
                                    cadrs_ui::Notification::warning(format!(
                                        "Could not refine here: {e}"
                                    )),
                                );
                                world.flush();
                            }
                        }
                    });
                    continue;
                }
                let pts = picked_points(&ts, h);
                let all_in = pts.iter().all(|i| s.selected.contains(i));
                if shift {
                    if all_in {
                        s.selected.retain(|i| !pts.contains(i));
                    } else {
                        for i in pts {
                            if !s.selected.contains(&i) {
                                s.selected.push(i);
                            }
                        }
                    }
                    continue;
                }
                if !all_in {
                    s.selected = pts.clone();
                }
                // a drag from here moves the selection in the plane facing the view
                let grabbed = match h {
                    Hover::Face(_, hs, ht) => v3(ts.eval(hs, ht)),
                    _ => {
                        pts.iter().map(|&i| v3(ts.verts[i].p)).sum::<Vec3>()
                            / pts.len().max(1) as f32
                    }
                };
                let normal = view.view.back();
                let (o, d) = view.view.ray(rect.offset(pos));
                let origin = ray_plane(o, d, grabbed, normal).unwrap_or(grabbed);
                pointer.drag = Some(Drag {
                    origin,
                    normal,
                    before: current.clone(),
                    now: current.clone(),
                });
            }
            PointerAction::Move { .. } => {
                let Some(dr) = pointer.drag.as_mut() else {
                    continue;
                };
                let (o, d) = view.view.ray(rect.offset(pos));
                let Some(at) = ray_plane(o, d, dr.origin, dr.normal) else {
                    continue;
                };
                let delta = at - dr.origin;
                let mut x = dr.before.clone();
                let dv = [delta.x as f64, delta.y as f64, delta.z as f64];
                if moved(&mut x, s.body, &s.selected, dv, s.symmetry) && x != dr.now {
                    dr.now = x.clone();
                    over.applied = Some((s.feature, FeatureKind::TSplineForm(x)));
                }
            }
            PointerAction::Release(PointerButton::Primary) if pointer.drag.is_some() => {
                commands.queue(|world: &mut World| {
                    world.resource_mut::<crate::parts::PartOverride>().applied = None;
                    let Some(d) = world.resource_mut::<FormPointer>().drag.take() else {
                        return;
                    };
                    if d.now != d.before {
                        set(world, d.now, "Move points");
                    }
                });
            }
            PointerAction::Cancel => {
                pointer.drag = None;
                over.applied = None;
            }
            _ => {}
        }
    }
}

// ---------------------------------------------------------------------------------------------
// Drawing the cage

#[derive(Default, Reflect, GizmoConfigGroup)]
pub struct FormGizmos;

fn configure_gizmos(mut store: ResMut<GizmoConfigStore>) {
    let (c, _) = store.config_mut::<FormGizmos>();
    c.line.width = 1.5;
    // over the surface it controls
    c.depth_bias = -1.0;
}

/// Draws the active body's cage: its edges, a square at each control point (filled with a
/// cross when selected), the hovered element orange, and Insert edge's or Subdivide's preview.
fn draw_cage(
    session: Option<Res<FormSession>>,
    pointer: Res<FormPointer>,
    doc: Option<Res<ActiveDocument>>,
    view: Res<crate::viewport::ViewportView>,
    mut gizmos: Gizmos<FormGizmos>,
) {
    let Some(s) = session else { return };
    let x = match &pointer.drag {
        Some(d) => d.now.clone(),
        None => match doc.as_ref().and_then(|d| {
            d.doc
                .element(s.element)?
                .feature(s.feature)
                .map(|f| f.kind.clone())
        }) {
            Some(FeatureKind::TSplineForm(x)) => x,
            _ => return,
        },
    };
    let Some(mut ts) = spline(&x, s.body) else {
        return;
    };
    let rgb = |c: [u8; 3]| Color::srgb_u8(c[0], c[1], c[2]);
    let blue = rgb(crate::manipulator::ARROW_BLUE);
    let hot = rgb(crate::manipulator::ARROW_HOT);
    let mm_px = view.view.scale;
    let (right, up) = (view.view.right(), view.view.up());
    for (a, b) in ts.cage_edges() {
        let hovered = pointer.hovered == Some(Hover::Edge(a, b))
            || pointer.hovered == Some(Hover::Edge(b, a));
        gizmos.line(
            v3(ts.verts[a].p),
            v3(ts.verts[b].p),
            if hovered { hot } else { blue },
        );
    }
    let h = POINT_PX * mm_px;
    for (i, v) in ts.verts.iter().enumerate() {
        let c = v3(v.p);
        let selected = s.selected.contains(&i);
        let col = if pointer.hovered == Some(Hover::Point(i)) || selected {
            hot
        } else {
            blue
        };
        let corners = [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)]
            .map(|(dx, dy)| c + right * (dx * h) + up * (dy * h));
        for k in 0..4 {
            gizmos.line(corners[k], corners[(k + 1) % 4], col);
        }
        if selected {
            gizmos.line(corners[0], corners[2], col);
            gizmos.line(corners[1], corners[3], col);
        }
    }
    if let Some(Hover::Face(f, hs, ht)) = pointer.hovered {
        // the face's outline on the surface
        let n = 16;
        let iso = |ts: &mut TSpline, from: (f64, f64), to: (f64, f64)| -> Vec<Vec3> {
            (0..=n)
                .map(|k| {
                    let u = k as f64 / n as f64;
                    v3(ts.eval(from.0 + (to.0 - from.0) * u, from.1 + (to.1 - from.1) * u))
                })
                .collect()
        };
        let sides = [
            ((f.l, f.b), (f.r, f.b)),
            ((f.r, f.b), (f.r, f.t)),
            ((f.r, f.t), (f.l, f.t)),
            ((f.l, f.t), (f.l, f.b)),
        ];
        for (a, b) in sides {
            gizmos.linestrip(iso(&mut ts, a, b), hot);
        }
        match s.mode {
            Mode::InsertEdge => {
                // the loop across the body
                let (sa, sb) = ts.domain_s();
                let (ta, tb) = ts.domain_t();
                let line = match insert_dir(&f, hs, ht) {
                    Dir::S => {
                        let m = (f.l + f.r) / 2.0;
                        ((m, ta), (m, tb))
                    }
                    Dir::T => {
                        let m = (f.b + f.t) / 2.0;
                        ((sa, m), (sb, m))
                    }
                };
                let pts: Vec<Vec3> = (0..=96)
                    .map(|k| {
                        let u = k as f64 / 96.0;
                        let (a, b) = line;
                        v3(ts.eval(a.0 + (b.0 - a.0) * u, a.1 + (b.1 - a.1) * u))
                    })
                    .collect();
                gizmos.linestrip(pts, hot);
            }
            Mode::Subdivide => {
                let (m, k) = ((f.l + f.r) / 2.0, (f.b + f.t) / 2.0);
                gizmos.linestrip(iso(&mut ts, (m, f.b), (m, f.t)), hot);
                gizmos.linestrip(iso(&mut ts, (f.l, k), (f.r, k)), hot);
            }
            Mode::Edit => {}
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_symmetric_move_mirrors_the_partner() {
        let mut x = TSplineFormFeature::primitive("plane").unwrap();
        let ts = spline(&x, 0).unwrap();
        // a point off the mirror plane and its partner
        let map = ts.mirror_map();
        let i = (0..ts.verts.len())
            .find(|&i| map[i].is_some_and(|m| m != i))
            .unwrap();
        let m = map[i].unwrap();
        assert!(moved(&mut x, 0, &[i], [3.0, 1.0, 2.0], true));
        let after = spline(&x, 0).unwrap();
        let (p, q) = (after.verts[i].p, after.verts[m].p);
        assert!(
            (p[0] + q[0]).abs() < 1e-9 && (p[1] - q[1]).abs() < 1e-9 && (p[2] - q[2]).abs() < 1e-9
        );
    }

    #[test]
    fn insert_edge_runs_along_the_nearest_side() {
        let f = Face {
            l: 1.0,
            r: 2.0,
            b: 1.0,
            t: 2.0,
        };
        assert_eq!(insert_dir(&f, 1.05, 1.5), Dir::S);
        assert_eq!(insert_dir(&f, 1.5, 1.95), Dir::T);
    }

    #[test]
    fn refinement_is_mirrored_with_symmetry() {
        let mut x = TSplineFormFeature::primitive("plane").unwrap();
        let s = FormSession {
            element: ElementId::new(),
            feature: FeatureId::new(),
            mark: 0,
            before: None,
            body: 0,
            mode: Mode::Subdivide,
            filter: Filter::All,
            selected: Vec::new(),
            symmetry: true,
            keep_shape: true,
        };
        let ts = spline(&x, 0).unwrap();
        let f = ts.face_at(1.5, 1.5).unwrap();
        refined(&mut x, &s, f, Dir::S).unwrap();
        let after = spline(&x, 0).unwrap();
        // both the face and its mirror image were split: points at s = 1.5 and 2.5
        assert!(after.vert_at(1.5, 1.5).is_some());
        assert!(after.vert_at(2.5, 1.5).is_some());
    }
}
