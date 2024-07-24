/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_context.hh"
#include "BKE_curves.hh"
#include "BKE_global.hh"
#include "BKE_grease_pencil.hh"
#include "BKE_image.h"
#include "BKE_layer.hh"
#include "BKE_lib_id.hh"
#include "BKE_object.hh"
#include "BKE_report.hh"

#include "BLI_assert.h"
#include "BLI_math_matrix.hh"
#include "BLI_math_vector.h"
#include "BLI_math_vector.hh"

#include "BLT_translation.hh"

#include "DEG_depsgraph.hh"
#include "DEG_depsgraph_build.hh"

#include "DNA_curves_types.h"
#include "DNA_grease_pencil_types.h"
#include "DNA_object_types.h"

#include "IMB_imbuf_types.hh"

#include "ED_grease_pencil.hh"
#include "ED_numinput.hh"
#include "ED_object.hh"
#include "ED_screen.hh"

#include "MEM_guardedalloc.h"

#include "RNA_access.hh"
#include "RNA_define.hh"

#include "grease_pencil_trace_util.hh"

namespace blender::ed::sculpt_paint::greasepencil {

/* -------------------------------------------------------------------- */
/** \name Trace Image Operator
 * \{ */

using image_trace::TurnPolicy;

/* Target object modes. */
enum class TargetObjectMode : int8_t {
  New = 0,
  Selected = 1,
};

enum class TraceMode : int8_t {
  Single = 0,
  Sequence = 1,
};

#ifdef WITH_POTRACE

struct TraceJob {
  /* from wmJob */
  Object *owner;
  bool *stop, *do_update;
  float *progress;

  bContext *C;
  wmWindowManager *wm;
  Main *bmain;
  Scene *scene;
  View3D *v3d;
  Base *base_active;
  Object *ob_active;
  Image *image;
  Object *ob_grease_pencil;
  bke::greasepencil::Layer *layer;

  bool was_ob_created;
  bool use_current_frame;

  /* Frame number where the output frame is generated. */
  int frame_target;
  float threshold;
  float radius;
  TurnPolicy turnpolicy;
  TraceMode mode;
  /** Frame to render to be used by python API. Not exposed in UI.
   * This feature is only used in Studios to run custom video trace for selected frames. */
  int frame_number;

  bool success;
  bool was_canceled;

  void ensure_output_object();
};

void TraceJob::ensure_output_object()
{
  using namespace blender::bke::greasepencil;

  /* Create a new grease pencil object. */
  if (this->ob_grease_pencil == nullptr) {
    const ushort local_view_bits = (this->v3d && this->v3d->localvd) ? this->v3d->local_view_uid :
                                                                       0;

    /* Copy transform from the active object. */
    this->ob_grease_pencil = ed::object::add_type(this->C,
                                                  OB_GREASE_PENCIL,
                                                  nullptr,
                                                  this->ob_active->loc,
                                                  this->ob_active->rot,
                                                  false,
                                                  local_view_bits);
    copy_v3_v3(this->ob_grease_pencil->scale, this->ob_active->scale);
    this->was_ob_created = true;
  }

  /* Create Layer. */
  GreasePencil &grease_pencil = *static_cast<GreasePencil *>(this->ob_grease_pencil->data);
  this->layer = grease_pencil.get_active_layer();
  if (this->layer == nullptr) {
    Layer &new_layer = grease_pencil.add_layer(DATA_("Trace"));
    grease_pencil.set_active_layer(&new_layer);
    this->layer = &new_layer;
  }
}

using PathSegment = potrace_dpoint_t[3];

static void trace_data_to_strokes(const potrace_state_t &st,
                                  Object & /*ob*/,
                                  bke::greasepencil::Drawing &drawing,
                                  const float4x4 &transform,
                                  const float radius)
{
  auto project_pixel = [&](const potrace_dpoint_t &point) -> float3 {
    return math::transform_point(transform, float3(point.x, point.y, 0));
  };

  ///* Find materials and create them if not found. */
  // BKE_object_material
  // int32_t mat_fill_idx = BKE_gpencil_material_find_index_by_name_prefix(ob, "Stroke");
  // int32_t mat_mask_idx = BKE_gpencil_material_find_index_by_name_prefix(ob, "Holdout");

  // const float default_color[4] = {0.0f, 0.0f, 0.0f, 1.0f};
  ///* Stroke and Fill material. */
  // if (mat_fill_idx == -1) {
  //   int32_t new_idx;
  //   Material *mat_gp = BKE_gpencil_object_material_new(bmain, ob, "Stroke", &new_idx);
  //   MaterialGPencilStyle *gp_style = mat_gp->gp_style;

  //  copy_v4_v4(gp_style->stroke_rgba, default_color);
  //  gp_style->flag |= GP_MATERIAL_STROKE_SHOW;
  //  gp_style->flag |= GP_MATERIAL_FILL_SHOW;
  //  mat_fill_idx = ob->totcol - 1;
  //}
  ///* Holdout material. */
  // if (mat_mask_idx == -1) {
  //   int32_t new_idx;
  //   Material *mat_gp = BKE_gpencil_object_material_new(bmain, ob, "Holdout", &new_idx);
  //   MaterialGPencilStyle *gp_style = mat_gp->gp_style;

  //  copy_v4_v4(gp_style->stroke_rgba, default_color);
  //  copy_v4_v4(gp_style->fill_rgba, default_color);
  //  gp_style->flag |= GP_MATERIAL_STROKE_SHOW;
  //  gp_style->flag |= GP_MATERIAL_FILL_SHOW;
  //  gp_style->flag |= GP_MATERIAL_IS_STROKE_HOLDOUT;
  //  gp_style->flag |= GP_MATERIAL_IS_FILL_HOLDOUT;
  //  mat_mask_idx = ob->totcol - 1;
  //}

  /* Count paths and points. */
  Vector<int> offsets;
  for (const potrace_path_t *path = st.plist; path != nullptr; path = path->next) {
    const Span<int> path_tags = {path->curve.tag, path->curve.n};
    const Span<PathSegment> path_segments = {path->curve.c, path->curve.n};

    int point_num = 0;
    for (const int segment_i : path_segments.index_range()) {
      switch (path_tags[segment_i]) {
        case POTRACE_CORNER:
          point_num += 2;
          break;
        case POTRACE_CURVETO:
          point_num += 1;
          break;
        default:
          BLI_assert_unreachable();
          break;
      }
    }
    offsets.append(point_num);
  }
  /* Last element stores total size. */
  offsets.append(0);
  const OffsetIndices points_by_curve = offset_indices::accumulate_counts_to_offsets(offsets);

  bke::CurvesGeometry curves(points_by_curve.total_size(), points_by_curve.size());
  curves.offsets_for_write().copy_from(offsets);

  /* Construct all curves as Bezier curves. */
  curves.curve_types_for_write().fill(CURVE_TYPE_BEZIER);
  curves.update_curve_types();
  /* All trace curves are cyclic. */
  curves.cyclic_for_write().fill(true);
  /* Uniform radius for all curves. */
  drawing.radii_for_write().fill(radius);

  bke::MutableAttributeAccessor attributes = curves.attributes_for_write();
  bke::SpanAttributeWriter<int> material_indices = attributes.lookup_or_add_for_write_span<int>(
      "material_index", bke::AttrDomain::Curve);
  MutableSpan<int8_t> handle_types_left = curves.handle_types_left_for_write();
  MutableSpan<int8_t> handle_types_right = curves.handle_types_right_for_write();
  MutableSpan<float3> handle_positions_left = curves.handle_positions_left_for_write();
  MutableSpan<float3> handle_positions_right = curves.handle_positions_right_for_write();
  MutableSpan<float3> positions = curves.positions_for_write();

  /* Draw each curve. */
  int curve_i = 0;
  for (const potrace_path_t *path = st.plist; path != nullptr; path = path->next, ++curve_i) {
    const Span<int> path_tags = {path->curve.tag, path->curve.n};
    const Span<PathSegment> path_segments = {path->curve.c, path->curve.n};

    const IndexRange points = points_by_curve[curve_i];
    if (points.is_empty()) {
      continue;
    }

    material_indices.span[curve_i] = 0 /*path->sign == '+' ? mat_fill_idx : mat_mask_idx*/;

    /* Potrace stores the last 3 points of a bezier segment.
     * The start point is the last segment's end point. */
    int point_i = points.last();
    auto next_point = [&]() {
      point_i = (point_i == points.last() ? points.first() : point_i + 1);
    };

    for (const int segment_i : path_segments.index_range()) {
      const PathSegment &segment = path_segments[segment_i];
      switch (path_tags[segment_i]) {
        case POTRACE_CORNER:
          /* Potrace corners are formed by straight lines from the previous/next point.
           * segment[0] is unused, segment[1] is the corner position, segment[2] is the next point.
           */
          handle_types_right[point_i] = BEZIER_HANDLE_VECTOR;

          next_point();
          positions[point_i] = project_pixel(segment[1]);
          handle_types_left[point_i] = BEZIER_HANDLE_VECTOR;
          handle_types_right[point_i] = BEZIER_HANDLE_VECTOR;

          next_point();
          positions[point_i] = project_pixel(segment[2]);
          handle_types_left[point_i] = BEZIER_HANDLE_VECTOR;
          break;
        case POTRACE_CURVETO:
          /* segment[0] is the previous point's right-side handle, segment[1] is the next point's
           * left-side handle, segment[2] is the next point. */
          handle_types_right[point_i] = BEZIER_HANDLE_FREE;
          handle_positions_right[point_i] = project_pixel(segment[0]);

          next_point();
          positions[point_i] = project_pixel(segment[2]);
          handle_types_left[point_i] = BEZIER_HANDLE_FREE;
          handle_positions_left[point_i] = project_pixel(segment[1]);
          break;
        default:
          BLI_assert_unreachable();
          break;
      }
    }
  }

  material_indices.finish();
  drawing.tag_topology_changed();
  drawing.tag_positions_changed();
  curves.tag_radii_changed();

  /* Calculate handles for all corner points (vector handle type). */
  bke::curves::bezier::calculate_auto_handles(true,
                                              handle_types_left,
                                              handle_types_right,
                                              positions,
                                              handle_positions_left,
                                              handle_positions_right);

  drawing.strokes_for_write() = curves;
}

static float4x4 pixel_to_object_transform(const Object &image_object,
                                          const ImBuf &ibuf,
                                          const float2 pixel_center = float2(0.5f))
{
  const float3 pixel_center_3d = float3(pixel_center.x, pixel_center.y, 0);
  const float3 pixel_size_3d = math::safe_rcp(float3(ibuf.x, ibuf.y, 0));
  const float3 image_offset_3d = float3(image_object.ima_ofs[0], image_object.ima_ofs[1], 0);
  const float max_image_scale = image_object.empty_drawsize;
  const float3 image_aspect_3d = (ibuf.x > ibuf.y ? float3(1, float(ibuf.y) / float(ibuf.x), 1) :
                                                    float3(float(ibuf.x) / float(ibuf.y), 1, 1));

  const float4x4 to_object = math::translate(
      math::from_scale<float4x4>(image_aspect_3d * max_image_scale), image_offset_3d);
  const float4x4 to_normalized = math::translate(math::scale(to_object, pixel_size_3d),
                                                 pixel_center_3d);
  return to_normalized;
}

static bool grease_pencil_trace_image(TraceJob &trace_job,
                                      const ImBuf &ibuf,
                                      bke::greasepencil::Drawing &drawing)
{
  /* Trace the image. */
  potrace_bitmap_t *bm = image_trace::image_to_bitmap(ibuf, [&](const ColorGeometry4f &color) {
    return math::average(float3(color.r, color.g, color.b)) * color.a > trace_job.threshold;
  });

  image_trace::TraceParams params;
  params.size_threshold = 0;
  params.turn_policy = trace_job.turnpolicy;
  image_trace::Trace *trace = image_trace::trace_bitmap(params, *bm);
  image_trace::free_bitmap(bm);

  /* Transform from bitmap index space to local image object space. */
  const float4x4 transform = pixel_to_object_transform(*trace_job.ob_active, ibuf);

  trace_data_to_strokes(*trace, *trace_job.ob_grease_pencil, drawing, transform, trace_job.radius);

  /* Free memory. */
  potrace_state_free(trace);

  return true;
}

static void trace_start_job(void *customdata, wmJobWorkerStatus *worker_status)
{
  TraceJob &trace_job = *static_cast<TraceJob *>(customdata);
  GreasePencil &grease_pencil = *static_cast<GreasePencil *>(trace_job.ob_grease_pencil->data);

  trace_job.stop = &worker_status->stop;
  trace_job.do_update = &worker_status->do_update;
  trace_job.progress = &worker_status->progress;
  trace_job.was_canceled = false;
  const int init_frame = max_ii((trace_job.use_current_frame) ? trace_job.frame_target : 0, 0);

  G.is_break = false;

  /* Single Image. */
  if (trace_job.image->source == IMA_SRC_FILE || trace_job.mode == TraceMode::Single) {
    void *lock;
    ImageUser &iuser = *trace_job.ob_active->iuser;

    iuser.framenr = ((trace_job.frame_number == 0) || (trace_job.frame_number > iuser.frames)) ?
                        init_frame :
                        trace_job.frame_number;
    ImBuf *ibuf = BKE_image_acquire_ibuf(trace_job.image, &iuser, &lock);
    if (ibuf) {
      /* Create frame. */
      bke::greasepencil::Drawing *drawing = grease_pencil.get_drawing_at(*trace_job.layer,
                                                                         trace_job.frame_target);
      if (drawing == nullptr) {
        drawing = grease_pencil.insert_frame(*trace_job.layer, trace_job.frame_target);
      }
      grease_pencil_trace_image(trace_job, *ibuf, *drawing);
      BKE_image_release_ibuf(trace_job.image, ibuf, lock);
      *(trace_job.progress) = 1.0f;
    }
  }
  /* Image sequence. */
  else if (trace_job.image->type == IMA_TYPE_IMAGE) {
    ImageUser &iuser = *trace_job.ob_active->iuser;
    for (int frame_number = init_frame; frame_number <= iuser.frames; frame_number++) {
      if (G.is_break) {
        trace_job.was_canceled = true;
        break;
      }

      *(trace_job.progress) = float(frame_number) / float(iuser.frames);
      worker_status->do_update = true;

      iuser.framenr = frame_number;

      void *lock;
      ImBuf *ibuf = BKE_image_acquire_ibuf(trace_job.image, &iuser, &lock);
      if (ibuf) {
        /* Create frame. */
        // bGPDframe *gpf = BKE_gpencil_layer_frame_get(trace_job.gpl, i, GP_GETFRAME_ADD_NEW);
        bke::greasepencil::Drawing *drawing = grease_pencil.get_drawing_at(*trace_job.layer,
                                                                           frame_number);
        if (drawing == nullptr) {
          drawing = grease_pencil.insert_frame(*trace_job.layer, frame_number);
        }
        grease_pencil_trace_image(trace_job, *ibuf, *drawing);

        BKE_image_release_ibuf(trace_job.image, ibuf, lock);
      }
    }
  }

  trace_job.success = !trace_job.was_canceled;
  worker_status->do_update = true;
  worker_status->stop = false;
}

static void trace_end_job(void *customdata)
{
  TraceJob &trace_job = *static_cast<TraceJob *>(customdata);
  GreasePencil &grease_pencil = *static_cast<GreasePencil *>(trace_job.ob_grease_pencil->data);

  /* If canceled, delete all previously created object and data-block. */
  if ((trace_job.was_canceled) && (trace_job.was_ob_created) && (trace_job.ob_grease_pencil)) {
    BKE_id_delete(trace_job.bmain, &trace_job.ob_grease_pencil->id);
    BKE_id_delete(trace_job.bmain, &grease_pencil.id);
  }

  if (trace_job.success) {
    DEG_relations_tag_update(trace_job.bmain);

    DEG_id_tag_update(&trace_job.scene->id, ID_RECALC_SELECT);
    DEG_id_tag_update(&grease_pencil.id, ID_RECALC_GEOMETRY | ID_RECALC_SYNC_TO_EVAL);

    WM_main_add_notifier(NC_OBJECT | NA_ADDED, nullptr);
    WM_main_add_notifier(NC_SCENE | ND_OB_ACTIVE, trace_job.scene);
  }
}

static void trace_free_job(void *customdata)
{
  TraceJob *tj = static_cast<TraceJob *>(customdata);
  MEM_freeN(tj);
}

/* Trace Image to Grease Pencil. */
static bool grease_pencil_trace_image_poll(bContext *C)
{
  Object *ob = CTX_data_active_object(C);
  if ((ob == nullptr) || (ob->type != OB_EMPTY) || (ob->data == nullptr)) {
    CTX_wm_operator_poll_msg_set(C, "No image empty selected");
    return false;
  }

  Image *image = (Image *)ob->data;
  if (!ELEM(image->source, IMA_SRC_FILE, IMA_SRC_SEQUENCE, IMA_SRC_MOVIE)) {
    CTX_wm_operator_poll_msg_set(C, "No valid image format selected");
    return false;
  }

  return true;
}

static int grease_pencil_trace_image_exec(bContext *C, wmOperator *op)
{
  TraceJob *job = static_cast<TraceJob *>(MEM_mallocN(sizeof(TraceJob), "TraceJob"));
  job->C = C;
  job->owner = CTX_data_active_object(C);
  job->wm = CTX_wm_manager(C);
  job->bmain = CTX_data_main(C);
  Scene *scene = CTX_data_scene(C);
  job->scene = scene;
  job->v3d = CTX_wm_view3d(C);
  job->base_active = CTX_data_active_base(C);
  job->ob_active = job->base_active->object;
  job->image = (Image *)job->ob_active->data;
  job->frame_target = scene->r.cfra;
  job->use_current_frame = RNA_boolean_get(op->ptr, "use_current_frame");

  /* Create a new grease pencil object or reuse selected. */
  const TargetObjectMode target = TargetObjectMode(RNA_enum_get(op->ptr, "target"));
  job->ob_grease_pencil = (target == TargetObjectMode::Selected) ?
                              BKE_view_layer_non_active_selected_object(
                                  scene, CTX_data_view_layer(C), job->v3d) :
                              nullptr;

  if (job->ob_grease_pencil != nullptr) {
    if (job->ob_grease_pencil->type != OB_GREASE_PENCIL) {
      BKE_report(op->reports, RPT_WARNING, "Target object not a grease pencil, ignoring!");
      job->ob_grease_pencil = nullptr;
    }
    else if (BKE_object_obdata_is_libdata(job->ob_grease_pencil)) {
      BKE_report(op->reports, RPT_WARNING, "Target object library-data, ignoring!");
      job->ob_grease_pencil = nullptr;
    }
  }

  job->was_ob_created = false;

  job->threshold = RNA_float_get(op->ptr, "threshold");
  job->radius = RNA_float_get(op->ptr, "radius");
  job->turnpolicy = TurnPolicy(RNA_enum_get(op->ptr, "turnpolicy"));
  job->mode = TraceMode(RNA_enum_get(op->ptr, "mode"));
  job->frame_number = RNA_int_get(op->ptr, "frame_number");

  job->ensure_output_object();

  /* Back to active base. */
  blender::ed::object::base_activate(job->C, job->base_active);

  if ((job->image->source == IMA_SRC_FILE) || (job->frame_number > 0)) {
    wmJobWorkerStatus worker_status = {};
    trace_start_job(job, &worker_status);
    trace_end_job(job);
    trace_free_job(job);
  }
  else {
    wmJob *wm_job = WM_jobs_get(job->wm,
                                CTX_wm_window(C),
                                job->scene,
                                "Trace Image",
                                WM_JOB_PROGRESS,
                                WM_JOB_TYPE_TRACE_IMAGE);

    WM_jobs_customdata_set(wm_job, job, trace_free_job);
    WM_jobs_timer(wm_job, 0.1, NC_GEOM | ND_DATA, NC_GEOM | ND_DATA);
    WM_jobs_callbacks(wm_job, trace_start_job, nullptr, nullptr, trace_end_job);

    WM_jobs_start(CTX_wm_manager(C), wm_job);
  }

  return OPERATOR_FINISHED;
}

static int grease_pencil_trace_image_invoke(bContext *C, wmOperator *op, const wmEvent * /*event*/)
{
  /* Show popup dialog to allow editing. */
  /* FIXME: hard-coded dimensions here are just arbitrary. */
  return WM_operator_props_dialog_popup(C, op, 250);
}

static void GREASE_PENCIL_OT_trace_image(wmOperatorType *ot)
{
  PropertyRNA *prop;

  static const EnumPropertyItem turnpolicy_type[] = {
      {int(TurnPolicy::Foreground),
       "FOREGROUND",
       0,
       "Foreground",
       "Prefers to connect foreground components"},
      {int(TurnPolicy::Background),
       "BACKGROUND",
       0,
       "Background",
       "Prefers to connect background components"},
      {int(TurnPolicy::Left), "LEFT", 0, "Left", "Always take a left turn"},
      {int(TurnPolicy::Right), "RIGHT", 0, "Right", "Always take a right turn"},
      {int(TurnPolicy::Minority),
       "MINORITY",
       0,
       "Minority",
       "Prefers to connect the color that occurs least frequently in the local "
       "neighborhood of the current position"},
      {int(TurnPolicy::Majority),
       "MAJORITY",
       0,
       "Majority",
       "Prefers to connect the color that occurs most frequently in the local "
       "neighborhood of the current position"},
      {int(TurnPolicy::Random), "RANDOM", 0, "Random", "Choose pseudo-randomly"},
      {0, nullptr, 0, nullptr, nullptr},
  };

  static const EnumPropertyItem trace_modes[] = {
      {int(TraceMode::Single), "SINGLE", 0, "Single", "Trace the current frame of the image"},
      {int(TraceMode::Sequence), "SEQUENCE", 0, "Sequence", "Trace full sequence"},
      {0, nullptr, 0, nullptr, nullptr},
  };

  static const EnumPropertyItem target_object_modes[] = {
      {int(TargetObjectMode::New), "NEW", 0, "New Object", ""},
      {int(TargetObjectMode::Selected), "SELECTED", 0, "Selected Object", ""},
      {0, nullptr, 0, nullptr, nullptr},
  };

  /* identifiers */
  ot->name = "Trace Image to Grease Pencil";
  ot->idname = "GREASE_PENCIL_OT_trace_image";
  ot->description = "Extract Grease Pencil strokes from image";

  /* callbacks */
  ot->invoke = grease_pencil_trace_image_invoke;
  ot->exec = grease_pencil_trace_image_exec;
  ot->poll = grease_pencil_trace_image_poll;

  /* flags */
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;

  /* properties */
  ot->prop = RNA_def_enum(ot->srna,
                          "target",
                          target_object_modes,
                          int(TargetObjectMode::New),
                          "Target Object",
                          "Target grease pencil");
  RNA_def_property_flag(ot->prop, PROP_SKIP_SAVE);

  RNA_def_float(ot->srna, "radius", 0.01f, 0.001f, 1.0f, "Radius", "", 0.001, 1.0f);

  RNA_def_float_factor(ot->srna,
                       "threshold",
                       0.5f,
                       0.0f,
                       1.0f,
                       "Color Threshold",
                       "Determine the lightness threshold above which strokes are generated",
                       0.0f,
                       1.0f);
  RNA_def_enum(ot->srna,
               "turnpolicy",
               turnpolicy_type,
               int(TurnPolicy::Minority),
               "Turn Policy",
               "Determines how to resolve ambiguities during decomposition of bitmaps into paths");
  RNA_def_enum(ot->srna,
               "mode",
               trace_modes,
               int(TraceMode::Single),
               "Mode",
               "Determines if trace simple image or full sequence");
  RNA_def_boolean(ot->srna,
                  "use_current_frame",
                  true,
                  "Start At Current Frame",
                  "Trace Image starting in current image frame");
  prop = RNA_def_int(
      ot->srna,
      "frame_number",
      0,
      0,
      9999,
      "Trace Frame",
      "Used to trace only one frame of the image sequence, set to zero to trace all",
      0,
      9999);
  RNA_def_property_flag(prop, PROP_SKIP_SAVE);
}

#endif

/** \} */

}  // namespace blender::ed::sculpt_paint::greasepencil

/* -------------------------------------------------------------------- */
/** \name Registration
 * \{ */

void ED_operatortypes_grease_pencil_trace()
{
  using namespace blender::ed::sculpt_paint::greasepencil;

#ifdef WITH_POTRACE
  WM_operatortype_append(GREASE_PENCIL_OT_trace_image);
#endif
}

/** \} */
