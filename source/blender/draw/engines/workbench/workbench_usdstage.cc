#include <iostream>
#include <memory>

#include "DNA_screen_types.h"
#include "DNA_view3d_types.h"

#include "BKE_appdir.h"
#include "BKE_camera.h"
#include "BKE_object.hh"
#include "BLI_fileops.h"
#include "BLI_path_util.h"
#include "DRW_render.h"
#include "GPU_compute.h"

#include "workbench_private.hh"

#include <pxr/pxr.h>

#include "pxr/usd/usd/prim.h"
#include <pxr/base/gf/camera.h>
#include <pxr/base/gf/matrix4d.h>
#include <pxr/base/gf/matrix4f.h>
#include <pxr/base/gf/vec2f.h>
#include <pxr/base/plug/plugin.h>
#include <pxr/base/plug/registry.h>
#include <pxr/base/tf/diagnostic.h>
#include <pxr/imaging/glf/simpleLightingContext.h>
#include <pxr/imaging/hd/engine.h>
#include <pxr/imaging/hd/light.h>
#include <pxr/imaging/hd/pluginRenderDelegateUniqueHandle.h>
#include <pxr/imaging/hd/renderBuffer.h>
#include <pxr/imaging/hd/renderIndex.h>
#include <pxr/imaging/hd/renderThread.h>
#include <pxr/imaging/hd/rendererPlugin.h>
#include <pxr/imaging/hd/rendererPluginRegistry.h>
#include <pxr/imaging/hd/sceneDelegate.h>
#include <pxr/imaging/hd/task.h>
#include <pxr/imaging/hd/tokens.h>
#include <pxr/imaging/hdSt/renderDelegate.h>
#include <pxr/imaging/hdx/tokens.h>
#include <pxr/imaging/hgi/hgi.h>
#include <pxr/imaging/hgi/tokens.h>
#include <pxr/usd/sdf/layer.h>
#include <pxr/usd/usd/primRange.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usdGeom/camera.h>
#include <pxr/usd/usdGeom/metrics.h>
#include <pxr/usd/usdGeom/sphere.h>
#include <pxr/usd/usdGeom/tokens.h>
#include <pxr/usd/usdGeom/xform.h>
#include <pxr/usd/usdGeom/xformCommonAPI.h>
#include <pxr/usdImaging/usdImaging/delegate.h>
#include <pxr/usdImaging/usdImagingGL/engine.h>
#include <pxr/usdImaging/usdImagingGL/renderParams.h>

#include "BKE_usd_stage.hh"
#include "DNA_usd_stage_types.h"

#include "../../../io/usd/hydra/hydra_scene_delegate.h"
#include "../../../render/hydra/viewport_engine.h"

using namespace blender::render::hydra;

extern "C" DrawEngineType draw_engine_workbench;

/* Stealing this function from Hydra */
namespace blender::render::hydra {

extern pxr::GfCamera gf_camera(const Depsgraph *depsgraph,
                               const View3D *v3d,
                               const ARegion *region,
                               const pxr::GfVec4f &border);

}  // namespace blender::render::hydra

namespace blender::workbench {

const pxr::TfToken rendererPluginId("HdStormRendererPlugin");
const pxr::TfToken render_delegate_name("HdStormRendererPlugin");
const pxr::SdfPath delegateID("/BlenderDelegate");
const pxr::SdfPath camera_path("/Camera");

// ======================================================================
struct ViewSettings {
  int screen_width;
  int screen_height;
  pxr::GfVec4i border;
  pxr::GfCamera camera;

  ViewSettings(const SceneState &scene_state)
  {
    const Scene *scene = scene_state.scene;

    const DRWContextState *ctx = DRW_context_state_get();

    View3D *view3d = ctx->v3d;
    RegionView3D *region_data = ctx->rv3d;
    ARegion *region = ctx->region;

    screen_width = region->winx;
    screen_height = region->winy;

    /* Getting render border. */
    int x1 = 0, y1 = 0;
    int x2 = screen_width, y2 = screen_height;

    if (region_data->persp == RV3D_CAMOB) {
      if (scene->r.mode & R_BORDER) {
        Object *camera_obj = scene->camera;

        float camera_points[4][3];
        BKE_camera_view_frame(scene, static_cast<Camera *>(camera_obj->data), camera_points);

        float screen_points[4][2];
        for (int i = 0; i < 4; i++) {
          float world_location[] = {
              camera_points[i][0], camera_points[i][1], camera_points[i][2], 1.0f};
          mul_m4_v4(camera_obj->object_to_world, world_location);
          mul_m4_v4(region_data->persmat, world_location);

          if (world_location[3] > 0.0) {
            screen_points[i][0] = screen_width * 0.5f +
                                  screen_width * 0.5f * (world_location[0] / world_location[3]);
            screen_points[i][1] = screen_height * 0.5f +
                                  screen_height * 0.5f * (world_location[1] / world_location[3]);
          }
        }

        /* Getting camera view region. */
        float x1_f = std::min(
            {screen_points[0][0], screen_points[1][0], screen_points[2][0], screen_points[3][0]});
        float x2_f = std::max(
            {screen_points[0][0], screen_points[1][0], screen_points[2][0], screen_points[3][0]});
        float y1_f = std::min(
            {screen_points[0][1], screen_points[1][1], screen_points[2][1], screen_points[3][1]});
        float y2_f = std::max(
            {screen_points[0][1], screen_points[1][1], screen_points[2][1], screen_points[3][1]});

        /* Adjusting region to border. */
        float x = x1_f, y = y1_f;
        float dx = x2_f - x1_f, dy = y2_f - y1_f;

        x1 = x + scene->r.border.xmin * dx;
        x2 = x + scene->r.border.xmax * dx;
        y1 = y + scene->r.border.ymin * dy;
        y2 = y + scene->r.border.ymax * dy;

        /* Adjusting to region screen resolution. */
        x1 = std::max(std::min(x1, screen_width), 0);
        x2 = std::max(std::min(x2, screen_width), 0);
        y1 = std::max(std::min(y1, screen_height), 0);
        y2 = std::max(std::min(y2, screen_height), 0);
      }
    }
    else {
      if (view3d->flag2 & V3D_RENDER_BORDER) {
        x1 = view3d->render_border.xmin * screen_width;
        x2 = view3d->render_border.xmax * screen_width;
        y1 = view3d->render_border.ymin * screen_height;
        y2 = view3d->render_border.ymax * screen_height;
      }
    }

    border = pxr::GfVec4i(x1, y1, x2, y2);

    camera = gf_camera(ctx->depsgraph,
                       view3d,
                       region,
                       pxr::GfVec4f(float(border[0]) / screen_width,
                                    float(border[1]) / screen_height,
                                    float(width()) / screen_width,
                                    float(height()) / screen_height));
  }

  int width()
  {
    return border[2] - border[0];
  }

  int height()
  {
    return border[3] - border[1];
  }
};

// ======================================================================
/* Populate Hydra render index using USD file export, for testing. */
class USDLiveStageDelegate {
 private:
  pxr::HdRenderIndex *render_index_;
  pxr::SdfPath const delegate_id_;
  pxr::UsdStageRefPtr stage_;
  std::unique_ptr<pxr::UsdImagingDelegate> delegate_;

  std::string temp_dir_;
  std::string temp_file_;

 public:
  USDLiveStageDelegate(pxr::HdRenderIndex *render_index, pxr::SdfPath const &delegate_id)
      : render_index_(render_index), delegate_id_(delegate_id)
  {
  }

  ~USDLiveStageDelegate()
  {
    //    BLI_delete(temp_dir_.c_str(), true, true);
  }

  void populate(const std::string stage_path)
  {
    /* Free previous stage first to save memory. */
    stage_.Reset();
    delegate_.reset();

    /* Convert depsgraph to stage + additional file in temp directory. */
    stage_ = pxr::UsdStage::Open(stage_path);
    delegate_ = std::make_unique<pxr::UsdImagingDelegate>(render_index_, delegate_id_);

    if ((bool)stage_) {
      delegate_->Populate(stage_->GetPseudoRoot());
      std::cerr << "UsdSceneDelegate::populate: loaded stage" << std::endl;
    }
    else {
      std::cerr << "UsdSceneDelegate::populate: can't find stage " << stage_path << std::endl;
    }
  }
};

// ======================================================================
class USDStagePassImpl {
  /* Using the PIMPL pattern to not have to pass as much
   * around in workbench_private.hh.  Also, the order of these
   * is very important-- reordering them will cause the wrong
   * things to be deconstructed first, leading to crashes. */

 protected:
  std::string render_delegate_name_;
  Depsgraph *depsgraph_ = nullptr;
  bContext *context_ = nullptr;
  Scene *scene_ = nullptr;

  pxr::HgiUniquePtr hgi_;
  pxr::HdDriver hgi_driver_;
  pxr::HdPluginRenderDelegateUniqueHandle render_delegate_;
  std::unique_ptr<pxr::HdRenderIndex> render_index_;

  std::unique_ptr<io::hydra::HydraSceneDelegate> hydra_scene_delegate_;
  std::unique_ptr<USDLiveStageDelegate> usd_scene_delegate_;

  std::unique_ptr<RenderTaskDelegate> render_task_delegate_;
  std::unique_ptr<pxr::HdxFreeCameraSceneDelegate> free_camera_delegate_;
  std::unique_ptr<LightTasksDelegate> light_tasks_delegate_;
  std::unique_ptr<pxr::HdEngine> engine_;

  double time_begin_;
  DrawTexture draw_texture_;

  pxr::HdTaskSharedPtrVector tasks();

 public:
  USDStagePassImpl();
  virtual ~USDStagePassImpl();

  void sync(Manager &manager,
            SceneResources &resources,
            const SceneState &scene_state,
            ObjectRef &ob_ref,
            float3 color);
  void render();

  void set_render_setting(const std::string &key, const pxr::VtValue &val);

  void init(const SceneState &scene_state);
  void setup_view(View &view, const SceneState &scene_state);
  void draw(Manager &manager,
            View &view,
            const SceneState &scene_state,
            SceneResources &resources);
};

// ======================================================================
USDStagesPass::USDStagesPass() : impl_(std::make_unique<USDStagePassImpl>()) {}

USDStagesPass::~USDStagesPass() {}

void USDStagesPass::init(const SceneState &scene_state)
{
  impl_->init(scene_state);
}

void USDStagesPass::sync(const SceneState &scene_state, SceneResources &resources)
{
  Manager *manager = DRW_manager_get();
  ObjectRef ref;
  impl_->sync(*manager, resources, scene_state, ref, {0, 0, 0});
}

void USDStagesPass::setup_view(View &view, const SceneState &scene_state)
{
  impl_->setup_view(view, scene_state);
}

void USDStagesPass::draw(Manager &manager,
                         View &view,
                         const SceneState &scene_state,
                         SceneResources &resources)
{

  impl_->draw(manager, view, scene_state, resources);
}

// ======================================================================
USDStagePassImpl::USDStagePassImpl() : render_delegate_name_("HdStormRendererPlugin")
{
  pxr::HdRendererPluginRegistry &registry = pxr::HdRendererPluginRegistry::GetInstance();

  pxr::TF_PY_ALLOW_THREADS_IN_SCOPE();

  if (GPU_backend_get_type() == GPU_BACKEND_VULKAN) {
    BLI_setenv("HGI_ENABLE_VULKAN", "1");
  }

  pxr::HdDriverVector hd_drivers;
  hgi_ = pxr::Hgi::CreatePlatformDefaultHgi();
  hgi_driver_.name = pxr::HgiTokens->renderDriver;
  hgi_driver_.driver = pxr::VtValue(hgi_.get());
  hd_drivers.push_back(&hgi_driver_);

  render_delegate_ = registry.CreateRenderDelegate(pxr::TfToken(render_delegate_name_));

  if (!render_delegate_) {
    throw std::runtime_error("Cannot create render delegate: " + render_delegate_name_);
  }

  render_index_.reset(pxr::HdRenderIndex::New(render_delegate_.Get(), hd_drivers));
  if (!render_index_) {
    std::cerr << ">>>>>>>>>>>> Unable to create render index!" << std::endl;
  }

  free_camera_delegate_ = std::make_unique<pxr::HdxFreeCameraSceneDelegate>(
      render_index_.get(), pxr::SdfPath::AbsoluteRootPath().AppendElementString("freeCamera"));

  render_task_delegate_ = std::make_unique<GPURenderTaskDelegate>(
      render_index_.get(), pxr::SdfPath::AbsoluteRootPath().AppendElementString("renderTask"));

  render_task_delegate_->set_camera(free_camera_delegate_->GetCameraId());

  light_tasks_delegate_ = std::make_unique<LightTasksDelegate>(
      render_index_.get(), pxr::SdfPath::AbsoluteRootPath().AppendElementString("lightTasks"));
  light_tasks_delegate_->set_camera(free_camera_delegate_->GetCameraId());

  engine_ = std::make_unique<pxr::HdEngine>();

  hydra_scene_delegate_ = nullptr;
  pxr::SdfPath scene_path = pxr::SdfPath::AbsoluteRootPath().AppendElementString("usd_scene");
  usd_scene_delegate_ = std::make_unique<USDLiveStageDelegate>(render_index_.get(), scene_path);
  usd_scene_delegate_->populate("/Users/kiki/Desktop/usd_test_scene_01.usd");
}

USDStagePassImpl::~USDStagePassImpl()
{
  std::cerr << "~USDStagePass" << std::endl;
}

void USDStagePassImpl::init(const SceneState &scene_state) {}

void USDStagePassImpl::sync(Manager &manager,
                            SceneResources &resources,
                            const SceneState &scene_state,
                            ObjectRef &ob_ref,
                            float3 color)
{
  std::cerr << "USDStagesPass::SYNC" << std::endl;

  const DRWContextState *ctx = DRW_context_state_get();

  depsgraph_ = ctx->depsgraph;
  scene_ = scene_state.scene;
}

void USDStagePassImpl::set_render_setting(const std::string &key, const pxr::VtValue &val)
{
  render_delegate_->SetRenderSetting(pxr::TfToken(key), val);
}

void USDStagePassImpl::setup_view(View &view, const SceneState &scene_state)
{
  std::cerr << "USDStagesPass::SETUP_VIEW" << std::endl;
}

pxr::HdTaskSharedPtrVector USDStagePassImpl::tasks()
{
  pxr::HdTaskSharedPtrVector res;
  if (light_tasks_delegate_) {
    if (scene_->r.alphamode != R_ALPHAPREMUL) {
#ifndef __APPLE__
      /* TODO: Temporary disable skydome task for MacOS due to crash with error:
       * Failed to created pipeline state, error depthAttachmentPixelFormat is not valid
       * and shader writes to depth */
      res.push_back(light_tasks_delegate_->skydome_task());
#endif
    }
    res.push_back(light_tasks_delegate_->simple_task());
  }
  res.push_back(render_task_delegate_->task());
  return res;
}

void USDStagePassImpl::draw(Manager &manager,
                            View &view,
                            const SceneState &scene_state,
                            SceneResources &resources)
{

  std::cerr << "USDStagesPass::Draw" << std::endl;

  setup_view(view, scene_state);

  ViewSettings view_settings(scene_state);
  if (view_settings.width() * view_settings.height() == 0) {
    return;
  };

  free_camera_delegate_->SetCamera(view_settings.camera);

  pxr::GfVec4d viewport(0.0, 0.0, view_settings.width(), view_settings.height());
  render_task_delegate_->set_viewport(viewport);
  if (light_tasks_delegate_) {
    light_tasks_delegate_->set_viewport(viewport);
  }

  render_task_delegate_->add_aov(pxr::HdAovTokens->color);
  render_task_delegate_->add_aov(pxr::HdAovTokens->depth);

  GPUFrameBuffer *view_framebuffer = GPU_framebuffer_active_get();
  auto t = tasks();

  render_task_delegate_->bind();
  engine_->Execute(render_index_.get(), &t);
  render_task_delegate_->unbind();

  GPU_framebuffer_bind(view_framebuffer);
  GPUShader *shader = GPU_shader_get_builtin_shader(GPU_SHADER_3D_IMAGE);
  GPU_shader_bind(shader);

  pxr::GfVec4d draw_viewport(view_settings.border[0] * 0.01f,
                             view_settings.border[1] * 0.01f,
                             view_settings.border[2] * 0.01f,
                             view_settings.border[3] * 0.01f);

  GPURenderTaskDelegate *gpu_task = dynamic_cast<GPURenderTaskDelegate *>(
      render_task_delegate_.get());
  if (gpu_task) {
    draw_texture_.draw(shader, draw_viewport, gpu_task->aov_texture(pxr::HdAovTokens->color));
  }
  else {
    /* This first line ensures the texture exists if it hasn't yet been created. */
    draw_texture_.write_data(view_settings.width(), view_settings.height(), nullptr);
    render_task_delegate_->read_aov(pxr::HdAovTokens->color, draw_texture_.texture());
    draw_texture_.draw(shader, draw_viewport);
  }

  GPU_shader_unbind();
}

}  // namespace blender::workbench
