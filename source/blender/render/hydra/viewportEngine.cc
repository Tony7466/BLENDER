/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#include <epoxy/gl.h>

#include <pxr/base/gf/camera.h>
#include <pxr/imaging/glf/drawTarget.h>
#include <pxr/usd/usdGeom/camera.h>

#include "DEG_depsgraph_query.h"
#include "BLI_math_matrix.h"
#include "DNA_camera_types.h"
#include "DNA_screen_types.h"
#include "BKE_camera.h"
#include "GPU_shader.h"

#include "glog/logging.h"

#include "viewportEngine.h"
#include "camera.h"
#include "utils.h"

using namespace std;
using namespace pxr;

namespace blender::render::hydra {

struct ViewSettings {
  ViewSettings(bContext *context);

  int width();
  int height();

  GfCamera gf_camera();

  CameraData camera_data;

  int screen_width;
  int screen_height;
  GfVec4i border;
};

ViewSettings::ViewSettings(bContext *context)
  : camera_data(context)
{
  View3D *view3d = CTX_wm_view3d(context);
  RegionView3D *region_data = (RegionView3D *)CTX_wm_region_data(context);
  ARegion *region = CTX_wm_region(context);

  screen_width = region->winx;
  screen_height = region->winy;

  Scene *scene = CTX_data_scene(context);

  //getting render border
  int x1 = 0, y1 = 0;
  int x2 = screen_width, y2 = screen_height;

  if (region_data->persp == RV3D_CAMOB) {
    if (scene->r.mode & R_BORDER) {
      Object *camera_obj = scene->camera;

      float camera_points[4][3];
      BKE_camera_view_frame(scene, (Camera *)camera_obj->data, camera_points);

      float screen_points[4][2];
      for (int i = 0 ; i < 4; i++) {
        float world_location[] = {camera_points[i][0], camera_points[i][1], camera_points[i][2], 1.0f};
        mul_m4_v4(camera_obj->object_to_world, world_location);
        mul_m4_v4(region_data->persmat, world_location);

        if (world_location[3] > 0.0) {
          screen_points[i][0] = screen_width * 0.5f + screen_width * 0.5f * (world_location[0] / world_location[3]);
          screen_points[i][1] = screen_height * 0.5f + screen_height * 0.5f * (world_location[1] / world_location[3]);
        }
      }

      // getting camera view region
      float x1_f = min({screen_points[0][0], screen_points[1][0], screen_points[2][0], screen_points[3][0]});
      float x2_f = max({screen_points[0][0], screen_points[1][0], screen_points[2][0], screen_points[3][0]});
      float y1_f = min({screen_points[0][1], screen_points[1][1], screen_points[2][1], screen_points[3][1]});
      float y2_f = max({screen_points[0][1], screen_points[1][1], screen_points[2][1], screen_points[3][1]});

      // adjusting region to border
      float x = x1_f, y = y1_f;
      float dx = x2_f - x1_f, dy = y2_f - y1_f;

      x1 = x + scene->r.border.xmin * dx;
      x2 = x + scene->r.border.xmax * dx;
      y1 = y + scene->r.border.ymin * dy;
      y2 = y + scene->r.border.ymax * dy;

      // adjusting to region screen resolution
      x1 = max(min(x1, screen_width), 0);
      x2 = max(min(x2, screen_width), 0);
      y1 = max(min(y1, screen_height), 0);
      y2 = max(min(y2, screen_height), 0);
    }
  }
  else {
    if (view3d->flag2 & V3D_RENDER_BORDER) {
      int x = x1, y = y1;
      int dx = x2 - x1, dy = y2 - y1;
      
      x1 = int(x + view3d->render_border.xmin * dx);
      x2 = int(x + view3d->render_border.xmax * dx);
      y1 = int(y + view3d->render_border.ymin * dy);
      y2 = int(y + view3d->render_border.ymax * dy);
    }
  }

  border = GfVec4i(x1, y1, x2 - x1, y2 - y1);
}

int ViewSettings::width()
{
  return border[2];
}

int ViewSettings::height()
{
  return border[3];
}

GfCamera ViewSettings::gf_camera()
{
  return camera_data.gf_camera(GfVec4f(
    (float)border[0] / screen_width, (float)border[1] / screen_height,
    (float)border[2] / screen_width, (float)border[3] / screen_height));
}

GLTexture::GLTexture()
  : textureId(0)
  , width(0)
  , height(0)
  , channels(4)
{
}

GLTexture::~GLTexture()
{
  if (textureId) {
    free();
  }
}

void GLTexture::setBuffer(HdRenderBuffer *buffer)
{
  if (!textureId) {
    create(buffer);
    return;
  }

  if (width != buffer->GetWidth() || height != buffer->GetHeight()) {
    free();
    create(buffer);
    return;
  }

  glBindTexture(GL_TEXTURE_2D, textureId);
    
  void *data = buffer->Map();
  glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, width, height, GL_RGBA, GL_FLOAT, data);
  buffer->Unmap();
}

void GLTexture::create(HdRenderBuffer *buffer)
{
  width = buffer->GetWidth();
  height = buffer->GetHeight();
  channels = HdGetComponentCount(buffer->GetFormat());

  glGenTextures(1, &textureId);
  glBindTexture(GL_TEXTURE_2D, textureId);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
  
  void *data = buffer->Map();
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA16F, width, height, 0, GL_RGBA, GL_FLOAT, data);
  buffer->Unmap();
}

void GLTexture::free()
{
  glDeleteTextures(1, &textureId);
  textureId = 0;
}

void GLTexture::draw(GLfloat x, GLfloat y)
{
  // INITIALIZATION

  // Getting shader program
  GLint shader_program;
  glGetIntegerv(GL_CURRENT_PROGRAM, &shader_program);

  // Generate vertex array
  GLuint vertex_array;
  glGenVertexArrays(1, &vertex_array);

  GLint texturecoord_location = glGetAttribLocation(shader_program, "texCoord");
  GLint position_location = glGetAttribLocation(shader_program, "pos");

  // Generate geometry buffers for drawing textured quad
  GLfloat position[8] = { x, y, x + width, y, x + width, y + height, x, y + height };
  GLfloat texcoord[8] = {0.0, 0.0, 1.0, 0.0, 1.0, 1.0, 0.0, 1.0};

  GLuint vertex_buffer[2];
  glGenBuffers(2, vertex_buffer);
  glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer[0]);
  glBufferData(GL_ARRAY_BUFFER, 32, position, GL_STATIC_DRAW);
  glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer[1]);
  glBufferData(GL_ARRAY_BUFFER, 32, texcoord, GL_STATIC_DRAW);
  glBindBuffer(GL_ARRAY_BUFFER, 0);

  // DRAWING
  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_2D, textureId);

  glBindVertexArray(vertex_array);
  glEnableVertexAttribArray(texturecoord_location);
  glEnableVertexAttribArray(position_location);

  glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer[0]);
  glVertexAttribPointer(position_location, 2, GL_FLOAT, GL_FALSE, 0, nullptr);
  glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer[1]);
  glVertexAttribPointer(texturecoord_location, 2, GL_FLOAT, GL_FALSE, 0, nullptr);
  glBindBuffer(GL_ARRAY_BUFFER, 0);

  glDrawArrays(GL_TRIANGLE_FAN, 0, 4);

  glBindVertexArray(0);
  glBindTexture(GL_TEXTURE_2D, 0);

  // DELETING
  glDeleteBuffers(2, vertex_buffer);
  glDeleteVertexArrays(1, &vertex_array);
}

void ViewportEngine::sync(Depsgraph *depsgraph, bContext *context, HdRenderSettingsMap &renderSettings)
{
  if (!scene_delegate) {
    scene_delegate = std::make_unique<BlenderSceneDelegate>(render_index.get(), 
      SdfPath::AbsoluteRootPath().AppendElementString("scene"), BlenderSceneDelegate::EngineType::Viewport);
  }
  scene_delegate->populate(depsgraph, context);

  for (auto const& setting : renderSettings) {
    render_delegate->SetRenderSetting(setting.first, setting.second);
  }
}

void ViewportEngine::render(Depsgraph *depsgraph, bContext *context)
{
  ViewSettings viewSettings(context);
  if (viewSettings.width() * viewSettings.height() == 0) {
    return;
  };

  GfCamera gfCamera = viewSettings.gf_camera();
  free_camera_delegate->SetCamera(gfCamera);
  render_task_delegate->SetCameraAndViewport(free_camera_delegate->GetCameraId(), 
    GfVec4d(viewSettings.border[0], viewSettings.border[1], viewSettings.border[2], viewSettings.border[3]));
  if (simple_light_task_delegate) {
    simple_light_task_delegate->SetCameraPath(free_camera_delegate->GetCameraId());
  }

  if ((bl_engine->type->flag & RE_USE_GPU_CONTEXT) == 0) {
    render_task_delegate->SetRendererAov(HdAovTokens->color);
  }

  if (renderer_percent_done() == 0.0f) {
    timeBegin = chrono::steady_clock::now();
  }

  GPUShader *shader = GPU_shader_get_builtin_shader(GPU_SHADER_3D_IMAGE);
  GPU_shader_bind(shader);

  HdTaskSharedPtrVector tasks;
  if (simple_light_task_delegate) {
    tasks.push_back(simple_light_task_delegate->GetTask());
  }
  tasks.push_back(render_task_delegate->GetTask());

  {
    /* Release the GIL before calling into hydra, in case any hydra plugins call into python. */
    TF_PY_ALLOW_THREADS_IN_SCOPE();
    engine->Execute(render_index.get(), &tasks);

    if ((bl_engine->type->flag & RE_USE_GPU_CONTEXT) == 0) {
      texture.setBuffer(render_task_delegate->GetRendererAov(HdAovTokens->color));
      texture.draw((GLfloat)viewSettings.border[0], (GLfloat)viewSettings.border[1]);
    }
  }

  GPU_shader_unbind();

  chrono::time_point<chrono::steady_clock> timeCurrent = chrono::steady_clock::now();
  chrono::milliseconds elapsedTime = chrono::duration_cast<chrono::milliseconds>(timeCurrent - timeBegin);

  string formattedTime = format_duration(elapsedTime);

  if (!render_task_delegate->IsConverged()) {
    notify_status("Time: " + formattedTime + " | Done: " + to_string(int(renderer_percent_done())) + "%", "Render");
    bl_engine->flag |= RE_ENGINE_DO_DRAW;
  }
  else {
    notify_status(("Time: " + formattedTime).c_str(), "Rendering Done");
  }
}

void ViewportEngine::render(Depsgraph *depsgraph)
{
  /* Empty function */
}

void ViewportEngine::notify_status(const string &info, const string &status)
{
  RE_engine_update_stats(bl_engine, status.c_str(), info.c_str());
}

}   // namespace blender::render::hydra
