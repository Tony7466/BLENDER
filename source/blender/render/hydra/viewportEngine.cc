/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#include <epoxy/gl.h>

#include <pxr/base/gf/camera.h>
#include <pxr/imaging/glf/drawTarget.h>
#include <pxr/usd/usdGeom/camera.h>

#include "BLI_math_matrix.h"
#include "DNA_camera_types.h"

#include "glog/logging.h"

#include "viewportEngine.h"
#include "camera.h"
#include "utils.h"

using namespace std;
using namespace pxr;

namespace blender::render::hydra {

struct ViewSettings {
  ViewSettings(BL::Context &b_context);

  int width();
  int height();

  GfCamera gf_camera();

  CameraData camera_data;

  int screen_width;
  int screen_height;
  GfVec4i border;
};

ViewSettings::ViewSettings(BL::Context &b_context)
  : camera_data(b_context)
{
  screen_width = b_context.region().width();
  screen_height = b_context.region().height();

  float width_half = screen_width / 2.0f;
  float height_half = screen_height / 2.0f;

  BL::Scene b_scene = b_context.scene();

  //getting render border
  int x1 = 0, y1 = 0;
  int x2 = screen_width, y2 = screen_height;

  if (b_context.region_data().view_perspective() == BL::RegionView3D::view_perspective_CAMERA) {
    if (b_scene.render().use_border()) {
      BL::Object b_camera_obj = b_scene.camera();
      BL::Camera b_camera = (BL::Camera)b_camera_obj.data();

      float camera_points[4][3];

      b_camera.view_frame(b_scene, camera_points[0], camera_points[1], camera_points[2], camera_points[3]);

      BL::Array<float, 16> region_persp_matrix = b_context.region_data().perspective_matrix();
      BL::Array<float, 16> camera_world_matrix = b_camera_obj.matrix_world();

      float screen_points[4][2];

      for (int i = 0 ; i < 4; i++) {
        float world_location[] = {camera_points[i][0], camera_points[i][1], camera_points[i][2], 1.0f};
        mul_m4_v4((float(*)[4])camera_world_matrix.data, world_location);
        mul_m4_v4((float(*)[4])region_persp_matrix.data, world_location);

        if (world_location[3] > 0.0) {
          screen_points[i][0] = width_half + width_half * (world_location[0] / world_location[3]);
          screen_points[i][1] = height_half + height_half * (world_location[1] / world_location[3]);
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

      x1 = x + b_scene.render().border_min_x() * dx;
      x2 = x + b_scene.render().border_max_x() * dx;
      y1 = y + b_scene.render().border_min_y() * dy;
      y2 = y + b_scene.render().border_max_y() * dy;

      // adjusting to region screen resolution
      x1 = max(min(x1, screen_width), 0);
      x2 = max(min(x2, screen_width), 0);
      y1 = max(min(y1, screen_height), 0);
      y2 = max(min(y2, screen_height), 0);
    }
  }
  else {
    if (((BL::SpaceView3D)b_context.space_data()).use_render_border()) {
      int x = x1, y = y1;
      int dx = x2 - x1, dy = y2 - y1;

      x1 = int(x + ((BL::SpaceView3D)b_context.space_data()).render_border_min_x() * dx);
      x2 = int(x + ((BL::SpaceView3D)b_context.space_data()).render_border_max_x() * dx);
      y1 = int(y + ((BL::SpaceView3D)b_context.space_data()).render_border_min_y() * dy);
      y2 = int(y + ((BL::SpaceView3D)b_context.space_data()).render_border_max_y() * dy);
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

void ViewportEngine::sync(BL::Depsgraph &b_depsgraph, BL::Context &b_context, HdRenderSettingsMap &renderSettings)
{
  if (!sceneDelegate) {
    sceneDelegate = std::make_unique<BlenderSceneDelegate>(renderIndex.get(), 
      SdfPath::AbsoluteRootPath().AppendElementString("scene"));
  }
  sceneDelegate->Populate(b_depsgraph, b_context);

  for (auto const& setting : renderSettings) {
    renderDelegate->SetRenderSetting(setting.first, setting.second);
  }
}

void ViewportEngine::viewDraw(BL::Depsgraph &b_depsgraph, BL::Context &b_context)
{
  ViewSettings viewSettings(b_context);
  if (viewSettings.width() * viewSettings.height() == 0) {
    return;
  };

  BL::Scene b_scene = b_depsgraph.scene_eval();
  GfCamera gfCamera = viewSettings.gf_camera();

  freeCameraDelegate->SetCamera(gfCamera);
  renderTaskDelegate->SetCameraAndViewport(freeCameraDelegate->GetCameraId(), 
    GfVec4d(viewSettings.border[0], viewSettings.border[1], viewSettings.border[2], viewSettings.border[3]));

  if (!b_engine.bl_use_gpu_context()) {
    renderTaskDelegate->SetRendererAov(HdAovTokens->color);
  }
  
  HdTaskSharedPtrVector tasks = renderTaskDelegate->GetTasks();

  if (getRendererPercentDone() == 0.0f) {
    timeBegin = chrono::steady_clock::now();
  }

  b_engine.bind_display_space_shader(b_scene);

  {
    // Release the GIL before calling into hydra, in case any hydra plugins call into python.
    TF_PY_ALLOW_THREADS_IN_SCOPE();
    engine->Execute(renderIndex.get(), &tasks);

    if (!b_engine.bl_use_gpu_context()) {
      texture.setBuffer(renderTaskDelegate->GetRendererAov(HdAovTokens->color));
      texture.draw((GLfloat)viewSettings.border[0], (GLfloat)viewSettings.border[1]);
    }
  }
  
  b_engine.unbind_display_space_shader();

  chrono::time_point<chrono::steady_clock> timeCurrent = chrono::steady_clock::now();
  chrono::milliseconds elapsedTime = chrono::duration_cast<chrono::milliseconds>(timeCurrent - timeBegin);

  string formattedTime = format_duration(elapsedTime);

  if (!renderTaskDelegate->IsConverged()) {
    notifyStatus("Time: " + formattedTime + " | Done: " + to_string(int(getRendererPercentDone())) + "%",
                 "Render");
    b_engine.tag_redraw();
  }
  else {
    notifyStatus(("Time: " + formattedTime).c_str(), "Rendering Done");
  }
}

void ViewportEngine::notifyStatus(const string &info, const string &status)
{
  b_engine.update_stats(status.c_str(), info.c_str());
}

}   // namespace blender::render::hydra
