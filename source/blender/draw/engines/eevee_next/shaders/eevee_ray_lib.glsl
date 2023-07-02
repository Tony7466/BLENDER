
struct Ray {
  vec3 origin;
  vec3 direction;
  float time;
};

/**
 * Screenspace ray ([0..1] "uv" range) where direction is normalize to be as small as one
 * full-resolution pixel. The ray is also clipped to all frustum sides.
 */
struct ScreenSpaceRay {
  vec4 origin;
  vec4 direction;
  float max_time;
};
