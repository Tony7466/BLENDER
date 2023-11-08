#include "curve_bezier_tess.hh"
#include "BKE_curve.h"

namespace blender::bke::curves::bezier {

/**
 * Calculates and caches points for all segments of given Nurb.
 * Precalculation needed to know final number of tess points in advance.
 */
void LegacyDeCasteljauTess::init(const struct Nurb *nurb)
{
  const bool cyclic = nurb->flagu & CU_NURB_CYCLIC;
  int segments = nurb->pntsu - 1;

  positions.clear();
  segment_offsets.clear();

  BezTriple *bezt = nurb->bezt;
  BezTriple *prevbezt = nurb->bezt;
  if (cyclic) {
    segments++;
    prevbezt = nurb->bezt + (nurb->pntsu - 1);
  }
  else {
    prevbezt = bezt;
    bezt++;
  }

  for (int s : IndexRange(segments)) {
    UNUSED_VARS(s);
    const float(&prev)[3][3] = prevbezt->vec;
    const float(&curr)[3][3] = bezt->vec;
    segment_offsets.append(positions.size());
    decasteljau_evaluate_segment(prev[1], prev[2], curr[0], curr[1], nurb->resolu, positions);
    prevbezt = bezt;
    bezt++;
  }
  segment_offsets.append(positions.size());
  positions.append((nurb->bezt + (nurb->pntsu - 1))->vec[1]);
}

int LegacyDeCasteljauTess::position_count()
{
  return positions.size();
}

void LegacyDeCasteljauTess::get_positions(const BezTriple * /*bezt*/,
                                          const BezTriple * /*prevbezt*/,
                                          const int segment,
                                          const int stride,
                                          float *dest,
                                          int &pnts)

{
  const int start = segment_offsets[segment];
  const int count = segment_offsets[segment + 1] - start;
  pnts = count;
  for (const float3 &p : positions.as_span().slice(start, count)) {
    copy_v3_v3(dest, p);
    dest = (float *)POINTER_OFFSET(dest, stride);
  }
}

void LegacyForwardTess::init(const struct Nurb *nurb)
{
  this->nurb = nurb;
}

int LegacyForwardTess::position_count()
{
  const int resolution = nurb->resolu;
  const int segment_count = SEGMENTSU(nurb);
  /* in case last point is not cyclic */
  return segment_count * resolution + 1;
}
void LegacyForwardTess::get_positions(const BezTriple *bezt,
                                      const BezTriple *prevbezt,
                                      const int /* segment */,
                                      const int /* stride */,
                                      float *dest,
                                      int &pnts)
{
  pnts = nurb->resolu;

  for (int j = 0; j < 3; j++) {
    BKE_curve_forward_diff_bezier(prevbezt->vec[1][j],
                                  prevbezt->vec[2][j],
                                  bezt->vec[0][j],
                                  bezt->vec[1][j],
                                  dest + j,
                                  pnts,
                                  sizeof(BevPoint));
  }
}
/**
 * Checks if angle between vecotors ab and cd is less than angle given as squared cosine.
 */
bool angle_test(float3 a, float3 b, float3 c, float3 d, float angle_cos_sqr)
{
  const float3 start = b - a;
  const float3 end = d - c;

  const float dot_prod = dot_v3v3(start, end);
  if (dot_prod < 0) {
    return true;
  }

  const float lengths_squared = len_squared_v3(start) * len_squared_v3(end);

  return dot_prod * dot_prod < angle_cos_sqr * lengths_squared;
}

bool subdivision_needed(const float3 points[], const int degree, float angle_cos_sqr)
{
  bool ends = angle_test(points[0], points[1], points[degree - 1], points[degree], angle_cos_sqr);
  if (ends) {
    return true;
  }
  for (int i : IndexRange(0, degree - 2)) {
    if (angle_test(points[i], points[i + 1], points[i + 1], points[i + 2], angle_cos_sqr)) {
      return true;
    }
  }
  return false;
}

/**
 * Subdivides Bezier curve segment.
 * \param degree: Degree of curve.
 * \param depth: Current recursion level.
 * \param angle_cos_sqr: Stop angle's cosine squared.
 */
void subdivide(
    const float3 points[], int degree, Vector<float3> &dest, const int depth, float angle_cos_sqr)
{
  if (depth < 15 && subdivision_needed(points, degree, angle_cos_sqr)) {
    float3 new_points[2 * degree + 1];
    for (int i : IndexRange(degree + 1)) {
      const int j = i << 1;
      new_points[j] = points[i];
    }

    for (int k : IndexRange(1, degree + 1)) {
      for (int i = k - 1; i < 2 * degree - k; i += 2) {
        new_points[i + 1] = (new_points[i] + new_points[i + 2]) * 0.5;
      }
    }

    subdivide(new_points, degree, dest, depth + 1, angle_cos_sqr);
    dest.append(new_points[degree]);
    subdivide(new_points + degree, degree, dest, depth + 1, angle_cos_sqr);
  }
}

/**
 * Calculates cubic Bezier curves points using De Casteljau's algorithm.
 * \param angle: Subdivision stops if angles between control point leg pairs is less than given.
 */
void decasteljau_evaluate_segment(const float3 &point_0,
                                  const float3 &point_1,
                                  const float3 &point_2,
                                  const float3 &point_3,
                                  const short angle,
                                  Vector<float3> &dest)
{
  const float angle_cos = cos(M_PI * angle / 180);
  const float3 points[] = {point_0, point_1, point_2, point_3};
  dest.append(point_0);
  subdivide(points, 3, dest, 0, angle_cos * angle_cos);
}

}  // namespace blender::bke::curves::bezier