#pragma once

#include "BKE_curves.hh"
#include "DNA_curve_types.h"

using blender::Vector;

namespace blender::bke::curves::bezier {

class LegacyBezierTess {
 public:
  virtual void init(const struct Nurb *nurb) = 0;
  virtual int position_count() = 0;
  virtual void get_positions(const BezTriple *bezt,
                             const BezTriple *prevbezt,
                             const int segment,
                             const int stride,
                             float *dest,
                             int &pnts) = 0;
  virtual ~LegacyBezierTess(){};
};

class LegacyForwardTess : public LegacyBezierTess {
  const Nurb *nurb;

 public:
  void init(const struct Nurb *nurb);
  int position_count();
  void get_positions(const BezTriple *bezt,
                     const BezTriple *prevbezt,
                     const int segment,
                     const int stride,
                     float *dest,
                     int &pnts);
};

class LegacyDeCasteljauTess : public LegacyBezierTess {
  Vector<float3> positions;
  Vector<int> segment_offsets;

 public:
  void init(const struct Nurb *nurb);
  int position_count();
  void get_positions(const BezTriple *bezt,
                     const BezTriple *prevbezt,
                     const int segment,
                     const int stride,
                     float *dest,
                     int &pnts);
};

void decasteljau_evaluate_segment(const float3 &point_0,
                                  const float3 &point_1,
                                  const float3 &point_2,
                                  const float3 &point_3,
                                  const short angle,
                                  Vector<float3> &dest);

}  // namespace blender::bke::curves::bezier
