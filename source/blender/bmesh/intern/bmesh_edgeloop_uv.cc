#include "bmesh_edgeloop_uv.hh"
#include "bmesh.hh"
using namespace blender;

void UV_get_edgeloops(
    const Scene *scene,
    BMesh *bm,
    std::vector<std::vector<std::vector<BMLoop *>>> *edgeloops,
    blender::FunctionRef<bool(const Scene *scene, BMLoop *l, const BMUVOffsets offsets)> callback)
{
  BMLoop *l;
  BMIter viter, liter;
  const BMUVOffsets offset = BM_uv_map_get_offsets(bm);

  int counter = 0;

  BM_ITER_MESH (l, &viter, bm, BM_VERTS_OF_MESH) {
    BM_ITER_ELEM (l, &liter, l, BM_LOOPS_OF_VERT) {
      if (callback(scene, l, offset)) {
        counter++;
      }
    }
  }
  return;
}
