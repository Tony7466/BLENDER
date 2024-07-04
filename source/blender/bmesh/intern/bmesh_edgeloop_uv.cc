#include "bmesh_edgeloop_uv.hh"
#include "bmesh.hh"
#include <queue>
using namespace blender;

void UV_get_edgeloops(
    const Scene *scene,
    BMesh *bm,
    std::vector<std::vector<std::vector<BMLoop *>>> *edgeloops,
    blender::FunctionRef<bool(const Scene *scene, BMLoop *l, const BMUVOffsets offsets)> callback)

{
  BMVert *v;
  BMLoop *l;
  BMIter viter, liter;
  const BMUVOffsets offsets = BM_uv_map_get_offsets(bm);

  std::queue<BMLoop *> queue;
  BM_ITER_MESH (v, &viter, bm, BM_VERTS_OF_MESH) {
    BM_ITER_ELEM (l, &liter, v, BM_LOOPS_OF_VERT) {
      if (callback(scene, l, offsets) and (l->head.index != -1 and l->head.index != -2)) {
        queue.push(l);
      }
      else {
        continue;
      }
      std::vector<std::vector<BMLoop *>> edgeloop;
      // while the queue is not empty
      while (!queue.empty()) {
        // pop the front of queue and store in queuel
        BMLoop *queuel = queue.front();
        queue.pop();

        // get UV location of queuel
        float *luv = BM_ELEM_CD_GET_FLOAT_P(queuel, offsets.uv);

        // get the vertex of queuel
        BMVert *queuev = queuel->v;

        // vector of current UV coord
        std::vector<BMLoop *> uvcoord;

        // iterate over the loops of the vertex
        BMLoop *currl;
        BMIter liter2;
        int selectedconnections = 0;
        BM_ITER_ELEM (currl, &liter2, queuev, BM_LOOPS_OF_VERT) {
          float *currluv = BM_ELEM_CD_GET_FLOAT_P(currl, offsets.uv);
          if (currluv[0] != luv[0] || currluv[1] != luv[1]) {
            continue;
          }
          BM_elem_index_set(currl, -1);
          uvcoord.push_back(currl);

          if (callback(scene, currl->next, offsets)) {
            selectedconnections++;
            if (currl->next->head.index != -1 and currl->next->head.index != -2) {
              queue.push(currl->next);
            }
          }
          if (callback(scene, currl->prev, offsets)) {
            selectedconnections++;
            if (currl->prev->head.index != -1 and currl->prev->head.index != -2) {
              queue.push(currl->prev);
            }
          }
        }
        if (selectedconnections < 2) {
          for (BMLoop *endpointloop : uvcoord) {
            BM_elem_index_set(endpointloop, -2);
          }
        }
        edgeloop.push_back(uvcoord);
      }
      if (!edgeloop.empty()) {
        edgeloops->push_back(edgeloop);
      }
    }
  }
  return;
}
