#pragma once
#include "BKE_customdata.hh"
#include <set>
#include <tuple>
#include <unordered_map>

struct ARegion;
struct ARegionType;
struct BMEditMesh;
struct BMFace;
struct BMLoop;
struct BMesh;
struct Image;
struct ImageUser;
struct Main;
struct Object;
struct Scene;
struct SpaceImage;
struct ToolSettings;
struct View2D;
struct ViewLayer;
struct bContext;
struct bNode;
struct bNodeTree;
struct wmKeyConfig;

void UV_get_edgeloops(
    const Scene *scene,
    BMesh *bm,
    std::vector<std::vector<std::vector<BMLoop *>>> *edgeloops,
    blender::FunctionRef<bool(const Scene *scene, BMLoop *l, const BMUVOffsets offsets)> callback);
