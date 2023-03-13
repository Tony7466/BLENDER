#include "BKE_grease_pencil.hh"

Span<GreasePencilDrawing> blender::bke::GreasePencil::drawings() const
{
  return Span<GreasePencilDrawing>{this->drawing_array, this->drawing_array_size};
}