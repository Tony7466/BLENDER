#include <cstdint>
#include <cstdio>
#include <vector>

#include <zstd.h>

int main(int argc, const char **argv)
{
  if (argc < 3) {
    return -1;
  }

  FILE *in = fopen(argv[1], "rb");
  if (in == nullptr) {
    return -1;
  }

  if (fseek(in, 0, SEEK_END) < 0) {
    return -1;
  }
  long in_size = ftell(in);
  if (in_size < 0) {
    return -1;
  }
  if (fseek(in, 0, SEEK_SET) < 0) {
    return -1;
  }

  std::vector<uint8_t> in_data(in_size);
  if (fread(in_data.data(), 1, in_size, in) != in_size) {
    return -1;
  }
  fclose(in);

  size_t out_size = ZSTD_compressBound(in_size);
  if (ZSTD_isError(out_size)) {
    return -1;
  }
  std::vector<uint8_t> out_data(out_size);

  out_size = ZSTD_compress(out_data.data(), out_data.size(), in_data.data(), in_data.size(), 19);
  if (ZSTD_isError(out_size)) {
    return -1;
  }

  FILE *out = fopen(argv[2], "wb");
  if (out == nullptr) {
    return -1;
  }

  if (fwrite(out_data.data(), 1, out_size, out) != out_size) {
    return -1;
  }
  fclose(out);

  return 0;
}
