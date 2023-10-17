/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

struct OccupancyBits {
  uint bits[8];
};

int occupancy_bit_index_from_depth(float depth, int bit_count)
{
  /* We want the occupancy at the center of each range a bit covers.
   * So we round the depth to the nearest bit. */
  return int(depth * float(bit_count) + 0.5);
}

/**
 * Example with for 16bits per layer and 2 layers.
 * 0    Layer0   15  16   Layer1   31  < Bits index from LSB to MSB (left to right)
 * |--------------|  |--------------|
 * 0000000001111111  1111111111111111  < Surf0
 * 0000000000000000  0000001111111111  < Surf1
 * 0000000001111111  1111110000000000  < Result occupancy at each froxel depths
 * \a depth in [0..1] range.
 * \a bit_count in [1..256] range.
 */
OccupancyBits occupancy_from_depth(float depth, int bit_count)
{
  /* We want the occupancy at the center of each range a bit covers.
   * So we round the depth to the nearest bit. */
  int depth_bit_index = occupancy_bit_index_from_depth(depth, bit_count);

  OccupancyBits occupancy;
  for (int i = 0; i < 8; i++) {
    int shift = clamp(depth_bit_index - i * 32, 0, 32);
    /* Cannot bit shift more than 31 positions. */
    occupancy.bits[i] = (shift == 32) ? 0x0u : (~0x0u << uint(shift));
  }
  return occupancy;
}

/**
 * Example with for 16bits per layer and 2 layers.
 * 0    Layer0   15  16   Layer1   31  < Bits index from LSB to MSB (left to right)
 * |--------------|  |--------------|
 * 0000000001000000  0000000000000000  < Surf0
 * 0000000000000000  0000001000000000  < Surf1
 * \a depth in [0..1] range.
 * \a bit_count in [1..256] range.
 */
OccupancyBits occupancy_bit_from_depth(float depth, int bit_count)
{
  /* We want the occupancy at the center of each range a bit covers.
   * So we round the depth to the nearest bit. */
  int depth_bit_index = occupancy_bit_index_from_depth(depth, bit_count);

  OccupancyBits occupancy;
  for (int i = 0; i < 8; i++) {
    int shift = depth_bit_index - i * 32;
    /* Cannot bit shift more than 31 positions. */
    occupancy.bits[i] = (shift >= 0 && shift < 32) ? (0x1u << uint(shift)) : 0x0u;
  }
  return occupancy;
}
