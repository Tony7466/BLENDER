
#if !(defined(SELECT_UNORDERED) || defined(SELECT_DEPTH_PICKING))
/* Avoid requesting the select_id when not in selection mode. */
#  define select_id_set(select_id)
#  define select_id_output(select_id)

#elif defined(GPU_VERTEX_SHADER)

void select_id_set(int id)
{
  /* Declared in the create info. */
  select_id = id;
}

#elif defined(GPU_FRAGMENT_SHADER)

void select_id_output(int id)
{
#  if defined(SELECT_UNORDERED)
  /* Used by rectangle selection.
   * Set the bit of the select id in the bitmap. */
  atomicOr(out_select_buf[id / 32u], 1u << (uint(id) % 32u));

#  elif defined(SELECT_DEPTH_PICKING)
  /* Used by mouse-clicking selection.
   * Stores the nearest depth for this select id. */
  atomicMin(out_select_buf[id], floatBitsToUint(gl_FragCoord.z));

#  else
#    error
#  endif
}

#endif
