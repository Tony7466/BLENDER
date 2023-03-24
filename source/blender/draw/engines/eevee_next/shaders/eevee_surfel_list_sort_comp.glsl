
/**
 * Sort a buffer of surfel list by distance along a direction.
 * The resulting surfel lists are then the equivalent of a series of ray cast in the same
 * direction. The fact that the surfels are sorted gives proper occlusion.
 *
 * Sort by increasing `ray_distance`. Start of list is smallest value.
 *
 * Outputs a flat array of surfel indices. Each ray is a range inside the array. This allows
 * parallel processing in the light propagation phase.
 * Dispatched as 1 thread per list.
 */

#pragma BLENDER_REQUIRE(gpu_shader_utildefines_lib.glsl)

/**
 * A doubly-linked list implementation.
 * IMPORTANT: It is not general purpose as it only cover the cases needed by this shader.
 */
struct List {
  int first, last;
};

/* Return the split list after link_index. */
List list_split_after(inout List original, int link_index)
{
  int next_link = surfel_buf[link_index].next;
  int last_link = original.last;

  original.last = link_index;

  List split;
  split.first = next_link;
  split.last = last_link;

  surfel_buf[link_index].next = -1;
  surfel_buf[next_link].prev = -1;

  return split;
}

void list_add_tail(inout List list, int link_index)
{
  surfel_buf[link_index].next = -1;
  surfel_buf[link_index].prev = list.last;
  surfel_buf[list.last].next = link_index;
  list.last = link_index;
}

void list_insert_link_before(inout List list, int next_link, int new_link)
{
  if (list.first == next_link) {
    /* At beginning of list. */
    list.first = new_link;
  }
  int prev_link = surfel_buf[next_link].prev;
  surfel_buf[new_link].next = next_link;
  surfel_buf[new_link].prev = prev_link;
  surfel_buf[next_link].prev = new_link;
  if (prev_link != -1) {
    surfel_buf[prev_link].next = new_link;
  }
}

void main()
{
  int list_index = int(gl_GlobalInvocationID);
  if (list_index >= list_info_buf.list_max) {
    return;
  }

  int list_start = list_start_buf[list_index];

  if (list_start == -1) {
    /* Empty list. */
    return;
  }

  /* Create Surfel.prev pointers. */
  int prev_id = -1;
  for (int i = list_start; i > -1; i = surfel_buf[i].next) {
    surfel_buf[i].prev = prev_id;
    prev_id = i;
  }

  List sorted_list;
  sorted_list.first = list_start;
  sorted_list.last = prev_id;

  if (sorted_list.first == sorted_list.last) {
    /* Only one item. Nothing to sort. */
    return;
  }

  /* Using insertion sort as it is easier to implement. */

  List unsorted_list = list_split_after(sorted_list, sorted_list.first);

  /* Mutable foreach. */
  for (int i = unsorted_list.first, next; i > -1; i = next) {
    next = surfel_buf[i].next;

    bool insert = false;
    for (int j = sorted_list.first; j > -1; j = surfel_buf[j].next) {
      if (surfel_buf[j].ray_distance < surfel_buf[i].ray_distance) {
        list_insert_link_before(sorted_list, j, i);
        insert = true;
        break;
      }
    }
    if (insert == false) {
      list_add_tail(sorted_list, i);
    }
  }
}
