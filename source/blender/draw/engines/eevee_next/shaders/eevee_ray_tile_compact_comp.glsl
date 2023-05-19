/**
 * This pass load tiles mask in regions and add them to the tile list in Z order.
 * This increases trace coherency and thus performance.
 */

void main()
{
  /* TODO(fclem): This is just placeholder high level description of the algorithm. */
  /* Load tile mask in region. */
  /* If tile is used, insert into local Zorder queue. */
  barrier();
  /* Get global offset into tile buffer and output the whole local queue. */
  /* Increment the dispatch argument by the number of tile passing the test. */
}