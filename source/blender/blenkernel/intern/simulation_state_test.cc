#include "BKE_simulation_state.hh"

#include "testing/testing.h"

namespace blender::bke::tests {
TEST(simulation, sim_state_cache_starts_empty)
{
  sim::ModifierSimulationCache sim_cache;
  EXPECT_FALSE(sim_cache.has_state_at_frame(SubFrame(0)));
  EXPECT_FALSE(sim_cache.has_state_at_frame(SubFrame(20)));
  EXPECT_FALSE(sim_cache.has_states());
}

TEST(simulation, sim_state_cache_caches_state)
{
  sim::ModifierSimulationCache sim_cache;

  auto &state20_old = sim_cache.get_state_at_frame_for_write(SubFrame(20));

  /* Set some data to make sure we get the same state back later on. */
  state20_old.meta_path_ = "test";

  /* Add frames before and after to spicy things up. */
  sim_cache.get_state_at_frame_for_write(SubFrame(0));
  sim_cache.get_state_at_frame_for_write(SubFrame(10));

  const auto *state20_new = sim_cache.get_state_at_exact_frame(SubFrame(20));
  EXPECT_EQ(state20_new->meta_path_, "test");
}

TEST(simulation, sim_state_cache_get_state_around_frame_empty)
{
  sim::ModifierSimulationCache sim_cache;

  auto stateAroundFrames = sim_cache.get_states_around_frame(SubFrame(0));
  EXPECT_EQ(stateAroundFrames.prev, nullptr);
  EXPECT_EQ(stateAroundFrames.current, nullptr);
  EXPECT_EQ(stateAroundFrames.next, nullptr);
}

TEST(simulation, sim_state_cache_get_state_around_frame_current)
{
  sim::ModifierSimulationCache sim_cache;

  sim_cache.get_state_at_frame_for_write(SubFrame(0));

  auto stateAroundFrames = sim_cache.get_states_around_frame(SubFrame(0));
  EXPECT_EQ(stateAroundFrames.prev, nullptr);
  EXPECT_NE(stateAroundFrames.current, nullptr);
  EXPECT_EQ(stateAroundFrames.next, nullptr);
}

TEST(simulation, sim_state_cache_get_state_around_frame_prev)
{
  sim::ModifierSimulationCache sim_cache;

  sim_cache.get_state_at_frame_for_write(SubFrame(0));

  auto stateAroundFrames = sim_cache.get_states_around_frame(SubFrame(1));
  EXPECT_NE(stateAroundFrames.prev, nullptr);
  EXPECT_EQ(stateAroundFrames.current, nullptr);
  EXPECT_EQ(stateAroundFrames.next, nullptr);
}

TEST(simulation, sim_state_cache_get_state_around_frame_next)
{
  sim::ModifierSimulationCache sim_cache;

  sim_cache.get_state_at_frame_for_write(SubFrame(2));

  auto stateAroundFrames = sim_cache.get_states_around_frame(SubFrame(1));
  EXPECT_EQ(stateAroundFrames.prev, nullptr);
  EXPECT_EQ(stateAroundFrames.current, nullptr);
  EXPECT_NE(stateAroundFrames.next, nullptr);
}

TEST(simulation, sim_state_cache_get_state_around_frame_all)
{
  sim::ModifierSimulationCache sim_cache;

  sim_cache.get_state_at_frame_for_write(SubFrame(0));
  sim_cache.get_state_at_frame_for_write(SubFrame(1));
  sim_cache.get_state_at_frame_for_write(SubFrame(2));

  auto stateAroundFrames = sim_cache.get_states_around_frame(SubFrame(1));
  EXPECT_NE(stateAroundFrames.prev, nullptr);
  EXPECT_NE(stateAroundFrames.current, nullptr);
  EXPECT_NE(stateAroundFrames.next, nullptr);
}

}  // namespace blender::bke::tests
