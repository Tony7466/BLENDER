/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: Apache-2.0 */

#include "integrator/work_balancer.h"

#include "util/math.h"

#include "util/log.h"

CCL_NAMESPACE_BEGIN

void work_balance_do_initial(vector<WorkBalanceInfo> &work_balance_infos, int cpu_index)
{
  const int num_infos = work_balance_infos.size();

  if (num_infos == 1) {
    work_balance_infos[0].weight = 1.0;
    return;
  }
  else if (num_infos == 0) {
    return;
  }

  /* There is no statistics available, so start with an equal distribution. */
  /* initially assume the CPU is much slower (100x) than the GPU */
  int cpu_used = ((cpu_index == -1) ? 0 : 1);
  const double GPU_WEIGHT = 100.0;
  const double total_weigth = GPU_WEIGHT*(num_infos - cpu_used) + cpu_used;
  //const double weight = 1.0 / num_infos;
  for (int device_index = 0;device_index < num_infos; device_index++) {
      work_balance_infos[device_index].weight = ((cpu_index == device_index) ? 1 : GPU_WEIGHT)/total_weigth;
  }
}

static double calculate_total_time(const vector<WorkBalanceInfo> &work_balance_infos)
{
  double total_time = 0;
  for (const WorkBalanceInfo &info : work_balance_infos) {
    total_time += info.time_spent;
  }
  return total_time;
}

/* The balance is based on equalizing time which devices spent performing a task. Assume that
 * average of the observed times is usable for estimating whether more or less work is to be
 * scheduled, and how difference in the work scheduling is needed. */

bool work_balance_do_rebalance(vector<WorkBalanceInfo> &work_balance_infos)
{
  const int num_infos = work_balance_infos.size();

  const double total_time = calculate_total_time(work_balance_infos);
  const double time_average = total_time / num_infos;

  double total_weight = 0;
  vector<double> new_weights;
  new_weights.reserve(num_infos);

  /* Equalize the overall average time. This means that we don't make it so every work will perform
   * amount of work based on the current average, but that after the weights changes the time will
   * equalize.
   * Can think of it that if one of the devices is 10% faster than another, then one device needs
   * to do 5% less of the current work, and another needs to do 5% more. */
  const double lerp_weight = 1.0 / num_infos;

  bool has_big_difference = false;

  for (const WorkBalanceInfo &info : work_balance_infos) {
    double work_time = (info.time_spent > time_average) ? time_average/2.0 : time_average;
    /* Quickly reduce weight if device is taking too long otherwise slowly increase the weight */
    double w = lerp_weight;
    if(time_average < info.time_spent) {
      w = 1.0f;
    }
    const double time_target = mix(info.time_spent, time_average, w);
    const double new_weight = info.weight * time_target / info.time_spent;
    new_weights.push_back(new_weight);
    total_weight += new_weight;

    double diff = time_target - time_average;
    if ((diff < 0.0) || (diff > 0.5)) {
      has_big_difference = true;
    }
  }

  if (!has_big_difference) {
    return false;
  }

  const double total_weight_inv = 1.0 / total_weight;
  for (int i = 0; i < num_infos; ++i) {
    WorkBalanceInfo &info = work_balance_infos[i];
    info.weight = new_weights[i] * total_weight_inv;
    info.time_spent = 0;
  }

  return true;
}

CCL_NAMESPACE_END
