/*
  planner.c - buffers movement commands and manages the acceleration profile plan
  Part of Grbl

  Copyright (c) 2011-2016 Sungeun K. Jeon for Gnea Research LLC
  Copyright (c) 2009-2011 Simen Svale Skogsrud
  Copyright (c) 2011 Jens Geisler

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "grbl.h"


static plan_block_t block_buffer[BLOCK_BUFFER_SIZE];  // A ring buffer for motion instructions
static uint8_t block_buffer_tail;     // Index of the block to process now
static uint8_t block_buffer_head;     // Index of the next block to be pushed
static uint8_t next_buffer_head;      // Index of the next buffer head
static uint8_t block_buffer_planned;  // Index of the optimally planned block

// Define planner variables
typedef struct {
  int32_t position[N_AXIS];          // The planner position of the tool in absolute steps. Kept separate
                                     // from g-code position for movements requiring multiple line motions,
                                     // i.e. arcs, canned cycles, and backlash compensation.
  float previous_unit_vec[N_AXIS];   // Unit vector of previous path line segment
  float previous_nominal_speed;  // Nominal speed of previous path line segment
} planner_t;
static planner_t pl;


// Returns the index of the next block in the ring buffer. Also called by stepper segment buffer.
uint8_t plan_next_block_index(uint8_t block_index)
{
  block_index++;
  if (block_index == BLOCK_BUFFER_SIZE) { block_index = 0; }
  return(block_index);
}


// Returns the index of the previous block in the ring buffer
// 该缓冲区是FIFO形式，head入队，tail出队，该函数是从tail往head方向移动的。
static uint8_t plan_prev_block_index(uint8_t block_index)
{
  if (block_index == 0) { block_index = BLOCK_BUFFER_SIZE; }
  block_index--;
  return(block_index);
}


/*                            PLANNER SPEED DEFINITION
                                     +--------+   <- current->nominal_speed（额定）
                                    /          \
         current->entry_speed ->   +            \
                                   |             + <- next->entry_speed (aka exit speed)
                                   +-------------+
                                       time -->

  根据以下基本准则重新计算运动计划:
  

    1. 从后往前遍历block计算entry_speed（连接点速度不能过大）
    Go over every feasible block sequentially in reverse order and calculate the junction speeds
        (i.e. current->entry_speed) such that:
      a. No junction speed exceeds the pre-computed maximum junction speed limit or nominal speeds of
         neighboring blocks.
      b. A block entry speed cannot exceed one reverse-computed from its exit speed (next->entry_speed)
         with a maximum allowable deceleration over the block travel distance.
      c. The last (or newest appended) block is planned from a complete stop (an exit speed of zero).
    2. 从前往后遍历block，检查exit_speed与entry_speed时候拟合。
    Go over every block in chronological (forward) order and dial down junction speed values if
      a. The exit speed exceeds the one forward-computed from its entry speed with the maximum allowable
         acceleration over the block travel distance.

  When these stages are complete, the planner will have maximized the velocity profiles throughout the all
  of the planner blocks, where every block is operating at its maximum allowable acceleration limits. In
  other words, for all of the blocks in the planner, the plan is optimal and no further speed improvements
  are possible. If a new block is added to the buffer, the plan is recomputed according to the said
  guidelines for a new optimal plan.
  当这些阶段完成后，规划器将在所有块中最大化速度曲线，其中每个块均以其最大允许加速度极限运行。 
  换句话说，对于计划器中的所有块，计划都是最佳的，无法进一步提高速度。 
  如果将新块添加到缓冲器，则根据用于新的最佳计划的所述准则重新计算计划。

  To increase computational efficiency of these guidelines, a set of planner block pointers have been
  created to indicate stop-compute points for when the planner guidelines cannot logically make any further
  changes or improvements to the plan when in normal operation and new blocks are streamed and added to the
  planner buffer. For example, if a subset of sequential blocks in the planner have been planned and are
  bracketed by junction velocities at their maximums (or by the first planner block as well), no new block
  added to the planner buffer will alter the velocity profiles within them. So we no longer have to compute
  them. Or, if a set of sequential blocks from the first block in the planner (or a optimal stop-compute
  point) are all accelerating, they are all optimal and can not be altered by a new block added to the
  planner buffer, as this will only further increase the plan speed to chronological blocks until a maximum
  junction velocity is reached. However, if the operational conditions of the plan changes from infrequently
  used feed holds or feedrate overrides, the stop-compute pointers will be reset and the entire plan is
  recomputed as stated in the general guidelines.

  Planner buffer index mapping:
  // block都是从head入队，从tail出队。FIFO先进先出。
  - block_buffer_tail: 指示即将出队的位置。blockPoints to the beginning of the planner buffer. First to be executed or being executed.
  - block_buffer_head: 指示即将入队的位置。Points to the buffer block after the last block in the buffer. Used to indicate whether
      the buffer is full or empty. As described for standard ring buffers, this block is always empty.
  - next_buffer_head: 相当于head++，如果next_buffer_head==block_buffer_tail，说明缓冲区满载。
      事实上还剩一个空位是head指向的位置。但是，对于环形缓冲区来说，一般都让head指向的位置保持为空。
      Points to next planner buffer block after the buffer head block. When equal to the
      buffer tail, this indicates the buffer is full.
  - block_buffer_planned: 缓冲区中存在2部分，一部分是planed的，一部分是to be planed的，该指针指向to be planed的的第一个块。
      Points to the first buffer block after the last optimally planned block for normal
      streaming operating conditions. Use for planning optimizations by avoiding recomputing parts of the
      planner buffer that don't change with the addition of a new block, as describe above. In addition,
      this block can never be less than block_buffer_tail and will always be pushed forward and maintain
      this requirement when encountered by the plan_discard_current_block() routine during a cycle.

  NOTE: Since the planner only computes on what's in the planner buffer, some motions with lots of short
  line segments, like G2/3 arcs or complex curves, may seem to move slow. This is because there simply isn't
  enough combined distance traveled in the entire buffer to accelerate up to the nominal speed and then
  decelerate to a complete stop at the end of the buffer, as stated by the guidelines. If this happens and
  becomes an annoyance, there are a few simple solutions: (1) Maximize the machine acceleration. The planner
  will be able to compute higher velocity profiles within the same combined distance. (2) Maximize line
  motion(s) distance per block to a desired tolerance. The more combined distance the planner has to use,
  the faster it can go. (3) Maximize the planner buffer size. This also will increase the combined distance
  for the planner to compute over. It also increases the number of computations the planner has to perform
  to compute an optimal plan, so select carefully. The Arduino 328p memory is already maxed out, but future
  ARM versions should have enough memory and speed for look-ahead blocks numbering up to a hundred or more.
  由于计划器仅根据计划器缓冲区中的内容进行计算，因此某些带有许多短线段的运动（例如G2 / 3弧或复杂曲线）似乎运动缓慢。
  这是因为，按照指南所述，在整个缓冲区中根本没有足够的组合距离来加速至标称速度，然后在缓冲区末端减速至完全停止。
  如果发生这种情况并成为烦人的事，有一些简单的解决方案：
  （1）最大化机器加速度。计划者将能够在相同的组合距离内计算出更高的速度分布。 
  （2）将每个块的直线运动距离最大化到所需的公差。计划者必须使用的距离越多，则走得越快。 
  （3）最大化计划程序缓冲区的大小。这也将增加计划者计算的总距离。它还增加了计划者为计算最佳计划而必须执行的计算量，因此请谨慎选择。 
  Arduino 328p的内存已经用完了，但是未来的ARM版本应该具有足够的内存和速度，以供多达100个或更多的超前块使用。

*/
// 在环形block_buffer中存放了足够多的block，然后为这些block规划最佳速度，让这些块衔接更流畅
static void planner_recalculate()
{
  // Initialize block index to the last block in the planner buffer.
  // 注意：因为head指向的内存总是空的，head指示了即将入队的块的位置。
  uint8_t block_index = plan_prev_block_index(block_buffer_head); // 获取刚刚入队的block，即FIFO队列的最后一个block

  // 缓冲区中存在2部分，一部分是planed的，一部分是to be planed的。如果最后一个block是planed的，则不必再重新规划。
  if (block_index == block_buffer_planned) { return; }

  // Reverse Pass: Coarsely maximize all possible deceleration curves back-planning from the last
  // block in buffer. Cease planning when the last optimal planned or tail pointer is reached.
  // NOTE: Forward pass will later refine and correct the reverse pass to create an optimal plan.
  // 反向遍历：从缓冲区的最后一个块开始，粗略地最大化所有可能的减速曲线的后向规划。 
  // 注意：正向遍历将在以后细化和纠正反向遍历的规划，以创建最佳计划。
  float entry_speed_sqr;
  plan_block_t *next;
  plan_block_t *current = &block_buffer[block_index];

  // 计算缓冲区中最后一个块的最大进入速度，该块的退出速度始终为零。
  current->entry_speed_sqr = min( current->max_entry_speed_sqr, 2*current->acceleration*current->millimeters);

  block_index = plan_prev_block_index(block_index); // 
  // 缓冲区中存在2部分，一部分是planed的，一部分是to be planed的。
  // 检查前一个block是否planed
  if (block_index == block_buffer_planned) { // Only two plannable blocks in buffer. Reverse pass complete.
    // Check if the first block is the tail. If so, notify stepper to update its current parameters.
	if (block_index == block_buffer_tail) { st_update_plan_block_parameters(); }
  }
  else { // 说明存在3个或3个以上的可以recalculate的block
    while (block_index != block_buffer_planned) {
      next = current;
      current = &block_buffer[block_index];
      block_index = plan_prev_block_index(block_index);

      // Check if next block is the tail block(=planned block). If so, update current stepper parameters.
      if (block_index == block_buffer_tail) { st_update_plan_block_parameters(); }

      // Compute maximum entry speed decelerating over the current block from its exit speed.
      if (current->entry_speed_sqr != current->max_entry_speed_sqr) {
        entry_speed_sqr = next->entry_speed_sqr + 2*current->acceleration*current->millimeters;
        if (entry_speed_sqr < current->max_entry_speed_sqr) {
          current->entry_speed_sqr = entry_speed_sqr;
        } else {
          current->entry_speed_sqr = current->max_entry_speed_sqr;
        }
      }
    }
  }

  // Forward Pass: Forward plan the acceleration curve from the planned pointer onward.
  // Also scans for optimal plan breakpoints and appropriately updates the planned pointer.
  next = &block_buffer[block_buffer_planned]; // Begin at buffer planned pointer
  block_index = plan_next_block_index(block_buffer_planned);
  while (block_index != block_buffer_head) {
    current = next;
    next = &block_buffer[block_index];

    // Any acceleration detected in the forward pass automatically moves the optimal planned
    // pointer forward, since everything before this is all optimal. In other words, nothing
    // can improve the plan from the buffer tail to the planned pointer by logic.
    if (current->entry_speed_sqr < next->entry_speed_sqr) {
      entry_speed_sqr = current->entry_speed_sqr + 2*current->acceleration*current->millimeters;
      // If true, current block is full-acceleration and we can move the planned pointer forward.
      if (entry_speed_sqr < next->entry_speed_sqr) {
        next->entry_speed_sqr = entry_speed_sqr; // Always <= max_entry_speed_sqr. Backward pass sets this.
        block_buffer_planned = block_index; // Set optimal plan pointer.
      }
    }

    // Any block set at its maximum entry speed also creates an optimal plan up to this
    // point in the buffer. When the plan is bracketed by either the beginning of the
    // buffer and a maximum entry speed or two maximum entry speeds, every block in between
    // cannot logically be further improved. Hence, we don't have to recompute them anymore.
    if (next->entry_speed_sqr == next->max_entry_speed_sqr) { block_buffer_planned = block_index; }
    block_index = plan_next_block_index( block_index );
  }
}


void plan_reset()
{
  memset(&pl, 0, sizeof(planner_t)); // Clear planner struct
  plan_reset_buffer();
}


void plan_reset_buffer()
{
  block_buffer_tail = 0;
  block_buffer_head = 0; // Empty = tail
  next_buffer_head = 1; // plan_next_block_index(block_buffer_head)
  block_buffer_planned = 0; // = block_buffer_tail;
}


void plan_discard_current_block()
{
  if (block_buffer_head != block_buffer_tail) { // Discard non-empty buffer.
    uint8_t block_index = plan_next_block_index( block_buffer_tail );
    // Push block_buffer_planned pointer, if encountered.
    if (block_buffer_tail == block_buffer_planned) { block_buffer_planned = block_index; }
    block_buffer_tail = block_index;
  }
}


// Returns address of planner buffer block used by system motions. Called by segment generator.
plan_block_t *plan_get_system_motion_block()
{
  return(&block_buffer[block_buffer_head]);
}


// Returns address of first planner block, if available. Called by various main program functions.
plan_block_t *plan_get_current_block()
{
  if (block_buffer_head == block_buffer_tail) { return(NULL); } // Buffer empty
  return(&block_buffer[block_buffer_tail]);
}


float plan_get_exec_block_exit_speed_sqr()
{
  uint8_t block_index = plan_next_block_index(block_buffer_tail);
  if (block_index == block_buffer_head) { return( 0.0 ); }
  return( block_buffer[block_index].entry_speed_sqr );
}


// Returns the availability status of the block ring buffer. True, if full.
uint8_t plan_check_full_buffer()
{
  if (block_buffer_tail == next_buffer_head) { return(true); }
  return(false);
}


// Computes and returns block nominal speed based on running condition and override values.
// NOTE: All system motion commands, such as homing/parking, are not subject to overrides.
// 计算和返回额定速度
// 注意：所有系统运动命令（例如归位/停车）均不受覆盖。
float plan_compute_profile_nominal_speed(plan_block_t *block)
{
  float nominal_speed = block->programmed_rate; 
  if (block->condition & PL_COND_FLAG_RAPID_MOTION) { nominal_speed *= (0.01*sys.r_override); }
  else {
    if (!(block->condition & PL_COND_FLAG_NO_FEED_OVERRIDE)) { nominal_speed *= (0.01*sys.f_override); }
    if (nominal_speed > block->rapid_rate) { nominal_speed = block->rapid_rate; }
  }
  if (nominal_speed > MINIMUM_FEED_RATE) { return(nominal_speed); }
  return(MINIMUM_FEED_RATE);
}


// Computes and updates the max entry speed (sqr) of the block, based on the minimum of the junction's
// previous and current nominal speeds and max junction speed.
// 计算并更新块的最大入口速度
static void plan_compute_profile_parameters(plan_block_t *block, float nominal_speed, float prev_nominal_speed)
{
  // Compute the junction maximum entry based on the minimum of the junction speed and neighboring nominal speeds.
  if (nominal_speed > prev_nominal_speed) { block->max_entry_speed_sqr = prev_nominal_speed*prev_nominal_speed; }
  else { block->max_entry_speed_sqr = nominal_speed*nominal_speed; }
  if (block->max_entry_speed_sqr > block->max_junction_speed_sqr) { block->max_entry_speed_sqr = block->max_junction_speed_sqr; }
}


// Re-calculates buffered motions profile parameters upon a motion-based override change.
void plan_update_velocity_profile_parameters()
{
  uint8_t block_index = block_buffer_tail;
  plan_block_t *block;
  float nominal_speed;
  float prev_nominal_speed = SOME_LARGE_VALUE; // Set high for first block nominal speed calculation.
  while (block_index != block_buffer_head) {
    block = &block_buffer[block_index];
    nominal_speed = plan_compute_profile_nominal_speed(block);
    plan_compute_profile_parameters(block, nominal_speed, prev_nominal_speed);
    prev_nominal_speed = nominal_speed;
    block_index = plan_next_block_index(block_index);
  }
  pl.previous_nominal_speed = prev_nominal_speed; // Update prev nominal speed for next incoming block.
}


/* Add a new linear movement to the buffer. target[N_AXIS] is the signed, absolute target position
   in millimeters. Feed rate specifies the speed of the motion. If feed rate is inverted, the feed
   rate is taken to mean "frequency" and would complete the operation in 1/feed_rate minutes.
   All position data passed to the planner must be in terms of machine position to keep the planner
   independent of any coordinate system changes and offsets, which are handled by the g-code parser.
   NOTE: Assumes buffer is available. Buffer checks are handled at a higher level by motion_control.
   In other words, the buffer head is never equal to the buffer tail.  Also the feed rate input value
   is used in three ways: as a normal feed rate if invert_feed_rate is false, as inverse time if
   invert_feed_rate is true, or as seek/rapids rate if the feed_rate value is negative (and
   invert_feed_rate always false).
   The system motion condition tells the planner to plan a motion in the always unused block buffer
   head. It avoids changing the planner state and preserves the buffer to ensure subsequent gcode
   motions are still planned correctly, while the stepper module only points to the block buffer head
   to execute the special system motion. */
// 被mc_line调用，向block_buffer添加新的线性运动block
uint8_t plan_buffer_line(float *target, plan_line_data_t *pl_data)
{
  // Prepare and initialize new block. Copy relevant pl_data for block execution.
  plan_block_t *block = &block_buffer[block_buffer_head];
  memset(block,0,sizeof(plan_block_t)); // Zero all block values.
  block->condition = pl_data->condition;
  // 如果主轴速度可变
  #ifdef VARIABLE_SPINDLE
    block->spindle_speed = pl_data->spindle_speed;
  #endif
  #ifdef USE_LINE_NUMBERS
    block->line_number = pl_data->line_number;
  #endif

  // Compute and store initial move distance data.
  int32_t target_steps[N_AXIS], position_steps[N_AXIS];
  float unit_vec[N_AXIS], delta_mm;
  uint8_t idx;

  // Copy position data based on type of motion being planned.
  // 基于运动类型，拷贝位置数据
  // sys_position与pl.position的区别是什么？
  if (block->condition & PL_COND_FLAG_SYSTEM_MOTION) { // 系统运动：home/park
    #ifdef COREXY
      position_steps[X_AXIS] = system_convert_corexy_to_x_axis_steps(sys_position);
      position_steps[Y_AXIS] = system_convert_corexy_to_y_axis_steps(sys_position);
      position_steps[Z_AXIS] = sys_position[Z_AXIS];
    #else
      memcpy(position_steps, sys_position, sizeof(sys_position)); 
    #endif
  } else { memcpy(position_steps, pl.position, sizeof(pl.position)); } 
  #ifdef COREXY
    target_steps[A_MOTOR] = lround(target[A_MOTOR]*settings.steps_per_mm[A_MOTOR]);
    target_steps[B_MOTOR] = lround(target[B_MOTOR]*settings.steps_per_mm[B_MOTOR]);
    block->steps[A_MOTOR] = labs((target_steps[X_AXIS]-position_steps[X_AXIS]) + (target_steps[Y_AXIS]-position_steps[Y_AXIS]));
    block->steps[B_MOTOR] = labs((target_steps[X_AXIS]-position_steps[X_AXIS]) - (target_steps[Y_AXIS]-position_steps[Y_AXIS]));
  #endif

  for (idx=0; idx<N_AXIS; idx++) {
    // Calculate target position in absolute steps, number of steps for each axis, and determine max step events.
    // Also, compute individual axes distance for move and prep unit vector calculations.
    // NOTE: Computes true distance from converted step values.
    #ifdef COREXY
      if ( !(idx == A_MOTOR) && !(idx == B_MOTOR) ) {
        target_steps[idx] = lround(target[idx]*settings.steps_per_mm[idx]);
        block->steps[idx] = labs(target_steps[idx]-position_steps[idx]);
      }
      block->step_event_count = max(block->step_event_count, block->steps[idx]);
      if (idx == A_MOTOR) {
        delta_mm = (target_steps[X_AXIS]-position_steps[X_AXIS] + target_steps[Y_AXIS]-position_steps[Y_AXIS])/settings.steps_per_mm[idx];
      } else if (idx == B_MOTOR) {
        delta_mm = (target_steps[X_AXIS]-position_steps[X_AXIS] - target_steps[Y_AXIS]+position_steps[Y_AXIS])/settings.steps_per_mm[idx];
      } else {
        delta_mm = (target_steps[idx] - position_steps[idx])/settings.steps_per_mm[idx];
      }
    #else
	
      target_steps[idx] = lround(target[idx]*settings.steps_per_mm[idx]); // 从绝对坐标地址转换到绝对steps
      block->steps[idx] = labs(target_steps[idx]-position_steps[idx]); // 计算运动到目标需要多少steps
      block->step_event_count = max(block->step_event_count, block->steps[idx]); // 记录最长的需要步数steps
      delta_mm = (target_steps[idx] - position_steps[idx])/settings.steps_per_mm[idx]; // block->steps[idx]换算成mm
	  #endif
    unit_vec[idx] = delta_mm; // Store unit vector numerator

    // 设置位移的方向
    if (delta_mm < 0.0 ) { block->direction_bits |= get_direction_pin_mask(idx); }
  }

  // Bail if this is a zero-length block. Highly unlikely to occur.
  if (block->step_event_count == 0) { return(PLAN_EMPTY_BLOCK); }

  // Calculate the unit vector of the line move and the block maximum feed rate and acceleration scaled
  // down such that no individual axes maximum values are exceeded with respect to the line direction.
  // NOTE: This calculation assumes all axes are orthogonal (Cartesian) and works with ABC-axes,
  // if they are also orthogonal/independent. Operates on the absolute value of the unit vector.
  // 计算该block存放的距离、加速度、转速r/m
  block->millimeters = convert_delta_vector_to_unit_vector(unit_vec); // millimeters=sqrt(x*x+y*y+z*z)
  block->acceleration = limit_value_by_axis_maximum(settings.acceleration, unit_vec);
  block->rapid_rate = limit_value_by_axis_maximum(settings.max_rate, unit_vec);

  // 存放进给速度，根据标志位区分转速r/m或线速mm/m
  if (block->condition & PL_COND_FLAG_RAPID_MOTION) { block->programmed_rate = block->rapid_rate; }
  else { 
  	// 对于线速来说，还要区分G93和G94
    block->programmed_rate = pl_data->feed_rate;
    if (block->condition & PL_COND_FLAG_INVERSE_TIME) { block->programmed_rate *= block->millimeters; }
  }

  // TODO: Need to check this method handling zero junction speeds when starting from rest.
  // 从静止开始时，需要检查此方法来处理零接合速度
  if ((block_buffer_head == block_buffer_tail) || (block->condition & PL_COND_FLAG_SYSTEM_MOTION)) {

    // Initialize block entry speed as zero. Assume it will be starting from rest. Planner will correct this later.
    // If system motion, the system motion block always is assumed to start from rest and end at a complete stop.
	// 对于系统运动来说，总是从0启动到最后停止
	block->entry_speed_sqr = 0.0;
    block->max_junction_speed_sqr = 0.0; // Starting from rest. Enforce start from zero velocity.

  } else {
    // Compute maximum allowable entry speed at junction by centripetal acceleration approximation.
    // Let a circle be tangent to both previous and current path line segments, where the junction
    // deviation is defined as the distance from the junction to the closest edge of the circle,
    // colinear with the circle center. The circular segment joining the two paths represents the
    // path of centripetal acceleration. Solve for max velocity based on max acceleration about the
    // radius of the circle, defined indirectly by junction deviation. This may be also viewed as
    // path width or max_jerk in the previous Grbl version. This approach does not actually deviate
    // from path, but used as a robust way to compute cornering speeds, as it takes into account the
    // nonlinearities of both the junction angle and junction velocity.
    //
    // NOTE: If the junction deviation value is finite, Grbl executes the motions in an exact path
    // mode (G61). If the junction deviation value is zero, Grbl will execute the motion in an exact
    // stop mode (G61.1) manner. In the future, if continuous mode (G64) is desired, the math here
    // is exactly the same. Instead of motioning all the way to junction point, the machine will
    // just follow the arc circle defined here. The Arduino doesn't have the CPU cycles to perform
    // a continuous mode path, but ARM-based microcontrollers most certainly do.
    //
    // NOTE: The max junction speed is a fixed value, since machine acceleration limits cannot be
    // changed dynamically during operation nor can the line move geometry. This must be kept in
    // memory in the event of a feedrate override changing the nominal speeds of blocks, which can
    // change the overall maximum entry speed conditions of all blocks.
	// 通过向心加速度逼近计算交点处的最大允许入口速度block->max_junction_speed_sqr。
    float junction_unit_vec[N_AXIS];
    float junction_cos_theta = 0.0;
    for (idx=0; idx<N_AXIS; idx++) {
      junction_cos_theta -= pl.previous_unit_vec[idx]*unit_vec[idx];
      junction_unit_vec[idx] = unit_vec[idx]-pl.previous_unit_vec[idx];
    }

    // NOTE: Computed without any expensive trig, sin() or acos(), by trig half angle identity of cos(theta).
    if (junction_cos_theta > 0.999999) {
      //  For a 0 degree acute junction, just set minimum junction speed.
      block->max_junction_speed_sqr = MINIMUM_JUNCTION_SPEED*MINIMUM_JUNCTION_SPEED;
    } else {
      if (junction_cos_theta < -0.999999) {
        // Junction is a straight line or 180 degrees. Junction speed is infinite.
        block->max_junction_speed_sqr = SOME_LARGE_VALUE;
      } else {
        convert_delta_vector_to_unit_vector(junction_unit_vec);
        float junction_acceleration = limit_value_by_axis_maximum(settings.acceleration, junction_unit_vec);
        float sin_theta_d2 = sqrt(0.5*(1.0-junction_cos_theta)); // Trig half angle identity. Always positive.
        block->max_junction_speed_sqr = max( MINIMUM_JUNCTION_SPEED*MINIMUM_JUNCTION_SPEED,
                       (junction_acceleration * settings.junction_deviation * sin_theta_d2)/(1.0-sin_theta_d2) );
      }
    }
  }

  // Block system motion from updating this data to ensure next g-code motion is computed correctly.
  // 本次block的处理已经完成，下面为下一次的block块运动准备一些数据。
  if (!(block->condition & PL_COND_FLAG_SYSTEM_MOTION)) {
    float nominal_speed = plan_compute_profile_nominal_speed(block); // 额定速度
    plan_compute_profile_parameters(block, nominal_speed, pl.previous_nominal_speed); // 
    pl.previous_nominal_speed = nominal_speed;
    
    // Update previous path unit_vector and planner position.
    memcpy(pl.previous_unit_vec, unit_vec, sizeof(unit_vec)); // pl.previous_unit_vec[] = unit_vec[]
    memcpy(pl.position, target_steps, sizeof(target_steps)); // pl.position[] = target_steps[]

    // New block is all set. Update buffer head and next buffer head indices.
    block_buffer_head = next_buffer_head;
    next_buffer_head = plan_next_block_index(block_buffer_head);

    // Finish up by recalculating the plan with the new block.
    // 推入1个block就重新规划1次
    planner_recalculate();
  }
  return(PLAN_OK);
}


// Reset the planner position vectors. Called by the system abort/initialization routine.
void plan_sync_position()
{
  // TODO: For motor configurations not in the same coordinate frame as the machine position,
  // this function needs to be updated to accomodate the difference.
  uint8_t idx;
  for (idx=0; idx<N_AXIS; idx++) {
    #ifdef COREXY
      if (idx==X_AXIS) {
        pl.position[X_AXIS] = system_convert_corexy_to_x_axis_steps(sys_position);
      } else if (idx==Y_AXIS) {
        pl.position[Y_AXIS] = system_convert_corexy_to_y_axis_steps(sys_position);
      } else {
        pl.position[idx] = sys_position[idx];
      }
    #else
      pl.position[idx] = sys_position[idx];
    #endif
  }
}


// Returns the number of available blocks are in the planner buffer.
uint8_t plan_get_block_buffer_available()
{
  if (block_buffer_head >= block_buffer_tail) { return((BLOCK_BUFFER_SIZE-1)-(block_buffer_head-block_buffer_tail)); }
  return((block_buffer_tail-block_buffer_head-1));
}


// Returns the number of active blocks are in the planner buffer.
// NOTE: Deprecated. Not used unless classic status reports are enabled in config.h
uint8_t plan_get_block_buffer_count()
{
  if (block_buffer_head >= block_buffer_tail) { return(block_buffer_head-block_buffer_tail); }
  return(BLOCK_BUFFER_SIZE - (block_buffer_tail-block_buffer_head));
}


// Re-initialize buffer plan with a partially completed block, assumed to exist at the buffer tail.
// Called after a steppers have come to a complete stop for a feed hold and the cycle is stopped.
void plan_cycle_reinitialize()
{
  // Re-plan from a complete stop. Reset planner entry speeds and buffer planned pointer.
  st_update_plan_block_parameters();
  block_buffer_planned = block_buffer_tail;
  planner_recalculate();
}
