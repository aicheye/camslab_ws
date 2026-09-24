/* Follower goal for the leader-follower chain.

Robot i+1 drives to a point "behind" robot i. The leader state gives its
position p and its unit heading q, both in the world frame.
*/

#include "gtg_ros/formation.hpp"

namespace gtg_ros {

// Return the goal p* for the robot that follows `leader`.
Vec2 followerGoal(const State &leader, const Params &params) {
  return leader.p - params.spacing * leader.q;
}

} // namespace gtg_ros
