#include <gtest/gtest.h>
#include <vector>
#include "boids_calculator.hpp"

/* Helper: gstest can't compare Vec2 directly, so check components with tolerance */
static void expect_vec2(const Vec2& v, double x, double y, double tol = 1e-6) {
    EXPECT_NEAR(v.x, x, tol);
    EXPECT_NEAR(v.y, y, tol);
}

/* === SEPARATION === */

/* One neighbor to the right at distance 1 -> force pushes me left (-x), 
    magnitude = diff/(dist*dist) = (-1.0)/1 = (-1.0) */
TEST(Separation, PushesAwayFromCloseNeighbor) {
    Vec2 my_pos(0.0, 0.0);
    std::vector<NeighborState> neighbors = {
        { Vec2(1.0, 0.0), Vec2(0.0, 0.0) } /* pos, vel */
    };
    Vec2 f = BoidsCore::compute_separation(my_pos, neighbors, 2.0);
    expect_vec2(f, -1.0, 0.0);
}

/* Neighbor outside radius -> no force */
TEST(Separation, IgnoresNeighborOutsideRadius) {
    Vec2 my_pos(0.0, 0.0);
    std::vector<NeighborState> neighbors = {
        { Vec2(5.0, 0.0), Vec2(0.0, 0.0)}
    };
    Vec2 f = BoidsCore::compute_separation(my_pos, neighbors, 2.0);
    expect_vec2(f, 0.0, 0.0);
}

/* === ALIGNMENT === */

/* Neighbor moving at (1,0), I'm at rest -> steer = avg_vel - my_vel = (1,0) */
TEST(Alignment, SteersTowardNeighborVelocity) {
    Vec2 my_pos(0.0, 0.0), my_vel(0.0, 0.0);
    std::vector<NeighborState> neighbors = {
        { Vec2(1.0, 0.0), Vec2(1.0, 0.0) }
    };
    Vec2 f = BoidsCore::compute_alignment(my_pos, my_vel, neighbors, 2.0);
    expect_vec2(f, 1.0, 0.0);
}

/* === COHESION === */

/* Two neighbors at (2, 0) and (0, 2), centroid = (1, 1), pull = centroid - my_pos = (1, 1) */
TEST(Cohesion, PullsTowardCentroid) {
    Vec2 my_pos(0.0, 0.0);
    std::vector<NeighborState> neighbors = {
        { Vec2(2.0, 0.0), Vec2(0.0, 0.0) },
        { Vec2(0.0, 2.0), Vec2(0.0, 0.0) }
    };
    Vec2 f = BoidsCore::compute_cohesion(my_pos, neighbors, 3.0);
    expect_vec2(f, 1.0, 1.0);
}

/* === CLAMP / Compute velocity === */
/* Velocity below max_speed should be returned unchanged */
TEST(Clamp, BelowMaxReturnUnchanged) {
    Vec2 v(0.3, 0.4); /* length = 0.5 */
    expect_vec2(v.clamp(1.0), 0.3, 0.4);
}

TEST(Clamp, AboveMaxScaledToMax) {
    Vec2 v(3.0, 4.0); /* length = 5.0 */
    expect_vec2(v.clamp(1.0), 0.6, 0.8); /* Scaled to length 1.0 */
}
