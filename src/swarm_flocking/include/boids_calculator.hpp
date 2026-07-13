/* boids_calculator.hpp */
#ifndef BOIDS_CALCULATOR_HPP
#define BOIDS_CALCULATOR_HPP

#include <vector>
#include <map>
#include <string>
#include <cmath>

struct Vec2 {
    double x;
    double y;

    /* Default constructor - needed for Vec2 v; or usage inside map/vector */
    Vec2() : x(0.0), y(0.0) {}

    /* Parameterized constructor - usage: Vec2 v(1.0, 2.0); */
    Vec2(double x_, double y_) : x(x_), y(y_) {}

    /* ===== OPERATOR =====*/
    Vec2 operator+(const Vec2& other) const {
        return Vec2(x + other.x, y + other.y);
    }

    Vec2& operator+=(const Vec2& other) {
        x += other.x;
        y += other.y;
        return *this;
    }

    Vec2 operator-(const Vec2& other) const {
        return Vec2(x - other.x, y - other.y);
    }

    Vec2 operator-() const {
        return Vec2(-x, -y);
    }

    Vec2 operator*(double scalar) const {
        return Vec2(x * scalar, y * scalar);
    }

    Vec2 operator/(double scalar) const {
        return Vec2(x / scalar, y / scalar);
    }

    /* ===== HELPER FUNCTIONS NEEDED FOR BOIDS ===== */
    /* Vector length - used to compute distance in separation */
    double length() const {
        return std::sqrt(x * x + y * y);
    }

    /* Clamp to a maximum length - used to cap velocity at max_speed */
    Vec2 clamp(double max_length) const {
        double len = length();
        if (len > max_length && len > 1e-6) { /* avoid division by zero */
            return (*this) * (max_length / len);
        }
        return *this;
    }
};

inline Vec2 operator*(double scalar, const Vec2& v) {
    return v * scalar;
}

inline Vec2 average(const std::vector<Vec2>& vectors) {
    if (vectors.empty()) {
        return Vec2(0.0, 0.0);
    }
    Vec2 sum(0.0, 0.0);
    for (const auto& v : vectors) {
        sum += v;
    }
    return sum / static_cast<double>(vectors.size());
}

struct NeighborState {
    Vec2 pos;
    Vec2 vel;
};

/**
  * @brief Weight coefficients controlling the contribution of each Boids
  *          behavior to the final velocity command.
  * These weights are tuned experimentally in Gazebo simulation - there is
  * no fixed "correct" value; the optimal weights depend on robot density,
  * environment size, and the desired swarm behavior.
*/
struct FlockingWeights {
    /**
      * @brief Strength of the separation behavior.
      *
      * Controls how strongly the robot avoids getting too close to its
      * neighbors. A higher value keeps more distance between robots
      * (safer, fewer collisions) but results in looser grouping.
    */
    double separation;

    /**
      * @brief Strength of the alignment behavior.
      *
      * Controls how strongly the robot matches the average velocity
      * (direction and speed) of nearby neighbors. A higher value makes
      * the robot move more "in sync" with the local group.
    */
    double alignment;

    /**
      * @brief Strength of the cohesion behavior.
      *
      * Controls how strongly the robot is pulled toward the centroid
      * (average position) of nearby neighbors. A higher value produces
      * tighter grouping and less spread-out formations.
    */
    double cohesion;

    /**
      * @brief Strength of the goal-seeking behavior.
      *
      * Controls how strongly the robot is pulled toward its assigned
      * goal or frontier point (typically provided by the Coordinator).
      * A higher value prioritizes reaching the destination over
      * flocking with neighbors.
    */
    double goal;
};

class BoidsCore {
    public:
        /**
          * @brief Computes the separation force pushing the robot away from nearby neighbors
        */
        static Vec2 compute_separation(const Vec2& my_pos, const std::vector<NeighborState>& neighbors, double radius) {
            Vec2 force = Vec2(0.0, 0.0);
            for (const auto& n : neighbors) {
                Vec2 diff = my_pos - n.pos;
                double dist = diff.length();
                if (dist > 0.0 && dist < radius) {
                    /* For each nearby neighbor within radius, add a push-away force: 
                        it points away from the neighbor (toward this robot), 
                        and gets much stronger the closer the neighbor is (inverse-square of distance).
                    */
                    force += diff / (dist * dist);
                }
            }
            return force;
        }

        /**
          * @brief Computes the alignment force that steers this robot to match
          *         the average velocity (speed and direction) of NEARBY neighbors (within radius).
          * This encourages the robot to move in sync with the local flock,
          * rather than moving independently in a different direction.
        */
        static Vec2 compute_alignment(const Vec2& my_pos, const Vec2& my_vel, const std::vector<NeighborState>& neighbors, double radius) {
            if (neighbors.empty()) {
                return Vec2(0.0, 0.0);
            }
            
            std::vector<Vec2> nearby_vels;
            for (const auto& n : neighbors) {
                double dist = (my_pos - n.pos).length();
                if (dist > 0.0 && dist < radius) {
                    nearby_vels.push_back(n.vel);
                }
            }
            
            if(nearby_vels.empty()) return Vec2(0.0, 0.0);

            return average(nearby_vels) - my_vel;
        }

        /**
          * @brief Computes the cohesion force that pulls this robot toward the
          *         centroid of NEARBY neighbors (within radius), not the entire flock.
        */
        static Vec2 compute_cohesion(const Vec2& my_pos, const std::vector<NeighborState>& neighbors, double radius) {
            if (neighbors.empty()) {
                return Vec2(0.0, 0.0);
            }

            std::vector<Vec2> nearby_positions;
            for (const auto& n : neighbors) {
                double dist = (my_pos - n.pos).length();
                if (dist > 0.0 && dist < radius) {
                    nearby_positions.push_back(n.pos);
                }
            }

            if (nearby_positions.empty()) return Vec2(0.0, 0.0);

            return average(nearby_positions) - my_pos;
        }

        /**
          * @brief Compute the velocity for the robot
          * TODO: (future improvement): use separate radius for each behavior instead of
          *         one shared radius -- e.g. smaller radius for separation (react early to
          *         avoid collision) and a larger radius for alignment/cohesion (keep group
          *         formation over a wider range).
        */
        static Vec2 compute_velocity(
                        const Vec2& my_pos, const Vec2& my_vel, 
                        std::vector<NeighborState>& neighbors,
                        Vec2 goal_pos, FlockingWeights weights, 
                        double radius, double max_speed
        ) {
            Vec2 sep        = compute_separation(my_pos, neighbors, radius);
            Vec2 align      = compute_alignment(my_pos, my_vel, neighbors, radius);
            Vec2 cohesion   = compute_cohesion(my_pos, neighbors, radius);
            Vec2 goal_dir   = goal_pos - my_pos;

            Vec2 vel = weights.separation * sep + weights.alignment * align 
                        + weights.cohesion * cohesion + weights.goal * goal_dir;
            
            return vel.clamp(max_speed);
        }
};

#endif