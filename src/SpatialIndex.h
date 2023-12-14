#pragma once

#include "ofVec2f.h"
#include "ofColor.h"

#include <vector>
#include <functional>

const int TILE_SIZE = 16;

struct Particle2D
{
	ofVec2f position = ofVec2f::zero();
	ofVec2f delta_position = ofVec2f::zero();

	float radius = 0.0F;

	ofColor color = ofColor::lightCyan;

	int64_t spatial_index() const;

	void update(float dt);
	void push(ofVec2f direction);

	static void resolve_collision(Particle2D& particle0, Particle2D& particle1, float bounce);
};

void spatial_index_sort(std::vector<Particle2D>& particles);

inline int64_t compute_spatial_index(int tile_x, int tile_y)
{
	// Pack Y to high bits, this will ensure that particles are sorted in a row-major order
	return (int64_t(tile_y) << 32) | int64_t(tile_x);
}

// Invoke function for all particles inside the circle
template <typename Fn>
inline size_t radius_query(std::vector<Particle2D>& particles, ofVec2f position, float radius, Fn fn)
{
	const int query_tile_x = int(std::floorf(position.x / float(TILE_SIZE)));
	const int query_tile_y = int(std::floorf(position.y / float(TILE_SIZE)));

	const int radius_in_tiles = int(std::ceilf(radius / float(TILE_SIZE)));

	size_t checked_particles = 0;

	for (int tile_y = (query_tile_y - radius_in_tiles); tile_y <= (query_tile_y + radius_in_tiles); ++tile_y)
	{
		const int64_t spatial_index_start = compute_spatial_index(query_tile_x - radius_in_tiles, tile_y);
		const int64_t spatial_index_end = compute_spatial_index(query_tile_x + radius_in_tiles, tile_y);

		auto found_particle = std::lower_bound(particles.begin(), particles.end(), spatial_index_start,
			[](const Particle2D& particle, int64_t spatial_index)
			{
				return particle.spatial_index() < spatial_index;
			});

		for (; found_particle != particles.end(); ++found_particle)
		{
			if (found_particle->spatial_index() > spatial_index_end) { break; }

			checked_particles++;

			float radius_sum = radius + found_particle->radius;
			if ((position - found_particle->position).lengthSquared() <= radius_sum * radius_sum)
			{
				fn(*found_particle);
			}
		}
	}

	return checked_particles;
}
