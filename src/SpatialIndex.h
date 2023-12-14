#pragma once

#include "ofVec2f.h"
#include "ofColor.h"

#include <vector>
#include <functional>

#pragma warning(push)
#pragma warning(disable:4267)
#include "flat_hash_map/bytell_hash_map.hpp"
#pragma warning(pop)

typedef ska::bytell_hash_map<int64_t, uint32_t> SpatialHashTable;

struct Particle2D
{
	ofVec2f position = ofVec2f::zero();
	ofVec2f delta_position = ofVec2f::zero();

	float radius = 0.0F;

	ofColor color = ofColor::lightCyan;

	int64_t spatial_index(size_t tile_size) const;

	void update(float dt);
	void push(ofVec2f direction);
	void teleport(ofVec2f new_position);

	static void resolve_collision(Particle2D& particle0, Particle2D& particle1, float bounce);
};

void spatial_index_sort(std::vector<Particle2D>& particles, size_t tile_size);

void compute_spatial_hash_table(const std::vector<Particle2D>& particles, size_t tile_size, SpatialHashTable& hash_table);

inline int64_t compute_spatial_index(int tile_x, int tile_y)
{
	// Pack Y to high bits, this will ensure that particles are sorted in a row-major order
	return (int64_t(tile_y) << 32) | int64_t(tile_x);
}

// Invoke function for all particles inside the circle
template <typename Fn>
inline size_t radius_query(std::vector<Particle2D>& particles, size_t tile_size, const SpatialHashTable* table, ofVec2f position, float radius, Fn fn)
{
	const int query_tile_x = int(std::floorf(position.x / float(tile_size)));
	const int query_tile_y = int(std::floorf(position.y / float(tile_size)));

	const int radius_in_tiles = int(std::ceilf(radius / float(tile_size)));

	size_t checked_particles = 0;

	for (int tile_y = (query_tile_y - radius_in_tiles); tile_y <= (query_tile_y + radius_in_tiles); ++tile_y)
	{
		const int64_t spatial_index_start = compute_spatial_index(query_tile_x - radius_in_tiles, tile_y);
		const int64_t spatial_index_end = compute_spatial_index(query_tile_x + radius_in_tiles, tile_y);

		auto found_particle = particles.end();
		if (table)
		{
			for (int tile_x = (query_tile_x - radius_in_tiles); tile_x <= (query_tile_x + radius_in_tiles); ++tile_x)
			{
				auto found_tile = table->find(compute_spatial_index(tile_x, tile_y));
				if (found_tile != table->end())
				{
					found_particle = particles.begin() + found_tile->second;
					break;
				}
			}
		}
		else
		{
			found_particle = std::lower_bound(particles.begin(), particles.end(), spatial_index_start,
				[tile_size](const Particle2D& particle, int64_t spatial_index)
				{
					return particle.spatial_index(tile_size) < spatial_index;
				});
		}

		for (; found_particle != particles.end(); ++found_particle)
		{
			if (found_particle->spatial_index(tile_size) > spatial_index_end) { break; }

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
