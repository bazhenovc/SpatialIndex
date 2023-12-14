
#include "SpatialIndex.h"

#include <algorithm>

int64_t Particle2D::spatial_index() const
{
	int tile_x = int(std::floorf(position.x / float(TILE_SIZE)));
	int tile_y = int(std::floorf(position.y / float(TILE_SIZE)));

	return compute_spatial_index(tile_x, tile_y);
}

void Particle2D::update(float dt)
{
	const ofVec2f acceleration(0.0F, 9.8F);

	delta_position += acceleration * dt * dt;
	position += delta_position;
}

void Particle2D::push(ofVec2f direction)
{
	delta_position += direction;
	//position += direction;
}

void Particle2D::resolve_collision(Particle2D& particle0, Particle2D& particle1, float bounce)
{
	const float radius_sum = particle0.radius + particle1.radius;

	const ofVec2f penetration_velocity = particle1.position - particle0.position;
	const float penetration_length_squared = fmaxf(penetration_velocity.lengthSquared(), 0.001F);

	if (penetration_length_squared < (radius_sum * radius_sum))
	{
		const float penetration_length = sqrt(penetration_length_squared);
		const float penetration_depth = radius_sum - penetration_length;

		const ofVec2f penetration_direction = penetration_velocity * (1.0F / penetration_length);
		const ofVec2f push_direction = penetration_direction * penetration_depth * 0.5F;

		particle0.push(-push_direction);
		particle1.push(push_direction);

		const float relative_velocity = (particle0.delta_position - particle1.delta_position).dot(penetration_direction);
		const float exchange_velocity = (1.0F + bounce) * relative_velocity;
		if (exchange_velocity > 0.0F)
		{
			const ofVec2f exchange_direction = penetration_direction * exchange_velocity * 0.5F;
			particle0.delta_position -= exchange_direction;
			particle1.delta_position += exchange_direction;
		}
	}
}

void spatial_index_sort(std::vector<Particle2D>& particles)
{
	std::sort(particles.begin(), particles.end(), [](const Particle2D& p0, const Particle2D& p1)
		{
			return p0.spatial_index() < p1.spatial_index();
		});
}

void compute_spatial_hash_table(const std::vector<Particle2D>& particles, SpatialHashTable& hash_table)
{
	hash_table.clear();

	for (const Particle2D& particle : particles)
	{
		int64_t spatial_index = particle.spatial_index();
		size_t particle_index = &particle - &(*particles.begin());

		auto table_value = hash_table.find(spatial_index);
		if (table_value == hash_table.end())
		{
			hash_table.insert({ spatial_index, uint32_t(particle_index) });
		}
		else
		{
			table_value->second = std::min(table_value->second, uint32_t(particle_index));
		}
	}
}
