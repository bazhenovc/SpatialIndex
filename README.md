# SpatialIndex
Implicit unbounded spatial acceleration structure based on spatial indexing.

![image](https://github.com/bazhenovc/SpatialIndex/assets/986127/17450101-b322-4502-bf65-dad546d95c33)

5000 colliding particles with different sizes, ~6ms when spatial hashing is used, ~14ms when spatial indexing is used, ~150ms when brute-force "each-to-each" approach is used.

## How it works

We define an "imaginary" grid with specific cell size, this cell size needs to be greater or equal to the largest particle radius. The grid is not store anywhere, we just have a grid size constant.

Each particle computes it's grid coordinates and packs them into a 64 bit value, this value is called a "spatial index". Y grid coordinate goes into the high 32 bits, X grid coordinate goes into the low 32 bits.

This spatial index is not stored anywhere, it's computed on the fly from particle position every time it's needed.

```cpp
int64_t Particle2D::spatial_index() const
{
	int tile_x = int(std::floorf(position.x / float(TILE_SIZE)));
	int tile_y = int(std::floorf(position.y / float(TILE_SIZE)));

	return compute_spatial_index(tile_x, tile_y);
}

inline int64_t compute_spatial_index(int tile_x, int tile_y)
{
	// Pack Y to high bits, this will ensure that particles are sorted in a row-major order
	return (int64_t(tile_y) << 32) | int64_t(tile_x);
}
```

Particle array is then sorted using this spatial index as an integer sorting key. This results in particles sorted by their tile coordinates in row-major order.

Now in order to find one specific particle at a given position, we compute the spatial index for that position and perform a binary search using that index.

Searching for all particles within a specific radius works in a similar fashion, but we need to check all tiles within the radius. To achieve that, we first compute the range of tiles within our radius, then we iterate that range row-by-row.
For each tile row we need to perform one binary search to find the first particle in a row, but then we can simply increment the spatial index to get to adjacent columns. Look at `radius_query` function for reference.

Now the next obvious improvement is to get rid of the binary searching. We still want to keep the main data densely packed, so what I suggest is to compute a hash table that maps spatial index to particle index in the array. This is trivial to implement, note that particles array has to be sorted:

```cpp
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
```

This allows us to replace the binary search by spatial index with hash table lookup by spatial index, rest of the code stays the same. Since we're now looking for an exact value instead of searching for a lesser value, we can't do the row trick anymore, so we need to do one individual lookup per adjacent tile. In practice this is still faster than binary search.

Of couse all this stull trivially extends to 3D - just pack the third coordinate into the spatial index. We have up to 64 bits here and tile coordinates don't need full 32 bits, so one way to pack it would be (20 bits Z, 20 bits Y, 20 bits X) and 4 spare bits left. ZYX packing order ensures that data is sorted in a cache friendly manner.

The demo uses spatial hashing (with bytell_hash_table by Malte Skarupke) or spatial indexing to both find and resolve collision contact pairs and to find and highlight particles near the mouse cursor. Brute-force reference implementation is also provided, but it is extremely slow.

## Applications and further improvements

I think the most interesting application for this would be GPU spatial searching, for instance this could be used for storing baked light probes that are not aligned to regular grid and are freely positioned in world space. After baking we could sort them by spatial index and compute a minimal perfect hash for them, this will allow to do constant time radius searches on the GPU to find and interpolate adjacent probes. Perfect hashing would work in the same way as regular hashing, the only difference is that to find the first probe in the tile we would use something like `probes = table[spatial_index & (table_size - 1)]`. Then we would iterate a small amount of probes in the tile and select the ones we need.

Another application is CPU and GPU physics, however GPU is going to have a lot of issues with sorting the data and hash tables in general. Static data could be baked with minimal perfect hashing, but for dynamic data there's still an open problem how to sort and hash it efficiently on the GPU. For CPU physics most production physics engines already use something similar, so it's a relatively common knowledge.

## Pros and cons

Pros:
* Zero memory overhead when spatial hashing is not used - tiles and spatial indices are implicit and not stored in memory.
* When spatial hashing is used, memory overhead is low - just an exra hash_table<int64_t, uint32_t>, this is entirely optional but provides big performance improvement.
* Particle data is densely packed with no gaps or wasted space.
* Easy to paralellize and can be trivially implemented on the GPU.

Cons:
* Data structure needs to be always in sync. Every time particle positions are updated you need to sort them, or spatial queries will not work properly.
* Binary searches are expensive, so you need a sufficiently large amount of particles and high particle density to outperform brute-force and other approaches.
* In some cases it might be impractical to sort the entire particle array after every update step, this demo deals with it by accumulating the collision response force and only applying this force after all collisions are resolved (particles are sorted again after applying forces).
* Spatial hashing mode will be hard to port to the GPU and run poorly there.
