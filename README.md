# SpatialIndex
Implicit unbounded spatial acceleration structure based on spatial indexing.

![image](https://github.com/bazhenovc/SpatialIndex/assets/986127/18575683-ef4d-4c16-84c5-a16998e78a83)

5000 colliding particles with different sizes, ~14ms when spatial indexing is used, ~150ms when brute-force "each-to-each" approach is used.

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

The demo uses spatial indexing to both find and resolve collision contact pairs and to find and highlight particles near the mouse cursor.

## Pros and cons

Pros:
* Zero memory overhead - tiles and spatial indices are implicit and not stored in memory.
* Particle data is densely packed with no gaps or wasted space.
* Easy to paralellize and can be trivially implemented on the GPU.

Cons:
* Data structure needs to be always in sync. Every time particle positions are updated you need to sort them, or spatial queries will not work properly.
* Binary searches are expensive, so you need a sufficiently large amount of particles and high particle density to outperform brute-force and other approaches.
* In some cases it might be impractical to sort the entire particle array after every update step, this demo deals with it by accumulating the collision response force and only applying this force after all collisions are resolved (particles are sorted again after applying forces).
