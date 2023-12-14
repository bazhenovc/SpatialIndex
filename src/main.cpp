
#pragma warning(push)
#pragma warning(disable:4267)
#include "ofMain.h"
#include "ofApp.h"
#pragma warning(pop)

#include "SpatialIndex.h"

#include <sstream>

static const int WINDOW_WIDTH = 1920;
static const int WINDOW_HEIGHT = 1080;

static const int PARTICLE_COUNT = 5000;
static const int SOLVER_ITERATIONS = 3;

static const float PARTICLE_BOUNCE = 0.05F;

class SpatialIndexApp : public ofApp
{
public:

	virtual void keyReleased(int key) override;
	virtual void mouseScrolled(int x, int y, float scrollX, float scrollY) override;

	virtual void setup() override;
	virtual void update() override;
	virtual void draw() override;

private:

	void reset_simulation();

	std::vector<Particle2D> m_particles;
	std::vector<std::pair<Particle2D*, Particle2D*>> m_contact_pairs;

	uint64_t m_update_time = 0;
	uint64_t m_query_time = 0;

	bool m_use_spatial_query = true;

	bool m_paused = false;
	bool m_run_step = false;

	bool m_draw_grid = false;

	float m_search_radius = 100.0F;
	size_t m_found_particles = 0;
	size_t m_checked_particles = 0;
	size_t m_solved_contacts = 0;
};

void SpatialIndexApp::keyReleased(int key)
{
	if (key == ofKey::OF_KEY_TAB)
	{
		m_use_spatial_query = !m_use_spatial_query;
	}

	if (key == 'g')
	{
		m_draw_grid = !m_draw_grid;
	}

	if (key == 'r')
	{
		reset_simulation();
	}

	if (key == ' ')
	{
		m_paused = !m_paused;
	}
	if (m_paused && key == ofKey::OF_KEY_RETURN)
	{
		m_run_step = true;
	}
}

void SpatialIndexApp::mouseScrolled(int x, int y, float scrollX, float scrollY)
{
	m_search_radius += scrollY;
}

void SpatialIndexApp::setup()
{
	m_particles.resize(PARTICLE_COUNT);
	
	// Conservatively estimate contact pairs count, this will grow if needed
	m_contact_pairs.reserve(m_particles.size() / 4);

	reset_simulation();

	ofApp::setup();
}

void SpatialIndexApp::update()
{
	// Resolve collisions
	if (m_paused && !m_run_step)
	{
		return;
	}
	m_run_step = false;

	uint64_t update_start_time = ofGetElapsedTimeMicros();
	Particle2D* particle_data = m_particles.data(); // Don't want bounds checking in the inner loop

	m_solved_contacts = 0;
	for (size_t iteration = 0; iteration < SOLVER_ITERATIONS; ++iteration)
	{
		if (m_use_spatial_query)
		{
			m_contact_pairs.clear();
			for (size_t particle_index0 = 0; particle_index0 < m_particles.size(); ++particle_index0)
			{
				Particle2D& particle0 = particle_data[particle_index0];

				radius_query(m_particles, particle0.position, particle0.radius, [this, &particle0](Particle2D& particle1)
					{
						Particle2D* ptr0 = &particle0;
						Particle2D* ptr1 = &particle1;

						if (ptr0 != ptr1)
						{
							if (ptr0 > ptr1) { std::swap(ptr0, ptr1); }
							m_contact_pairs.push_back({ ptr0, ptr1 });
						}
					});
			}

			std::sort(m_contact_pairs.begin(), m_contact_pairs.end());
			m_contact_pairs.erase(std::unique(m_contact_pairs.begin(), m_contact_pairs.end()), m_contact_pairs.end());

			for (const auto& contact : m_contact_pairs)
			{
				Particle2D::resolve_collision(*contact.first, *contact.second, PARTICLE_BOUNCE);
			}

			m_solved_contacts += m_contact_pairs.size();
		}
		else
		{
			for (size_t particle_index0 = 0; particle_index0 < m_particles.size(); ++particle_index0)
			{
				Particle2D& particle0 = particle_data[particle_index0];
				for (size_t particle_index1 = particle_index0 + 1; particle_index1 < m_particles.size(); ++particle_index1)
				{
					Particle2D& particle1 = particle_data[particle_index1];
					Particle2D::resolve_collision(particle0, particle1, PARTICLE_BOUNCE);

					m_solved_contacts++;
				}
			}
		}

		// Bounce off window bounds
		for (Particle2D& particle : m_particles)
		{
			if (particle.position.x <= particle.radius || particle.position.x >= (float(WINDOW_WIDTH) - particle.radius) ||
				particle.position.y <= particle.radius || particle.position.y >= (float(WINDOW_HEIGHT) - particle.radius))
			{
				ofVec2f border_point = ofVec2f(
					ofClamp(particle.position.x, particle.radius, float(WINDOW_WIDTH) - particle.radius),
					ofClamp(particle.position.y, particle.radius, float(WINDOW_HEIGHT) - particle.radius));

				particle.push(border_point - particle.position);
			}
		}

		// Resolve PBD and update spatial index
		float dt = 0.033F / float(SOLVER_ITERATIONS);// float(ofGetLastFrameTime());
		for (Particle2D& particle : m_particles)
		{
			particle.color = ofColor::lightCyan;
			particle.update(dt);
		}
		spatial_index_sort(m_particles);
	}

	m_update_time = ofGetElapsedTimeMicros() - update_start_time;

	ofVec2f mouse_position = ofVec2f(float(ofGetMouseX()), float(ofGetMouseY()));
	m_found_particles = 0;
	m_checked_particles = 0;

	// Paint particle inside the search radius
	uint64_t query_start_time = ofGetElapsedTimeMicros();
	if (m_use_spatial_query)
	{
		m_checked_particles = radius_query(m_particles, mouse_position, m_search_radius,
			[this](Particle2D& particle)
			{
				particle.color = ofColor::orangeRed;
				m_found_particles++;
			});
	}
	else
	{
		for (Particle2D& particle : m_particles)
		{
			float radius_sum = m_search_radius + particle.radius;
			if ((mouse_position - particle.position).lengthSquared() <= radius_sum * radius_sum)
			{
				particle.color = ofColor::orangeRed;
				m_found_particles++;
			}
		}

		m_checked_particles = m_particles.size();
	}
	m_query_time = ofGetElapsedTimeMicros() - query_start_time;

	ofApp::update();
}

void SpatialIndexApp::draw()
{
	ofClear(ofColor::darkSlateGray);

	for (const Particle2D& particle : m_particles)
	{
		ofSetColor(particle.color);
		ofDrawCircle(particle.position.x, particle.position.y, 0.0F, particle.radius);
	}

	ofColor circle_color = ofColor::orangeRed;
	circle_color.a = 50;
	ofSetColor(circle_color);
	ofDrawCircle(float(ofGetMouseX()), float(ofGetMouseY()), m_search_radius);

	if (m_draw_grid)
	{
		ofDrawGrid(float(TILE_SIZE), WINDOW_WIDTH / TILE_SIZE, false, false, false, true);
	}

	std::ostringstream oss;
	oss << "FPS: " << ofGetFrameRate()
		<< "\nUsing spatial indexing (TAB to toggle): " << m_use_spatial_query
		<< "\nPhysics update time: " << double(m_update_time) / 1000.0 << "ms"
		<< "\nParticles: " << PARTICLE_COUNT << " Solved contacts: " << m_solved_contacts
		<< "\nHighlight spatial query time: " << double(m_query_time) / 1000.0 << "ms"
		<< "\nHighlight search radius (mouse wheel to change): " << m_search_radius
		<< "\nHighlight checked/found particles: " << m_checked_particles << "/" << m_found_particles
		<< "\nSPACE to pause simulation, ENTER to run one simulation step when paused"
		<< "\nG to toggle grid, R to reset simulation";

	ofDrawBitmapStringHighlight(oss.str(), 20, 20);

	ofApp::draw();
}

void SpatialIndexApp::reset_simulation()
{
	for (int particle_index = 0; particle_index < PARTICLE_COUNT; ++particle_index)
	{
		Particle2D particle = {};

		particle.radius = ofRandom(4.0F, 8.0F);
		particle.position = {
			ofRandom(particle.radius, float(WINDOW_WIDTH) - particle.radius),
			ofRandom(particle.radius, float(WINDOW_HEIGHT) - particle.radius),
		};

		// particle.radius = float(TILE_SIZE / 2);
		// particle.position.x = 500.0F;
		// particle.position.y = 500.0F + float(particle_index * 10);

		m_particles[particle_index] = particle;
	}
}

int main()
{
	ofGLWindowSettings settings;
	settings.setSize(WINDOW_WIDTH, WINDOW_HEIGHT);
	settings.windowMode = OF_WINDOW;

	auto window = ofCreateWindow(settings);

	ofRunApp(window, make_shared<SpatialIndexApp>());
	ofRunMainLoop();
}
