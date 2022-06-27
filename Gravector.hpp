#include "ColorTextureProgram.hpp"

#include "GL.hpp"
#include "Mode.hpp"

#include <glm/glm.hpp>

#include <deque>
#include <random>
#include <vector>

#define RED glm::u8vec4(255.f, 0.f, 0.f, 255.f)
#define GREEN glm::u8vec4(0.f, 255.f, 0.f, 255.f)
#define BLUE glm::u8vec4(0.f, 0.f, 255.f, 255.f)
#define CYAN glm::u8vec4(0.f, 255.f, 255.f, 255.f)
#define YELLOW glm::u8vec4(255.f, 255.f, 0.f, 255.f)
#define WHITE glm::u8vec4(255.f, 255.f, 255.f, 255.f)

struct Ball
{
    Ball(const glm::vec2 &p, const glm::vec2 &v, const glm::vec2 &a, const glm::u8vec4 col = WHITE)
        : pos(p), vel(v), accel(a)
    {
        radius = 0.3f;
        mass = 1.0f;
        color = col;
    }

    glm::vec2 pos, vel, accel;
    float radius, mass;
    glm::u8vec4 color;
};

struct Goal
{

    void new_pos(const glm::vec2 &court_radius)
    {
        const float rand_x = (rand() / float(rand.max())) * 2.f * court_radius.x - court_radius.x;
        const float rand_y = (rand() / float(rand.max())) * 2.f * court_radius.y - court_radius.y;
        pos = glm::vec2(rand_x, rand_y);
    }
    std::mt19937 rand;

    glm::vec2 pos;
    float radius = 0.5;
    glm::u8vec4 color = GREEN;
};

/*
 * Gravector is a game mode that implements a single-player game of Gravector.
 */

struct Gravector : Mode
{
    Gravector();
    virtual ~Gravector();

    // functions called by main loop:
    virtual bool handle_event(SDL_Event const &, glm::uvec2 const &window_size) override;
    virtual void update(float elapsed) override;
    virtual void draw(glm::uvec2 const &drawable_size) override;
    //----- game state -----

    glm::vec2 court_radius = glm::vec2(7.0f, 7.0f);

    const size_t num_init_balls = 1;
    float gravity_scale = 1.f;
    const float gravity_scale_incr = 0.001f;
    const float ball_ball_collision_damping = 1.0f; // how much "energy" is preserved after ball-ball collision
    const float ball_wall_collision_damping = 0.3f; // how much "energy" is preserved after ball-wall collision

    const float ball_growth_rate = -0.0001f;
    const float spawn_rate = 3; // random spawning every ~3 s

    float current_time = 0.f;
    std::vector<Ball> balls;
    glm::vec2 main_ball_prev_pos;
    int num_lives = 5;
    float breather = 0.f;
    const float breather_amnt_sec = 1.f;
    Ball main_ball = Ball(glm::vec2(0.0f, 0.0f), glm::vec2(0.0f, 0.0f), glm::vec2(0.0f, 0.0f), BLUE);
    void new_ball();

    Goal goal = Goal();
    const float max_energy = court_radius.y;
    float energy = 3.f;
    float energy_use_rate = -0.5f;
    float energy_recharge_rate = 0.25f;
    int score = 0;
    const float score_scale = 2.0f; // make larger to have to score less, smaller to need to score more

    float next_spawn_t = 0.f;
    std::mt19937 rand_next_time;

    //--- direction vector
    float direction_heading = 0;
    // radius is (x, y) where x is the equilateral length & y is only the tip length (direction_magnitude)
    const float magnitude_scale = 1.f; // how much the mouse scale affects the gravity magnitude
    glm::vec2 triangle_radius = glm::vec2(0.5f, 1.f);
    glm::vec2 triangle = glm::vec2(0.0f, 0.0f);

    //----- opengl assets / helpers ------

    // draw functions will work on vectors of vertices, defined as follows:
    struct Vertex
    {
        Vertex(glm::vec3 const &Position_, glm::u8vec4 const &Color_, glm::vec2 const &TexCoord_)
            : Position(Position_), Color(Color_), TexCoord(TexCoord_)
        {
        }
        glm::vec3 Position;
        glm::u8vec4 Color;
        glm::vec2 TexCoord;
    };
    static_assert(sizeof(Vertex) == 4 * 3 + 1 * 4 + 4 * 2, "Gravector::Vertex should be packed");

    // Shader program that draws transformed, vertices tinted with vertex colors:
    ColorTextureProgram color_texture_program;

    // Buffer used to hold vertex data during drawing:
    GLuint vertex_buffer = 0;

    // Vertex Array Object that maps buffer locations to color_texture_program attribute locations:
    GLuint vertex_buffer_for_color_texture_program = 0;

    // Solid white texture:
    GLuint white_tex = 0;

    // matrix that maps from clip coordinates to court-space coordinates:
    glm::mat3x2 clip_to_court = glm::mat3x2(1.0f);
    // computed in draw() as the inverse of OBJECT_TO_CLIP
};
