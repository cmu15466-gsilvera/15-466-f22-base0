#include "ColorTextureProgram.hpp"

#include "GL.hpp"
#include "Mode.hpp"

#include <glm/glm.hpp>

#include <deque>
#include <vector>

#define RED glm::u8vec4(255.f, 0.f, 0.f, 255.f)
#define GREEN glm::u8vec4(0.f, 255.f, 0.f, 255.f)
#define BLUE glm::u8vec4(0.f, 0.f, 255.f, 255.f)
#define YELLOW glm::u8vec4(255.f, 255.f, 0.f, 255.f)

struct Ball
{
    Ball(const glm::vec2 &p, const glm::vec2 &v, const glm::vec2 &a) : pos(p), vel(v), accel(a)
    {
        radius = 0.2f;
        mass = 1.0f;
        color_idx = Ball::index % 4;
        if (color_idx == 0)
            color = RED;
        else if (color_idx == 1)
            color = GREEN;
        else if (color_idx == 2)
            color = BLUE;
        else if (color_idx == 3)
            color = YELLOW;
        Ball::index += 1;
    }
    glm::vec2 pos, vel, accel;
    float radius, mass;
    glm::u8vec4 color;
    int color_idx;
    bool finished = false;
    static int index;
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

    glm::vec2 court_radius = glm::vec2(7.0f, 5.0f);

    const size_t num_init_balls = 1;
    const float gravity_scale = 4.f;
    const float radius_growth = 0.001f;
    const float mass_growth = 0.001f;
    const float radius_shrink_factor = 0.5f; // during collisions
    const float mass_shrink_factor = 0.5f;   // during collisions
    const float min_ball_radius_before_death = 0.01f;
    const float ball_ball_collision_damping = 1.1f; // how much "energy" is preserved after ball-ball collision
    const float ball_wall_collision_damping = 0.3f; // how much "energy" is preserved after ball-wall collision

    float current_time = 0.f;
    std::vector<Ball> balls;
    void new_ball();
    size_t score[4];

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
