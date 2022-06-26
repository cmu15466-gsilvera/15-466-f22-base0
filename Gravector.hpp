#include "ColorTextureProgram.hpp"

#include "GL.hpp"
#include "Mode.hpp"

#include <glm/glm.hpp>

#include <deque>
#include <vector>

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
    glm::vec2 ball_radius = glm::vec2(0.2f, 0.2f);

    glm::vec2 ball = glm::vec2(0.0f, 0.0f);
    glm::vec2 ball_velocity = glm::vec2(0.0f, 0.0f);
    glm::vec2 ball_accel = glm::vec2(0.0f, 0.0f);

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
