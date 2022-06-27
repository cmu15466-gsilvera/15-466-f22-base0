#include "Gravector.hpp"

// for the GL_ERRORS() macro:
#include "gl_errors.hpp"

// for glm::value_ptr() :
#include <glm/gtc/type_ptr.hpp>

int Ball::index = 0;
bool Ball::color_win[4];
Gravector::Gravector()
{

    //----- allocate OpenGL resources -----
    { // vertex buffer:
        glGenBuffers(1, &vertex_buffer);
        // for now, buffer will be un-filled.

        GL_ERRORS(); // PARANOIA: print out any OpenGL errors that may have happened
    }

    for (size_t i = 0; i < num_init_balls; i++)
    {
        new_ball();
    }

    for (size_t i = 0; i < 4; i++)
    {
        score[i] = 0;
        Ball::color_win[i] = false;
    }

    { // vertex array mapping buffer for color_texture_program:
        // ask OpenGL to fill vertex_buffer_for_color_texture_program with the name of an unused vertex array object:
        glGenVertexArrays(1, &vertex_buffer_for_color_texture_program);

        // set vertex_buffer_for_color_texture_program as the current vertex array object:
        glBindVertexArray(vertex_buffer_for_color_texture_program);

        // set vertex_buffer as the source of glVertexAttribPointer() commands:
        glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer);

        // set up the vertex array object to describe arrays of Gravector::Vertex:
        glVertexAttribPointer(color_texture_program.Position_vec4, // attribute
                              3,                                   // size
                              GL_FLOAT,                            // type
                              GL_FALSE,                            // normalized
                              sizeof(Vertex),                      // stride
                              (GLbyte *)0 + 0                      // offset
        );
        glEnableVertexAttribArray(color_texture_program.Position_vec4);
        //[Note that it is okay to bind a vec3 input to a vec4 attribute -- the w component will be filled with 1.0
        // automatically]

        glVertexAttribPointer(color_texture_program.Color_vec4, // attribute
                              4,                                // size
                              GL_UNSIGNED_BYTE,                 // type
                              GL_TRUE,                          // normalized
                              sizeof(Vertex),                   // stride
                              (GLbyte *)0 + 4 * 3               // offset
        );
        glEnableVertexAttribArray(color_texture_program.Color_vec4);

        glVertexAttribPointer(color_texture_program.TexCoord_vec2, // attribute
                              2,                                   // size
                              GL_FLOAT,                            // type
                              GL_FALSE,                            // normalized
                              sizeof(Vertex),                      // stride
                              (GLbyte *)0 + 4 * 3 + 4 * 1          // offset
        );
        glEnableVertexAttribArray(color_texture_program.TexCoord_vec2);

        // done referring to vertex_buffer, so unbind it:
        glBindBuffer(GL_ARRAY_BUFFER, 0);

        // done setting up vertex array object, so unbind it:
        glBindVertexArray(0);

        GL_ERRORS(); // PARANOIA: print out any OpenGL errors that may have happened
    }

    { // solid white texture:
        // ask OpenGL to fill white_tex with the name of an unused texture object:
        glGenTextures(1, &white_tex);

        // bind that texture object as a GL_TEXTURE_2D-type texture:
        glBindTexture(GL_TEXTURE_2D, white_tex);

        // upload a 1x1 image of solid white to the texture:
        glm::uvec2 size = glm::uvec2(1, 1);
        std::vector<glm::u8vec4> data(size.x * size.y, glm::u8vec4(0xff, 0xff, 0xff, 0xff));
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, size.x, size.y, 0, GL_RGBA, GL_UNSIGNED_BYTE, data.data());

        // set filtering and wrapping parameters:
        //(it's a bit silly to mipmap a 1x1 texture, but I'm doing it because you may want to use this code to load
        // different sizes of texture)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

        // since texture uses a mipmap and we haven't uploaded one, instruct opengl to make one for us:
        glGenerateMipmap(GL_TEXTURE_2D);

        // Okay, texture uploaded, can unbind it:
        glBindTexture(GL_TEXTURE_2D, 0);

        GL_ERRORS(); // PARANOIA: print out any OpenGL errors that may have happened
    }

    next_spawn_t += 4 * rand_next_time() / float(rand_next_time.max());
}

Gravector::~Gravector()
{

    //----- free OpenGL resources -----
    glDeleteBuffers(1, &vertex_buffer);
    vertex_buffer = 0;

    glDeleteVertexArrays(1, &vertex_buffer_for_color_texture_program);
    vertex_buffer_for_color_texture_program = 0;

    glDeleteTextures(1, &white_tex);
    white_tex = 0;
}

bool Gravector::handle_event(SDL_Event const &evt, glm::uvec2 const &window_size)
{

    if (evt.type == SDL_MOUSEMOTION)
    {
        // convert mouse from window pixels (top-left origin, +y is down) to clip space ([-1,1]x[-1,1], +y is up):
        glm::vec2 clip_mouse = glm::vec2((evt.motion.x + 0.5f) / window_size.x * 2.0f - 1.0f,
                                         (evt.motion.y + 0.5f) / window_size.y * -2.0f + 1.0f);
        direction_heading = glm::atan(clip_mouse.y, clip_mouse.x);
        triangle_radius.y = magnitude_scale * glm::length(clip_mouse);
    }

    if (evt.type == SDL_KEYDOWN && evt.key.keysym.sym == SDLK_SPACE)
    {
        new_ball();
    }

    return false;
}

void Gravector::new_ball()
{
    static std::mt19937 mt;
    const float random_x = (mt() / float(mt.max())) * 2.f * court_radius.x - court_radius.x;
    const float random_y = (mt() / float(mt.max())) * 2.f * court_radius.y - court_radius.y;
    const float random_vel_x = (mt() / float(mt.max())) * 2.f - 1;
    const float random_vel_y = (mt() / float(mt.max())) * 2.f - 1;
    balls.push_back(Ball(glm::vec2(random_x, random_y), glm::vec2(random_vel_x, random_vel_y), glm::vec2(0.0f, 0.0f)));
}

void Gravector::update(float elapsed)
{
    current_time += elapsed;

    static std::mt19937 mt; // mersenne twister pseudo-random number generator

    if (current_time > next_spawn_t)
    {
        new_ball();
        next_spawn_t += 4 * rand_next_time() / float(rand_next_time.max());
    }

    //----- ball update -----

    for (Ball &b : balls)
    {
        // eulerian update
        b.accel = glm::vec2(glm::cos(direction_heading), glm::sin(direction_heading)) * triangle_radius.y;
        b.vel += elapsed * b.accel * gravity_scale;
        b.pos += elapsed * b.vel;
        b.radius += radius_growth;
        b.mass += mass_growth;
    }

    for (Ball &b : balls)
    {
        // handle collisions
        for (Ball &other_b : balls)
        {
            if (&b != &other_b)
            {
                const float dist = glm::length(b.pos - other_b.pos);
                const float overlap = (b.radius + other_b.radius) - dist;
                if (overlap > 0)
                {
                    // compute overlap offset
                    const float b_speed = glm::length(b.vel);
                    const float other_b_speed = glm::length(other_b.vel);
                    const glm::vec2 disp = overlap * ((b.pos - other_b.pos) / dist);
                    b.pos += (b_speed / (b_speed + other_b_speed)) * disp;
                    other_b.pos -= (other_b_speed / (b_speed + other_b_speed)) * disp;

                    // update their positions by one velocity tick
                    b.pos -= elapsed * b.vel;
                    other_b.pos -= elapsed * other_b.vel;

                    // compute collision
                    const float sum_mass = b.mass + other_b.mass;
                    glm::vec2 new_b_vel = ball_ball_collision_damping *
                                          (b.vel * (b.mass - other_b.mass) + (2 * other_b.mass * other_b.vel)) /
                                          sum_mass;
                    other_b.vel = ball_ball_collision_damping *
                                  (other_b.vel * (other_b.mass - b.mass) + (2 * b.mass * b.vel)) / sum_mass;
                    b.vel = new_b_vel;

                    // update their positions by one velocity tick
                    b.pos += elapsed * b.vel;
                    other_b.pos += elapsed * other_b.vel;

                    if (b.radius > min_ball_radius)
                    {
                        // shrink these balls
                        b.radius *= radius_shrink_factor;
                        b.mass *= mass_shrink_factor;
                    }
                    if (other_b.radius > min_ball_radius)
                    {
                        other_b.radius *= radius_shrink_factor;
                        other_b.mass *= mass_shrink_factor;
                    }
                }
            }
        }
    }

    std::vector<Ball> remaining_balls;
    for (Ball &b : balls)
    {
        if (b.finished == false)
        {
            remaining_balls.push_back(b);
        }
        else
        {
            score[b.color_idx] += b.mass;
            /// TODO: don't assume just court_radius.x
            if (score[b.color_idx] > (court_radius.x / score_scale))
            {
                score[b.color_idx] = (court_radius.x / score_scale);
                b.color_win[b.color_idx] = true;
            }
        }
    }
    balls.clear();
    balls.swap(remaining_balls);
    // now balls contains only remaining balls

    /// TODO: refactor
    for (Ball &ball : balls)
    {
        // court walls:
        if (ball.pos.y > court_radius.y - ball.radius) // top wall
        {
            ball.pos.y = court_radius.y - ball.radius;
            if (ball.vel.y > 0.0f)
            {
                ball.vel.y = -ball_wall_collision_damping * ball.vel.y;
                ball.accel.y = 0;
            }

            if (ball.color_idx == 0)
            {
                ball.finished = true;
            }
        }
        if (ball.pos.y < -court_radius.y + ball.radius) // bottom wall
        {
            ball.pos.y = -court_radius.y + ball.radius;
            if (ball.vel.y < 0.0f)
            {
                ball.vel.y = -ball_wall_collision_damping * ball.vel.y;
                ball.accel.y = 0;
            }
            if (ball.color_idx == 2)
            {
                ball.finished = true;
            }
        }
        if (ball.pos.x > court_radius.x - ball.radius) // right wall
        {
            ball.pos.x = court_radius.x - ball.radius;
            if (ball.vel.x > 0.0f)
            {
                ball.vel.x = -ball_wall_collision_damping * ball.vel.x;
                ball.accel.x = 0;
            }

            if (ball.color_idx == 1)
            {
                ball.finished = true;
            }
        }
        if (ball.pos.x < -court_radius.x + ball.radius) // left wall
        {
            ball.pos.x = -court_radius.x + ball.radius;
            if (ball.vel.x < 0.0f)
            {
                ball.vel.x = -ball_wall_collision_damping * ball.vel.x;
                ball.accel.x = 0;
            }

            if (ball.color_idx == 3)
            {
                ball.finished = true;
            }
        }
    }
}

void Gravector::draw(glm::uvec2 const &drawable_size)
{
// some nice colors from the course web page:
#define HEX_TO_U8VEC4(HX) (glm::u8vec4((HX >> 24) & 0xff, (HX >> 16) & 0xff, (HX >> 8) & 0xff, (HX)&0xff))
    const glm::u8vec4 bg_color = HEX_TO_U8VEC4(0x193b59ff);
    // const glm::u8vec4 fg_color = HEX_TO_U8VEC4(0xf2d2b6ff);
    // const glm::u8vec4 shadow_color = HEX_TO_U8VEC4(0xf2ad94ff);
    const std::vector<glm::u8vec4> trail_colors = {
        HEX_TO_U8VEC4(0xf2ad9488),
        HEX_TO_U8VEC4(0xf2897288),
        HEX_TO_U8VEC4(0xbacac088),
    };
#undef HEX_TO_U8VEC4

    // other useful drawing constants:
    const float wall_radius = 0.05f;
    const float padding = 0.54f; // padding between outside of walls and edge of window

    //---- compute vertices to draw ----

    // vertices will be accumulated into this list and then uploaded+drawn at the end of this function:
    std::vector<Vertex> vertices;

    // inline helper function for rectangle drawing:
    auto draw_rectangle = [&vertices](glm::vec2 const &center, glm::vec2 const &radius, glm::u8vec4 const &color) {
        // draw rectangle as two CCW-oriented triangles:
        vertices.emplace_back(glm::vec3(center.x - radius.x, center.y - radius.y, 0.0f), color, glm::vec2(0.5f, 0.5f));
        vertices.emplace_back(glm::vec3(center.x + radius.x, center.y - radius.y, 0.0f), color, glm::vec2(0.5f, 0.5f));
        vertices.emplace_back(glm::vec3(center.x + radius.x, center.y + radius.y, 0.0f), color, glm::vec2(0.5f, 0.5f));

        vertices.emplace_back(glm::vec3(center.x - radius.x, center.y - radius.y, 0.0f), color, glm::vec2(0.5f, 0.5f));
        vertices.emplace_back(glm::vec3(center.x + radius.x, center.y + radius.y, 0.0f), color, glm::vec2(0.5f, 0.5f));
        vertices.emplace_back(glm::vec3(center.x - radius.x, center.y + radius.y, 0.0f), color, glm::vec2(0.5f, 0.5f));
    };

    auto draw_circle = [&vertices](glm::vec2 const &center, glm::vec2 const &radius, glm::u8vec4 const &color,
                                   const int num_verts = 20) {
        // draw a circle by drawing a bunch of triangles
        const int N = num_verts; // of vertices used to draw circle

        glm::vec2 circ_verts[N];
        for (int i = 0; i < N; i++)
        {
            circ_verts[i] = glm::vec2((radius.x * glm::cos(i * 2 * M_PI / N)), (radius.y * glm::sin(i * 2 * M_PI / N)));
        }

        for (int i = 0; i < N; i++)
        {
            vertices.emplace_back(glm::vec3(center.x, center.y, 0.0f), color, glm::vec2(0.5f, 0.5f));
            vertices.emplace_back(glm::vec3(center.x + circ_verts[(i) % N].x, center.y + circ_verts[i % N].y, 0.0f),
                                  color, glm::vec2(0.5f, 0.5f));
            vertices.emplace_back(
                glm::vec3(center.x + circ_verts[(i + 1) % N].x, center.y + circ_verts[(i + 1) % N].y, 0.0f), color,
                glm::vec2(0.5f, 0.5f));
        }
    };

    // inline helper function for triangle drawing:
    auto draw_triangle = [&vertices](glm::vec2 const &center, const glm::vec2 radius, const float heading,
                                     glm::u8vec4 const &color) {
        /// NOTE: heading is in radians!
        const float bottomLeft = heading + 2 * M_PI / 3;
        const float bottomRight = heading + 4 * M_PI / 3;
        const float Up = heading;

        // bottom left vertex
        vertices.emplace_back(
            glm::vec3(center.x + glm::cos(bottomLeft) * radius.x, center.y + glm::sin(bottomLeft) * radius.x, 0.0f),
            color, glm::vec2(0.5f, 0.5f));
        // bottom right vertex
        vertices.emplace_back(
            glm::vec3(center.x + glm::cos(bottomRight) * radius.x, center.y + glm::sin(bottomRight) * radius.x, 0.0f),
            color, glm::vec2(0.5f, 0.5f));
        // top vertex (colored red)
        vertices.emplace_back(glm::vec3(center.x + glm::cos(Up) * radius.y, center.y + glm::sin(Up) * radius.y, 0.0f),
                              color, glm::vec2(0.5f, 0.5f));
    };

    // solid objects:

    // walls:
    draw_rectangle(glm::vec2(-court_radius.x - wall_radius, 0.0f),
                   glm::vec2(wall_radius, court_radius.y + 2.0f * wall_radius), YELLOW);
    draw_rectangle(glm::vec2(court_radius.x + wall_radius, 0.0f),
                   glm::vec2(wall_radius, court_radius.y + 2.0f * wall_radius), GREEN);
    draw_rectangle(glm::vec2(0.0f, -court_radius.y - wall_radius), glm::vec2(court_radius.x, wall_radius), BLUE);
    draw_rectangle(glm::vec2(0.0f, court_radius.y + wall_radius), glm::vec2(court_radius.x, wall_radius), RED);

    // ball:
    for (const Ball &b : balls)
    {
        draw_circle(b.pos, glm::vec2(b.radius, b.radius), b.color);
        const float velocity_dir = glm::atan(b.vel.y, b.vel.x);
        draw_triangle(b.pos, glm::vec2(b.radius, 2 * b.radius), velocity_dir, b.color);
    }

    draw_triangle(triangle, triangle_radius, direction_heading, glm::u8vec4(255, 0, 255, 255));

    // draw scores
    const float score_height = 0.05f;

    draw_rectangle(glm::vec2(0.0f, court_radius.y + wall_radius + 4 * score_height),
                   glm::vec2(score_scale * score[0], score_height), RED);
    draw_rectangle(glm::vec2(court_radius.x + wall_radius + 4 * score_height, 0.0f),
                   glm::vec2(score_height, score_scale * score[1]), GREEN);
    draw_rectangle(glm::vec2(0.0f, -court_radius.y - wall_radius - 4 * score_height),
                   glm::vec2(score_scale * score[2], score_height), BLUE);
    draw_rectangle(glm::vec2(-court_radius.x - wall_radius - 4 * score_height, 0.0f),
                   glm::vec2(score_height, score_scale * score[3]), YELLOW);

    //------ compute court-to-window transform ------

    // compute area that should be visible:
    glm::vec2 scene_min =
        glm::vec2(-court_radius.x - 2.0f * wall_radius - padding, -court_radius.y - 2.0f * wall_radius - padding);
    glm::vec2 scene_max =
        glm::vec2(court_radius.x + 2.0f * wall_radius + padding, court_radius.y + 2.0f * wall_radius + padding);

    // compute window aspect ratio:
    float aspect = drawable_size.x / float(drawable_size.y);
    // we'll scale the x coordinate by 1.0 / aspect to make sure things stay square.

    // compute scale factor for court given that...
    float scale = std::min((2.0f * aspect) / (scene_max.x - scene_min.x), //... x must fit in [-aspect,aspect] ...
                           (2.0f) / (scene_max.y - scene_min.y)           //... y must fit in [-1,1].
    );

    glm::vec2 center = 0.5f * (scene_max + scene_min);

    // build matrix that scales and translates appropriately:
    glm::mat4 court_to_clip = glm::mat4(glm::vec4(scale / aspect, 0.0f, 0.0f, 0.0f), glm::vec4(0.0f, scale, 0.0f, 0.0f),
                                        glm::vec4(0.0f, 0.0f, 1.0f, 0.0f),
                                        glm::vec4(-center.x * (scale / aspect), -center.y * scale, 0.0f, 1.0f));
    // NOTE: glm matrices are specified in *Column-Major* order,
    // so each line above is specifying a *column* of the matrix(!)

    // also build the matrix that takes clip coordinates to court coordinates (used for mouse handling):
    clip_to_court =
        glm::mat3x2(glm::vec2(aspect / scale, 0.0f), glm::vec2(0.0f, 1.0f / scale), glm::vec2(center.x, center.y));

    //---- actual drawing ----

    // clear the color buffer:
    glClearColor(bg_color.r / 255.0f, bg_color.g / 255.0f, bg_color.b / 255.0f, bg_color.a / 255.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    // use alpha blending:
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    // don't use the depth test:
    glDisable(GL_DEPTH_TEST);

    // upload vertices to vertex_buffer:
    glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer); // set vertex_buffer as current
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(vertices[0]), vertices.data(),
                 GL_STREAM_DRAW); // upload vertices array
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    // set color_texture_program as current program:
    glUseProgram(color_texture_program.program);

    // upload OBJECT_TO_CLIP to the proper uniform location:
    glUniformMatrix4fv(color_texture_program.OBJECT_TO_CLIP_mat4, 1, GL_FALSE, glm::value_ptr(court_to_clip));

    // use the mapping vertex_buffer_for_color_texture_program to fetch vertex data:
    glBindVertexArray(vertex_buffer_for_color_texture_program);

    // bind the solid white texture to location zero so things will be drawn just with their colors:
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, white_tex);

    // run the OpenGL pipeline:
    glDrawArrays(GL_TRIANGLES, 0, GLsizei(vertices.size()));

    // unbind the solid white texture:
    glBindTexture(GL_TEXTURE_2D, 0);

    // reset vertex array to none:
    glBindVertexArray(0);

    // reset current program to none:
    glUseProgram(0);

    GL_ERRORS(); // PARANOIA: print errors just in case we did something wrong.
}
