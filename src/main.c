#include <JEL/jel.h>
#include <SDL2/SDL.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>

// Game stuff
#define FPS 60
#define PI 3.14

int           running = 1;
SDL_Window   *window = NULL;
SDL_Renderer *renderer = NULL;
int           mouse_x = 0;
int           mouse_y = 0;
int           mouse_down = 0;
int           mouse_down_prev = 0; // If the mouse was down on the previous frame
int           mouse_right_down = 0;
int           mouse_right_down_prev;
int           window_w = 960;
int           window_h = 640;
int           current_size = 2;
int           gravity_on = 1;

// Components
JEL_COMPONENT_DEFINE(Position, float, x, float, y);
JEL_COMPONENT_CREATE(Position, float, x, float, y);

JEL_COMPONENT_DEFINE(Physics, float, x_vel, float, y_vel, float, magnitude, float, angle, float, mass);
JEL_COMPONENT_CREATE(Physics, float, x_vel, float, y_vel, float, magnitude, float, angle, float, mass);

// Other stuff
#define MASS_AMPLIFY 100
// 2D arrays to store the results of individual magnitudes and angles
size_t buffer_count = 128;
int    num_points = 0;
float *magnitudes = NULL;
float *angles = NULL;

void allocate_buffers(size_t count)
{
  if (count <= buffer_count) {
    return;
  }

  float *new_magnitudes;
  float *new_angles;

  new_magnitudes = malloc(sizeof(float) * count);
  new_angles = malloc(sizeof(float) * count);

  void *m_bp = magnitudes;
  void *a_bp = angles;
  for (size_t i = 0; i < buffer_count; ++i) {
    memcpy(m_bp, new_magnitudes + i * count, sizeof(float) * buffer_count);
    memcpy(a_bp, new_angles + i * count, sizeof(float) * buffer_count);

    m_bp += buffer_count;
    a_bp += buffer_count;
  }

  free(magnitudes);
  free(angles);

  magnitudes = new_magnitudes;
  angles = new_angles;

  buffer_count = count;
}

// Systems
void gravity(void)
{
 struct JEL_Query *q;
 JEL_QUERY(q, Position, Physics);

  for (JEL_ComponentInt i = 0; i < q->tables_num; ++i) {
    struct PositionFragment *position;
    struct PhysicsFragment *physics;
    JEL_FRAGMENT_GET(position, q->tables[i], Position);
    JEL_FRAGMENT_GET(physics, q->tables[i], Physics);

    for (JEL_EntityInt j = 0; j < q->tables[i]->num; ++j) {
      magnitudes[j * buffer_count + j] = 0;
      angles[j * buffer_count + j] = 0;

      for (JEL_EntityInt k = j + 1; k < q->tables[i]->num; ++k) {
        float x_diff = position->x[k] - position->x[j];
        float y_diff = position->y[k] - position->y[j];

        float r = sqrt(pow(x_diff, 2) + pow(y_diff, 2));
        float m = r == 0 ? 0 : physics->mass[j] * physics->mass[k] / (r * r);
        // The angle is how i points to j
        
        float a = x_diff == 0 ? (y_diff > 0 ? 3 * PI / 2 : PI / 2) : atan(y_diff / x_diff);
        if (x_diff < 0) {
          a = PI;
        }

        magnitudes[j * buffer_count + k] = m;
        angles[j * buffer_count + k] = a ;
        magnitudes[k * buffer_count + j] = m;
        angles[k * buffer_count + j] = a + PI;

      }

      // Figure out resultant
      physics->magnitude[j] = 0;
      physics->angle[j] = 0;
      
      for (size_t k = 0; k < buffer_count; ++k) {
        if (magnitudes[j * buffer_count + k] == 0) {
          continue;
        }

        float x_1 = physics->magnitude[j] * cos(physics->angle[j]);
        float y_1 = physics->magnitude[j] * sin(physics->angle[j]);

        float x_comp = x_1 + (magnitudes[j * buffer_count + k] * cos(angles[j * buffer_count + k]));
        float y_comp = y_1 + (magnitudes[j * buffer_count + k] * sin(angles[j * buffer_count + k]));

        physics->magnitude[j] = sqrt(x_comp * x_comp + y_comp * y_comp);
        physics->angle[j] = x_comp == 0 ? PI / 2: atan(y_comp / x_comp);
        if (x_comp == 0) {
          if (y_comp < 0) {
            physics->angle[j] += PI;
          }
        }
        else if (x_comp < 0) {
          physics->angle[j] += PI;
        }
      }
    }
  }

 JEL_query_destroy(q);

}

void physics(void)
{
 struct JEL_Query *q;
 JEL_QUERY(q, Position, Physics);

  for (JEL_ComponentInt i = 0; i < q->tables_num; ++i) {
    struct PositionFragment *position;
    struct PhysicsFragment *physics;
    struct JEL_EntityCFragment *entity;
    JEL_FRAGMENT_GET(position, q->tables[i], Position);
    JEL_FRAGMENT_GET(physics, q->tables[i], Physics);
    JEL_FRAGMENT_GET(entity, q->tables[i], JEL_EntityC);

    for (JEL_EntityInt j = 0; j < q->tables[i]->num; ++j) {
      if (gravity_on) {
        physics->x_vel[j] += physics->magnitude[j] * cos(physics->angle[j]) / physics->mass[j];
        physics->y_vel[j] += physics->magnitude[j] * sin(physics->angle[j]) / physics->mass[j];
      }

      position->x[j] += physics->x_vel[j];
      position->y[j] += physics->y_vel[j];
    }
  }

 JEL_query_destroy(q);
}

void draw_points(void)
{
  struct JEL_Query *q;
  JEL_QUERY(q, Position, Physics);

  SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);

  for (JEL_ComponentInt i = 0; i < q->tables_num; ++i) {
    struct PositionFragment *position;
    struct PhysicsFragment *physics;
    JEL_FRAGMENT_GET(position, q->tables[i], Position);
    JEL_FRAGMENT_GET(physics, q->tables[i], Physics);

    for (JEL_EntityInt j = 0; j < q->tables[i]->num; ++j) {
      // Draw the square
      float m_squared = pow(physics->mass[j] / MASS_AMPLIFY, 2);
      SDL_Rect r = {position->x[j] - m_squared / 2, position->y[j] - m_squared / 2, m_squared, m_squared};
      SDL_RenderDrawRect(renderer, &r);
      // Draw the net force
      if (physics->magnitude[j] == 0) {
        continue;
      }
      int amplify = 1;
      SDL_RenderDrawLine(renderer, position->x[j], position->y[j], position->x[j] + physics->magnitude[j] * amplify * cos(physics->angle[j]), position->y[j] + physics->magnitude[j] * amplify * sin(physics->angle[j]));
    }
  }

  JEL_query_destroy(q);
}

// Game loop functions
void input(void)
{
  SDL_Event e;

  while (SDL_PollEvent(&e)) {
    switch (e.type) {
      case SDL_QUIT:
        running = 0;
        break;
      case SDL_MOUSEBUTTONDOWN:
        switch (e.button.button) {
          case SDL_BUTTON_LEFT:
            mouse_down = 1;
            break;
        }
        break;
      case SDL_MOUSEBUTTONUP:
        switch (e.button.button) {
          case SDL_BUTTON_LEFT:
            mouse_down = 0;
            break;
        }
        break;
      case SDL_KEYDOWN:
        switch (e.key.keysym.sym) {
          case SDLK_1:
            current_size = 2;
            break;
          case SDLK_2:
            current_size = 3;
            break;
          case SDLK_3:
            current_size = 4;
            break;
          case SDLK_4:
            current_size = 5;
            break;
          case SDLK_5:
            current_size = 6;
            break;
          case SDLK_6:
            current_size = 7;
            break;
          case SDLK_7:
            current_size = 8;
            break;
          case SDLK_8:
            current_size = 9;
            break;
          case SDLK_9:
            current_size = 10;
            break;
        }
        break;
    }
  }

  SDL_GetMouseState(&mouse_x, &mouse_y);
  SDL_GetWindowSize(window, &window_h, &window_w);
}

void update(void)
{
  if (mouse_down && !mouse_down_prev) {
    if (num_points == buffer_count) {
      allocate_buffers(buffer_count * 2);
    }
    JEL_Entity point = JEL_entity_create();
    JEL_ENTITY_ADD(point, Position, Physics);
    JEL_ENTITY_SET(point, Position, x, mouse_x);
    JEL_ENTITY_SET(point, Position, y, mouse_y);
    JEL_ENTITY_SET(point, Physics, x_vel, 0);
    JEL_ENTITY_SET(point, Physics, y_vel, 0);
    JEL_ENTITY_SET(point, Physics, magnitude, 0);
    JEL_ENTITY_SET(point, Physics, angle, 0);
    JEL_ENTITY_SET(point, Physics, mass, current_size * MASS_AMPLIFY);
  }

  if (gravity_on) {
    gravity();
  }

  physics();

  // Update mouse down prev
  if (mouse_down) {
    mouse_down_prev = 1;
  }
  else {
    mouse_down_prev = 0;
  }
}

void draw(void)
{
  SDL_SetRenderDrawColor(renderer, 0, 1, 20, 255);
  SDL_RenderClear(renderer);

  draw_points();

  // Draw a dot at the cursor with the current size
  SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
  SDL_Rect r = {mouse_x - current_size * current_size / 2, mouse_y - current_size * current_size / 2, current_size * current_size, current_size * current_size};
  SDL_RenderFillRect(renderer, &r);

  SDL_RenderPresent(renderer);
}

// Main
int main(void)
{
  SDL_Init(SDL_INIT_EVERYTHING);
  JEL_init();

  window = SDL_CreateWindow("Simulation", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, window_w, window_h, SDL_WINDOW_RESIZABLE);
  renderer = SDL_CreateRenderer(window, -1, 0);

  magnitudes = calloc(buffer_count * buffer_count, sizeof(float));
  angles = calloc(buffer_count * buffer_count, sizeof(float));

  JEL_COMPONENT_REGISTER(Position);
  JEL_COMPONENT_REGISTER(Physics);

  while (running) {
    uint32_t frame_start = SDL_GetTicks();

    input();
    update();
    draw();

    int frame_time = SDL_GetTicks() - frame_start;

    if (1000.0 / FPS > frame_time) {
      SDL_Delay(1000.0 / FPS - frame_time);
    }
  }

  SDL_DestroyRenderer(renderer);
  SDL_DestroyWindow(window);

  JEL_quit();
  SDL_Quit();

  return 0;
}
