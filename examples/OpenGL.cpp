#include "Transform.hpp"
#include "PhysicsObject.hpp"
#include "PhysicsSystem.hpp"
#include "GravitySystem.hpp"
#include "Keywords.hpp"
#include "GravitySystem.hpp"

#include <optional>

#include <chrono>
#include <thread>
#pragma warning(push)
#pragma warning(disable : 4068)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Weverything"

#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/hash.hpp>

#pragma clang diagnostic pop
#pragma warning(pop)

class Colored {
public:
  ml::vec3 color{1.0f, 1.0f, 0.0f};
  bool     draw{true};
};

/*
 * OGL01Shape3D.cpp: 3D Shapes
 */

#include <GL/glut.h>  // GLUT, include glu.h and gl.h
#include "Shapes/AABB.hpp"
#include "Shapes/OBB.hpp"
#include "Shapes/Sphere.hpp"
#include "Shapes/Capsule.hpp"
#include <math.h> /* sin */
#include <iostream>

/* Global variables */
char  title[]   = "3D Shapes";
float angleCube = 0.0f;  // Rotational angle for cube [NEW]
float alpha     = 0.0f;  // Rotational angle for cube [NEW]
float gamma2    = 0.0f;  // Rotational angle for cube [NEW]
float beta      = 0.0f;  // Rotational angle for cube [NEW]
float x1        = 3.0f;
float yy1       = 0.0f;
float z1        = 0.0f;
float x2        = 5.0f;
float y2        = 2.0f;
float z2        = 0.0f;
float x3        = 5.0f;
float y3        = -2.0f;
float z3        = 0.0f;
float xSouris   = 0.0f;
float ySouris   = 0.0f;
// angle of rotation for the camera direction
float angle = 0.0;
// actual vector representing the camera's direction
float lx = 0.0f, lz = -1.0f;
// XZ position of the camera
float x = 0.0f;
float z = 12.0f;

float deltaAngle = 0.0f;
float deltaMove  = 0;
int   xOrigin    = -1;

/* Initialize OpenGL Graphics */
void initGL() {
  glClearColor(0.0f, 0.0f, 0.0f, 1.0f);               // Set background color to black and opaque
  glClearDepth(1.0f);                                 // Set background depth to farthest
  glEnable(GL_DEPTH_TEST);                            // Enable depth testing for z-culling
  glDepthFunc(GL_LEQUAL);                             // Set the type of depth-test
  glShadeModel(GL_SMOOTH);                            // Enable smooth shading
  glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);  // Nice perspective corrections
}

class UserCommand final {
public:
  enum Buttons {
    Reset    = (1 << 0),
    Left     = (1 << 1),
    Right    = (1 << 2),
    Forward  = (1 << 3),
    Backward = (1 << 4),
    Sprint   = (1 << 5),
    Duck     = (1 << 6),
    Jump     = (1 << 7),
  };

public:
  std::uint64_t buttons{};
};

UserCommand m_userCommand{};

void processSpecialKeys(int key, int xx, int yy) {
  switch (key) {
    case GLUT_KEY_UP:
      m_userCommand.buttons |= UserCommand::Buttons::Forward;
      break;
    case GLUT_KEY_LEFT:
      m_userCommand.buttons |= UserCommand::Buttons::Left;
      break;
    case GLUT_KEY_DOWN:
      m_userCommand.buttons |= UserCommand::Buttons::Backward;
      break;
    case GLUT_KEY_RIGHT:
      m_userCommand.buttons |= UserCommand::Buttons::Right;
      break;
    case GLUT_KEY_PAGE_UP:
      m_userCommand.buttons |= UserCommand::Buttons::Jump;
      break;
    case GLUT_KEY_PAGE_DOWN:
      m_userCommand.buttons |= UserCommand::Buttons::Duck;
      break;
  }
}

void processSpecialUpKeys(int key, int xx, int yy) {
  switch (key) {
    case GLUT_KEY_UP:
      m_userCommand.buttons &= ~UserCommand::Buttons::Forward;
      break;
    case GLUT_KEY_LEFT:
      m_userCommand.buttons &= ~UserCommand::Buttons::Left;
      break;
    case GLUT_KEY_DOWN:
      m_userCommand.buttons &= ~UserCommand::Buttons::Backward;
      break;
    case GLUT_KEY_RIGHT:
      m_userCommand.buttons &= ~UserCommand::Buttons::Right;
      break;
    case GLUT_KEY_PAGE_UP:
      m_userCommand.buttons &= ~UserCommand::Buttons::Jump;
      break;
    case GLUT_KEY_PAGE_DOWN:
      m_userCommand.buttons &= ~UserCommand::Buttons::Duck;
      break;
  }
}

glm::vec3 GetAnyPerpendicularUnitVector(const glm::vec3 &vec) {
  if (vec.y != 0.0f || vec.z != 0.0f)
    return glm::vec3(1, 0, 0);
  else
    return glm::vec3(0, 1, 0);
}

void drawTriangle(const glm::vec3 p0, const glm::vec3 p1, const glm::vec3 p2, glm::vec3 color) {
  float vertices[] = {
  p0.x,
  p0.y,
  p0.z,  // top corner
  p1.x,
  p1.y,
  p1.z,  // bottom left corner
  p2.x,
  p2.y,
  p2.z  // bottom right corner
  };
  glColor3f(color.r, color.g, color.b);       // Green
  glEnableClientState(GL_VERTEX_ARRAY);       // tell OpenGL that you're using a vertex array for fixed-function attribute
  glVertexPointer(3, GL_FLOAT, 0, vertices);  // point to the vertices to be used
  glDrawArrays(GL_TRIANGLES, 0, 3);           // draw the vertixes
  glDisableClientState(GL_VERTEX_ARRAY);      // tell OpenGL that you're finished using the vertex arrayattribute
}

void drawCapsule(glm::vec3 start, glm::vec3 end, float radius, glm::vec3 color) {
  const glm::vec3 axis   = end - start;
  const float     length = glm::length(axis);
  const glm::vec3 localZ = axis / length;
  const glm::vec3 localX = GetAnyPerpendicularUnitVector(localZ);
  const glm::vec3 localY = glm::cross(localZ, localX);

  using glm::cos;
  using glm::sin;
  constexpr float pi = glm::pi<float>();

  const glm::vec3 startP(0.0f);
  const glm::vec3 endP(1.0f);
  const float     resolution = 16.0f;

  const glm::vec3 step = (endP - startP) / resolution;

  auto cylinder = [localX, localY, localZ, start, length, radius](const float u, const float v) {
    return start                                   //
           + localX * cos(2.0f * pi * u) * radius  //
           + localY * sin(2.0f * pi * u) * radius  //
           + localZ * v * length;                  //
  };

  auto sphereStart = [localX, localY, localZ, start, radius](const float u, const float v) -> glm::vec3 {
    const float latitude = (pi / 2.0f) * (v - 1);

    return start                                                   //
           + localX * cos(2.0f * pi * u) * cos(latitude) * radius  //
           + localY * sin(2.0f * pi * u) * cos(latitude) * radius  //
           + localZ * sin(latitude) * radius;
  };

  auto sphereEnd = [localX, localY, localZ, end, radius](const float u, const float v) {
    const float latitude = (pi / 2.0f) * v;
    return end                                                     //
           + localX * cos(2.0f * pi * u) * cos(latitude) * radius  //
           + localY * sin(2.0f * pi * u) * cos(latitude) * radius  //
           + localZ * sin(latitude) * radius;
  };

  for (float i = 0; i < resolution; ++i) {
    for (float j = 0; j < resolution; ++j) {
      const float u = i * step.x + startP.x;
      const float v = j * step.y + startP.y;

      const float un = (i + 1 == resolution) ? endP.x : (i + 1) * step.x + startP.x;
      const float vn = (j + 1 == resolution) ? endP.y : (j + 1) * step.y + startP.y;

      // Draw Cylinder
      {
        const glm::vec3 p0 = cylinder(u, v);
        const glm::vec3 p1 = cylinder(u, vn);
        const glm::vec3 p2 = cylinder(un, v);
        const glm::vec3 p3 = cylinder(un, vn);

        drawTriangle(p0, p1, p2, color);
        drawTriangle(p3, p1, p2, color);
      }

      // Draw Sphere start
      {
        const glm::vec3 p0 = sphereStart(u, v);
        const glm::vec3 p1 = sphereStart(u, vn);
        const glm::vec3 p2 = sphereStart(un, v);
        const glm::vec3 p3 = sphereStart(un, vn);
        drawTriangle(p0, p1, p2, color);
        drawTriangle(p3, p1, p2, color);
      }

      // Draw Sphere end
      {
        const glm::vec3 p0 = sphereEnd(u, v);
        const glm::vec3 p1 = sphereEnd(u, vn);
        const glm::vec3 p2 = sphereEnd(un, v);
        const glm::vec3 p3 = sphereEnd(un, vn);
        drawTriangle(p0, p1, p2, color);
        drawTriangle(p3, p1, p2, color);
      }
    }
  }
}

/* Handler for window-repaint event. Called back when the window first appears and
   whenever the window needs to be re-painted. */

void drawAABB(std::vector<ml::vec3> points, float red, float green, float blue) {
  glBegin(GL_QUADS);            // Begin drawing the color cube with 6 quads
  glColor3f(red, green, blue);  // Green
                                //
  // Top face (y = 1.0f)
  // Define vertices in counter-clockwise (CCW) order with normal pointing out
  // glColor3f(1.0f, 0.0f, 0.0f);  // Red
  // glColor3f(red, green, blue);  // Green
  glVertex3f(points[2].x, points[2].y, points[2].z);
  glVertex3f(points[3].x, points[3].y, points[3].z);
  glVertex3f(points[4].x, points[4].y, points[4].z);
  glVertex3f(points[7].x, points[7].y, points[7].z);

  // Bottom face  (y = -1.0f)
  // glColor3f(1.0f, 0.5f, 0.0f);  // Orange
  // glColor3f(red, green, blue);  // Green
  glVertex3f(points[6].x, points[6].y, points[6].z);
  glVertex3f(points[5].x, points[5].y, points[5].z);
  glVertex3f(points[0].x, points[0].y, points[0].z);
  glVertex3f(points[1].x, points[1].y, points[1].z);

  // Front face  (z = 1.0f)
  // glColor3f(1.0f, 0.0f, 0.0f);  // Red
  // glColor3f(red, green, blue);  // Green
  glVertex3f(points[7].x, points[7].y, points[7].z);
  glVertex3f(points[4].x, points[4].y, points[4].z);
  glVertex3f(points[5].x, points[5].y, points[5].z);
  glVertex3f(points[6].x, points[6].y, points[6].z);

  // Back face (z = -1.0f)
  // glColor3f(1.0f, 1.0f, 0.0f);  // Yellow
  // glColor3f(red, green, blue);  // Green
  glVertex3f(points[1].x, points[1].y, points[1].z);
  glVertex3f(points[0].x, points[0].y, points[0].z);
  glVertex3f(points[3].x, points[3].y, points[3].z);
  glVertex3f(points[2].x, points[2].y, points[2].z);

  // Left face (x = -1.0f)
  // glColor3f(1.0f, 1.0f, 0.0f);  // Blue
  // glColor3f(red, green, blue);  // Green
  glVertex3f(points[4].x, points[4].y, points[4].z);
  glVertex3f(points[3].x, points[3].y, points[3].z);
  glVertex3f(points[0].x, points[0].y, points[0].z);
  glVertex3f(points[5].x, points[5].y, points[5].z);

  // Right face (x = 1.0f)
  // glColor3f(1.0f, 0.0f, 1.0f);  // Magenta
  // glColor3f(red, green, blue);  // Green
  glVertex3f(points[2].x, points[2].y, points[2].z);
  glVertex3f(points[7].x, points[7].y, points[7].z);
  glVertex3f(points[6].x, points[6].y, points[6].z);
  glVertex3f(points[1].x, points[1].y, points[1].z);
  glEnd();  // End of drawing color-cube
  // dram my cube
}

float windowWidth{0};
float windowHeight{0};
bool  m_pressed{false};

void reshape(GLsizei width, GLsizei height) {
  if (height == 0)
    height = 1;

  float aspect = (float)width / (float)height;

  glViewport(0, 0, width, height);

  windowWidth  = width;
  windowHeight = height;
}

int prevX{0};
int prevY{0};

void mouseMove(int x, int y) {
  int realX{x - prevX};
  int realY{y - prevY};

  // if (abs(realX) < 100.0) {
  /* FIX : if (m_pressed && m_camera.has_value()) {
    auto &cameraComponent{admin.getComponent<Components::Camera>(m_camera.value())};
    cameraComponent.angles += ml::vec3(static_cast<float>(-realY) * 0.033f * 9.0f, static_cast<float>(realX) * 0.033f * 9.0f, 0.0f);
    while (cameraComponent.angles.y >= 360.0f)
      cameraComponent.angles.y -= 360.0f;
    while (cameraComponent.angles.y < 0.0f)
      cameraComponent.angles.y += 360.0f;
    if (cameraComponent.angles.x < -89.0f)
      cameraComponent.angles.x = -89.0f;
    if (cameraComponent.angles.x > 89.0f)
      cameraComponent.angles.x = 89.0f;
  }

  // std::cout << realX << '\t' << realY << '\n';

  prevX = x;
  prevY = y;
  */
}

void mouseButton(int button, int state, int x, int y) {
  if (button == GLUT_LEFT_BUTTON) {
    if (state == GLUT_UP) {
      m_pressed = false;
      // std::cout << "GLUT_UP" << '\n';
    } else if (state == GLUT_DOWN) {
      // std::cout << "GLUT_DOWN" << '\n';
      m_pressed = true;
    }
  }
}

void display() {
  static auto previous{std::chrono::high_resolution_clock::now()};
  auto        delta{std::chrono::high_resolution_clock::now() - previous};
  previous = std::chrono::high_resolution_clock::now();

  float dt{std::chrono::duration_cast<std::chrono::nanoseconds>(delta).count() / 1000000000.0f};
  // std::cout << dt << " ms\t|\t" << 1.0f / dt << " FPS" << std::string(16, ' ') << '\r';

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);  // Clear color and depth buffers

  {
    /* FIX : auto &physics{admin.getComponent<Components::Physics>(m_camera.value())};
    auto &camera{admin.getComponent<Components::Camera>(m_camera.value())};
    auto &transform{admin.getComponent<Components::Transform>(m_camera.value())};

    auto  position{ml::vec3{0.0f, 0.0f, 0.0f}};  // transform.matrix.getTranslation()};
    float speed{(m_userCommand.buttons & UserCommand::Buttons::Sprint) ? 150.0f : 50.0f};
    if (m_userCommand.buttons & UserCommand::Buttons::Forward)
      position += camera.m_front * speed * dt;
    if (m_userCommand.buttons & UserCommand::Buttons::Backward)
      position -= camera.m_front * speed * dt;
    if (m_userCommand.buttons & UserCommand::Buttons::Left)
      position -= camera.m_right * speed * dt;
    if (m_userCommand.buttons & UserCommand::Buttons::Right)
      position += camera.m_right * speed * dt;
    if (m_userCommand.buttons & UserCommand::Buttons::Jump)
      position += camera.m_worldUp * speed * dt;
    if (m_userCommand.buttons & UserCommand::Buttons::Duck)
      position -= camera.m_worldUp * speed * dt;

    physics.applyLinearImpulse(position);

    position          = transform.matrix.getTranslation();
    camera.m_position = position;

    // m_userCommand.buttons = 0;

    auto viewMatrix{camera.viewMatrix()};
    auto projectionMatrix{camera.projectionMatrix(windowWidth / windowHeight)};
    
    float viewMatrixRaw[4 * 4]{
    viewMatrix[0][0],
    viewMatrix[0][1],
    viewMatrix[0][2],
    viewMatrix[0][3],
    viewMatrix[1][0],
    viewMatrix[1][1],
    viewMatrix[1][2],
    viewMatrix[1][3],
    viewMatrix[2][0],
    viewMatrix[2][1],
    viewMatrix[2][2],
    viewMatrix[2][3],
    viewMatrix[3][0],
    viewMatrix[3][1],
    viewMatrix[3][2],
    viewMatrix[3][3],
    };

    float projectionMatrixRaw[4 * 4]{
    projectionMatrix[0][0],
    projectionMatrix[0][1],
    projectionMatrix[0][2],
    projectionMatrix[0][3],
    projectionMatrix[1][0],
    projectionMatrix[1][1],
    projectionMatrix[1][2],
    projectionMatrix[1][3],
    projectionMatrix[2][0],
    projectionMatrix[2][1],
    projectionMatrix[2][2],
    projectionMatrix[2][3],
    projectionMatrix[3][0],
    projectionMatrix[3][1],
    projectionMatrix[3][2],
    projectionMatrix[3][3],
    };

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glLoadMatrixf(projectionMatrixRaw);
    // gluPerspective(45.0f, aspect, 0.1f, 100.0f);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glLoadMatrixf(viewMatrixRaw); */
  }

  //FIX : 
  //admin.cacheSystems();
  //admin.updateSystems(dt, 0);

  Ray          ray(ml::vec3(-20.0f, 106.0f, -1.0f), ml::vec3(1.0f, 0.0f, 0.0f));
    RayCollision rayCollision;

    /* FIX : if (admin.getSystem<Systems::Physics>().RayIntersection(ray, rayCollision)) {
      std::cout << rayCollision.node << std::endl;
      auto &colored{admin.getComponent<Colored>(rayCollision.node)};
      colored.color = ml::vec3{1.0f, 1.0f, 0.0f};
    }*/

  glBegin(GL_LINES);
  glColor3f(1.0f, 1.0f, 0.0f);
  glVertex3f(-20.0f, 106.0f, -1.0f);
  glVertex3f(-20.0f + 1.0f * 100.0f, 106.0f, -1.0f);
  glEnd();
  // FIX
  /* for (auto &&[entity, transform, physics] : admin.getEntitiesWithComponents<Components::Transform, Components::Physics>()) {
    ml::vec3 color{1.0f, 0.0f, 0.0f};
    if (admin.hasComponent<Colored>(entity)) {
      auto &colored{admin.getComponent<Colored>(entity)};
      color = colored.color;

      if (!colored.draw)
        continue;
    }

    auto &shape{physics.m_shape};
    switch (shape->m_shapeType) {
      case ShapeType::AABB:
        use(AABB & aabb{reinterpret_cast<AABB &>(*shape.get())}) {
          drawAABB(aabb.getPoints(transform.matrix), color.x, color.y, color.z);
        }
        break;
      case ShapeType::SPHERE:
        use(Sphere & sphere{reinterpret_cast<Sphere &>(*shape.get())}) {
          glPushMatrix();
          glTranslatef(sphere.getPoints(transform.matrix).x, sphere.getPoints(transform.matrix).y, sphere.getPoints(transform.matrix).z);  // Move right and into the screen
          glColor3f(color.x, color.y, color.z);
          GLUquadric *quadric = gluNewQuadric();
          gluQuadricTexture(quadric, true);
          gluSphere(quadric, sphere.getRadius(), 20, 20);
          glPopMatrix();
        }
        break;
      case ShapeType::CAPSULE:
        use(Capsule & capsule{reinterpret_cast<Capsule &>(*shape.get())}) {
          auto points = capsule.getPoints(transform.matrix);
          drawCapsule(glm::vec3(points[0].x, points[0].y, points[0].z), glm::vec3(points[1].x, points[1].y, points[1].z), capsule.getRadius(), glm::vec3(color.x, color.y, color.z));
        }
        break;
      case ShapeType::OBB:
        use(OBB & obb{reinterpret_cast<OBB &>(*shape.get())}) {
          drawAABB(obb.getPoints(transform.matrix), color.x, color.y, color.z);
        }
        break;
    }
  } */

  glutSwapBuffers();  // Swap the front and back frame buffers (double buffering)
  glutPostRedisplay();
}

void callbackCollision(int i, int j) {
  std::cout << "Collision " << i << " | " << j << std::endl;
}

/* Main function: GLUT runs as a console application starting at main() */
int main(int argc, char **argv) {
  /*   use (auto entity{admin.createEntity()}) {
      auto &translate{admin.createComponent<Components::Transform>(entity)};
      translate.matrix.setTranslation(ml::vec3{-3.0f, 6.0f, 0.0f});

      auto &physics{admin.createComponent<Components::Physics>(entity, std::make_unique<AABB>(ml::vec3(-1.0f, -1.0f, -6.0f), ml::vec3(1.0f, 1.0f, -4.0f)))};
      physics.applyLinearImpulse(ml::vec3{0.0f, -15.0f, 0.0f});

      auto &colored{admin.createComponent<Colored>(entity)};
      colored.color = ml::vec3{0.25f, 0.5f, 1.0f};
    }
  */

  //admin.createSystem<Systems::Physics>();
  //admin.createSystem<Systems::Gravity>();

//  admin.getSystem<Systems::Physics>().setCallbackCollision(callbackCollision);

  ///////////////////////////////////////
  // Objects
  ///////////////////////////////////////
  /* FIX : use(auto entity{admin.createEntity()}) {
    // Physics
    auto &physics{admin.createComponent<Components::Physics>(entity, std::make_unique<AABB>(ml::vec3{-50.0f, 0.0f, -50.0f}, ml::vec3{50.0f, 100.0f, 50.0f}))};
    physics.setIsRigid(true);

    // Transform
    auto &transform{admin.createComponent<Components::Transform>(entity)};

    auto &colored{admin.createComponent<Colored>(entity)};
    colored.color = ml::vec3{0.25f, 0.5f, 1.0f};
  }

  use(auto entity{admin.createEntity()}) {
    // Physics
    auto &physics{admin.createComponent<Components::Physics>(entity, std::make_unique<AABB>(ml::vec3{-1.0f, -1.0f, -1.0f}, ml::vec3{1.0f, 1.0f, 1.0f}))};

    // Transform
    auto &transform{admin.createComponent<Components::Transform>(entity)};
    transform.matrix.setTranslation(ml::vec3{0.0f, 101.0f, 0.0f});
    admin.createComponent<Components::Gravity>(entity);
    auto &colored{admin.createComponent<Colored>(entity)};
    auto &gravity{admin.createComponent<Components::Gravity>(entity)};
    colored.color = ml::vec3{1.0f, 0.5f, 0.0f};
    physics.applyLinearImpulse(ml::vec3{0.0f, 0.0f, 0.0f});
  }

  use(auto entity{admin.createEntity()}) {
    // Physics
    auto &physics{admin.createComponent<Components::Physics>(entity, std::make_unique<Capsule>(ml::vec3{0.0, 2.0f, 0.0f}, ml::vec3{0.0, -1.0f, 0.0f}, 1.0f))};

    // Transform
    auto &transform{admin.createComponent<Components::Transform>(entity)};
    transform.matrix.setTranslation(ml::vec3{0.0f, 110.0f, 0.0f});

    auto &colored{admin.createComponent<Colored>(entity)};
    auto &gravity{admin.createComponent<Components::Gravity>(entity)};
    colored.color = ml::vec3{1.0f, 0.0f, 0.0f};
    physics.applyLinearImpulse(ml::vec3{0.0f, -10.0f, 0.0f});
  }

  ///////////////////////////////////////
  // Camera
  ///////////////////////////////////////
  use(auto entity{admin.createEntity()}) {
    // Transform
    auto &transform{admin.createComponent<Components::Transform>(entity)};
    // transform.matrix.setTranslation(ml::vec3{1.72966f, 4.71852f, 4.25082f});
    transform.matrix.setTranslation(ml::vec3{1.72966f, 120.71852f, 4.25082f});

    // Camera
    auto &camera{admin.createComponent<Components::Camera>(entity)};
    camera.angles = ml::vec3{-44.847f, 270.009f, 0.0f};

    // Physics
    // auto &physics{admin.createComponent<Components::Physics>(entity, std::make_unique<Capsule>(ml::vec3{0.0, 1.0f, 0.0f}, ml::vec3{0.0f, -1.0f, 0.0f}, 2.0f))};
    auto &physics{admin.createComponent<Components::Physics>(entity, std::make_unique<AABB>(ml::vec3{-1.0, -1.0f, -1.0f}, ml::vec3{1.0f, 1.0f, 1.0f}))};

    m_camera = entity;

    auto &colored{admin.createComponent<Colored>(entity)};
    colored.draw = false;
  }*/

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  glutInit(&argc, argv);             // Initialize GLUT
  glutInitDisplayMode(GLUT_DOUBLE);  // Enable double buffered mode
  glutInitWindowSize(1600, 900);     // Set the window's initial width & height
  glutInitWindowPosition(50, 50);    // Position the window's initial top-left corner
  glutCreateWindow(title);           // Create window with the given title
  glutDisplayFunc(display);          // Register callback handler for window re-paint event
  glutReshapeFunc(reshape);          // Register callback handler for window re-size event
  glutIdleFunc(display);

  glutSpecialFunc(processSpecialKeys);
  glutSpecialUpFunc(processSpecialUpKeys);

  glutMouseFunc(mouseButton);
  glutPassiveMotionFunc(mouseMove);
  glutMotionFunc(mouseMove);

  // glutTimerFunc(0, timer, 0);     // First timer call immediately [NEW]
  initGL();        // Our own OpenGL initialization
  glutMainLoop();  // Enter the infinite event-processing loop
  return 0;
}
