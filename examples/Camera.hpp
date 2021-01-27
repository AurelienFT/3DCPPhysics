#pragma once

#include "Maths/Vectors.hpp"
#include "Maths/Matrix.hpp"

class Camera {
public:
  enum CameraType {
    Orthographic,
    Perspective,
  };

  Camera();
  Camera(float _pitch, float _yaw, const Vector3f &_position);
  ~Camera() = default;

  Matrix4<float>                BuildViewMatrix();
  [[nodiscard]] float           getNearPlane() const;
  void                          setNearPlane(float nearPlane);
  [[nodiscard]] float           getFarPlane() const;
  void                          setFarPlane(float farPlane);
  [[nodiscard]] float           getLeft() const;
  void                          setLeft(float left);
  [[nodiscard]] float           getRight() const;
  void                          setRight(float right);
  [[nodiscard]] float           getTop() const;
  void                          setTop(float top);
  [[nodiscard]] float           getBottom() const;
  void                          setBottom(float bottom);
  [[nodiscard]] float           getFov() const;
  void                          setFov(float fov);
  [[nodiscard]] float           getYaw() const;
  void                          setYaw(float yaw);
  [[nodiscard]] float           getPitch() const;
  void                          setPitch(float pitch);
  [[nodiscard]] const Vector3f &getPosition() const;
  void                          setPosition(const Vector3f &position);
  [[nodiscard]] Matrix4<float>  BuildProjectionMatrix(float currentAspect) const;

protected:
  float nearPlane;
  float farPlane;

  float left;
  float right;
  float top;
  float bottom;

  float    fov;
  float    yaw;
  float    pitch;
  Vector3f position;
};
