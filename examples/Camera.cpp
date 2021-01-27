
#include "Camera.hpp"
Matrix4<float> Camera::BuildViewMatrix() {
  return Matrix4<float>::rotation(-pitch, Vector3<float>(1, 0, 0)) * Matrix4<float>::rotation(-yaw, Vector3<float>(0, 1, 0)) * Matrix4<float>::translate(position * -1);
}
Camera::Camera() : yaw(0.0f), pitch(0.0f), position(0, 0, 0) {}
Camera::Camera(float _pitch, float _yaw, const Vector3f &_position) : pitch(_pitch), yaw(_yaw), position(_position) {}

float Camera::getNearPlane() const {
  return nearPlane;
}
void Camera::setNearPlane(float nearPlane) {
  Camera::nearPlane = nearPlane;
}
float Camera::getFarPlane() const {
  return farPlane;
}
void Camera::setFarPlane(float farPlane) {
  Camera::farPlane = farPlane;
}
float Camera::getLeft() const {
  return left;
}
void Camera::setLeft(float left) {
  Camera::left = left;
}
float Camera::getRight() const {
  return right;
}
void Camera::setRight(float right) {
  Camera::right = right;
}
float Camera::getTop() const {
  return top;
}
void Camera::setTop(float top) {
  Camera::top = top;
}
float Camera::getBottom() const {
  return bottom;
}
void Camera::setBottom(float bottom) {
  Camera::bottom = bottom;
}
float Camera::getFov() const {
  return fov;
}
void Camera::setFov(float fov) {
  Camera::fov = fov;
}
float Camera::getYaw() const {
  return yaw;
}
void Camera::setYaw(float yaw) {
  Camera::yaw = yaw;
}
float Camera::getPitch() const {
  return pitch;
}
void Camera::setPitch(float pitch) {
  Camera::pitch = pitch;
}
const Vector3f &Camera::getPosition() const {
  return position;
}
void Camera::setPosition(const Vector3f &position) {
  Camera::position = position;
}
Matrix4<float> Camera::BuildProjectionMatrix(float currentAspect) const {
  return Matrix4<float>::perspective(fov, currentAspect, nearPlane, farPlane);
}
