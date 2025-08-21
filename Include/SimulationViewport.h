#pragma once

#include <QOpenGLWindow>
#include <mujoco/mujoco.h>
#include <QTimer>
#include <QWheelEvent>
#include <qpoint.h>
#include <GL/glu.h>
#include <Eigen/Dense>
#include <array>
#include "MujocoContext.h"
#include "iostream"
#include "Constants.h"

enum DragType
{
    dragNormal, /**< drag for moving objects along the axis of the scene or for rotating the camera */
    dragNormalObject, /**< drag for moving object along their axis */
    dragRotate, /**< drag for rotating objects relative to their orientation */
    dragRotateWorld, /**< drag for rotating objects around the axis of the scene */
};

enum DragAndDropPlane
{
    xyPlane,
    xzPlane,
    yzPlane
};

using Vector2f = Eigen::Vector2f;
using Vector3f = Eigen::Vector3f;
using Vector3d = Eigen::Vector3d;
using Vector4d = Eigen::Vector4d;
using Matrix3f = Eigen::Matrix3f;
using Matrix4d = Eigen::Matrix4d;

namespace spqr {
class SimulationViewport : public QOpenGLWindow {
public:
    SimulationViewport(MujocoContext& mujContext);

protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;

    void wheelEvent(QWheelEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void dragBodyWithMouse(QMouseEvent* event);
    int getClickedBodyIdFromMouse(QMouseEvent* event);

    // DragHandler stuffs
    void computePerspective(double fovY, float aspect, float near, float far, double matrix[]);
    void resize(double fovY, unsigned int width, unsigned int height);
    Vector3f projectClick(int x, int y) const;
    Vector2f trigonometricDelta(QMouseEvent* event, QPointF deltaMouse);
    Vector2f unprojectDelta(QMouseEvent* event);
    Vector3f projectClickOnPlane(int x, int y, const Vector3f& planePoint, const Vector3f& planeNormal) const;
    QPointF projectWorldToScreen(const Vector3f& worldPos) const;
    QPointF worldToScreenTopView(const Vector3f& p) const;
    QPointF convertMetersToPixel(const Vector3f& p) const;
    //QPointF projectDeltaOnPlane(const Vector3f& delta3D, const Vector3f& planeNormal);
    QPointF projectDeltaOnPlane(const Vector3f& prevPos3D, const Vector3f& currPos3D, const Vector3f& planeNormal);
    Vector3f getCameraInWorld() const;
    Matrix3f getCameraRotation() const;
    Vector3f getBodyTranslation(int body_id) const;
    Matrix3f getBodyRotationMatrix(int body_id);
    bool intersectRayAndPlane(const Vector3f& point, const Vector3f& v,  const Vector3f& plane, const Vector3f& n, Vector3f& intersection) const;
    void setDragPlane(DragAndDropPlane plane);
    void calcDragPlaneVector();
    Vector3f computeDelta(QMouseEvent* event);


private:
    QPointF lastMousePosition;
    mjtMouse mouseAction = mjMOUSE_NONE;
    bool leftButtonDown = false;
    bool rightButtonDown = false;
    bool objectSelected = true;

    mjModel* model;
    mjData* data;
    mjvCamera* cam;
    mjvOption* opt;
    mjvScene* scene;
    mjrContext context;
    mjvPerturb* perturb;
    QTimer* timer;

    int width = initialWindowWidth, height = initialWindowHeight;

    // DragHandler stuffs
    GLdouble cameraTransformation[16];
    GLdouble projection[16];
    double fovY;
    int viewport[4];

    DragAndDropPlane dragPlane = xyPlane;
    Vector3f dragPlaneVector;
    Vector3f dragStartPos;
    bool dragging = false;
    DragType dragType;
    int dragSelection = 0;
    unsigned int dragStartTime;

};

}
