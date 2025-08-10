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

using Vector3f = Eigen::Vector3f;
using Matrix3f = Eigen::Matrix3f;

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
    Vector3f getCameraInWorld() const;
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
