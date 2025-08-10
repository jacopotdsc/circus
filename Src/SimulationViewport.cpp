#include "SimulationViewport.h"
#include <memory>
#include <mujoco/mjvisualize.h>
#include <qpoint.h>

namespace spqr {

SimulationViewport::SimulationViewport(MujocoContext& mujContext)
    : model(mujContext.model), data(mujContext.data), cam(&mujContext.cam), opt(&mujContext.opt), scene(&mujContext.scene) {

    perturb = new mjvPerturb(); 

    fovY = 40.f;//*(model->cam_fovy);

    timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, QOverload<>::of(&SimulationViewport::update));
    timer->start(16);
}

void SimulationViewport::initializeGL() {
    mjr_defaultContext(&context);
    mjr_makeContext(model, &context, mjFONTSCALE_100);
}

void SimulationViewport::resizeGL(int w, int h) {
    width = w;
    height = h;

    glViewport(0, 0, width, height);
    viewport[0] = viewport[1] = 0;
    viewport[2] = width;
    viewport[3] = height; 

    float projF[16];
    for (int i = 0; i < 16; ++i)
        projF[i] = static_cast<float>(projection[i]);

    glMatrixMode(GL_PROJECTION);
    computePerspective(fovY * (M_PI / 180.f), float(width) / float(height), 0.1f, 500.f, projection);
    glLoadMatrixf(projF);

    glMatrixMode(GL_MODELVIEW);
}

// Wrapper
void SimulationViewport::paintGL() {
    glViewport(0, 0, width, height);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    mjv_updateScene(model, data, opt, nullptr, cam, mjCAT_ALL, scene);
    mjrRect viewport = {0, 0, width, height};

    glGetDoublev(GL_MODELVIEW_MATRIX, cameraTransformation); 
    glGetDoublev(GL_PROJECTION_MATRIX, projection); 

    mjr_setBuffer(mjFB_WINDOW, &context);
    mjr_render(viewport, scene, &context);
}

void SimulationViewport::wheelEvent(QWheelEvent* event) {
    mjv_moveCamera(model, mjMOUSE_ZOOM, 0, 0.0005 * event->angleDelta().y(), scene, cam);
}

void SimulationViewport::dragBodyWithMouse(QMouseEvent* event) {
    //std::cout << "------------------------------\n";

    QPointF deltaMouse = (event->position() - lastMousePosition) * devicePixelRatioF();
    mjtNum normDX = deltaMouse.x() / viewport[2];
    mjtNum normDY = deltaMouse.y() / viewport[3];
    
    mjtNum elRad = cam->elevation * mjPI / 180.0;


    float sgnDx = normDX >= 0 ? 1.0f : -1.0f;
    float sgnDy = normDY >= 0 ? 1.0f : -1.0f;

    mjtNum adjustedNormDz = sgnDx * abs(normDX);
    mjtNum adjustedNormDy = sgnDy * abs(normDY/sin(elRad));

    mjtMouse perturbationType = event->modifiers() == Qt::ShiftModifier ? mjMOUSE_ROTATE_V : mjMOUSE_MOVE_H;
    mjv_movePerturb(model, data, perturbationType, adjustedNormDz , adjustedNormDy, scene, perturb);
    mjv_applyPerturbPose(model, data, perturb, 1);
}


void SimulationViewport::mousePressEvent(QMouseEvent* event) {
    lastMousePosition = event->position();

    dragSelection = getClickedBodyIdFromMouse(event);
    if ( dragSelection > 0 ){

        perturb->active = event->modifiers() == Qt::ShiftModifier ? mjPERT_ROTATE : mjPERT_TRANSLATE;  
        perturb->select = dragSelection;
        mjv_initPerturb(model, data, scene, perturb);
        mouseAction = mjMOUSE_SELECT;
    }
    else if (event->button() == Qt::LeftButton) {
        if (event->modifiers() & Qt::ShiftModifier) {
            mouseAction = mjMOUSE_ROTATE_V;
        } else {
            mouseAction = mjMOUSE_MOVE_H;
        }
    }
}

void SimulationViewport::mouseReleaseEvent(QMouseEvent* event) {
    Q_UNUSED(event);

    if (mouseAction == mjMOUSE_SELECT) {
        perturb->active = 0;
        perturb->select = -1;
    }

    mouseAction = mjMOUSE_NONE;
}

void SimulationViewport::mouseMoveEvent(QMouseEvent* event) {
    if (mouseAction == mjMOUSE_NONE)
        return;
    
    if(mouseAction == mjMOUSE_SELECT){
        dragBodyWithMouse(event);
    }
    else{
        const QPointF delta = (event->position() - lastMousePosition) * devicePixelRatioF();
        mjv_moveCamera(model, mouseAction, 0.003*delta.x(), 0.003*delta.y(), scene, cam);
    }
        

    lastMousePosition = event->position();
    update();
}

int SimulationViewport::getClickedBodyIdFromMouse(QMouseEvent* event) {
    mjtNum relx = event->position().x() / static_cast<mjtNum>(width);
    mjtNum rely = 1.0 - event->position().y() / static_cast<mjtNum>(height);
    mjtNum aspectratio = static_cast<mjtNum>(width) / static_cast<mjtNum>(height);

    mjtNum selpnt[3];
    int geomid[1] = {-1};
    int flexid[1] = {-1};
    int skinid[1] = {-1};

    if (!model || !data || !opt || !cam || !scene)
        return -2;

    int bodyId = mjv_select(model, data, opt, aspectratio, relx, rely, scene, selpnt, geomid, flexid, skinid);
          
    std::string bodyName = bodyId < 0? "" : std::string(mj_id2name(model, mjOBJ_BODY, bodyId));

    std::cout << "DEBUG: Body name \"" << bodyName << "\" , " << bodyId << " clicked." << std::endl;
    return bodyId;

}

void SimulationViewport::computePerspective(double fovY, float aspect, float near, float far, double matrix[]){
    matrix[5] = 1.f / std::tan(fovY * 0.5f);
    matrix[0] = matrix[5] / aspect;
    const float nearMFarInv = 1.f / (near - far);
    matrix[10] = (far + near) * nearMFarInv;
    matrix[11] = -1.f;
    matrix[14] = 2.f * far * near * nearMFarInv;
    matrix[1] = matrix[2] = matrix[3] = matrix[4] = matrix[6] = matrix[7] = matrix[8] = matrix[9] = matrix[12] = matrix[13] = matrix[15] = 0.f;
}

Vector3f SimulationViewport::projectClick(int x, int y) const{
  GLdouble mvMatrix[16];
  GLdouble projMatrix[16];
  for(int i = 0; i < 16; ++i){
    mvMatrix[i] = static_cast<GLdouble>(cameraTransformation[i]);
    projMatrix[i] = static_cast<GLdouble>(projection[i]);
  }

  GLdouble tx, ty, tz;
  gluUnProject(static_cast<GLdouble>(x), static_cast<GLdouble>(height - y), 1.0, mvMatrix, projMatrix, viewport, &tx, &ty, &tz);
  return Vector3f(static_cast<float>(tx), static_cast<float>(ty), static_cast<float>(tz));
}

Vector3f SimulationViewport::getCameraInWorld() const {
    float azimuth_rad = cam->azimuth * M_PI / 180.0f;
    float elevation_rad = cam->elevation * M_PI / 180.0f;
    float distance = cam->distance;

    return Vector3f(
        cam->lookat[0] + distance * cos(elevation_rad) * cos(azimuth_rad),
        cam->lookat[1] + distance * cos(elevation_rad) * sin(azimuth_rad),
        cam->lookat[2] + distance * sin(elevation_rad)
    );
}

Vector3f SimulationViewport::getBodyTranslation(int body_id) const {
    const mjtNum* posMuJoCo = data->xpos + 3 * body_id;
    return Vector3f(static_cast<float>(posMuJoCo[0]),
                           static_cast<float>(posMuJoCo[1]),
                           static_cast<float>(posMuJoCo[2]));
}

Matrix3f SimulationViewport::getBodyRotationMatrix(int body_id) {
    const mjtNum* rotMatMuJoCo = data->xmat + 9 * body_id;

    Matrix3f rotMatrix;
    rotMatrix << 
        rotMatMuJoCo[0], rotMatMuJoCo[3], rotMatMuJoCo[6],
        rotMatMuJoCo[1], rotMatMuJoCo[4], rotMatMuJoCo[7],
        rotMatMuJoCo[2], rotMatMuJoCo[5], rotMatMuJoCo[8];

    return rotMatrix;
}

bool SimulationViewport::intersectRayAndPlane(const Vector3f& point, const Vector3f& v,
                                             const Vector3f& plane, const Vector3f& n,
                                             Vector3f& intersection) const{
  Vector3f p = plane - point;
  float denominator = n.dot(v);
  if(denominator == 0.f)
    return false;
  float r = n.dot(p) / denominator;
  if(r < 0.f)
    return false;
  intersection = v;
  intersection *= r;
  intersection += point;
  return true;
}

void SimulationViewport::setDragPlane(DragAndDropPlane plane)
{
  dragPlane = plane;
  calcDragPlaneVector();
}

void SimulationViewport::calcDragPlaneVector(){
    dragPlaneVector = Vector3f(0.f, 0.f, 1.f);
    return;
    /*switch(dragPlane)
    {
        case xyPlane:
        dragPlaneVector = Vector3f(0.f, 0.f, 1.f);
        break;
        case xzPlane:
        dragPlaneVector = Vector3f(0.f, 1.f, 0.f);
        break;
        case yzPlane:
        dragPlaneVector = Vector3f(1.f, 0.f, 0.f);
        break;
    }*/
}

Vector3f SimulationViewport::computeDelta(QMouseEvent* event) {
    
    Vector3f bodyTranslation = getBodyTranslation(dragSelection);
    Matrix3f bodyRotation = getBodyRotationMatrix(dragSelection);
    Vector3f cameraPos = getCameraInWorld();
    calcDragPlaneVector();

    // Intersection for current mouse position
    Vector3f currentPos;
    Vector3f projectedClick = projectClick( event->position().x() * devicePixelRatio(), event->position().y() * devicePixelRatio());
    bool currBool = intersectRayAndPlane(cameraPos, (projectedClick - cameraPos).normalized(), bodyTranslation, dragPlaneVector, currentPos);

    //  Intersection for current mouse position
    Vector3f previousPos;
    Vector3f prevProjectedClick = projectClick(lastMousePosition.x() * devicePixelRatio(), lastMousePosition.y() * devicePixelRatio());
    bool prevBool = intersectRayAndPlane(cameraPos, (prevProjectedClick - cameraPos).normalized(), bodyTranslation, dragPlaneVector, previousPos);

    return (currBool && prevBool) ? (currentPos - previousPos).eval() : Vector3f::Zero();
}

}
