#include "SimulationViewport.h"
#include <memory>
#include <mujoco/mjvisualize.h>
#include <qpoint.h>

namespace spqr {

SimulationViewport::SimulationViewport(MujocoContext& mujContext)
    : model(mujContext.model), data(mujContext.data), cam(&mujContext.cam), opt(&mujContext.opt), scene(&mujContext.scene) {

    perturb = new mjvPerturb(); 

    timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, QOverload<>::of(&SimulationViewport::update));
    timer->start(16);

    // --- Salva i parametri della camera in variabili ---
    int   cameraId     = model->vis.global.cameraid;
    int   orthographic = model->vis.global.orthographic;
    float fovY         = model->vis.global.fovy;
    float ipd          = model->vis.global.ipd;
    float azimuth      = model->vis.global.azimuth;
    float elevation    = model->vis.global.elevation;
    float realtime     = model->vis.global.realtime;
    int   offWidth     = model->vis.global.offwidth;
    int   offHeight    = model->vis.global.offheight;

    // --- Stampa i valori ---
    std::cout << "Camera ID: "        << cameraId     << std::endl;
    std::cout << "Orthographic: "     << orthographic << std::endl;
    std::cout << "FOV Y: "            << fovY         << std::endl;
    std::cout << "IPD: "              << ipd          << std::endl;
    std::cout << "Azimuth: "          << azimuth      << std::endl;
    std::cout << "Elevation: "        << elevation    << std::endl;
    std::cout << "Real-time factor: " << realtime     << std::endl;
    std::cout << "Offscreen buffer: " << offWidth 
              << " x " << offHeight   << std::endl;

    fovY = model->vis.global.fovy;
}

void SimulationViewport::initializeGL() {
    mjr_defaultContext(&context);
    mjr_makeContext(model, &context, mjFONTSCALE_100);
}

void SimulationViewport::resizeGL(int w, int h) {
    width = w;
    height = h;

    glViewport(0, 0, width, height);
    viewport[0] = 0;
    viewport[1] = 0;
    viewport[2] = width;
    viewport[3] = height; 

    glMatrixMode(GL_PROJECTION);
    glLoadMatrixd(projection);

    glMatrixMode(GL_MODELVIEW);
    glLoadMatrixd(cameraTransformation);
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

Vector2f SimulationViewport::trigonometricDelta(QMouseEvent* event, QPointF deltaMouse) {

    Vector3f cameraPos = getCameraInWorld();
    Vector2f cameraDimension = Vector2f(viewport[2], viewport[3]);
    float elRad = cam->elevation * mjPI / 180.0;

    Vector2f lastPosInCamera = Vector2f(lastMousePosition.x(), lastMousePosition.y()) - Vector2f(cameraDimension[0]/2, cameraDimension[1]/2);
    Vector2f currentPosInCamera = Vector2f(event->position().x(), event->position().y()) - Vector2f(cameraDimension[0]/2, cameraDimension[1]/2);

    float lastPosHeightInWorld = cameraPos[2] + cos(elRad) * lastPosInCamera.norm();
    float currentPosHeightInWorld = cameraPos[2] + cos(elRad) * currentPosInCamera.norm();

    float lastDistanceFromXYplane = lastPosHeightInWorld / cos( (mjPI/2) - elRad );
    float currentDistanceFromXYplane = currentPosHeightInWorld / cos( (mjPI/2) - elRad );

    float deltaNormXYSquared = pow(lastDistanceFromXYplane,2) + pow(currentDistanceFromXYplane,2) - 2*lastDistanceFromXYplane*currentDistanceFromXYplane*cos( (mjPI/2) - elRad );
    float deltaNormXY = std::sqrt(deltaNormXYSquared);

    Vector2f deltaMouseVector = Vector2f(deltaMouse.x(), deltaMouse.y());
    Vector2f computedDelta = deltaNormXY * deltaMouseVector/std::min(viewport[2], viewport[3]);

    std::cout << "DEBUG: deltaNorm: " << deltaNormXY << ", computed delta: " << computedDelta.transpose() << std::endl;

    return computedDelta;
}

Vector2f SimulationViewport::unprojectDelta(QMouseEvent* event) {
    // Matrici OpenGL (row-major in Eigen, ma OpenGL è column-major -> attenzione al load)
    Matrix4d proj, model;
    for (int i = 0; i < 16; ++i) {
        proj(i % 4, i / 4) = projection[i];             // col-major → row-major
        model(i % 4, i / 4) = cameraTransformation[i];  // col-major → row-major
    }

    Matrix4d invPV = (proj * model).inverse();

    auto unproject = [&](QPointF p, double winZ) -> Vector3d {
        double ndcX =  (2.0 * p.x()) / viewport[2] - 1.0;
        double ndcY =  1.0 - (2.0 * p.y()) / viewport[3];
        double ndcZ =  2.0 * winZ - 1.0; // 0 near, 1 far

        Vector4d clip(ndcX, ndcY, ndcZ, 1.0);
        Vector4d world = invPV * clip;
        world /= world.w();
        return world.head<3>();
    };

    auto intersectWithZPlane = [&](Vector3d p0, Vector3d p1, double zPlane) -> Vector3d {
        Vector3d dir = p1 - p0;
        double t = (zPlane - p0.z()) / dir.z();
        return p0 + t * dir;
    };

    // Unproject last mouse pos
    Vector3d nearLast = unproject(lastMousePosition, 0.0);
    Vector3d farLast  = unproject(lastMousePosition, 1.0);
    Vector3d P_last   = intersectWithZPlane(nearLast, farLast, 0.0); // piano z=0

    // Unproject current mouse pos
    Vector3d nearCur = unproject(event->position(), 0.0);
    Vector3d farCur  = unproject(event->position(), 1.0);
    Vector3d P_cur   = intersectWithZPlane(nearCur, farCur, 0.0);

    // Delta in XY
    return (P_cur - P_last).head<2>().cast<float>();
}

QPointF SimulationViewport::worldToScreenTopView(const Vector3f& p) const {
    
    Vector3f cameraPos = getCameraInWorld();

    float az = cam->azimuth * M_PI / 180.0f;  
    float el = cam->elevation * M_PI / 180.0f;

    float x = cos(el) * cos(az);
    float y = cos(el) * sin(az);
    float z = sin(el);

    Vector3f viewDir(x, y, z);
    viewDir.normalize();

    float d = cam->distance; // std::abs((cameraPos - Vector3f(0, 0, 0)).dot(viewDir)); // distanza piano XY = z=0
    
    float fovy_rad = fovY * M_PI / 180.0f; // converti in radianti
    float some_height =0.0;// 2.0f * d * std::tan(fovy_rad / 2.0f);

    float sx = viewport[0] + p.x() * devicePixelRatio();
    float sy = viewport[1] + (some_height - p.y()) * devicePixelRatio(); // inverte y se necessario

    return QPointF(sx, sy);
}

void SimulationViewport::dragBodyWithMouse(QMouseEvent* event) {
    std::cout << "------------------------------\n";

    float az = cam->azimuth * M_PI / 180.0f;
    float el = cam->elevation * M_PI / 180.0f;

    //Vector3f planePoint = getBodyTranslation(dragSelection);
    //Vector3f planeNormal = Vector3f(cos(el)*cos(az), cos(el)*sin(az), sin(el));

    //Vector3f prevPos3D = projectClickOnPlane(lastMousePosition.x() * devicePixelRatio(), lastMousePosition.y() * devicePixelRatio(), planePoint, planeNormal);
    //Vector3f currPos3D = projectClickOnPlane(event->position().x() * devicePixelRatio(), event->position().y() * devicePixelRatio(), planePoint, planeNormal);

    //Vector3f delta3D = currPos3D - prevPos3D;
    //float dist = delta3D.dot(planeNormal);
    //Vector3f deltaOnPlane = delta3D - dist * planeNormal;

    //QPointF screenPrev = convertMetersToPixel(prevPos3D);
    //QPointF screenCurr = convertMetersToPixel(currPos3D);
    //QPointF deltaMouse = screenCurr - screenPrev;

    //Eigen::Vector2f delta3D2D = (currPos3D.head<2>() - prevPos3D.head<2>()) *500;
    //QPointF deltaMouse(delta3D2D.x(), -delta3D2D.y());

    //std::cout << "DEBUG: deltaMouse: " << deltaMouse.x() << ", " << deltaMouse.y() << std::endl;
    //std::cout << "DEBUG: " << (event->position().x() - lastMousePosition.x()) * devicePixelRatioF() << ", " << (event->position().y() - lastMousePosition.y()) * devicePixelRatioF() << std::endl;
    QPointF deltaMouse = (event->position() - lastMousePosition) * devicePixelRatioF();
    float minViewport = std::min(viewport[2], viewport[3]);
    mjtNum normDX = deltaMouse.x() / minViewport;
    mjtNum normDY = deltaMouse.y() / minViewport;

    //mjtNum mouseAngle = std::tan(deltaMouse) 
    mjtNum elRad = cam->elevation * mjPI / 180.0;

    mjtNum adjustedNormDz = normDX;
    mjtNum adjustedNormDy = normDY;

    mjtMouse perturbationType = event->modifiers() == Qt::ShiftModifier ? mjMOUSE_ROTATE_V : mjMOUSE_MOVE_H;
    mjv_movePerturb(model, data, perturbationType, adjustedNormDz , adjustedNormDy, scene, perturb);
    mjv_applyPerturbPose(model, data, perturb, 1);
}

void SimulationViewport::mousePressEvent(QMouseEvent* event) {

    std::cout << "------------------------------\n";
    std::cout << "Camera ID: "      << model->vis.global.cameraid   << std::endl;
    std::cout << "Orthographic: "   << model->vis.global.orthographic << std::endl;
    std::cout << "FOV Y: "          << model->vis.global.fovy       << std::endl;
    std::cout << "IPD: "            << model->vis.global.ipd        << std::endl;
    std::cout << "Azimuth: "        << model->vis.global.azimuth    << std::endl;
    std::cout << "Elevation: "      << model->vis.global.elevation  << std::endl;
    std::cout << "Real-time: "      << model->vis.global.realtime   << std::endl;

    lastMousePosition = event->position();

    //std::cout << "DEBUG: Mouse position: " << lastMousePosition.x() << ", " << lastMousePosition.y() << std::endl;

    Vector3f planePoint = (dragSelection > 0) ? getBodyTranslation(dragSelection) : Vector3f(0.f, 0.f, 0.f);
    Vector3f planeNormal(0.f, 0.f, 1.f);

    //std::cout << "DEBUG: Mouse position projected on plane: " << projectClickOnPlane(event->position().x() * devicePixelRatio(), event->position().y() * devicePixelRatio(), planePoint, planeNormal).transpose() << std::endl;

    dragSelection = getClickedBodyIdFromMouse(event);
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

QPointF SimulationViewport::convertMetersToPixel(const Vector3f& p_world) const {

    int camId = model->vis.global.cameraid;
    Vector3f camPos = getCameraInWorld();      // posizione camera corrente
    Matrix3f R_world = getCameraRotation();    // rotazione camera corrente

    Matrix3f T;
    T << 0, 0, 1,
        1, 0, 0,
        0,-1, 0;

    Matrix3f R_cam = T * R_world.transpose();
    Vector3f p_cam = R_cam.transpose() * (p_world - camPos);

    // --- Parametri intrinseci ---
    int cam_width  = viewport[2];
    int cam_height = viewport[3];
    double fovy_rad = model->cam_fovy[camId] * M_PI / 180.0; // da gradi a radianti

    std::cout << "Type of cam_resolution in model: " << typeid(model->cam_resolution).name() << std::endl;
    std::cout << "DEBUG: Camera resolution: height = " << cam_height << ", fovy: " << model->cam_fovy[camId] << std::endl;

    double fy = (cam_height * 0.5) / tan(fovy_rad * 0.5);
    double fx = fy;  // pixel quadrati
    double cx = cam_width  * 0.5;
    double cy = cam_height * 0.5;

    // --- Proiezione prospettica ---
    std::cout << p_cam.transpose() << std::endl;
    if (p_cam.z() <= 0) return QPointF(-1e6, -1e6); // punto dietro la camera

    double x_pixel = (p_cam.x() / p_cam.z()) * fx + cx;
    double y_pixel = -(p_cam.y() / p_cam.z()) * fy + cy;

    std::cout << "DEBUG: p_cam: " << p_cam.transpose()
              << ", x_pixel: " << x_pixel
              << ", y_pixel: " << y_pixel << std::endl;

    return QPointF(x_pixel, y_pixel);
}


/*
QPointF SimulationViewport::projectDeltaOnPlane(const Vector3f& delta3D, const Vector3f& planeNormal) {
    Vector3f n = planeNormal.normalized();
    Vector3f arbitrary = (fabs(n.z()) < 0.9f) ? Vector3f(0,0,1) : Vector3f(1,0,0);
    Vector3f u = n.cross(arbitrary).normalized();
    Vector3f v = n.cross(u);  // inverti segno se serve

    float deltaU = delta3D.dot(u);
    float deltaV = delta3D.dot(v);
    return QPointF(deltaU, deltaV);
}*/

QPointF SimulationViewport::projectDeltaOnPlane(const Vector3f& prevPos3D, const Vector3f& currPos3D, const Vector3f& planeNormal)
{
    Vector3f delta3D = currPos3D - prevPos3D;
    float dist = delta3D.dot(planeNormal);
    Vector3f deltaOnPlane = delta3D - dist * planeNormal;

    // Calcolo assi ortogonali sul piano
    Vector3f arbitrary = (fabs(planeNormal.z()) < 0.9f) ? Vector3f(0,0,1) : Vector3f(1,0,0);
    Vector3f u = planeNormal.cross(arbitrary).normalized();
    Vector3f v = planeNormal.cross(u);

    // Proiezione delta sui due assi del piano
    float deltaU = deltaOnPlane.dot(u);
    float deltaV = deltaOnPlane.dot(v);

    // Proiezione punti per calcolare scala pixel/unità world
    QPointF screenBase = projectWorldToScreen(prevPos3D);
    QPointF screenU = projectWorldToScreen(prevPos3D + u);
    QPointF screenV = projectWorldToScreen(prevPos3D + v);

    float scaleU = QLineF(screenBase, screenU).length();
    float scaleV = QLineF(screenBase, screenV).length();

    return QPointF(deltaV * scaleV, deltaU * scaleU);
}

QPointF SimulationViewport::projectWorldToScreen(const Vector3f& worldPos) const {
    GLdouble mvMatrix[16], projMatrix[16];
    for (int i = 0; i < 16; ++i) {
        mvMatrix[i] = static_cast<GLdouble>(cameraTransformation[i]);
        projMatrix[i] = static_cast<GLdouble>(projection[i]);
    }
    GLdouble sx, sy, sz;
    gluProject(worldPos.x(), worldPos.y(), worldPos.z(), mvMatrix, projMatrix, viewport, &sx, &sy, &sz);
    return QPointF(sx, viewport[3] - sy);
}

Vector3f SimulationViewport::projectClickOnPlane(int x, int y, const Vector3f& planePoint, const Vector3f& planeNormal) const {
    GLdouble mvMatrix[16];
    GLdouble projMatrix[16];
    for (int i = 0; i < 16; ++i) {
        mvMatrix[i] = static_cast<GLdouble>(cameraTransformation[i]);
        projMatrix[i] = static_cast<GLdouble>(projection[i]);
    }

    GLdouble nearX, nearY, nearZ;
    GLdouble farX, farY, farZ;

    gluUnProject(static_cast<GLdouble>(x), static_cast<GLdouble>(height - y), 0.0, mvMatrix, projMatrix, viewport, &nearX, &nearY, &nearZ);
    gluUnProject(static_cast<GLdouble>(x), static_cast<GLdouble>(height - y), 1.0, mvMatrix, projMatrix, viewport, &farX, &farY, &farZ);

    Vector3f rayOrigin(static_cast<float>(nearX), static_cast<float>(nearY), static_cast<float>(nearZ));
    Vector3f rayDir(static_cast<float>(farX) - static_cast<float>(nearX),
                    static_cast<float>(farY) - static_cast<float>(nearY),
                    static_cast<float>(farZ) - static_cast<float>(nearZ));
    rayDir.normalize();

    Vector3f intersection;
    if (intersectRayAndPlane(rayOrigin, rayDir, planePoint, planeNormal, intersection)) {
        return intersection;
    } else {
        // Se non c'è intersezione ritorna qualcosa di significativo, es. il punto origine
        return rayOrigin;
    }
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

Matrix3f SimulationViewport::getCameraRotation() const {
    float azimuth_rad   = cam->azimuth   * M_PI / 180.0f;
    float elevation_rad = cam->elevation * M_PI / 180.0f;

    Matrix3f Rz;  
    Rz << cos(azimuth_rad), -sin(azimuth_rad), 0,
        sin(azimuth_rad),  cos(azimuth_rad), 0,
        0,                 0,                1;

    Matrix3f Rx; 
    Rx <<   cos(elevation_rad), 0, sin(elevation_rad),
            0, 1, 0,
            -sin(elevation_rad), 0,  cos(elevation_rad);

    Matrix3f R = Rz * Rx;

    return R;
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
