#include "ModelDialog.h"
#include "ui_ModelDialog.h"

using namespace isnl;
#include "RBSharedMemory.h"
#include "RBMotorController.h"
#include "index_notation.h"
const double R2Df = 57.2957802f;
const double D2Rf = 0.0174533f;

extern pRBCORE_SHM sharedData;
extern RBMotorController    _DEV_MC[MAX_MC];
struct QuadParams
{
    double P2HX,P2HY,ULEG,LLEG,R2P;
    QuadParams()
    {
        P2HX = 0.203;
        P2HY = 0.08;
        ULEG = 0.20;
        LLEG = 0.20;
        R2P = 0.035-0.0015;
    }
};


#define R2Df 57.2958
enum JOINT_ID{
    J_RHR,J_RHP,J_RKN,
    J_LHR,J_LHP,J_LKN,
    J_RSR,J_RSP,J_REB,
    J_LSR,J_LSP,J_LEB,
    NJOINT
};


enum BONE_ID{
    B_WST,
    B_RHR,B_RHP,B_RKN,
    B_LHR,B_LHP,B_LKN,
    B_RSR,B_RSP,B_REB,
    B_LSR,B_LSP,B_LEB,

    NBONE
};
enum COMP_ID{
    JRHR,JRHP,JRKN,
    JLHR,JLHP,JLKN,
    JRSR,JRSP,JREB,
    JLSR,JLSP,JLEB,
    BWST,
    BRHR,BRHP,BRKN,
    BLHR,BLHP,BLKN,
    BRSR,BRSP,BREB,
    BLSR,BLSP,BLEB,

    NCOMP
};
static const char COMP_NAME[NCOMP+1][10] = {
    "JLSR","JLSP","JLEB",
    "JRSR","JRSP","JREB",
    "JLHR","JLHP","JLKN",
    "JRHR","JRHP","JRKN",
    "BWST",
    "BLSR","BLSP","BLEB",
    "BRSR","BRSP","BREB",
    "BLHR","BLHP","BLKN",
    "BRHR","BRHP","BRKN",
    "null"
};

inline float JointReference(int jnum){
    return _DEV_MC[jnum].MoveJoints.RefAngleCurrent;
}
inline float JointEncoder(int jnum){
    return _DEV_MC[jnum].CurrentPosition;
}

std::string STLPath = "../share/GUI/stl/";

ModelDialog::ModelDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::ModelDialog)
{
    ui->setupUi(this);

    glWidget = new GLWidget(this);

    QScrollArea *glWidgetArea = new QScrollArea;
    glWidgetArea->setWidget(glWidget);
    glWidgetArea->setWidgetResizable(true);
    glWidgetArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    glWidgetArea->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    glWidgetArea->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
    glWidgetArea->setMinimumSize(50, 50);

    ui->LAYOUT_MODEL->addWidget(glWidgetArea, 0, 0);

    displayTimer = new QTimer(this);
    connect(displayTimer, SIGNAL(timeout()), this, SLOT(DisplayUpdate()));
    displayTimer->start(30);
}

ModelDialog::~ModelDialog()
{
    delete ui;
}


void ModelDialog::DisplayUpdate(){
    if(ui->CB_USE_REFERENCE->isChecked())
    {
        glWidget->model->ref[FLR+7] = JointReference(FLR)*D2Rf;
        glWidget->model->ref[FLP+7] = JointReference(FLP)*D2Rf;
        glWidget->model->ref[FLK+7] = (JointReference(FLK) + JointReference(FLP)/11.0)*D2Rf;
        glWidget->model->ref[FRR+7] = JointReference(FRR)*D2Rf;
        glWidget->model->ref[FRP+7] = JointReference(FRP)*D2Rf;
        glWidget->model->ref[FRK+7] = (JointReference(FRK) + JointReference(FRP)/11.0)*D2Rf;
        glWidget->model->ref[HLR+7] = JointReference(HLR)*D2Rf;
        glWidget->model->ref[HLP+7] = JointReference(HLP)*D2Rf;
        glWidget->model->ref[HLK+7] = (JointReference(HLK) + JointReference(HLP)/11.0)*D2Rf;
        glWidget->model->ref[HRR+7] = JointReference(HRR)*D2Rf;
        glWidget->model->ref[HRP+7] = JointReference(HRP)*D2Rf;
        glWidget->model->ref[HRK+7] = (JointReference(HRK) + JointReference(HRP)/11.0)*D2Rf;
    }

    if(ui->CB_USE_ENCODER->isChecked()){
        glWidget->model_true->ref[FLR+7] = JointEncoder(FLR)*D2Rf;
        glWidget->model_true->ref[FLP+7] = JointEncoder(FLP)*D2Rf;
        glWidget->model_true->ref[FLK+7] = (JointEncoder(FLK) + JointEncoder(FLP)/11.0)*D2Rf;
        glWidget->model_true->ref[FRR+7] = JointEncoder(FRR)*D2Rf;
        glWidget->model_true->ref[FRP+7] = JointEncoder(FRP)*D2Rf;
        glWidget->model_true->ref[FRK+7] = (JointEncoder(FRK) + JointEncoder(FRP)/11.0)*D2Rf;
        glWidget->model_true->ref[HLR+7] = JointEncoder(HLR)*D2Rf;
        glWidget->model_true->ref[HLP+7] = JointEncoder(HLP)*D2Rf;
        glWidget->model_true->ref[HLK+7] = (JointEncoder(HLK) + JointEncoder(HLP)/11.0)*D2Rf;
        glWidget->model_true->ref[HRR+7] = JointEncoder(HRR)*D2Rf;
        glWidget->model_true->ref[HRP+7] = JointEncoder(HRP)*D2Rf;
        glWidget->model_true->ref[HRK+7] = (JointEncoder(HRK) + JointEncoder(HRP)/11.0)*D2Rf;
    }else{
        glWidget->model_true->ref[FLR+7] = JointReference(FLR)*D2Rf;
        glWidget->model_true->ref[FLP+7] = JointReference(FLP)*D2Rf;
        glWidget->model_true->ref[FLK+7] = (JointReference(FLK) + JointReference(FLP)/11.0)*D2Rf;
        glWidget->model_true->ref[FRR+7] = JointReference(FRR)*D2Rf;
        glWidget->model_true->ref[FRP+7] = JointReference(FRP)*D2Rf;
        glWidget->model_true->ref[FRK+7] = (JointReference(FRK) + JointReference(FRP)/11.0)*D2Rf;
        glWidget->model_true->ref[HLR+7] = JointReference(HLR)*D2Rf;
        glWidget->model_true->ref[HLP+7] = JointReference(HLP)*D2Rf;
        glWidget->model_true->ref[HLK+7] = (JointReference(HLK) + JointReference(HLP)/11.0)*D2Rf;
        glWidget->model_true->ref[HRR+7] = JointReference(HRR)*D2Rf;
        glWidget->model_true->ref[HRP+7] = JointReference(HRP)*D2Rf;
        glWidget->model_true->ref[HRK+7] = (JointReference(HRK) + JointReference(HRP)/11.0)*D2Rf;

    }

    glWidget->updateGL();
}





GLWidget::GLWidget(QWidget *parent)
    : QGLWidget(parent)
{
    xRot = 0;
    yRot = 0;
    zRot = -60*16;



    model = newModel();
    model->ref.resize(NJOINT+7);//NO_OF_JOINTS+7);
    model->ref[2] = 0.f;


    model_true = newModel_true();
    model_true->ref.resize(NJOINT+7);//NO_OF_JOINTS+7);
    model_true->ref[2] = 0.f;




}
GLWidget::~GLWidget()
{
    makeCurrent();
}

void GLWidget::setXRotation(int angle){
    normalizeAngle(&angle);
    if (angle != xRot) {
        xRot = angle;
        emit xRotationChanged(angle);
    }
}
void GLWidget::setYRotation(int angle){
    normalizeAngle(&angle);
    if (angle != yRot) {
        yRot = angle;
        emit yRotationChanged(angle);
    }
}
void GLWidget::setZRotation(int angle){
    normalizeAngle(&angle);
    if (angle != zRot) {
        zRot = angle;
        emit zRotationChanged(angle);
    }
}
void GLWidget::initializeGL()
{
    GLfloat lightColor[] = {1.0f, 1.0f, 1.0f, 1.0f};

    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_LIGHT1);
    glEnable(GL_LIGHT2);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, lightColor);
    glLightfv(GL_LIGHT1, GL_DIFFUSE, lightColor);
    glLightfv(GL_LIGHT2, GL_DIFFUSE, lightColor);
    glEnable(GL_DEPTH_TEST);

    globjs.initialize();

    glEnable(GL_NORMALIZE);
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
}
void GLWidget::paintGL()
{
    GLfloat lightPos0[4] = { 10.0f,-10.0f, 7.0f, 1.0f };
    GLfloat lightPos1[4] = {-10.0f, 10.0f, 7.0f, 1.0f };
    GLfloat lightPos2[4] = {-10.0f,-10.0f, 7.0f, 1.0f };

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


    glPushMatrix();

    glRotated(xRot / 16.0 -180, 1.0, 0.0, 0.0);
    glRotated(yRot / 16.0 -180, 0.0, 1.0, 0.0);
    glRotated(zRot / 16.0 -180, 0.0, 0.0, 1.0);




    model->render();



    model_true->render(); // encoder model



    glPushMatrix();

    glRotated(xRot / 16.0 -180, 1.0, 0.0, 0.0);
    glRotated(yRot / 16.0 -180, 0.0, 1.0, 0.0);
    glRotated(zRot / 16.0 -180, 0.0, 0.0, 1.0);

    glLightfv(GL_LIGHT0, GL_POSITION, lightPos0);
    glLightfv(GL_LIGHT1, GL_POSITION, lightPos1);
    glLightfv(GL_LIGHT2, GL_POSITION, lightPos2);


    globjs.render();

    glPopMatrix();

    glPopMatrix();

}

void GLWidget::resizeGL(int width, int height)
{
    int side = qMin(width, height);
    glViewport((width - side) / 2, (height - side) / 2, side, side);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glFrustum(-0.1, +0.1, -0.1, 0.1, 0.1, 60.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glTranslated(0.0, 0.0, -1.0);
    // change camera coord to world coord :
    glRotatef(2.0944*R2Df, -0.5774, -0.5774, -0.5774);
}
void GLWidget::mousePressEvent(QMouseEvent *event){
    lastPos = event->pos();
}
void GLWidget::mouseMoveEvent(QMouseEvent *event){
    currPos = event->pos();
    int dx = currPos.x() - lastPos.x();
    int dy = currPos.y() - lastPos.y();

    if(event->buttons() & Qt::RightButton){
        setYRotation(yRot - 3 * dy);
        setZRotation(zRot + 3 * dx);
    }else if (event->buttons() & Qt::LeftButton){
        glTranslatef(0, dx/500.0, -dy/500.0);
    }
    lastPos = event->pos();
}
void GLWidget::wheelEvent(QWheelEvent *event){
    glTranslatef(-event->delta()/1000.0, 0, 0);
}

void GLWidget::normalizeAngle(int *angle){
    while (*angle < 0)
        *angle += 360 * 16;
    while (*angle > 360 * 16)
        *angle -= 360 * 16;
}


// ===========================
// ===========================


HUBOModel* GLWidget::newModel(){

    Joints joints(NJOINT);
    Bones  bones(NBONE);
    Bones tempbone(35);
    std::vector<GLComplex*> objs(1);
    objs[0] = new GLComplex();

    // Lower body joints
    joints[J_LHR]     = new RevoluteXJoint("LHR");
    joints[J_LHP]     = new RevoluteYJoint("LHP");
    joints[J_LKN]     = new RevoluteYJoint("LKN");
    joints[J_RHR]     = new RevoluteXJoint("RHR");
    joints[J_RHP]     = new RevoluteYJoint("RHP");
    joints[J_RKN]     = new RevoluteYJoint("RKN");
    // Upper body joints
    joints[J_LSR]     = new RevoluteXJoint("LSR");
    joints[J_LSP]     = new RevoluteYJoint("LSP");
    joints[J_LEB]     = new RevoluteYJoint("LEB");
    joints[J_RSR]     = new RevoluteXJoint("RSR");
    joints[J_RSP]     = new RevoluteYJoint("RSP");
    joints[J_REB]     = new RevoluteYJoint("REB");


    QuadParams Oi;
    GLObject *body_wst	   = newBox(0.0,0.0,-0.03,Oi.P2HX*2+0.1,Oi.P2HY*2,0.15);

    //LARM
    GLObject *body_lsr	   = newBox(-0.05, 0, 0.0,Oi.P2HX+0.05,0.12,0.08);
    GLObject *body_lsp     = newBox(0.0, 0.0, -Oi.ULEG*0.5,0.05,0.05,Oi.ULEG);
    GLObject *body_leb     = newBox(0.0, 0.0, -Oi.LLEG*0.5,0.05,0.05,Oi.LLEG);

    //RARM
    GLObject *body_rsr	   = newBox(-0.05, 0, 0.0,Oi.P2HX+0.05,0.12,0.08);
    GLObject *body_rsp     = newBox(0.0, 0.0, -Oi.ULEG*0.5,0.05,0.05,Oi.ULEG);
    GLObject *body_reb     = newBox(0.0, 0.0, -Oi.LLEG*0.5,0.05,0.05,Oi.LLEG);
    //LLEG
    GLObject *body_lhr	   = newBox(+0.05,0, 0.0,Oi.P2HX+0.05,0.12,0.08);
    GLObject *body_lhp	   = newBox(0.0, 0.0, -Oi.ULEG*0.5,0.05,0.05,Oi.ULEG);
    GLObject *body_lkn	   = newBox(0.0, 0.0, -Oi.LLEG*0.5,0.05,0.05,Oi.LLEG);

    //RLEG
    GLObject *body_rhr	   = newBox(+0.05, 0, 0.0,Oi.P2HX+0.05,0.12,0.08);
    GLObject *body_rhp	   = newBox(0.0, 0.0, -Oi.ULEG*0.5,0.05,0.05,Oi.ULEG);
    GLObject *body_rkn	   = newBox(0.0, 0.0, -Oi.LLEG*0.5,0.05,0.05,Oi.LLEG);
//    GLObject *body_rhr	   = newCylinder(+0.05, 0, 0.0,Oi.P2HX+0.05,0.12,0.08);
//    GLObject *body_rhp	   = newCylinder(0.0, 0.0, -Oi.ULEG*0.5,0.05,0.05,Oi.ULEG);
//    GLObject *body_rkn	   = newCylinder(0.0, 0.0, -Oi.LLEG*0.5,0.05,0.05,Oi.LLEG);


    bones[B_WST]    = new Bone(COMP_NAME[BWST],    isnl::pos(0.0, 0.0, 0.0), body_wst);

    bones[B_LSP]    = new Bone(COMP_NAME[BLSP],    isnl::pos(0.0, 0.0, 0.0), body_lsp);
    bones[B_LSR]    = new Bone(COMP_NAME[BLSR],    isnl::pos(0.0, 0.0, 0.0), body_lsr);
    bones[B_LEB]    = new Bone(COMP_NAME[BLEB],    isnl::pos(0.0, 0.0, 0.0), body_leb);

    bones[B_RSP]    = new Bone(COMP_NAME[BRSP],    isnl::pos(0.0, 0.0, 0.0), body_rsp);
    bones[B_RSR]    = new Bone(COMP_NAME[BRSR],    isnl::pos(0.0, 0.0, 0.0), body_rsr);
    bones[B_REB]    = new Bone(COMP_NAME[BREB],    isnl::pos(0.0, 0.0, 0.0), body_reb);

    bones[B_LHR]    = new Bone(COMP_NAME[BLHR],    isnl::pos(0.0, 0.0, 0.0), body_lhr);
    bones[B_LHP]    = new Bone(COMP_NAME[BLHP],    isnl::pos(0.0, 0.0, 0.0), body_lhp);
    bones[B_LKN]    = new Bone(COMP_NAME[BLKN],    isnl::pos(0.0, 0.0, 0.0), body_lkn);

    bones[B_RHR]    = new Bone(COMP_NAME[BRHR],    isnl::pos(0.0, 0.0, 0.0), body_rhr);
    bones[B_RHP]    = new Bone(COMP_NAME[BRHP],    isnl::pos(0.0, 0.0, 0.0), body_rhp);
    bones[B_RKN]    = new Bone(COMP_NAME[BRKN],    isnl::pos(0.0, 0.0, 0.0), body_rkn);


    tempbone[0]  = new Bone("T_LS",       isnl::pos(Oi.P2HX,Oi.P2HY,0.0));
    tempbone[1]  = new Bone("LS_LS2",       isnl::pos(0.0,Oi.R2P,0.0));
    tempbone[2]  = new Bone("LS2_LEB",     isnl::pos(0.0,0.0,-Oi.ULEG));
    tempbone[3]  = new Bone("T_RS",       isnl::pos(Oi.P2HX,-Oi.P2HY,0.0));
    tempbone[4]  = new Bone("RS_RS2",       isnl::pos(0.0,-Oi.R2P,0.0));
    tempbone[5]  = new Bone("RS2_REB",    isnl::pos(0.0,0.0,-Oi.ULEG));

    tempbone[6]  = new Bone("T_LH",        isnl::pos(-Oi.P2HX,Oi.P2HY,0.0));
    tempbone[7]  = new Bone("LH_LH2",        isnl::pos(0.0,Oi.R2P,0.0));
    tempbone[8]  = new Bone("LH2_LKN",      isnl::pos(0.0,0.0,-Oi.ULEG));
    tempbone[9]  = new Bone("T_RH",        isnl::pos(-Oi.P2HX,-Oi.P2HY,0.0));
    tempbone[10]  = new Bone("RH_RH2",       isnl::pos(0.0,-Oi.R2P,0.0));
    tempbone[11]  = new Bone("RH2_RKN",    isnl::pos(0.0,0.0,-Oi.ULEG));
    //tree
    bones[B_WST]->setParent(NULL);
    *bones[B_WST] + tempbone[0]+joints[J_LSR] + bones[B_LSR] + tempbone[1]+ joints[J_LSP] + bones[B_LSP]
            + tempbone[2]+ joints[J_LEB] + bones[B_LEB];

    *bones[B_WST] + tempbone[3]+joints[J_RSR] + bones[B_RSR] + tempbone[4]+ joints[J_RSP] + bones[B_RSP]
            + tempbone[5]+ joints[J_REB] + bones[B_REB];


    *bones[B_WST] + tempbone[6]+joints[J_LHR] + bones[B_LHR] + tempbone[7]+ joints[J_LHP] + bones[B_LHP]
            + tempbone[8]+ joints[J_LKN] + bones[B_LKN];

    *bones[B_WST] + tempbone[9]+joints[J_RHR] + bones[B_RHR] + tempbone[10]+ joints[J_RHP] + bones[B_RHP]
            + tempbone[11]+ joints[J_RKN] + bones[B_RKN];





    return new HUBOModel(bones, joints, objs);
}


HUBOModel* GLWidget::newModel_true(){ // encoder model

    Joints joints(NJOINT);
    Bones  bones(NBONE);
    Bones tempbone(35);
    std::vector<GLComplex*> objs(1);
    objs[0] = new GLComplex();

    // Lower body joints

    joints[J_LHR]     = new RevoluteXJoint("LHR");
    joints[J_LHP]     = new RevoluteYJoint("LHP");
    joints[J_LKN]     = new RevoluteYJoint("LKN");
    joints[J_RHR]     = new RevoluteXJoint("RHR");
    joints[J_RHP]     = new RevoluteYJoint("RHP");
    joints[J_RKN]     = new RevoluteYJoint("RKN");
    // Upper body joints
    joints[J_LSR]     = new RevoluteXJoint("LSR");
    joints[J_LSP]     = new RevoluteYJoint("LSP");
    joints[J_LEB]     = new RevoluteYJoint("LEB");
    joints[J_RSR]     = new RevoluteXJoint("RSR");
    joints[J_RSP]     = new RevoluteYJoint("RSP");
    joints[J_REB]     = new RevoluteYJoint("REB");


    QuadParams Oi;
    GLObject *body_wst	   = newBox_true(0.0,0.0,-0.03,Oi.P2HX*2+0.1,Oi.P2HY*2,0.15);

    //LARM
    GLObject *body_lsr	   = newBox_true(-0.05, 0, 0.0,Oi.P2HX+0.05,0.12,0.08);
    GLObject *body_lsp     = newBox_true(0.0, 0.0, -Oi.ULEG*0.5,0.05,0.05,Oi.ULEG);
    GLObject *body_leb     = newBox_true(0.0, 0.0, -Oi.LLEG*0.5,0.05,0.05,Oi.LLEG);

    //RARM
    GLObject *body_rsr	   = newBox_true(-0.05, 0, 0.0,Oi.P2HX+0.05,0.12,0.08);
    GLObject *body_rsp     = newBox_true(0.0, 0.0, -Oi.ULEG*0.5,0.05,0.05,Oi.ULEG);
    GLObject *body_reb     = newBox_true(0.0, 0.0, -Oi.LLEG*0.5,0.05,0.05,Oi.LLEG);
    //LLEG
    GLObject *body_lhr	   = newBox_true(+0.05,0, 0.0,Oi.P2HX+0.05,0.12,0.08);
    GLObject *body_lhp	   = newBox_true(0.0, 0.0, -Oi.ULEG*0.5,0.05,0.05,Oi.ULEG);
    GLObject *body_lkn	   = newBox_true(0.0, 0.0, -Oi.LLEG*0.5,0.05,0.05,Oi.LLEG);

    //RLEG
    GLObject *body_rhr	   = newBox_true(+0.05, 0, 0.0,Oi.P2HX+0.05,0.12,0.08);
    GLObject *body_rhp	   = newBox_true(0.0, 0.0, -Oi.ULEG*0.5,0.05,0.05,Oi.ULEG);
    GLObject *body_rkn	   = newBox_true(0.0, 0.0, -Oi.LLEG*0.5,0.05,0.05,Oi.LLEG);


    bones[B_WST]    = new Bone(COMP_NAME[BWST],    isnl::pos(0.0, 0.0, 0.0), body_wst);

    bones[B_LSP]    = new Bone(COMP_NAME[BLSP],    isnl::pos(0.0, 0.0, 0.0), body_lsp);
    bones[B_LSR]    = new Bone(COMP_NAME[BLSR],    isnl::pos(0.0, 0.0, 0.0), body_lsr);
    bones[B_LEB]    = new Bone(COMP_NAME[BLEB],    isnl::pos(0.0, 0.0, 0.0), body_leb);

    bones[B_RSP]    = new Bone(COMP_NAME[BRSP],    isnl::pos(0.0, 0.0, 0.0), body_rsp);
    bones[B_RSR]    = new Bone(COMP_NAME[BRSR],    isnl::pos(0.0, 0.0, 0.0), body_rsr);
    bones[B_REB]    = new Bone(COMP_NAME[BREB],    isnl::pos(0.0, 0.0, 0.0), body_reb);

    bones[B_LHR]    = new Bone(COMP_NAME[BLHR],    isnl::pos(0.0, 0.0, 0.0), body_lhr);
    bones[B_LHP]    = new Bone(COMP_NAME[BLHP],    isnl::pos(0.0, 0.0, 0.0), body_lhp);
    bones[B_LKN]    = new Bone(COMP_NAME[BLKN],    isnl::pos(0.0, 0.0, 0.0), body_lkn);

    bones[B_RHR]    = new Bone(COMP_NAME[BRHR],    isnl::pos(0.0, 0.0, 0.0), body_rhr);
    bones[B_RHP]    = new Bone(COMP_NAME[BRHP],    isnl::pos(0.0, 0.0, 0.0), body_rhp);
    bones[B_RKN]    = new Bone(COMP_NAME[BRKN],    isnl::pos(0.0, 0.0, 0.0), body_rkn);


    tempbone[0]  = new Bone("T_LS",       isnl::pos(Oi.P2HX,Oi.P2HY,0.0));
    tempbone[1]  = new Bone("LS_LS2",       isnl::pos(0.0,Oi.R2P,0.0));
    tempbone[2]  = new Bone("LS2_LEB",     isnl::pos(0.0,0.0,-Oi.ULEG));
    tempbone[3]  = new Bone("T_RS",       isnl::pos(Oi.P2HX,-Oi.P2HY,0.0));
    tempbone[4]  = new Bone("RS_RS2",       isnl::pos(0.0,-Oi.R2P,0.0));
    tempbone[5]  = new Bone("RS2_REB",    isnl::pos(0.0,0.0,-Oi.ULEG));

    tempbone[6]  = new Bone("T_LH",        isnl::pos(-Oi.P2HX,Oi.P2HY,0.0));
    tempbone[7]  = new Bone("LH_LH2",        isnl::pos(0.0,Oi.R2P,0.0));
    tempbone[8]  = new Bone("LH2_LKN",      isnl::pos(0.0,0.0,-Oi.ULEG));
    tempbone[9]  = new Bone("T_RH",        isnl::pos(-Oi.P2HX,-Oi.P2HY,0.0));
    tempbone[10]  = new Bone("RH_RH2",       isnl::pos(0.0,-Oi.R2P,0.0));
    tempbone[11]  = new Bone("RH2_RKN",    isnl::pos(0.0,0.0,-Oi.ULEG));
    //tree
    bones[B_WST]->setParent(NULL);
    *bones[B_WST] + tempbone[0]+joints[J_LSR] + bones[B_LSR] + tempbone[1]+ joints[J_LSP] + bones[B_LSP]
            + tempbone[2]+ joints[J_LEB] + bones[B_LEB];

    *bones[B_WST] + tempbone[3]+joints[J_RSR] + bones[B_RSR] + tempbone[4]+ joints[J_RSP] + bones[B_RSP]
            + tempbone[5]+ joints[J_REB] + bones[B_REB];


    *bones[B_WST] + tempbone[6]+joints[J_LHR] + bones[B_LHR] + tempbone[7]+ joints[J_LHP] + bones[B_LHP]
            + tempbone[8]+ joints[J_LKN] + bones[B_LKN];

    *bones[B_WST] + tempbone[9]+joints[J_RHR] + bones[B_RHR] + tempbone[10]+ joints[J_RHP] + bones[B_RHP]
            + tempbone[11]+ joints[J_RKN] + bones[B_RKN];





    return new HUBOModel(bones, joints, objs);
}
