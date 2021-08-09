#include "func_kinematics.h"
using namespace std;
Matrix3d func_kinematics::L2R(Matrix3d in)
{
    Matrix3d out;
    Matrix3d mY = Matrix3d::Identity();
    mY(1,1) = -1;
    out = mY*in*mY;
    return out;
}
Matrix3d func_kinematics::F2B(Matrix3d in)
{
    Matrix3d out;
    Matrix3d mX = Matrix3d::Identity();
    mX(0,0) = -1;
    out = mX*in*mX;
    return out;
}
Vector3d func_kinematics::F2B(Vector3d in)
{
    Vector3d out = in;
    out[0] = -in[0];
    return out;
}
Vector3d func_kinematics::L2R(Vector3d in)
{
    Vector3d out = in;
    out[1] = -in[1];
    return out;
}

//MatrixNd func_kinematics::Tmat(Matrix3d R, Vector3d p)
//{
//    MatrixNd Tmat = MatrixNd::Zero(4,4);
//    Tmat.block(0,0,3,3) << R;
//    Tmat.block(0,3,3,1) << p;
//    return Tmat;
//}

Matrix3d func_kinematics::getOneLegJacobian(Vector3d JointPos, int LR)
{
    Matrix3d Jacobian = Matrix3d::Zero();

    double R = JointPos[0];  // Roll angle
    double P = JointPos[1];  // Pitch angle
    double K = JointPos[2];  // Knee angle

    double l1 = R2P * LR; // -1 for the right and 1 for the left
    double l2 = -ULEG;
    double l3 = -LLEG;

//    Jacobian(0, 0) = 0;
//    Jacobian(0, 1) = -l2*cos(P) - l3*cos(P+K);
//    Jacobian(0, 2) = -l3*cos(P+K);

//    Jacobian(1, 0) = -l1*sin(R) + cos(R)*(l2*cos(P) + l3*cos(P+K));
//    Jacobian(1, 1) = -sin(R)*(l2*sin(P) + l3*sin(P+K));
//    Jacobian(1, 2) = -l3*sin(R)*sin(P+K);

//    Jacobian(2, 0) = l1*cos(R) + sin(R)*(l2*cos(P) + l3*cos(P+K));
//    Jacobian(2, 1) = cos(R)*(l2*sin(P) + l3*sin(P+K));
//    Jacobian(2, 2) = l3*cos(R)*sin(P+K);

    Jacobian(0, 0) = 0;
    Jacobian(1, 0) = -l1*sin(R) - cos(R)*(l2*cos(P) + l3*cos(P+K));
    Jacobian(2, 0) = l1*cos(R) - sin(R)*(l2*cos(P) + l3*cos(P+K));

    Jacobian(0, 1) = l3*cos(K+P) + l2*cos(P);
    Jacobian(1, 1) = sin(R)*(l3*sin(K+P)+l2*sin(P));
    Jacobian(2, 1) = -cos(R)*(l3*sin(K+P)+l2*sin(P));

    Jacobian(0, 2) = l3*cos(K+P);
    Jacobian(1, 2) = l3*sin(K+P)*sin(R);
    Jacobian(2, 2) = -l3*sin(K+P)*cos(R);

    return Jacobian;
}

Matrix3d func_kinematics::getOneLegJacobian_Global(SensorInfo sensor, Vector3d JointPos, int LR)
{
    Matrix3d Jacobian_local = Matrix3d::Zero();
    Matrix3d Jacobian_global = Matrix3d::Zero();

    Matrix3d R_Pel = quat2R(sensor.IMU.quat);

    double R = JointPos[0];  // Roll angle
    double P = JointPos[1];  // Pitch angle
    double K = JointPos[2];  // Knee angle

    double l1 = R2P * LR; // -1 for the right and 1 for the left
    double l2 = -ULEG;
    double l3 = -LLEG;

//    Jacobian(0, 0) = 0;
//    Jacobian(0, 1) = -l2*cos(P) - l3*cos(P+K);
//    Jacobian(0, 2) = -l3*cos(P+K);

//    Jacobian(1, 0) = -l1*sin(R) + cos(R)*(l2*cos(P) + l3*cos(P+K));
//    Jacobian(1, 1) = -sin(R)*(l2*sin(P) + l3*sin(P+K));
//    Jacobian(1, 2) = -l3*sin(R)*sin(P+K);

//    Jacobian(2, 0) = l1*cos(R) + sin(R)*(l2*cos(P) + l3*cos(P+K));
//    Jacobian(2, 1) = cos(R)*(l2*sin(P) + l3*sin(P+K));
//    Jacobian(2, 2) = l3*cos(R)*sin(P+K);

    Jacobian_local(0, 0) = 0;
    Jacobian_local(1, 0) = -l1*sin(R) - cos(R)*(l2*cos(P) + l3*cos(P+K));
    Jacobian_local(2, 0) = l1*cos(R) - sin(R)*(l2*cos(P) + l3*cos(P+K));

    Jacobian_local(0, 1) = l3*cos(K+P) + l2*cos(P);
    Jacobian_local(1, 1) = sin(R)*(l3*sin(K+P)+l2*sin(P));
    Jacobian_local(2, 1) = -cos(R)*(l3*sin(K+P)+l2*sin(P));

    Jacobian_local(0, 2) = l3*cos(K+P);
    Jacobian_local(1, 2) = l3*sin(K+P)*sin(R);
    Jacobian_local(2, 2) = -l3*sin(K+P)*cos(R);

    Jacobian_global =  R_Pel*Jacobian_local;

    return Jacobian_global;
}

Matrix3d func_kinematics::getOneLegJacobian_angleRef(Matrix3d angleRef, Vector3d JointPos, int LR)
{
    Matrix3d Jacobian_local = Matrix3d::Zero();
    Matrix3d Jacobian_global = Matrix3d::Zero();

    Matrix3d R_Pel = angleRef;

    double R = JointPos[0];  // Roll angle
    double P = JointPos[1];  // Pitch angle
    double K = JointPos[2];  // Knee angle

    double l1 = R2P * LR; // -1 for the right and 1 for the left
    double l2 = -ULEG;
    double l3 = -LLEG;

//    Jacobian(0, 0) = 0;
//    Jacobian(0, 1) = -l2*cos(P) - l3*cos(P+K);
//    Jacobian(0, 2) = -l3*cos(P+K);

//    Jacobian(1, 0) = -l1*sin(R) + cos(R)*(l2*cos(P) + l3*cos(P+K));
//    Jacobian(1, 1) = -sin(R)*(l2*sin(P) + l3*sin(P+K));
//    Jacobian(1, 2) = -l3*sin(R)*sin(P+K);

//    Jacobian(2, 0) = l1*cos(R) + sin(R)*(l2*cos(P) + l3*cos(P+K));
//    Jacobian(2, 1) = cos(R)*(l2*sin(P) + l3*sin(P+K));
//    Jacobian(2, 2) = l3*cos(R)*sin(P+K);

    Jacobian_local(0, 0) = 0;
    Jacobian_local(1, 0) = -l1*sin(R) - cos(R)*(l2*cos(P) + l3*cos(P+K));
    Jacobian_local(2, 0) = l1*cos(R) - sin(R)*(l2*cos(P) + l3*cos(P+K));

    Jacobian_local(0, 1) = l3*cos(K+P) + l2*cos(P);
    Jacobian_local(1, 1) = sin(R)*(l3*sin(K+P)+l2*sin(P));
    Jacobian_local(2, 1) = -cos(R)*(l3*sin(K+P)+l2*sin(P));

    Jacobian_local(0, 2) = l3*cos(K+P);
    Jacobian_local(1, 2) = l3*sin(K+P)*sin(R);
    Jacobian_local(2, 2) = -l3*sin(K+P)*cos(R);

    //R_Pel = diagonalize(Vector3d(1.0,1.0,1.0));

    Jacobian_global =  R_Pel*Jacobian_local;

    return Jacobian_global;
}

Vector3d func_kinematics::FK_OneLeg_local(Vector3d JointPos, int LR)
{
    Vector3d foot_pos;

    double R = JointPos[0];  // Roll angle
    double P = JointPos[1];  // Pitch angle
    double K = JointPos[2];  // Knee angle

    // -1 for the right and 1 for the left
    double l1 = R2P * LR; // -1 for the right and 1 for the left
    double l2 = -ULEG;
    double l3 = -LLEG;

    foot_pos[0] = l2*sin(P) + l3*sin(P+K);
    foot_pos[1] = l1*cos(R) - sin(R)*(l2*cos(P) + l3*cos(P+K));
    foot_pos[2] = l1*sin(R) + cos(R)*(l2*cos(P) + l3*cos(P+K));


    return foot_pos;
}

Vector3d func_kinematics::FK_OneLeg_localize(RobotState state, Vector3d vec_global, int idx)
{
    Vector3d vec_local;
    Matrix3d R = state.rotate;
    Vector3d p = state.COM.pos;

    MatrixNd Tpel = Tmat(p,R);
    MatrixNd Tpel_inv = Tmat_inv(Tpel);
    Matrix3d R_inv = Tpel_inv.block(0,0,3,3);
    Vector3d p_inv = Tpel_inv.block(0,3,3,1);

    vec_local = p_inv + R_inv*vec_global;
    int LR = 2*(idx%2) - 1;
    int FH = sign(idx-1.5);
    vec_local = vec_local - Vector3d(FH*P2HX, LR*P2HY, 0);

    return vec_local;
}

Vector3d func_kinematics::IK_OneLeg(Vector3d xyz, int LR)
{
    Vector3d legAngle;
    double a,b,c,d;
    if(xyz[2]>-0.08){ xyz[2] = -0.08;}
    if(xyz[0]>xlim){xyz[0] = xlim;}
    if(xyz[0]<-xlim){xyz[0] = -xlim;}
    if(xyz[1]>ylim){xyz[1] = ylim;}
    if(xyz[1]<-ylim){xyz[1] = -ylim;}

    double ul = ULEG;
    double ll = LLEG;
    double r2p = R2P*LR;
    double y = xyz[1];
    double z = xyz[2];

    double t_l = sqrtp(y*y + z*z - r2p*r2p);

    if(t_l>ULEG+LLEG-0.02)
    {
        t_l = ULEG+LLEG-0.02;
    }
    if(t_l<0.03)
    {
        t_l = 0.03;
    }

    a = r2p;    b  = -t_l;
    c = -t_l;   d = -r2p;


    double cR = (d*y+-b*z)/(a*d-b*c);
    double sR = (-c*y+a*z)/(a*d-b*c);


    legAngle[0] = -atan2(sR,cR);
    double newx = xyz[0];
    double newy = t_l;
    if(sqrtp(newx*newx+newy*newy) > ULEG+LLEG-0.02)
    {
        double nxt, nyt;
        nxt = newx*((ULEG+LLEG-0.02)/sqrtp(newx*newx+newy*newy));
        nyt = newy*((ULEG+LLEG-0.02)/sqrtp(newx*newx+newy*newy));
        newx = nxt;
        newy = nyt;
    }

    double cPminusK = (ul*ul+ll*ll-(newx*newx+newy*newy))/(2*ul*ll);

    legAngle[2] = -(M_PI-acos(cPminusK));

    a = -ll*sin(legAngle[2]);
    b  = -ul-ll*cos(legAngle[2]);
    c = ul+ll*cos(legAngle[2]);
    d = -ll*sin(legAngle[2]);

    double cP = (d*newx+-b*newy)/(a*d-b*c);
    double sP = (-c*newx+a*newy)/(a*d-b*c);

    legAngle[1] = atan2(sP,cP);

    for(int i=0;i<3;i++)
    {
        if(std::isnan(legAngle[i]))
        {
            legAngle[i] = 0.0;
            std::cout << "nan!" << std::endl;
        }
    }

    return legAngle;
}

QuadJoint func_kinematics::IK_FullBody(RobotState state)
{
    QuadJoint QJ;
    Matrix3d R = state.rotate;
    Vector3d p = state.COM.pos;
//    cout << state.COM.pos[0] << endl;

    MatrixNd Tpel = Tmat(p,R);
//    MatrixNd Tpel_inv = Tpel.inverse();
    MatrixNd Tpel_inv = Tmat_inv(Tpel);
    Matrix3d R_inv = Tpel_inv.block(0,0,3,3);
    Vector3d p_inv = Tpel_inv.block(0,3,3,1);
//    cout << "Tpel = " << Tpel << endl;
//    cout << "Tpel_inv = " << Tpel_inv << endl;
//    cout << "R_inv = " << R_inv << endl;
//    cout << "p_inv = " << p_inv << endl;

    Vector3d foot_HR_local = p_inv + R_inv*state.foot.HR.pos;
    Vector3d foot_HL_local = p_inv + R_inv*state.foot.HL.pos;
    Vector3d foot_FR_local = p_inv + R_inv*state.foot.FR.pos;
    Vector3d foot_FL_local = p_inv + R_inv*state.foot.FL.pos;
    foot_HR_local = foot_HR_local - offset_p2rh;
    foot_HL_local = foot_HL_local - offset_p2lh;
    foot_FR_local = foot_FR_local - offset_p2rs;
    foot_FL_local = foot_FL_local - offset_p2ls;

    Vector3d inv_angle;
//    cout << "HR_local = " << foot_HR_local << endl;
    inv_angle = IK_OneLeg(foot_HR_local, -1);
    QJ.HRR = inv_angle[0];
    QJ.HRP = inv_angle[1];
    QJ.HRK = inv_angle[2];

    inv_angle = IK_OneLeg(foot_HL_local, 1);
    QJ.HLR = inv_angle[0];
    QJ.HLP = inv_angle[1];
    QJ.HLK = inv_angle[2];

    inv_angle = IK_OneLeg(foot_FR_local, -1);
    QJ.FRR = inv_angle[0];
    QJ.FRP = inv_angle[1];
    QJ.FRK = inv_angle[2];

    inv_angle = IK_OneLeg(foot_FL_local, 1);
    QJ.FLR = inv_angle[0];
    QJ.FLP = inv_angle[1];
    QJ.FLK = inv_angle[2];

    if(QJ.HRR> 35.0*D2Rf) QJ.HRR =  35.0*D2Rf;
    if(QJ.HRR<-35.0*D2Rf) QJ.HRR = -35.0*D2Rf;
    if(QJ.FRR> 35.0*D2Rf) QJ.FRR =  35.0*D2Rf;
    if(QJ.FRR<-35.0*D2Rf) QJ.FRR = -35.0*D2Rf;
    if(QJ.HLR> 35.0*D2Rf) QJ.HLR =  35.0*D2Rf;
    if(QJ.HLR<-35.0*D2Rf) QJ.HLR = -35.0*D2Rf;
    if(QJ.FLR> 35.0*D2Rf) QJ.FLR =  35.0*D2Rf;
    if(QJ.FLR<-35.0*D2Rf) QJ.FLR = -35.0*D2Rf;

    return QJ;
}

RobotState func_kinematics::FK_FullBody(SensorInfo sensor)
{
    RobotState state_temp;
    QuadJoint QJ = sensor.Encoder.pos;
    Vector3d pPel = Vector3d::Zero();
    Vector4d qPel = Vector4d::Zero();
    //Vector4d qPel = sensor.IMU.quat;
    Matrix3d R = quat2R(qPel);

    MatrixNd T_PEL = Tmat(pPel, R);
//    cout << "T_PEL = " << T_PEL << endl;

    MatrixNd T_HRR = T_PEL * Tmat(offset_p2rh,  rotate_3D(Vector3d(1,0,0),  QJ.HRR));
    MatrixNd T_HRP = T_HRR * Tmat(offset_rh2rp, rotate_3D(Vector3d(0,1,0),  QJ.HRP));
    MatrixNd T_HRK = T_HRP * Tmat(offset_uleg,  rotate_3D(Vector3d(0,1,0),  QJ.HRK));
    MatrixNd T_HR  = T_HRK * Tmat(offset_lleg,  rotate_3D(Vector3d(1,0,0), 0));

    MatrixNd T_HLR = T_PEL * Tmat(offset_p2lh,  rotate_3D(Vector3d(1,0,0),  QJ.HLR));
    MatrixNd T_HLP = T_HLR * Tmat(offset_lh2lp, rotate_3D(Vector3d(0,1,0),  QJ.HLP));
    MatrixNd T_HLK = T_HLP * Tmat(offset_uleg,  rotate_3D(Vector3d(0,1,0),  QJ.HLK));
    MatrixNd T_HL  = T_HLK * Tmat(offset_lleg,  rotate_3D(Vector3d(1,0,0), 0));

    MatrixNd T_FRR = T_PEL * Tmat(offset_p2rs,  rotate_3D(Vector3d(1,0,0),  QJ.FRR));
    MatrixNd T_FRP = T_FRR * Tmat(offset_rs2rp, rotate_3D(Vector3d(0,1,0),  QJ.FRP));
    MatrixNd T_FRK = T_FRP * Tmat(offset_uleg,  rotate_3D(Vector3d(0,1,0),  QJ.FRK));
    MatrixNd T_FR  = T_FRK * Tmat(offset_lleg,  rotate_3D(Vector3d(1,0,0), 0));

    MatrixNd T_FLR = T_PEL * Tmat(offset_p2ls,  rotate_3D(Vector3d(1,0,0),  QJ.FLR));
    MatrixNd T_FLP = T_FLR * Tmat(offset_ls2lp, rotate_3D(Vector3d(0,1,0),  QJ.FLP));
    MatrixNd T_FLK = T_FLP * Tmat(offset_uleg,  rotate_3D(Vector3d(0,1,0),  QJ.FLK));
    MatrixNd T_FL  = T_FLK * Tmat(offset_lleg,  rotate_3D(Vector3d(1,0,0), 0));

    state_temp.foot.HR.pos = T_HR.block(0,3,3,1);
    state_temp.foot.HL.pos = T_HL.block(0,3,3,1);
    state_temp.foot.FR.pos = T_FR.block(0,3,3,1);
    state_temp.foot.FL.pos = T_FL.block(0,3,3,1);

    state_temp.hip.HR.pos = T_HRP.block(0,3,3,1);
    state_temp.hip.HL.pos = T_HLP.block(0,3,3,1);
    state_temp.hip.FR.pos = T_FRP.block(0,3,3,1);
    state_temp.hip.FL.pos = T_FLP.block(0,3,3,1);

    state_temp.torso.pos = pPel;
    state_temp.COM.pos = pPel; //temporary
    state_temp.rotate = R;

    return state_temp;
}

RobotState func_kinematics::FK_FullBody_ACC(Vector4d ACC_quat, SensorInfo sensor)
{
    RobotState state_temp;
    QuadJoint QJ = sensor.Encoder.pos;
    Vector3d pPel = Vector3d::Zero();
    //Vector4d qPel = Vector4d::Zero();
    Vector4d qPel = ACC_quat;
    Matrix3d R = quat2R(qPel);

    MatrixNd T_PEL = Tmat(pPel, R);
//    cout << "T_PEL = " << T_PEL << endl;

    MatrixNd T_HRR = T_PEL * Tmat(offset_p2rh,  rotate_3D(Vector3d(1,0,0),  QJ.HRR));
    MatrixNd T_HRP = T_HRR * Tmat(offset_rh2rp, rotate_3D(Vector3d(0,1,0),  QJ.HRP));
    MatrixNd T_HRK = T_HRP * Tmat(offset_uleg,  rotate_3D(Vector3d(0,1,0),  QJ.HRK));
    MatrixNd T_HR  = T_HRK * Tmat(offset_lleg,  rotate_3D(Vector3d(1,0,0), 0));

    MatrixNd T_HLR = T_PEL * Tmat(offset_p2lh,  rotate_3D(Vector3d(1,0,0),  QJ.HLR));
    MatrixNd T_HLP = T_HLR * Tmat(offset_lh2lp, rotate_3D(Vector3d(0,1,0),  QJ.HLP));
    MatrixNd T_HLK = T_HLP * Tmat(offset_uleg,  rotate_3D(Vector3d(0,1,0),  QJ.HLK));
    MatrixNd T_HL  = T_HLK * Tmat(offset_lleg,  rotate_3D(Vector3d(1,0,0), 0));

    MatrixNd T_FRR = T_PEL * Tmat(offset_p2rs,  rotate_3D(Vector3d(1,0,0),  QJ.FRR));
    MatrixNd T_FRP = T_FRR * Tmat(offset_rs2rp, rotate_3D(Vector3d(0,1,0),  QJ.FRP));
    MatrixNd T_FRK = T_FRP * Tmat(offset_uleg,  rotate_3D(Vector3d(0,1,0),  QJ.FRK));
    MatrixNd T_FR  = T_FRK * Tmat(offset_lleg,  rotate_3D(Vector3d(1,0,0), 0));

    MatrixNd T_FLR = T_PEL * Tmat(offset_p2ls,  rotate_3D(Vector3d(1,0,0),  QJ.FLR));
    MatrixNd T_FLP = T_FLR * Tmat(offset_ls2lp, rotate_3D(Vector3d(0,1,0),  QJ.FLP));
    MatrixNd T_FLK = T_FLP * Tmat(offset_uleg,  rotate_3D(Vector3d(0,1,0),  QJ.FLK));
    MatrixNd T_FL  = T_FLK * Tmat(offset_lleg,  rotate_3D(Vector3d(1,0,0), 0));

    state_temp.foot.HR.pos = T_HR.block(0,3,3,1);
    state_temp.foot.HL.pos = T_HL.block(0,3,3,1);
    state_temp.foot.FR.pos = T_FR.block(0,3,3,1);
    state_temp.foot.FL.pos = T_FL.block(0,3,3,1);

    state_temp.hip.HR.pos = T_HRP.block(0,3,3,1);
    state_temp.hip.HL.pos = T_HLP.block(0,3,3,1);
    state_temp.hip.FR.pos = T_FRP.block(0,3,3,1);
    state_temp.hip.FL.pos = T_FLP.block(0,3,3,1);

    state_temp.torso.pos = pPel;
    state_temp.COM.pos = pPel; //temporary
    state_temp.rotate = R;

    return state_temp;
}

RobotState func_kinematics::FK_FullBody_IMU(SensorInfo sensor)
{
    RobotState state_temp;
    QuadJoint QJ = sensor.Encoder.pos;
    Vector3d pPel = Vector3d::Zero();
    //Vector4d qPel = Vector4d::Zero();
    Vector4d qPel = sensor.IMU.quat;
    Matrix3d R = quat2R(qPel);

    MatrixNd T_PEL = Tmat(pPel, R);
//    cout << "T_PEL = " << T_PEL << endl;

    MatrixNd T_HRR = T_PEL * Tmat(offset_p2rh,  rotate_3D(Vector3d(1,0,0),  QJ.HRR));
    MatrixNd T_HRP = T_HRR * Tmat(offset_rh2rp, rotate_3D(Vector3d(0,1,0),  QJ.HRP));
    MatrixNd T_HRK = T_HRP * Tmat(offset_uleg,  rotate_3D(Vector3d(0,1,0),  QJ.HRK));
    MatrixNd T_HR  = T_HRK * Tmat(offset_lleg,  rotate_3D(Vector3d(1,0,0), 0));

    MatrixNd T_HLR = T_PEL * Tmat(offset_p2lh,  rotate_3D(Vector3d(1,0,0),  QJ.HLR));
    MatrixNd T_HLP = T_HLR * Tmat(offset_lh2lp, rotate_3D(Vector3d(0,1,0),  QJ.HLP));
    MatrixNd T_HLK = T_HLP * Tmat(offset_uleg,  rotate_3D(Vector3d(0,1,0),  QJ.HLK));
    MatrixNd T_HL  = T_HLK * Tmat(offset_lleg,  rotate_3D(Vector3d(1,0,0), 0));

    MatrixNd T_FRR = T_PEL * Tmat(offset_p2rs,  rotate_3D(Vector3d(1,0,0),  QJ.FRR));
    MatrixNd T_FRP = T_FRR * Tmat(offset_rs2rp, rotate_3D(Vector3d(0,1,0),  QJ.FRP));
    MatrixNd T_FRK = T_FRP * Tmat(offset_uleg,  rotate_3D(Vector3d(0,1,0),  QJ.FRK));
    MatrixNd T_FR  = T_FRK * Tmat(offset_lleg,  rotate_3D(Vector3d(1,0,0), 0));

    MatrixNd T_FLR = T_PEL * Tmat(offset_p2ls,  rotate_3D(Vector3d(1,0,0),  QJ.FLR));
    MatrixNd T_FLP = T_FLR * Tmat(offset_ls2lp, rotate_3D(Vector3d(0,1,0),  QJ.FLP));
    MatrixNd T_FLK = T_FLP * Tmat(offset_uleg,  rotate_3D(Vector3d(0,1,0),  QJ.FLK));
    MatrixNd T_FL  = T_FLK * Tmat(offset_lleg,  rotate_3D(Vector3d(1,0,0), 0));

    state_temp.foot.HR.pos = T_HR.block(0,3,3,1);
    state_temp.foot.HL.pos = T_HL.block(0,3,3,1);
    state_temp.foot.FR.pos = T_FR.block(0,3,3,1);
    state_temp.foot.FL.pos = T_FL.block(0,3,3,1);

    state_temp.hip.HR.pos = T_HRP.block(0,3,3,1);
    state_temp.hip.HL.pos = T_HLP.block(0,3,3,1);
    state_temp.hip.FR.pos = T_FRP.block(0,3,3,1);
    state_temp.hip.FL.pos = T_FLP.block(0,3,3,1);

    state_temp.torso.pos = pPel;
    state_temp.COM.pos = pPel; //temporary
    state_temp.rotate = R;

    return state_temp;
}

RobotState func_kinematics::FK_foot(SensorInfo sensor, Vector3d pfoot, int idx_leg)
{
    RobotState state_temp;
    state_temp = FK_FullBody(sensor);

    Vector3d pfoot_temp;
    if(idx_leg == 0) {pfoot_temp = state_temp.foot.HR.pos;}
    else if(idx_leg == 1) {pfoot_temp = state_temp.foot.HL.pos;}
    else if(idx_leg == 2) {pfoot_temp = state_temp.foot.FR.pos;}
    else if(idx_leg == 3) {pfoot_temp = state_temp.foot.FL.pos;}
    Vector3d offset = pfoot - pfoot_temp;

    state_temp.foot.HR.pos = state_temp.foot.HR.pos + offset;
    state_temp.foot.HL.pos = state_temp.foot.HL.pos + offset;
    state_temp.foot.FR.pos = state_temp.foot.FR.pos + offset;
    state_temp.foot.FL.pos = state_temp.foot.FL.pos + offset;

    state_temp.hip.HR.pos  = state_temp.hip.HR.pos + offset;
    state_temp.hip.HL.pos  = state_temp.hip.HL.pos + offset;
    state_temp.hip.FR.pos  = state_temp.hip.FR.pos + offset;
    state_temp.hip.FL.pos  = state_temp.hip.FL.pos + offset;

    state_temp.torso.pos   = state_temp.torso.pos + offset;
    state_temp.COM.pos     = state_temp.COM.pos + offset;

     return state_temp;
}

RobotState func_kinematics::FK_COM(SensorInfo sensor, Vector3d COM)
{
    RobotState state_temp;
    state_temp = FK_FullBody(sensor);

    Vector3d COM_temp = state_temp.COM.pos;
    Vector3d offset = COM - COM_temp;

    state_temp.foot.HR.pos = state_temp.foot.HR.pos + offset;
    state_temp.foot.HL.pos = state_temp.foot.HL.pos + offset;
    state_temp.foot.FR.pos = state_temp.foot.FR.pos + offset;
    state_temp.foot.FL.pos = state_temp.foot.FL.pos + offset;

    state_temp.hip.HR.pos  = state_temp.hip.HR.pos + offset;
    state_temp.hip.HL.pos  = state_temp.hip.HL.pos + offset;
    state_temp.hip.FR.pos  = state_temp.hip.FR.pos + offset;
    state_temp.hip.FL.pos  = state_temp.hip.FL.pos + offset;

    state_temp.torso.pos   = state_temp.torso.pos + offset;
    state_temp.COM.pos     = state_temp.COM.pos + offset;

    return state_temp;

}

RobotState func_kinematics::FK_COM_IMU(SensorInfo sensor, Vector3d COM)
{
    RobotState state_temp;
    state_temp = FK_FullBody_IMU(sensor);

    Vector3d COM_temp = state_temp.COM.pos;
    Vector3d offset = COM - COM_temp;

    state_temp.foot.HR.pos = state_temp.foot.HR.pos + offset;
    state_temp.foot.HL.pos = state_temp.foot.HL.pos + offset;
    state_temp.foot.FR.pos = state_temp.foot.FR.pos + offset;
    state_temp.foot.FL.pos = state_temp.foot.FL.pos + offset;

    state_temp.hip.HR.pos  = state_temp.hip.HR.pos + offset;
    state_temp.hip.HL.pos  = state_temp.hip.HL.pos + offset;
    state_temp.hip.FR.pos  = state_temp.hip.FR.pos + offset;
    state_temp.hip.FL.pos  = state_temp.hip.FL.pos + offset;

    state_temp.torso.pos   = state_temp.torso.pos + offset;
    state_temp.COM.pos     = state_temp.COM.pos + offset;

    return state_temp;

}

RobotState func_kinematics::FK_COM_ACC(Vector4d ACC_quat, SensorInfo sensor, Vector3d COM)
{
    RobotState state_temp;
    state_temp = FK_FullBody_ACC(ACC_quat, sensor);

    Vector3d COM_temp = state_temp.COM.pos;
    Vector3d offset = COM - COM_temp;

    state_temp.foot.HR.pos = state_temp.foot.HR.pos + offset;
    state_temp.foot.HL.pos = state_temp.foot.HL.pos + offset;
    state_temp.foot.FR.pos = state_temp.foot.FR.pos + offset;
    state_temp.foot.FL.pos = state_temp.foot.FL.pos + offset;

    state_temp.hip.HR.pos  = state_temp.hip.HR.pos + offset;
    state_temp.hip.HL.pos  = state_temp.hip.HL.pos + offset;
    state_temp.hip.FR.pos  = state_temp.hip.FR.pos + offset;
    state_temp.hip.FL.pos  = state_temp.hip.FL.pos + offset;

    state_temp.torso.pos   = state_temp.torso.pos + offset;
    state_temp.COM.pos     = state_temp.COM.pos + offset;

    return state_temp;

}
