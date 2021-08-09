#ifndef FUNC_KINEMATICS_H
#define FUNC_KINEMATICS_H

#include "hubodog5_struct.h"
#include "BasicMath.h"
//#include <algorithm>
#include "func_general.h"

class func_kinematics
{
public:
    double m_rhr;
    double m_rhp;
    double m_rkn;
    double m_rf;

    double m_lhr;
    double m_lhp;
    double m_lkn;
    double m_lf;

    double m_rsr;
    double m_rsp;
    double m_reb;
    double m_rh;

    double m_lsr;
    double m_lsp;
    double m_leb;
    double m_lh;

    double m_torso;
    double M_total;

    double P2HX;
    double P2HY;
    double R2P;
    double ULEG;
    double LLEG;
    double xlim, ylim;

    Vector3d offset_p2rh, offset_p2lh, offset_uleg, offset_lleg;
    Vector3d offset_rh2rp, offset_lh2lp;
    Vector3d c_rhr, c_rhp, c_rkn;
    Vector3d c_lhr, c_lhp, c_lkn;
    Matrix3d I_rhr, I_rhp, I_rkn;
    Matrix3d I_lhr, I_lhp, I_lkn;

    Vector3d offset_p2rs, offset_p2ls;
    Vector3d offset_rs2rp, offset_ls2lp;
    Vector3d c_rsr, c_rsp, c_reb;
    Vector3d c_lsr, c_lsp, c_leb;
    Matrix3d I_rsr, I_rsp, I_reb;
    Matrix3d I_lsr, I_lsp, I_leb;


    Vector3d c_torso;
    Matrix3d I_torso;

    Matrix3d L2R(Matrix3d in);
    Matrix3d F2B(Matrix3d in);
    Vector3d F2B(Vector3d in);
    Vector3d L2R(Vector3d in);

    func_kinematics()
    {
        m_rhr = 1.6;
        m_rhp = 1.8;
        m_rkn = 0.2;
        m_rf = 0.0;
        m_torso = 23 - 4*m_rhr-4*m_rhp-4*m_rkn-4*m_rf+5;

        m_lsr = m_rsr = m_lhr = m_rhr;
        m_lsp = m_rsp = m_lhp = m_rhp;
        m_leb = m_reb = m_lkn = m_rkn;
        m_lh = m_rh = m_lf = m_rf;

        M_total = (m_rhr+m_rhp+m_rkn+m_rf)*4+m_torso;

        P2HX = 0.224;
        P2HY = 0.079;
        ULEG = 0.225;
        LLEG = 0.225 + 0.015; // 0.015
        R2P = 0.068;
        xlim = (ULEG+LLEG)/2.0*1.5;
        ylim = (ULEG+LLEG)/2.0*1.5;

        //offsets
        offset_p2rs =   Vector3d(P2HX,-P2HY,0);
        offset_p2ls =   Vector3d(P2HX,+P2HY,0);
        offset_rs2rp =  Vector3d(0,-R2P,0);
        offset_ls2lp =  Vector3d(0,+R2P,0);
        offset_p2rh =   Vector3d(-P2HX,-P2HY,0);
        offset_p2lh =   Vector3d(-P2HX,+P2HY,0);
        offset_rh2rp =  Vector3d(0,-R2P,0);
        offset_lh2lp =  Vector3d(0,+R2P,0);
        offset_uleg =   Vector3d(0,0,-ULEG);
        offset_lleg =   Vector3d(0,0,-LLEG);

        c_torso     = Vector3d(0,0,0);
        c_lsr = Vector3d(0.00011,0.00283,0.00140);
        c_lsp = Vector3d(-0.00026,0.03369,-0.02516);
        c_leb = Vector3d(0.00144,0.0,-0.04022);

        c_lhr = F2B(c_lsr);
        c_lhp = (c_lsp);
        c_lkn = (c_leb);

        c_rsr = L2R(c_lsr);
        c_rsp = L2R(c_lsp);
        c_reb = L2R(c_leb);

        c_rhr = F2B(c_rsr);
        c_rhp = (c_rsp);
        c_rkn = (c_reb);

        I_torso = 0.001*0.001*0.001*0.001*
                Matrix3d(	70908096.75, 1976068.94, 9267259.42,
                       197068.94, 476924345.97, -149917.60,
                       9267259.42, -149917.60, 485757972.02);

        I_lsr = 0.001*0.001*0.001*0.001*
                Matrix3d(1541447.91, -13604.15, 6180.19,
                     -13604.15, 1976742.64, -83792.55,
                     6180.19, -83792.55, 1469855.63);

        I_lsp = 0.001*0.001*0.001*0.001*
                Matrix3d(11590515.71, 21740.84, 496432.79,
                     21740.84, 11996886.62, 1678318.13,
                     496432.79, 1678318.13, 2999047.65);


        I_leb =0.001*0.001*0.001*0.001*
                Matrix3d(586375.77, -42.53, 10702.72,
                     -42.53, 586750.08, -0.06,
                     10702.72, -0.069, 15962.16);

        I_lhr = F2B(I_lsr);
        I_lhp = (I_lsp);
        I_lkn = (I_leb);

        I_rsr = L2R(I_lsr);
        I_rsp = L2R(I_lsp);
        I_reb = L2R(I_leb);

        I_rhr = F2B(I_rsr);
        I_rhp = (I_rsp);
        I_rkn = (I_reb);
    }

//    MatrixNd Tmat(Matrix3d R, Vector3d p);

    Matrix3d getOneLegJacobian(Vector3d EncPos, int LR);

    Matrix3d getOneLegJacobian_Global(SensorInfo sensor, Vector3d JointPos, int LR);

    Matrix3d getOneLegJacobian_angleRef(Matrix3d angleRef, Vector3d JointPos, int LR);

    Vector3d FK_OneLeg_local(Vector3d legAngle, int LR);

    Vector3d FK_OneLeg_localize(RobotState state, Vector3d vec_global, int idx);

    Vector3d IK_OneLeg(Vector3d foot_pos, int LR);

    QuadJoint IK_FullBody(RobotState state);

    RobotState FK_FullBody(SensorInfo sensor);

    RobotState FK_FullBody_IMU(SensorInfo sensor);

    RobotState FK_FullBody_ACC(Vector4d ACC_quat, SensorInfo sensor);

    RobotState FK_foot(SensorInfo sensor, Vector3d pfoot, int idx_leg);

    RobotState FK_COM(SensorInfo sensor, Vector3d COM);

    RobotState FK_COM_IMU(SensorInfo sensor, Vector3d COM);

    RobotState FK_COM_ACC(Vector4d ACC_quat, SensorInfo sensor, Vector3d COM);
};

#endif // FUNC_KINEMATICS_H
