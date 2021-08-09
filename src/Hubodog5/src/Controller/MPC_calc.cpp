#include "hubodog5_general.h"
#include "func_general.h"
#include <unsupported/Eigen/MatrixFunctions>

void hubodog5_general::MPC_calc()
{
//    cout << "test" << endl;
    if(isMPCON==false)
    {
        return;
    }

    int lock_cnt = 0;
    while(isSamplingLock)
    {
        // wait
        lock_cnt++;
        if(lock_cnt>lock_size)
            break;
    }
    isMPCLock = true;

    if(cnt_MPC==0)
    {
        cout << "MPC Initial Setup." << endl;
        make_ineq(x, R, x.segment(12,12));
    }

    timer mpc_timer, mpc_timer_setup;
    tic(&mpc_timer);
    tic(&mpc_timer_setup);

    stepping = 0;

    swing_num = 0;
    for(int i=0; i<4; i++) // find # of swing foot during horizon
    {
        for(int j=0; j<Nhorizon; j++)
        {
            if(gait_boolean(i,j)<0.5)
            {
                swing_num++;
            }
        }
    }
    total_stepping = 0; // j
    for(int i=0; i<Nhorizon; i++)
    {
        if(eta_new[i]>0.5)
        {
            total_stepping++;
        }
    }

    ucurrent = uref;

    // add if slope changes (later)
//    nx_RH = nx_LH = nx_RF = nx_LF = Vector3d(1,0,0);
//    ny_RH = ny_LH = ny_RF = ny_LF = Vector3d(0,1,0);
//    nz_RH = nz_LH = nz_RF = nz_LF = Vector3d(0,0,1);
//    nvec_mat.block(0,0,3,1) = nx_RH;
//    nvec_mat.block(0,1,3,1) = ny_RH;
//    nvec_mat.block(0,2,3,1) = nz_RH;
//    nvec_mat.block(0,3,3,1) = nx_LH;
//    nvec_mat.block(0,4,3,1) = ny_LH;
//    nvec_mat.block(0,5,3,1) = nz_LH;
//    nvec_mat.block(0,6,3,1) = nx_RF;
//    nvec_mat.block(0,7,3,1) = ny_RF;
//    nvec_mat.block(0,8,3,1) = nz_RF;
//    nvec_mat.block(0,9,3,1) = nx_LF;
//    nvec_mat.block(0,10,3,1) = ny_LF;
//    nvec_mat.block(0,11,3,1) = nz_LF;

    make_ABC(x, ucurrent, R, x.segment(12,12));

    // init part
    int size_margin = 0;
    Aeq.setZero((m*Nhorizon+swing_num*3+Nhorizon*4)*1+size_margin, (m+n)*Nhorizon+pf_num*total_stepping);
    beq.setZero((m*Nhorizon+swing_num*3+Nhorizon*4)*1+size_margin, 1);
    Aineq.setZero((m2+n2)*Nhorizon*1+size_margin, (m+n)*Nhorizon+pf_num*total_stepping);
    bineq.setZero((m2+n2)*Nhorizon*1+size_margin, 1);
    //

    Aeq.block(0,0,m,m).setIdentity(m,m);
    Aeq.block(0,m,m,n) << -Bop;
    beq.segment(0,m) << Cop+Aop*x;

    MatrixNd selection;
    if(eta_new[0]>0.5 && total_stepping>0)
    {
        stepping++;
        selection = selection_mat_MPC(gait_current, gait_boolean.block(0,0,4,1));
        Aeq.block(0, (m+n)*Nhorizon, m, pf_num) << -Aop*selection;
    }

    Aineq.block(0,0,m2,m) << Aineq_x;
    Aineq.block(m2,m,n2,n) << Aineq_u;
    bineq.segment(0,m2) << bineq_x;
    bineq.segment(m2,n2) << bineq_u;

    Matrix3d Rop_;
    for(int i=1;i<Nhorizon;i++)
    {
        xop = xref_vec.block(0,i,m,1);
        Rop_ = mat_reshape(Rref_vec.block(0,i,9,1),3,3);
        uop = uref_vec.block(0,i,n,1);
        pfop = xref_vec.block(12,i,n,1);
        make_ABC(xop, uop, Rop_, pfop);
//        make_ineq(xop, Rop_, pfop);
        Aeq.block(i*m, (i-1)*(m+n), m, m) << -Aop;
        Aeq.block(i*m, (i-1)*(m+n)+m+n, m, m).setIdentity(m,m);
        Aeq.block(i*m, (i-1)*(m+n)+m+n+m, m, n) << -Bop;
        if(eta_new[i]>0.5 && total_stepping>0)
        {
            stepping++;
            selection = selection_mat_MPC(gait_boolean.block(0,i-1,4,1), gait_boolean.block(0,i,4,1));
            Aeq.block(i*m, (m+n)*Nhorizon+(stepping-1)*pf_num, m, pf_num) << -Aop*selection;
        }
        beq.segment(i*m,m) << Cop;
        Aineq.block((m2+n2)*i, (m+n)*i, m2, m) << Aineq_x;
        Aineq.block((m2+n2)*i+m2, (m+n)*i+m, n2, n) << Aineq_u;
        bineq.segment((m2+n2)*i,m2) << bineq_x;
        bineq.segment((m2+n2)*i+m2,n2) << bineq_u;
    }

    // Swing leg & plane-contact constraints
    int swing_i = 0;
    int k = 0;
    VectorNd k_vec;
    MatrixNd eye_swing;
    for(int i=1; i<=Nhorizon; i++)
    {
        if(gait_boolean.block(0,i-1,4,1).norm() < 1.9 && swing_num > 0)
        {
            k_vec = find_index(gait_boolean.block(0,i-1,4,1), 0);
            k = int(k_vec.size());

            eye_swing = MatrixNd::Zero(k*3, leg_num*3);
            for(int j=0; j<k; j++)
            {
                eye_swing.block(j*3, int(k_vec[j])*3, 3, 3) = Matrix3d::Identity();
            }

            Aeq.block(m*Nhorizon+swing_i*3, (i-1)*(m+n)+m, k*3, leg_num*3) << eye_swing;
            swing_i += k;
        }

        // Surface-contact constraints
        // foot : [RH(hind) - LH - RF(front) - LF]
        // init part
        Aeq.block(m*Nhorizon+swing_num*3+i*4-4, (i-1)*(m+n)+12, 1, 3) << nz_RH.transpose();
        Aeq.block(m*Nhorizon+swing_num*3+i*4-3, (i-1)*(m+n)+12+3, 1, 3) << nz_LH.transpose();
        Aeq.block(m*Nhorizon+swing_num*3+i*4-2, (i-1)*(m+n)+12+6, 1, 3) << nz_RF.transpose();
        Aeq.block(m*Nhorizon+swing_num*3+i*4-1, (i-1)*(m+n)+12+9, 1, 3) << nz_LF.transpose();
//        beq.segment(m*Nhorizon+swing_num*3+i*4-4, 4) = VectorNd::Zero(4,1);
    }

    ys.setZero((m+n)*Nhorizon+pf_num*total_stepping+size_margin);
    Matrix3d logm_temp;
    for(int i=1; i<=Nhorizon; i++)
    {
        Rop_ = mat_reshape(Rref_vec.block(0,i-1,9,1),3,3);
        ys.segment((m+n)*(i-1),6) << xref_vec.block(0,i-1,6,1);
        logm_temp = Rop_.transpose()*R;
        ys.segment((m+n)*(i-1)+6,3) << -hat2vec(logm_temp.log());
        ys.segment((m+n)*(i-1)+9,3) << xref_vec.block(9,i-1,3,1);
        ys.segment((m+n)*(i-1)+12,n) << xref_vec.block(12,i-1,n,1);

        ys.segment((m+n)*(i-1)+m,n) = sharedData->uref_gain * uref_vec.block(0,i-1,n,1); // after
//        ys.segment((m+n)*(i-1)+m,n) = ucurrent; // not good
    }

    Qvec.setZero((m+n)*Nhorizon+pf_num*total_stepping+size_margin);
    for(int i=0; i<Nhorizon; i++)
    {
        Qvec.segment((m+n)*i, m+n) << weight_mpc.segment(0,m+n);
    }
    Qvec.segment((m+n)*Nhorizon, pf_num*total_stepping) << weight_mpc[m+n]*VectorNd::Ones(pf_num*total_stepping,1);

    Qmat.setZero(Qvec.size(), Qvec.size());
    for(int i=0; i<Qvec.size(); i++)
    {
        Qmat(i,i) = Qvec[i];
    }

    isMPCLock = false;

    sharedData->mpc_timer_setup = 1/toc(&mpc_timer_setup);

    VectorNd result = qpSwift_MPC();

    for(int i=0; i<12; i++)
    {
        if(total_stepping>0)
        {
            dpf_new[i] = result[(m+n)*Nhorizon+i];
        }
        else{
            dpf_new[i] = 0;
        }
        sharedData->mpc_dpf[i] = dpf_new[i];
    }

    uref = mpc_gain * result.segment(m,12);
    for(int i=0; i<n; i++){sharedData->mpc_GRF[i] = uref[i];}
    if(isMPCON)
        cnt_MPC++;
    sharedData->mpc_timer_total = 1/toc(&mpc_timer);



}
