#include "hubodog5_general.h"

//extern SensorInfo sensor;
//extern hubodog5_general Quad;

#include <unsupported/Eigen/MatrixFunctions>
#include "qpSWIFT/include/Prime.h"
#include "RBSharedMemory.h"
extern pRBCORE_SHM sharedData;
#define zero_threshold 1e-8

void hubodog5_general::make_ineq(VectorNd _xop, MatrixNd _Rop, VectorNd _pfop)
{
    // Box constraint
//    Aineq_x = MatrixNd::Zero(4*leg_num,m);
//    bineq_x = VectorNd::Zero(4*leg_num,1);
//    m2 = int(Aineq_x.rows());
//    Vector3d pb = _xop.segment(0,3);
//    Vector3d r = Vector3d::Zero();
//    Matrix3d _RopT = _Rop.transpose();
//    for(int i=0; i<leg_num; i++)
//    {
//        r = _pfop.segment(i*3,3) - pb;
//        Aineq_x(i*4, 12+i*3) = proj_x.transpose()*_RopT*r;
//        Aineq_x(i*4+1, 12+i*3) = -proj_x.transpose()*_RopT*r;
//        Aineq_x(i*4+2, 12+i*3+1) = proj_y.transpose()*_RopT*r;
//        Aineq_x(i*4+3, 12+i*3+1) = -proj_y.transpose()*_RopT*r;
//        bineq_x.segment(i*4,2) = Vector2d(UB_pfx, UB_pfx);
//        bineq_x.segment(i*4+2,2) = Vector2d(UB_pfy, UB_pfy);
//    }

    // for test
    Aineq_x.setZero(1,m);
    bineq_x.setZero(1,1);
    m2 = int(Aineq_x.rows());
    Aineq_x(0,2) = 1.0;
    bineq_x[0] = 1.0;



    // Friction pyramid constraints
    Aineq_u.setZero(leg_num*6, n);
    bineq_u.setZero(leg_num*6, 1);
    n2 = int(Aineq_u.rows());

    for(int i=0; i<leg_num; i++)
    {
        Aineq_u.block(i*6,i*3,1,3) << nvec_mat.block(0,i*3,3,1).transpose() - mu*nvec_mat.block(0,i*3+2,3,1).transpose();
        Aineq_u.block(i*6+1,i*3,1,3) << -nvec_mat.block(0,i*3,3,1).transpose() - mu*nvec_mat.block(0,i*3+2,3,1).transpose();
        Aineq_u.block(i*6+2,i*3,1,3) << nvec_mat.block(0,i*3+1,3,1).transpose() - mu*nvec_mat.block(0,i*3+2,3,1).transpose();
        Aineq_u.block(i*6+3,i*3,1,3) << -nvec_mat.block(0,i*3+1,3,1).transpose() - mu*nvec_mat.block(0,i*3+2,3,1).transpose();
        Aineq_u.block(i*6+4,i*3,1,3) << -nvec_mat.block(0,i*3+2,3,1).transpose();
        Aineq_u.block(i*6+5,i*3,1,3) << nvec_mat.block(0,i*3+2,3,1).transpose();
        bineq_u(i*6+4) = 0;
        bineq_u(i*6+5) = Fz_max;
    }

//    // for test
//    Aineq_x = MatrixNd::Zero(1,m);
//    bineq_x = VectorNd::Zero(1,1);
//    m2 = 1;
//    Aineq_x(0,2) = 1.0;
//    bineq_x[0] = 1.0;

//    Aineq_u = MatrixNd::Zero(1,n);
//    bineq_u = VectorNd::Zero(1,1);
//    n2 = 1;
//    Aineq_x(0,2) = 1.0;
//    bineq_x[0] = 100;
}


void hubodog5_general::make_ABC(VectorNd _xop, VectorNd _uop, MatrixNd _Rop, VectorNd _pfop)
{
//    Aop = MatrixNd::Zero(m,m);
    Aop.setIdentity(m,m);
    Bop.setZero(m,leg_num*3);
    Cop.setZero(m,1);

    Vector3d pbop = Vector3d(_xop[0], _xop[1], _xop[2]);
    Vector3d wop = Vector3d(_xop[9], _xop[10], _xop[11]);

    Vector3d uop_sum = Vector3d::Zero();
    MatrixNd uop_mat = MatrixNd::Zero(3,12);
    MatrixNd r_mat = MatrixNd::Zero(3,12);

    for(int i=1;i<5;i++)
    {
        Vector3d uop_temp = Vector3d(_uop[i*3-3], _uop[i*3-2], _uop[i*3-1]);
        Vector3d pf_temp = Vector3d(_pfop[i*3-3], _pfop[i*3-2], _pfop[i*3-1]);
        Vector3d r_temp = pf_temp - pbop;
        uop_sum += uop_temp;
        r_mat.block(0, i*3-3, 3, 3) = vec2hat(r_temp);
        uop_mat.block(0, i*3-3, 3, 3) = vec2hat(uop_temp);
    }
    VectorNd torque = r_mat*_uop;
    Matrix3d wop_hat = vec2hat(wop);
    Matrix3d uop_sum_hat = vec2hat(uop_sum);
    Matrix3d inertia_wop_hat = vec2hat(inertia_mat*wop);
    MatrixNd kron_eye_Rop = mat_kronecker(Matrix3d::Identity(), _Rop);
    MatrixNd kron_eye_RopT = mat_kronecker(Matrix3d::Identity(), _Rop.transpose());

    // Define factors in ABC matrix
    VectorNd C1_0 = VectorNd::Zero(9,1);
    MatrixNd C1_gamma = MatrixNd::Zero(9,3);
    MatrixNd C1_w = MatrixNd::Zero(9,3);

    VectorNd C2_0 = VectorNd::Zero(3,1);
    MatrixNd C2_pb = MatrixNd::Zero(3,3);
    MatrixNd C2_gamma = MatrixNd::Zero(3,3);
    MatrixNd C2_w = MatrixNd::Zero(3,3);
    MatrixNd C2_u = MatrixNd::Zero(3,12);
    MatrixNd C2_pf = MatrixNd::Zero(3,12);

    C1_0 = mat_reshape(_Rop*wop_hat, 9, 1) - kron_eye_Rop*Nmat*wop;
    C1_gamma = -kron_eye_Rop*Nmat*wop_hat + mat_kronecker(Matrix3d::Identity(), _Rop*wop_hat)*Nmat;
    C1_w = kron_eye_Rop*Nmat;

    C2_0 = _Rop.transpose()*(torque - uop_sum_hat*pbop + uop_mat*_pfop) - inertia_wop_hat*wop;
    C2_pb = _Rop.transpose()*uop_sum_hat;
    C2_gamma = mat_kronecker(Matrix3d::Identity(), torque.transpose())*Nmat - (inertia_wop_hat-wop_hat*inertia_mat)*wop_hat;
    C2_w = inertia_wop_hat - wop_hat*inertia_mat;
    C2_u = _Rop.transpose()*r_mat;
    C2_pf = -_Rop.transpose()*uop_mat;

    // Define Aop, Bop, Cop matrix
//    cout << "Nmat_inv = " << Nmat_inv(2,1) << endl;
    Aop.block(0,3,3,3) << Ts_mpc * Matrix3d::Identity();
    Aop.block(6,6,3,3) << Matrix3d::Identity() + Nmat_inv*(kron_eye_RopT*C1_gamma)*Ts_mpc;
    Aop.block(6,9,3,3) << Nmat_inv*(kron_eye_RopT*C1_w)*Ts_mpc;
    Aop.block(9,0,3,3) << Imat_inverse*C2_pb*Ts_mpc;
    Aop.block(9,6,3,3) << Imat_inverse*C2_gamma*Ts_mpc;
    Aop.block(9,9,3,3) << Matrix3d::Identity() + Imat_inverse*C2_w*Ts_mpc;
    Aop.block(9,12,3,12) << Imat_inverse*C2_pf*Ts_mpc;

    Bop.block(0,0,3,n) << pow(Ts_mpc, 2)*eye_mat/(2*Kinematics.M_total);
    Bop.block(3,0,3,n) << Ts_mpc*eye_mat/Kinematics.M_total;
    Bop.block(9,0,3,n) << Imat_inverse*C2_u*Ts_mpc;

    Cop.block(0,0,3,1) << -pow(Ts_mpc,2)*gravity/2;
    Cop.block(3,0,3,1) << -Ts_mpc*gravity;
    Cop.block(9,0,3,1) << Imat_inverse*(C2_0 - C2_u*_uop)*Ts_mpc;
    Cop.block(6,0,3,1) << Nmat_inv*kron_eye_RopT*C1_0*Ts_mpc;
}

MatrixNd hubodog5_general::selection_mat_MPC(VectorNd boolean_prev, VectorNd boolean_next)
{
    MatrixNd mat = MatrixNd::Zero(24,12);
    for(int i=0;i<4;i++)
    {
        if(boolean_next[i]-boolean_prev[i]>0.5)
        {
            mat.block(12+i*3, i*3, 3,3) = Matrix3d::Identity();
        }
    }

    return mat;
}


VectorNd hubodog5_general::find_index(VectorNd input_vec, int value)
{
    int k = 0;
    for(int j=0; j<input_vec.size(); j++)
    {
        if(int(input_vec[j]) == value){k++;}
    }

    VectorNd k_vec;
    if(k!=0)
    {
        k_vec = VectorNd::Zero(k,1);

        k = 0;
        for(int j=0; j<input_vec.size(); j++)
        {
            if(int(input_vec[j]) == value){k_vec[k] = j; k++;}
        }
    }
    else {
        k_vec = VectorNd::Zero(2,1);
    }

    return k_vec;
}


//////////////////////
// qpSWIFT function //
void hubodog5_general::get_Air(MatrixNd mat, qp_int* Air) // row indices of Apr matrix
{
    int cnt=0;
    for(int i=0; i<int(mat.cols()); i++)
    {
        for(int j=0; j<int(mat.rows()); j++)
        {
            if(fabs(mat(j,i)) > zero_threshold)
            {
                Air[cnt] = long(j);
                cnt++;
            }
        }
    }
}

void hubodog5_general::get_Ajc(MatrixNd mat,qp_int* Ajc)
{
    int cnt = 0;
    int indexing = 1;

    Ajc[0] = 0;
    for(int i=0; i<int(mat.cols()); i++)
    {
        cnt = 0;
        for(int j=0; j<int(mat.rows()); j++)
        {

            if(fabs(mat(j,i)) > zero_threshold){cnt++;}
        }
        Ajc[indexing] = Ajc[indexing-1] + cnt;
        indexing++;

    }
}

void hubodog5_general::get_Apr(MatrixNd mat, qp_real* Apr)
{
    int cnt = 0;
    for(int i=0; i<int(mat.cols()); i++)
    {
        for(int j=0; j<int(mat.rows()); j++)
        {
            if(fabs(mat(j,i)) > zero_threshold)
            {
                Apr[cnt] = mat(j,i);
//                cout << "Apr["<<cnt<<"] = " << Apr[cnt] << endl;
//                cout << "mat("<<j<<","<<i<<") = " << mat(j,i) << endl;

                cnt++;
            }
        }
    }
}

void hubodog5_general::vec_to_realtype(VectorNd vec, qp_real* vec_pnt)
{
    for(int i=0; i<int(vec.size()); i++)
    {
        vec_pnt[i] = vec[i];
    }
}

void hubodog5_general::CCS_CLEANUP()
{
    cout << "CCS CLEANUP." << endl;
    free(Pjc);  free(Pir);  free(Ppr);
    free(Ajc);
    free(Air);  free(Apr);
    free(Gjc);  free(Gir);  free(Gpr);
    free(qpswift_c);    free(qpswift_h);    free(qpswift_b);
    std::cout << "CCS CLEANUP FINISHED" << std::endl;
}






VectorNd hubodog5_general::qpSwift_MPC()
{
//    double tset = 0.0, tsol = 0.0, t_ldl = 0.0;
    timer qpswift_CCS_timer;
    tic(&qpswift_CCS_timer);

    if(isMPCstarted)
    {
        cout << "CCS Initialized." << endl;
        int memory_multisize = 2;
        H = 2*Qmat;
        F = -2*Qmat.transpose()*ys;

        const int mat_size_Pjc = H.cols()+1;
        Pjc = (qp_int*)MALLOC(mat_size_Pjc*sizeof(qp_int)*memory_multisize);
        int cnt=0;
        for(int i=0; i<int(H.cols()); i++)
        {for(int j=0; j<int(H.rows()); j++){if(fabs(H(j,i)) > zero_threshold){cnt++;}}}
        const int mat_size_Pir = cnt;
        Pir = (qp_int*)MALLOC(mat_size_Pir*sizeof(qp_int)*memory_multisize);
        const int mat_size_Ppr = cnt;
        Ppr = (qp_real*)MALLOC(mat_size_Ppr*sizeof(qp_real)*memory_multisize);

        const int mat_size_Ajc = Aeq.cols()+1;
        Ajc = (qp_int*)MALLOC(mat_size_Ajc*sizeof(qp_int)*memory_multisize);

        cnt=0;
        for(int i=0; i<int(Aeq.cols()); i++)
        {for(int j=0; j<int(Aeq.rows()); j++){if(fabs(Aeq(j,i)) > zero_threshold){cnt++;}}}
        const int mat_size_Air = cnt;
        Air = (qp_int*)MALLOC(mat_size_Air*sizeof(qp_int)*memory_multisize);
        const int mat_size_Apr = cnt;
//        cout << "Apr size(init) = " << mat_size_Apr << endl;
        Apr = (qp_real*)MALLOC(mat_size_Apr*sizeof(qp_real)*memory_multisize);

        const int mat_size_Gjc = Aineq.cols()+1;
        Gjc = (qp_int*)MALLOC(mat_size_Gjc*sizeof(qp_int)*memory_multisize);
        cnt=0;
        for(int i=0; i<int(Aineq.cols()); i++)
        {for(int j=0; j<int(Aineq.rows()); j++){if(fabs(Aineq(j,i)) > zero_threshold){cnt++;}}}
        const int mat_size_Gir = cnt;
        Gir = (qp_int*)MALLOC(mat_size_Gir*sizeof(qp_int)*memory_multisize);
        const int mat_size_Gpr = cnt;
        Gpr = (qp_real*)MALLOC(mat_size_Gpr*sizeof(qp_real)*memory_multisize);

        const int mat_size_c = int(F.size());
        qpswift_c = (qp_real*)MALLOC(mat_size_c*sizeof(qp_real)*memory_multisize);
        const int mat_size_h = int(bineq.size());
        qpswift_h = (qp_real*)MALLOC(mat_size_h*sizeof(qp_real)*memory_multisize);
        const int mat_size_b = int(beq.size());
        qpswift_b = (qp_real*)MALLOC(mat_size_b*sizeof(qp_real)*memory_multisize);

        // P mat (weight)
        get_Ajc(H, Pjc);
        get_Air(H, Pir);
        get_Apr(H, Ppr);

        // G mat (ineq)
        get_Ajc(Aineq, Gjc);
        get_Air(Aineq, Gir);
        get_Apr(Aineq, Gpr);

        // vectors
        vec_to_realtype(bineq, qpswift_h);

        optvar_size = long(H.rows());
        eq_size = long(Aeq.rows());
        ineq_size = long(Aineq.rows());
        sigma_d = 0.0;

        isMPCstarted = false;
        cout << "CCS Setup done." << endl;
    }



//    MatrixNd H = 2*Qmat;
//    MatrixNd F = -2*Qmat.transpose()*ys;
//    // P mat
//    const int mat_size_Pjc = H.cols()+1;
//    qp_int *Pjc;
//    Pjc = (qp_int*)MALLOC(mat_size_Pjc*sizeof(qp_int));
//    get_Ajc(H, Pjc);
//    int cnt=0;
//    for(int i=0; i<int(H.cols()); i++)
//    {for(int j=0; j<int(H.rows()); j++){if(fabs(H(j,i)) != 0){cnt++;}}}
//    const int mat_size_Pir = cnt;
//    qp_int *Pir;
//    Pir = (qp_int*)MALLOC(mat_size_Pir*sizeof(qp_int));
//    get_Air(H, Pir);
//    const int mat_size_Ppr = cnt;
//    qp_real *Ppr;
//    Ppr = (qp_real*)MALLOC(mat_size_Ppr*sizeof(qp_real));
//    get_Apr(H, Ppr);


//    // A mat
//    const int mat_size_Ajc = Aeq.cols()+1;
//    qp_int *Ajc;
//    Ajc = (qp_int*)MALLOC(mat_size_Ajc*sizeof(qp_int));
//    get_Ajc(Aeq, Ajc);
//    cnt=0;
//    for(int i=0; i<int(Aeq.cols()); i++)
//    {for(int j=0; j<int(Aeq.rows()); j++){if(fabs(Aeq(j,i)) != 0){cnt++;}}}
//    const int mat_size_Air = cnt;
//    qp_int *Air;
//    Air = (qp_int*)MALLOC(mat_size_Air*sizeof(qp_int));
//    get_Air(Aeq, Air);
//    const int mat_size_Apr = cnt;
//    qp_real *Apr;
//    Apr = (qp_real*)MALLOC(mat_size_Apr*sizeof(qp_real)); // if removed, crash.. why?
//    get_Apr(Aeq, Apr);

//    // move sementic
//    // memory manage class
//    // smart pointer


//    // G mat
//    const int mat_size_Gjc = Aineq.cols()+1;
//    qp_int *Gjc;
//    Gjc = (qp_int*)MALLOC(mat_size_Gjc*sizeof(qp_int));
//    get_Ajc(Aineq, Gjc);
//    cnt=0;
//    for(int i=0; i<int(Aineq.cols()); i++)
//    {for(int j=0; j<int(Aineq.rows()); j++){if(fabs(Aineq(j,i)) != 0){cnt++;}}}
//    const int mat_size_Gir = cnt;
//    qp_int *Gir;
//    Gir = (qp_int*)MALLOC(mat_size_Gir*sizeof(qp_int));
//    get_Air(Aineq, Gir);
//    const int mat_size_Gpr = cnt;
//    qp_real *Gpr;
//    Gpr = (qp_real*)MALLOC(mat_size_Gpr*sizeof(qp_real));
//    get_Apr(Aineq, Gpr);

//    // vectors
//    const int mat_size_c = int(F.size());
////    qp_real qpswift_c[mat_size_c];
//    qp_real *qpswift_c;
//    qpswift_c = (qp_real*)MALLOC(mat_size_c*sizeof(qp_real));
//    vec_to_realtype(F, qpswift_c);
//    const int mat_size_h = int(bineq.size());
////    qp_real qpswift_h[mat_size_h];
//    qp_real *qpswift_h;
//    qpswift_h = (qp_real*)MALLOC(mat_size_h*sizeof(qp_real));
//    vec_to_realtype(bineq, qpswift_h);

//    const int mat_size_b = int(beq.size());
////    qp_real qpswift_b[mat_size_b];
//    qp_real *qpswift_b;
//    qpswift_b = (qp_real*)MALLOC(mat_size_b*sizeof(qp_real));
//    vec_to_realtype(beq, qpswift_b);


//    // other constants
//    qp_int optvar_size = long(H.rows());
//    qp_int ineq_size = long(Aineq.rows());
//    qp_int eq_size = long(Aeq.rows());
//    qp_real sigma_d = 0.0;





    F = -2*Qmat.transpose()*ys;

    get_Ajc(Aeq, Ajc);
//    int cnt=0;
//    for(int i=0; i<int(Aeq.cols()); i++)
//    {for(int j=0; j<int(Aeq.rows()); j++){if(fabs(Aeq(j,i)) > zero_threshold){cnt++;}}}
//    const int mat_size_Air = cnt;
//    Air = (qp_int*)MALLOC(mat_size_Air*sizeof(qp_int));
    get_Air(Aeq, Air);
//    const int mat_size_Apr = cnt;
//    qp_real *Apr;
//    Apr = (qp_real*)MALLOC(mat_size_Apr*sizeof(qp_real)); // if removed, crash.. why?
    get_Apr(Aeq, Apr);

    vec_to_realtype(F, qpswift_c);
    vec_to_realtype(beq, qpswift_b);


    // test
    // P mat (weight)
    get_Ajc(H, Pjc);
    get_Air(H, Pir);
    get_Apr(H, Ppr);

    // G mat (ineq)
    get_Ajc(Aineq, Gjc);
    get_Air(Aineq, Gir);
    get_Apr(Aineq, Gpr);

    // vectors
    vec_to_realtype(bineq, qpswift_h);

    optvar_size = long(H.rows());
    eq_size = long(Aeq.rows());
    ineq_size = long(Aineq.rows());
    //


    double CCStime_temp = toc(&qpswift_CCS_timer);

//    cout << "Pjc size = " << mat_size_Pjc << endl;
//    cout << "Pir size = " << mat_size_Pir << endl;
//    cout << "Ppr size = " << mat_size_Ppr << endl;
//    cout << "Ajc size = " << mat_size_Ajc << endl;
//    cout << "Air size = " << mat_size_Air << endl;
//    cout << "Apr size = " << mat_size_Apr << endl;
//    cout << "Gjc size = " << mat_size_Gjc << endl;
//    cout << "Gir size = " << mat_size_Gir << endl;
//    cout << "Gpr size = " << mat_size_Gpr << endl;
//    cout << "qpswift_c size = " << mat_size_c << endl;
//    cout << "qpswift_h size = " << mat_size_h << endl;
//    cout << "qpswift_b size = " << mat_size_b << endl;



    QPswift* myQP;
//myQP = QP_SETUP(n, m, p, Pjc, Pir, Ppr, Ajc, Air, Apr, Gjc, Gir, Gpr, c, h, b, sigma_d, P);

//    cout << "QPswift Setup" << endl;
    myQP = QP_SETUP(optvar_size, ineq_size, eq_size, Pjc, Pir, Ppr, Ajc, Air, Apr, Gjc, Gir, Gpr, qpswift_c, qpswift_h, qpswift_b, sigma_d, nullptr);


//    cout << "Setup Done. Do QP_SOLVE" << endl;
    qp_int ExitCode = QP_SOLVE(myQP);


//    cout << "Solve done" << endl;

//    if (myQP != NULL)
//        printf("Setup Time     : %f ms\n", myQP->stats->tsetup*1000.0);
//    if (ExitCode == QP_OPTIMAL){
//        printf("Solve Time     : %f ms\n", (myQP->stats->tsolve + myQP->stats->tsetup)*1000.0);
//        printf("KKT_Solve Time : %f ms\n", myQP->stats->kkt_time*1000.0);
//        printf("LDL Time       : %f ms\n", myQP->stats->ldl_numeric*1000.0);
//        printf("Diff	       : %f ms\n", (myQP->stats->kkt_time - myQP->stats->ldl_numeric)*1000.0);
//        printf("Iterations     : %d\n", myQP->stats->IterationCount);
//        printf("Optimal Solution Found\n");
//    }
//    if (ExitCode == QP_MAXIT){
//        printf("Solve Time     : %f ms\n", myQP->stats->tsolve*1000.0);
//        printf("KKT_Solve Time : %f ms\n", myQP->stats->kkt_time*1000.0);
//        printf("LDL Time       : %f ms\n", myQP->stats->ldl_numeric*1000.0);
//        printf("Diff	       : %f ms\n", (myQP->stats->kkt_time - myQP->stats->ldl_numeric)*1000.0);
//        printf("Iterations     : %d\n", myQP->stats->IterationCount);
//        printf("Maximum Iterations reached\n");
//    }

//    if (ExitCode == QP_FATAL){
//        printf("Unknown Error Detected\n");
//    }

//    if (ExitCode == QP_KKTFAIL){
//        printf("LDL Factorization fail\n");
//    }

    VectorNd result = VectorNd::Zero(ys.size(),1);

    for(int i=0; i<optvar_size; i++)
    {
        result[i] = myQP->x[i];
    }

    // GUI info
    sharedData->mpc_flag = int(ExitCode);
    sharedData->mpc_iter = myQP->stats->IterationCount;
    sharedData->mpc_cnt = cnt_MPC;
    sharedData->mpc_tsim = T_sim;
    sharedData->mpc_tsolve = 1/(myQP->stats->tsolve + myQP->stats->tsetup);



    QP_CLEANUP(myQP);
//    free(Pjc);  free(Pir);  free(Ppr);
//    free(Ajc);  free(Air);  free(Apr);
//    free(Gjc);  free(Gir);  free(Gpr);
//    free(qpswift_c);    free(qpswift_h);    free(qpswift_b);
//    CCS_CLEANUP();
//    free(Air);
//    free(Apr);

    sharedData->mpc_tCCS = 1/CCStime_temp;
//    cout << "clean finished" << endl;
    return result;
}


