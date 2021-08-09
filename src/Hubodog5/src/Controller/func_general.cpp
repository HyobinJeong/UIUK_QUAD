#include "func_general.h"

Matrix3d quat2R(Vector4d quat)
{
    Matrix3d R;
    double w=quat[0], x=quat[1], y=quat[2], z=quat[3];
    double xx = 2*x*x, yy = 2*y*y, zz = 2*z*z;
    double wx = 2*w*x, wy = 2*w*y, wz = 2*w*z;
    double xy = 2*x*y, yz = 2*y*z, zx = 2*z*x;

    R(0,0) = 1.0f-yy-zz; R(0,1) =      xy-wz; R(0,2) =      zx+wy;
    R(1,0) =      xy+wz; R(1,1) = 1.0f-xx-zz; R(1,2) =      yz-wx;
    R(2,0) =      zx-wy; R(2,1) =      yz+wx; R(2,2) = 1.0f-xx-yy;
    return R;
}

Matrix3d Euler2R(Vector3d e)
{
    // z-y-x Euler angle to Rotation Matrix
    Matrix3d R;
    double r = e[0], p = e[1], y = e[2];
    double cx = cos(r), cy = cos(p), cz = cos(y);
    double sx = sin(r), sy = sin(p), sz = sin(y);

    R(0,0) = cy*cz; R(0,1) = -cx*sz + sx*sy*cz; R(0,2) =  sx*sz + cx*sy*cz;
    R(1,0) = cy*sz; R(1,1) =  cx*cz + sx*sy*sz; R(1,2) = -sx*cz + cx*sy*sz;
    R(2,0) =   -sy; R(2,1) =             sx*cy; R(2,2) =             cx*cy;
    return R;
}

Matrix3d rotate_3D(Vector3d axis, double angle)
{
    Matrix3d R;
    double c = cos(angle), s = sin(angle);
    double C = 1-c;
    double l = axis.norm();
    double x = axis[0]/l, y = axis[1]/l, z = axis[2]/l;
    R(0,0) =   c + x*x*C; R(0,1) = x*y*C - z*s; R(0,2) = x*z*C + y*s;
    R(1,0) = y*x*C + z*s; R(1,1) =   c + y*y*C; R(1,2) = y*z*C - x*s;
    R(2,0) = z*x*C - y*s; R(2,1) = z*y*C + x*s; R(2,2) =   c + z*z*C;
    return R;
}

Vector4d R2Quat(Matrix3d m) {
    Vector4d q;
    q.w() = 0.5f*sqrt(1.0f+m(0,0)+m(1,1)+m(2,2));
    q.x() = sign(m(2,1)-m(1,2))*0.5f*sqrtp(1+m(0,0)-m(1,1)-m(2,2));
    q.y() = sign(m(0,2)-m(2,0))*0.5f*sqrtp(1-m(0,0)+m(1,1)-m(2,2));
    q.z() = sign(m(1,0)-m(0,1))*0.5f*sqrtp(1-m(0,0)-m(1,1)+m(2,2));
    if(q.w()*q.w() + q.x()*q.x() + q.y()*q.y() + q.z()*q.z() <0.001)
    {q.w() = 1;}
    return q;
}

MatrixNd diagonalize(VectorNd vector)
{
    int size = vector.size();
    MatrixNd matrix = MatrixNd::Zero(size, size);
    for(int i=0; i<size; i++)
    {
        matrix(i,i) = vector(i);
    }

    return matrix;
}

MatrixNd Tmat(Vector3d p, Matrix3d R)
{
    MatrixNd Tmat = MatrixNd::Zero(4,4);
    Tmat.block(0,0,3,3) << R;
    Tmat.block(0,3,3,1) << p;
    Tmat(3,3) = 1.0;
    return Tmat;
}

MatrixNd Tmat_inv(MatrixNd Tmat)
{
    MatrixNd Tmat_inv = MatrixNd::Zero(4,4);
    Matrix3d R_inv = Tmat.block(0,0,3,3).inverse();
    Vector3d p = Tmat.block(0,3,3,1);
    Tmat_inv.block(0,0,3,3) << R_inv;
    Tmat_inv.block(0,3,3,1) << -R_inv*p;

    return Tmat_inv;
}

Matrix3d vec2hat(Vector3d vec)
{
    Matrix3d hatMat = Matrix3d::Zero();
    hatMat(0,1) = -vec[2];
    hatMat(0,2) = vec[1];
    hatMat(1,0) = vec[2];
    hatMat(1,2) = -vec[0];
    hatMat(2,0) = -vec[1];
    hatMat(2,1) = vec[0];

    return hatMat;
}

Vector3d hat2vec(Matrix3d mat)
{
    Vector3d vec = Vector3d(mat(2,1), mat(0,2), mat(1,0));
    return vec;
}

MatrixNd mat_reshape(MatrixNd mat, int row, int col)
{
    int row0 = mat.rows();
    int col0 = mat.cols();
    VectorNd mat_expanded = VectorNd::Zero(mat.size());
    int cnt_temp = 0;
    for(int i=0;i<col0;i++)
    {
        for(int j=0;j<row0;j++)
        {
            mat_expanded(cnt_temp) = mat(j,i);
            cnt_temp++;
        }
    }

    cnt_temp = 0;
    MatrixNd mat_new = MatrixNd::Zero(row, col);
    for(int i=0;i<col;i++)
    {
        for(int j=0;j<row;j++)
        {
            mat_new(j,i) = mat_expanded(cnt_temp);
            cnt_temp++;
        }
    }

    return mat_new;
}

MatrixNd mat_repeat(MatrixNd mat, int row, int col)
{
    int row0 = mat.rows();
    int col0 = mat.cols();
    MatrixNd mat_out = MatrixNd::Zero(row0*row, col0*col);

    for(int i=0; i<row; i++)
    {
        for(int j=0; j<col; j++)
        {
            mat_out.block(i*row0, j*col0, row0, col0) = mat;
        }
    }
    return mat_out;
}

MatrixNd mat_kronecker(MatrixNd A, MatrixNd B)
{
    int Arow = A.rows();
    int Acol = A.cols();
    int Brow = B.rows();
    int Bcol = B.cols();

    // kronecker product
    MatrixNd mat_new = MatrixNd::Zero(Arow*Brow, Acol*Bcol);

    for(int i=0; i<Arow; i++)
    {
        for(int j=0; j<Acol; j++)
        {
            mat_new.block(i*Brow, j*Bcol, Brow, Bcol) = A(i,j)*B;
        }
    }
    return mat_new;
}

Vector3d bezier_calc(double t_now, double t_end, Vector3d xdxddx_start, Vector3d xdxddx_end)
{
    VectorNd coeff, xdxddx;
    Vector3d xdxddx_ref;
    double t_temp = t_now/t_end;
    double x_next, dx_next, ddx_next;

    xdxddx = VectorNd::Zero(6,1);
    xdxddx[0] = xdxddx_start[0];
    xdxddx[2] = xdxddx_start[1];
    xdxddx[4] = xdxddx_start[2];
    xdxddx[1] = xdxddx_end[0];
    xdxddx[3] = xdxddx_end[1];
    xdxddx[5] = xdxddx_end[2];

    coeff = bezier_coeff(xdxddx, 5);
    x_next = bezier_polynomial_x(coeff, t_temp);
    dx_next = bezier_polynomial_dx(coeff, t_temp);
    ddx_next = bezier_polynomial_ddx(coeff, t_temp);

    xdxddx_ref = Vector3d(x_next, dx_next, ddx_next);
    return xdxddx_ref;
}

Vector3d bezier_calc_3rd(double t_now, double t_end, Vector3d xdxddx_start, Vector3d xdxddx_end)
{
    VectorNd coeff, xdxddx;
    Vector3d xdxddx_ref;
    double t_temp = t_now/t_end;
    double x_next, dx_next, ddx_next;

    xdxddx = VectorNd::Zero(6,1);
    xdxddx[0] = xdxddx_start[0];
    xdxddx[2] = xdxddx_start[1];
    xdxddx[4] = xdxddx_start[2];
    xdxddx[1] = xdxddx_end[0];
    xdxddx[3] = xdxddx_end[1];
    xdxddx[5] = xdxddx_end[2];

    coeff = bezier_coeff(xdxddx, 3);
    x_next = bezier_polynomial_x(coeff, t_temp);
    dx_next = bezier_polynomial_dx(coeff, t_temp);
    ddx_next = bezier_polynomial_ddx(coeff, t_temp);

    xdxddx_ref = Vector3d(x_next, dx_next, ddx_next);
    return xdxddx_ref;
}

Vector3d bezier_calc_2rd(double t_now, double t_end, Vector3d xdxddx_start, Vector3d xdxddx_end)
{
    Vector3d x_ref;
    double t_temp = t_now/t_end;
    double t_temp2;
    double coeff[4];
    double v_max = fabs(xdxddx_end[0] - xdxddx_start[0]);
    double acc = 4*v_max;
    double x_ref_temp;
//    coeff[0] = xdxddx_start[0];
//    coeff[1] = (xdxddx_start[0]+xdxddx_end[0])/3;
//    coeff[2] = (xdxddx_start[0]+xdxddx_end[0])/3*2;
//    coeff[3] = xdxddx_end[0];
//    double x_ref_temp = pow(1-t_temp,3)*coeff[0] + 3*pow(1-t_temp,2)*t_temp*coeff[1] + 3*(1-t_temp)*pow(t_temp,2)*coeff[2] + pow(t_temp,3)*coeff[3];

//    coeff[0] = xdxddx_start[0];
//    coeff[1] = (xdxddx_start[0] + xdxddx_end[0])/2;
//    coeff[2] = xdxddx_end[0];
//    double x_ref_temp = pow(1-t_temp,2)*coeff[0] + 2*(1-t_temp)*t_temp*coeff[1] + pow(t_temp,2)*coeff[2];

    if(t_temp < 0.5) {
        x_ref_temp = 0.5*acc*t_temp*t_temp;
    }
    else {
        x_ref_temp = 0.5*acc*0.5*0.5 + acc*(0.5*(t_temp-0.5) - 0.5*(t_temp-0.5)*(t_temp-0.5));
    }

//    float a_t = MoveJoints.acctime;
//    float cur_t = MoveJoints.CurrentTimeCount * (double)RT_TIMER_PERIOD_MS / 1000.0 - a_t;
//    moving_angle = 0.5*MoveJoints.STP_acc*a_t*a_t + MoveJoints.STP_acc*(a_t*cur_t - 0.5*cur_t*cur_t);

    if(xdxddx_end[0] - xdxddx_start[0] >= 0){
        x_ref[0] = xdxddx_start[0] + x_ref_temp;
    }else{
        x_ref[0] = xdxddx_start[0] - x_ref_temp;
    }
    return x_ref;
}

VectorNd bezier_coeff(VectorNd xdxddx, int order)
{
    VectorNd coeff_out;
    switch(order)
    {
        case 3:
        {
            coeff_out = VectorNd::Zero(4,1);
            MatrixNd A = MatrixNd::Zero(4,4);
            VectorNd b = VectorNd::Zero(4,1);
            for(int i=0; i<4; i++)
            {
                b[i] = xdxddx[i];
            }
            A(0,0) = 1;
            A(1,3) = 1;

            A(2,0) = -3;
            A(2,1) = 3;
            A(3,2) = -3;
            A(3,3) = 3;

            coeff_out = A.inverse()*b;
            break;
        }
        case 5:
        {
            coeff_out = VectorNd::Zero(6,1);
            MatrixNd A = MatrixNd::Zero(6,6);
            VectorNd b = VectorNd::Zero(6,1);
            for(int i=0; i<6; i++)
            {
                b[i] = xdxddx[i];
            }
            A(0,0) = 1;
            A(1,5) = 1;

            A(2,0) = -5;
            A(2,1) = 5;
            A(3,4) = -5;
            A(3,5) = 5;

            A(4,0) = 20;
            A(4,1) = -40;
            A(4,2) = 20;
            A(5,3) = 20;
            A(5,4) = -40;
            A(5,5) = 20;

            coeff_out = A.inverse()*b;
            break;
        }
        default:
        {
            coeff_out = VectorNd::Zero(4,1);
            break;
        }

    }
    return coeff_out;
}

double bezier_polynomial_x(VectorNd coeff, double t)
{
    int order = int(coeff.size());
    double x=0;

    for(int i=0; i<order; i++)
    {
        x += coeff[i] * factorial(order-1)/(factorial(i)*factorial(order-1-i)) * pow(t,i) * pow(1-t, order-1-i);
    }

    return x;
}

double bezier_polynomial_dx(VectorNd coeff, double t)
{
    int order = int(coeff.size())-1;
    double dx=0;

    switch(order)
    {
        case 3:
        {
            dx = t*(t-1)*6*(coeff[1]-coeff[2]) - pow(t-1,2)*3*(coeff[0]-coeff[1]) - pow(t,2)*3*(coeff[2]-coeff[3]);
            break;
        }
        case 5:
        {
            dx = t*pow(t-1,3)*20*(coeff[1]-coeff[2]) - pow(t-1,4)*5*(coeff[0]-coeff[1]) - pow(t,2)*pow(t-1,2)*30*(coeff[2]-coeff[3]) - pow(t,4)*5*(coeff[4]-coeff[5]) + pow(t,3)*pow(t-1,1)*20*(coeff[3]-coeff[4]);
            break;
        }
        default:
        {
            dx = 0;
            break;
        }
    }

    return dx;
}

double bezier_polynomial_ddx(VectorNd coeff, double t)
{
    int order = int(coeff.size())-1;
    double ddx=0;

    switch(order)
    {
        case 5:
        {
            ddx = pow(t,3)*20*(coeff[3]-2*coeff[4]+coeff[5]) - pow(t-1,3)*20*(coeff[0]-2*coeff[1]+coeff[2]) + t*pow(t-1,2)*60*(coeff[1]-2*coeff[2]+coeff[3]) - pow(t,2)*(t-1)*60*(coeff[2]-2*coeff[3]+coeff[4]);
            break;
        }
        default:
        {
            ddx = 0;
            break;
        }
    }


    return ddx;
}

int factorial(int a)
{
    int output = 1;
    if(a!=0)
    {
        for(int i=2; i<=a; i++)
        {
            output = output*i;
        }
    }

    return output;
}

Vector3d Lowpass_filter_3d(Vector3d input, Vector3d output, double f_cut, double ts) {
    double tau;
    Vector3d output_;
    if(f_cut != 0) {
        tau = 1/(2*PI*f_cut);
        for(int i=0; i<3; i++) {
            output_[i] = (tau *  output[i] + ts *input[i]) / (tau + ts);
        }
    }
    else output_ = Vector3d();
    return output_;
}

Vector3d Highpass_filter_3d(Vector3d input, Vector3d input_pre, Vector3d output, double f_cut, double ts) {
    double tau;
    Vector3d output_;
    if(f_cut != 0) {
        tau = 1/(2*PI*f_cut);
        for(int i=0; i<3; i++) {
            output_[i] = tau/(tau+ts)*output[i] + tau/(tau+ts)*(input[i]-input_pre[i]);
        }
    }
    else output_ = Vector3d();
    return output_;
}

double Lowpass_filter(double input, double output, double f_cut, double ts) {
    double tau;
    double output_;
    if(f_cut != 0) {
//        tau = 1/(2*PI*f_cut);
//        output_ = (tau *  output + ts *input) / (tau + ts);
        tau = 1.0/(1.0 + 2*PI*f_cut*ts);
        output_ = tau*output + (1.0 - tau)*input;
    }
    else output_=0;
    return output_;
}

double Highpass_filter(double input, double input_pre, double output, double f_cut, double ts) {
    double tau;
    double output_;
    if(f_cut != 0) {
        tau = 1/(2*PI*f_cut);
        output_ = tau/(tau+ts)*output + tau/(tau+ts)*(input-input_pre);
    }
    else output_=0;
    return output_;
}

Vector3d calc_3rd(double t_now, double t_e, Vector3d p_init, Vector3d p_end, double ts){

    // calculate 3rd order polynomial coefficient fot t_now to t_e
    // t_now : p_init(pi, dpi ,ddpi)
    // t_e   : p_end(pf, 0, 0)
    // ts    : system time interval
    // and return p, dp ,ddp for t_now + dt


    double A, B, C, D;

    Vector3d p_dp_ddp = Vector3d::Zero();


    //for safety
//    if (t_now <= 0){
//        p_dp_ddp = p_init;
//        return p_dp_ddp;
//    }
    if(t_now >= t_e){
        p_dp_ddp = p_end;
        return p_dp_ddp;
    }


    double tf1, tf2, tf3, tf4, pi, dpi, pf, dpf;

    tf1 = t_e - t_now;
    tf2 = tf1*tf1;
    tf3 = tf2*tf1;
    tf4 = tf3*tf1;

    pi = p_init[0];
    dpi = p_init[1];

    pf = p_end[0];
    dpf = p_end[1];

    C = dpi;
    D = pi;

    double det = -tf4;
    A = 2*tf1/det*(pf-C*tf1-D) - tf2/det*(dpf - C);
    B = -3*tf2/det*(pf-C*tf1-D) + tf3/det*(dpf - C);

    double t1 = ts;
    double t2 = t1*t1;
    double t3 = t2*t1;

    p_dp_ddp[0] = A*t3 + B*t2 + C*t1 + D;
    p_dp_ddp[1] = 3*A*t2 + 2*B*t1 + C;
    p_dp_ddp[2] = 6*A*t1 + 2*B;

    return p_dp_ddp;
}
