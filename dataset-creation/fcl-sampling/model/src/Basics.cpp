#include "Basics.h"
using namespace std;

double truncate_command(double in, double up, double down)
{
    if(in>up)
        return up;
    else if(in<down)
        return down;
    else
        return in;
}

double gauss_random(double sigma)
{
    int N = 20;
    double X = 0;
    for(int i=0;i<N;i++)
        X += double(rand()%1000)/1000.0;
    X = X - double(N/2);
    X = X * sqrt(12.0 / double(N));
    X = sigma * X;
    return X;
}

void load_matrix(double * dest, Cmatrix in)
{
    int m = in.rows();
    int n = in.cols();
    for (int j = 0; j < n; j++)
        for (int i = 0; i < m; i++)
            dest[i+j*m] = in(i,j);
}

void load_cvector(double * dest, Cvector in)
{
    memcpy(dest, &in[0], sizeof(double)*in.size());
}

void load_cvector3(double * dest, Cvector3 in)
{
    for (int i = 0; i < 3; i++)
        dest[i] = in[i];
}

void save_Cvector3(Cvector3 &dest, double * in)
{
    for (int i = 0; i < 3; i++)
        dest[i] = in[i];
}

Cvector vectorbig(Cvector a, Cvector b)
{
    Cvector res(a.size()+b.size());
    for(int i=0;i<a.size();i++) res[i] = a[i];
    for(int i=0;i<b.size();i++) res[i+a.size()] = b[i];
    return res;
}

void Cvector3_convert(Cvector3& in, double out[3])
{
    for(int i=0;i<3;i++)
        out[i] = in[i];
}

Cvector3 Cvector3_convert(double in[3])
{
    return Cvector3(in[0], in[1], in[2]);
}

void matrix3_convert(Cmatrix& in, double out[3][3])
{
    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
            out[i][j]=in(i,j);
}

Cmatrix matrix3_convert(double in[3][3])
{
    Cmatrix out(3,3);
    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
            out(i,j)=in[i][j];
    return out;
}

// skew symmetric matrix 4 dimensional
Cmatrix skew_symmetric_4d(Cvector I)
{
    Cmatrix O = Cmatrix::Zero(4,4);
    // a1 -b1 -c1 -d1
    // b1  a1 -d1  c1
    // c1  d1  a1 -b1
    // d1 -c1  b1  a1
    // all rotated to account for "real at the end"
    O(3,3) = I[0];
    O(0,0) = I[0];
    O(1,1) = I[0];
    O(2,2) = I[0];
    O(3,0) = -I[1];
    O(0,3) = I[1];
    O(1,2) = -I[1];
    O(2,1) = I[1];
    O(3,1) = -I[2];
    O(0,2) = I[2];
    O(1,3) = I[2];
    O(2,0) = -I[2];
    O(3,2) = -I[3];
    O(0,1) = -I[3];
    O(1,0) = I[3];
    O(2,3) = I[3];

    return O;
}

// reading keyboard functions
int khbit()
{
    struct timeval tv;
    fd_set fds;
    tv.tv_sec = 0;
    tv.tv_usec = 0;
    FD_ZERO(&fds);
    FD_SET(STDIN_FILENO, &fds);
    select(STDIN_FILENO+1, &fds, NULL, NULL, &tv);
    return FD_ISSET(STDIN_FILENO, &fds);
}

void nonblock(int state)
{
    struct termios ttystate;
    tcgetattr(STDIN_FILENO, &ttystate);

    if ( state == 1)
    {
        ttystate.c_lflag &= (~ICANON & ~ECHO); //Not display character
        ttystate.c_cc[VMIN] = 1;
    }
    else if (state == 0)
    {
        ttystate.c_lflag |= ICANON;
        ttystate.c_lflag |= ECHO;
    }
    tcsetattr(STDIN_FILENO, TCSANOW, &ttystate);
}

bool keyState(char key)
{
    bool pressed = false;
    int i = khbit(); //Alow to read from terminal
    if (i != 0)
    {
        char c = fgetc(stdin);
        if (c == key)
        {
            pressed = true;
        }
        else
        {
            pressed = false;
        }
    }

    return pressed;
}
