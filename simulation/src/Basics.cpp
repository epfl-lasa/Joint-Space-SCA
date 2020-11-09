#include "Basics.h"
using namespace std;

VectorXd vectorbig(VectorXd a, VectorXd b)
{
    VectorXd res(a.size()+b.size());
    for(int i=0;i<a.size();i++) 
		res[i] = a[i];
    for(int i=0;i<b.size();i++) 
		res[i+a.size()] = b[i];
    return res;
}

VectorXd truncate_command(VectorXd in, double up, double down)
{
	VectorXd res(in.size());
	for(int i=0; i<in.size(); i++)
		res[i] = truncate_command(in[i], up, down);
	return res;
}

double truncate_command(double in, double up, double down)
{
    if(in>up)
        return up;
    else if(in<down)
        return down;
    else
        return in;
}

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
