import numpy as np

def control_user(pos_x,vel_x,acc_x,pos_y,vel_y,acc_y,i,num_agent,leader):
    force_x=0
    force_y=0
    if i != leader:
        beta = [1 for i in range(num_agent)]
        beta[0]=0.95
        z_pos_x = pos_x[i] - pos_x[i-1]
        z_vel_x = vel_x[i] - vel_x[i-1]

        z_pos_y = pos_y[i] - pos_y[i-1]
        z_vel_y = vel_y[i] - vel_y[i-1]

        s_z_x = z_pos_x + (z_vel_x*abs(z_vel_x))/(2*(beta[i]-beta[0]))
        s_z_y = z_pos_y + (z_vel_y*abs(z_vel_y))/(2*(beta[i]-beta[0]))

        if(s_z_x==0):
            force_x = beta[i]*np.sign(z_pos_x)
        else:
            force_x = -beta[i]*np.sign(s_z_x)

        if(s_z_y==0):
            force_y = beta[i]*np.sign(z_pos_y)
        else:
            force_y = -beta[i]*np.sign(s_z_y)


    return force_x,force_y