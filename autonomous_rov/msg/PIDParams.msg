Header    header# contains time stamp
float32   z     # current depth
float32   z_err # depth_err
float32   z_I   # integral error
float32   z_d   # derivative error
float32   kp    # proportional gain
float32   ki    # integral gain
float32   kd    # derivative gain
float32   g     # gravity correction
float32   step  # const. step
float32   dt    # variable dt (depends on time between each 2 readings)
bool      use_step #use the const. step or not (else will use the variable dt)
bool      use_kd
bool      use_ki
bool      use_alpha_beta
float32   alpha # alpha of the alpha-beta filter
float32   beta # beta of the alpha-beta filter
