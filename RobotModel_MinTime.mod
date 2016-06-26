# Race track,  complete dynamics
# uses the outer loop pd controller for wg = 1rad/s and PM = 80, kp = 0.869 kd = 0.396 
# inner loop is wg = 2 rad/s and pm = 60

param N; # no. of integration steps 
param ds:= 0.001; # integration step size 
set kset ordered:= 0..N; 

param x_t{kset}; 
param z_t{kset}; 
param theta_t{kset}; 
param k_t{kset};

param L;   #length between wheels
param r;   #radius of wheels

var vref{kset};
var tref{kset}; 

var scale_f{kset};
var distance{kset};

var accelr{kset};
var accel{kset};
var w{kset};
var wref{kset};
var ang_accelr{kset};
var ang_accel{kset};
var jerkr{kset};
var jerk{kset};

var xpos{kset}; 
var zpos{kset}; 
var x1{kset};
var x2{kset};
var x3{kset};
var x4{kset};
var x5{kset};
var v{kset};
var theta{kset}; 
var time{kset};

minimize obj: time[N]; 


subject to
## dynamics low freq
c1{k in 1..N}: (xpos[k] - xpos[k-1])/ds = scale_f[k-1]*(v[k-1]*sin(theta[k-1])); 
c2{k in 1..N}: (zpos[k] - zpos[k-1])/ds = scale_f[k-1]*(v[k-1]*cos(theta[k-1]));
c3{k in 1..N}: (x1[k] - x1[k-1])/ds = scale_f[k-1]*(-2.616*x1[k-1]-2.489*x2[k-1]+2*vref[k-1]);
c4{k in 1..N}: (x2[k] - x2[k-1])/ds = scale_f[k-1]*(2*x1[k-1]);
c5{k in 0..N}: v[k] = 1.244*x2[k];

c6{k in 1..N}: (x3[k] - x3[k-1])/ds = scale_f[k-1]*(-2.556*x3[k-1]-1.703*x4[k-1]-1.08*x5[k-1]+tref[k-1]);
c7{k in 1..N}: (x4[k] - x4[k-1])/ds = scale_f[k-1]*(4*x3[k-1]);
c8{k in 1..N}: (x5[k] - x5[k-1])/ds = scale_f[k-1]*(x4[k-1]);
c9{k in 0..N}: theta[k] = 0.4924*x4[k]+1.08*x5[k];

c14{k in 1..N}: (time[k] - time[k-1])/ds = scale_f[k-1];


c80{k in 0..N}: scale_f[k] = (1 - (distance[k]*k_t[k]) )/(v[k]*cos(theta[k]-theta_t[k]));


##accels
c15{k in 1..N}: (vref[k]-vref[k-1])/ds = (scale_f[k-1])*accelr[k-1];
c16{k in 1..N}: (v[k]-v[k-1])/ds = (scale_f[k-1])*accel[k-1]; 

c59{k in 1..N}: (tref[k]-tref[k-1])/ds = (scale_f[k-1])*wref[k-1];
c60{k in 1..N}: (theta[k]-theta[k-1])/ds = (scale_f[k-1])*w[k-1];
c61{k in 1..N}: (wref[k]-wref[k-1])/ds = (scale_f[k-1])*ang_accelr[k-1];
c62{k in 1..N}: (w[k]-w[k-1])/ds = (scale_f[k-1])*ang_accel[k-1];


##controls
c17{k in 0..N}:   v[k] <= 2.3;   #2.3
c18{k in 0..N}:   v[k] >= 0.00001;  
#c19{k in 0..N}:   w[k] <= -7.14*v[k]+16.42;  #before 32.85 
#c20{k in 0..N}:   w[k] >=  7.14*v[k]-16.42;  
c21{k in 0..N}:   vref[k] <= 2;   #
c22{k in 0..N}:   vref[k] >= 0.00001;  #0.0001  
c23{k in 0..N}:   wref[k] <= 10;  #1.57,  max is +/-12.85 @0.5m/s
c24{k in 0..N}:   wref[k] >=  -10;  #       


## path constraint
c25{k in kset}: theta[k]-theta_t[k]<=1.5; 
c26{k in kset}: theta[k]-theta_t[k]>=-1.5;

c90{k in kset}: distance[k] =(xpos[k] - x_t[k])*cos(theta_t[k]) - (zpos[k] - z_t[k])*sin(theta_t[k]); 
c27{k in kset}: distance[k] <= 0.01;
c28{k in kset}: distance[k] >= -0.01;


## intial conditions
c29: time[0]=0;
c30: xpos[0]=0;
c31: zpos[0]=0;
c32: theta[0]=0;
#c33: v[0]= 0.01;
c34: x1[0]=0;
c35: x2[0]=0.0080;
c36: x3[0]=0;
c37: x4[0]=0;
c38: x5[0]=0;


## max allowable wheel speed
#c43{k in 1..N}: vref[k]/r + L*(wref[k])/(2*r) <= 46; # wr 
#c44{k in 1..N}: vref[k]/r + L*(wref[k])/(2*r) >= 0;
#c45{k in 1..N}: vref[k]/r - L*(wref[k])/(2*r) <= 46;  #wl
#c46{k in 1..N}: vref[k]/r - L*(wref[k])/(2*r) >= 0;

c47{k in 1..N}: v[k]/r + L*(w[k])/(2*r) <= 46; # wl 
c48{k in 1..N}: v[k]/r + L*(w[k])/(2*r) >= 0;
c49{k in 1..N}: v[k]/r - L*(w[k])/(2*r) <= 46;  #wr
c50{k in 1..N}: v[k]/r - L*(w[k])/(2*r) >= 0;


## constraint accel
c51{k in 0..N}: accelr[k] <= 1;  
c52{k in 0..N}: accelr[k] >= -1;
c53{k in 0..N}: accel[k] <= 1;  
c54{k in 0..N}: accel[k] >= -1;

c63{k in 0..N}: ang_accelr[k] <= 1;   
c64{k in 0..N}: ang_accelr[k] >= -1;
c65{k in 0..N}: ang_accel[k] <= 1; 
c66{k in 0..N}: ang_accel[k] >= -1;

#jerk
c55{k in 1..N}: (accelr[k]-accelr[k-1])/ds = (scale_f[k-1])*jerkr[k-1];
c57{k in 1..N}: (accel[k]-accel[k-1])/ds = (scale_f[k-1])*jerk[k-1];

c67{k in 0..N}: jerkr[k] <= 1;
c68{k in 0..N}: jerkr[k] >= -1;
c69{k in 0..N}: jerk[k] <= 1;
c70{k in 0..N}: jerk[k] >= -1;