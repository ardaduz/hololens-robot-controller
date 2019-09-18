% Intrinsic and Extrinsic Camera Parameters
%
% This script file can be directly executed under Matlab to recover the camera intrinsic and extrinsic parameters.
% IMPORTANT: This file contains neither the structure of the calibration objects nor the image coordinates of the calibration points.
%            All those complementary variables are saved in the complete matlab data file Calib_Results.mat.
% For more information regarding the calibration model visit http://www.vision.caltech.edu/bouguetj/calib_doc/


%-- Focal length:
fc = [ 449.616616629183056 ; 111.135864115088182 ];

%-- Principal point:
cc = [ 227.372375558537954 ; 76.951132806750820 ];

%-- Skew coefficient:
alpha_c = 0.000000000000000;

%-- Distortion coefficients:
kc = [ 0.005217164303602 ; -0.026623079946337 ; 0.000000000000000 ; 0.000000000000000 ; 0.000000000000000 ];

%-- Focal length uncertainty:
fc_error = [ 5.563680337661523 ; 1.340159189879572 ];

%-- Principal point uncertainty:
cc_error = [ 2.245601901800431 ; 0.653903796154889 ];

%-- Skew coefficient uncertainty:
alpha_c_error = 0.000000000000000;

%-- Distortion coefficients uncertainty:
kc_error = [ 0.011804296050598 ; 0.032548342558057 ; 0.000000000000000 ; 0.000000000000000 ; 0.000000000000000 ];

%-- Image size:
nx = 480;
ny = 160;


%-- Various other variables (may be ignored if you do not use the Matlab Calibration Toolbox):
%-- Those variables are used to control which intrinsic parameters should be optimized

n_ima = 30;						% Number of calibration images
est_fc = [ 1 ; 1 ];					% Estimation indicator of the two focal variables
est_aspect_ratio = 1;				% Estimation indicator of the aspect ratio fc(2)/fc(1)
center_optim = 1;					% Estimation indicator of the principal point
est_alpha = 0;						% Estimation indicator of the skew coefficient
est_dist = [ 1 ; 1 ; 0 ; 0 ; 0 ];	% Estimation indicator of the distortion coefficients


%-- Extrinsic parameters:
%-- The rotation (omc_kk) and the translation (Tc_kk) vectors for every calibration image and their uncertainties

%-- Image #1:
omc_1 = [ -2.054344e+00 ; -2.199837e+00 ; -3.136664e-01 ];
Tc_1  = [ -2.413263e+02 ; -3.265883e+02 ; 8.356011e+02 ];
omc_error_1 = [ 3.541942e-03 ; 2.909902e-03 ; 7.862120e-03 ];
Tc_error_1  = [ 4.238264e+00 ; 5.315362e+00 ; 1.072982e+01 ];

%-- Image #2:
omc_2 = [ -2.200921e+00 ; -2.182383e+00 ; -3.323962e-01 ];
Tc_2  = [ -3.871998e+02 ; -4.008318e+02 ; 1.164926e+03 ];
omc_error_2 = [ 6.050694e-03 ; 4.378725e-03 ; 1.294325e-02 ];
Tc_error_2  = [ 6.085866e+00 ; 7.331069e+00 ; 1.519629e+01 ];

%-- Image #3:
omc_3 = [ 2.104338e+00 ; 1.917832e+00 ; 6.602480e-01 ];
Tc_3  = [ -4.197213e+02 ; -4.042512e+02 ; 1.170401e+03 ];
omc_error_3 = [ 5.931522e-03 ; 5.711807e-03 ; 1.086674e-02 ];
Tc_error_3  = [ 6.373672e+00 ; 7.546442e+00 ; 1.534364e+01 ];

%-- Image #4:
omc_4 = [ 2.031067e+00 ; 1.662710e+00 ; 8.544535e-01 ];
Tc_4  = [ -3.263149e+02 ; -4.557599e+02 ; 1.294120e+03 ];
omc_error_4 = [ 6.558626e-03 ; 4.213167e-03 ; 1.022806e-02 ];
Tc_error_4  = [ 6.819341e+00 ; 8.274387e+00 ; 1.800485e+01 ];

%-- Image #5:
omc_5 = [ 2.210604e+00 ; 1.744600e+00 ; 9.254192e-01 ];
Tc_5  = [ -1.123135e+02 ; -5.484123e+02 ; 1.523655e+03 ];
omc_error_5 = [ 7.197853e-03 ; 5.636664e-03 ; 1.285731e-02 ];
Tc_error_5  = [ 7.697174e+00 ; 1.003342e+01 ; 2.201575e+01 ];

%-- Image #6:
omc_6 = [ -2.031739e+00 ; -2.075827e+00 ; -7.049617e-01 ];
Tc_6  = [ -1.263404e+02 ; -4.247451e+02 ; 1.212970e+03 ];
omc_error_6 = [ 5.453413e-03 ; 5.914530e-03 ; 1.453802e-02 ];
Tc_error_6  = [ 6.098967e+00 ; 7.857861e+00 ; 1.624145e+01 ];

%-- Image #7:
omc_7 = [ -1.946965e+00 ; -2.213217e+00 ; -2.875921e-01 ];
Tc_7  = [ -3.267331e+02 ; -2.743486e+02 ; 1.277823e+03 ];
omc_error_7 = [ 6.352891e-03 ; 4.723522e-03 ; 1.271213e-02 ];
Tc_error_7  = [ 6.380883e+00 ; 7.906346e+00 ; 1.634045e+01 ];

%-- Image #8:
omc_8 = [ -2.203252e+00 ; -2.187776e+00 ; -2.305569e-01 ];
Tc_8  = [ -2.575416e+02 ; -4.044307e+02 ; 1.286953e+03 ];
omc_error_8 = [ 6.373445e-03 ; 5.394660e-03 ; 1.418285e-02 ];
Tc_error_8  = [ 6.558231e+00 ; 7.986525e+00 ; 1.673673e+01 ];

%-- Image #9:
omc_9 = [ 2.173163e+00 ; 2.147876e+00 ; 1.471346e-01 ];
Tc_9  = [ -3.326475e+02 ; -5.014153e+02 ; 1.258801e+03 ];
omc_error_9 = [ 6.043478e-03 ; 6.167587e-03 ; 1.225292e-02 ];
Tc_error_9  = [ 6.520342e+00 ; 7.697265e+00 ; 1.640430e+01 ];

%-- Image #10:
omc_10 = [ 2.234812e+00 ; 2.110099e+00 ; 5.488379e-01 ];
Tc_10  = [ -3.329897e+02 ; -6.462141e+02 ; 1.158114e+03 ];
omc_error_10 = [ 5.890047e-03 ; 7.194638e-03 ; 1.297991e-02 ];
Tc_error_10  = [ 6.138862e+00 ; 7.612022e+00 ; 1.554045e+01 ];

%-- Image #11:
omc_11 = [ -2.215004e+00 ; -2.084964e+00 ; -4.722718e-01 ];
Tc_11  = [ -3.041504e+02 ; -6.317900e+02 ; 1.570672e+03 ];
omc_error_11 = [ 9.934759e-03 ; 7.633395e-03 ; 2.084789e-02 ];
Tc_error_11  = [ 8.095226e+00 ; 1.017447e+01 ; 2.138805e+01 ];

%-- Image #12:
omc_12 = [ -2.136907e+00 ; -2.155090e+00 ; -1.427301e-01 ];
Tc_12  = [ -3.077624e+02 ; -4.561118e+02 ; 1.604891e+03 ];
omc_error_12 = [ 1.016115e-02 ; 7.862364e-03 ; 2.005984e-02 ];
Tc_error_12  = [ 8.019354e+00 ; 9.872062e+00 ; 2.075686e+01 ];

%-- Image #13:
omc_13 = [ -1.662556e+00 ; -2.018981e+00 ; 3.237000e-02 ];
Tc_13  = [ 2.581200e+01 ; -6.035383e+02 ; 1.396228e+03 ];
omc_error_13 = [ 7.337547e-03 ; 5.894046e-03 ; 1.285523e-02 ];
Tc_error_13  = [ 7.132000e+00 ; 8.453483e+00 ; 1.528586e+01 ];

%-- Image #14:
omc_14 = [ -1.775064e+00 ; -2.082885e+00 ; -1.959139e-01 ];
Tc_14  = [ -1.463972e+02 ; -6.240880e+02 ; 1.374235e+03 ];
omc_error_14 = [ 8.259459e-03 ; 5.747508e-03 ; 1.433319e-02 ];
Tc_error_14  = [ 6.761891e+00 ; 8.719352e+00 ; 1.684496e+01 ];

%-- Image #15:
omc_15 = [ -2.105524e+00 ; -1.926277e+00 ; -6.259156e-01 ];
Tc_15  = [ -3.297639e+02 ; -6.667720e+02 ; 1.265117e+03 ];
omc_error_15 = [ 8.690862e-03 ; 6.162089e-03 ; 1.644569e-02 ];
Tc_error_15  = [ 6.487093e+00 ; 8.337310e+00 ; 1.688769e+01 ];

%-- Image #16:
omc_16 = [ NaN ; NaN ; NaN ];
Tc_16  = [ NaN ; NaN ; NaN ];
omc_error_16 = [ NaN ; NaN ; NaN ];
Tc_error_16  = [ NaN ; NaN ; NaN ];

%-- Image #17:
omc_17 = [ NaN ; NaN ; NaN ];
Tc_17  = [ NaN ; NaN ; NaN ];
omc_error_17 = [ NaN ; NaN ; NaN ];
Tc_error_17  = [ NaN ; NaN ; NaN ];

%-- Image #18:
omc_18 = [ NaN ; NaN ; NaN ];
Tc_18  = [ NaN ; NaN ; NaN ];
omc_error_18 = [ NaN ; NaN ; NaN ];
Tc_error_18  = [ NaN ; NaN ; NaN ];

%-- Image #19:
omc_19 = [ NaN ; NaN ; NaN ];
Tc_19  = [ NaN ; NaN ; NaN ];
omc_error_19 = [ NaN ; NaN ; NaN ];
Tc_error_19  = [ NaN ; NaN ; NaN ];

%-- Image #20:
omc_20 = [ NaN ; NaN ; NaN ];
Tc_20  = [ NaN ; NaN ; NaN ];
omc_error_20 = [ NaN ; NaN ; NaN ];
Tc_error_20  = [ NaN ; NaN ; NaN ];

%-- Image #21:
omc_21 = [ NaN ; NaN ; NaN ];
Tc_21  = [ NaN ; NaN ; NaN ];
omc_error_21 = [ NaN ; NaN ; NaN ];
Tc_error_21  = [ NaN ; NaN ; NaN ];

%-- Image #22:
omc_22 = [ NaN ; NaN ; NaN ];
Tc_22  = [ NaN ; NaN ; NaN ];
omc_error_22 = [ NaN ; NaN ; NaN ];
Tc_error_22  = [ NaN ; NaN ; NaN ];

%-- Image #23:
omc_23 = [ 2.032299e+00 ; 1.842090e+00 ; 6.806103e-01 ];
Tc_23  = [ -3.241533e+02 ; -1.745531e+02 ; 1.687671e+03 ];
omc_error_23 = [ 7.411442e-03 ; 5.920021e-03 ; 1.473127e-02 ];
Tc_error_23  = [ 8.861279e+00 ; 1.050467e+01 ; 2.271651e+01 ];

%-- Image #24:
omc_24 = [ 2.166213e+00 ; 2.086406e+00 ; 4.793903e-01 ];
Tc_24  = [ -2.968181e+02 ; -2.989133e+02 ; 1.560722e+03 ];
omc_error_24 = [ 7.572113e-03 ; 8.171782e-03 ; 1.670678e-02 ];
Tc_error_24  = [ 8.116242e+00 ; 9.734722e+00 ; 2.077658e+01 ];

%-- Image #25:
omc_25 = [ -1.922085e+00 ; -2.207680e+00 ; -2.449594e-01 ];
Tc_25  = [ -2.970731e+02 ; -4.802686e+02 ; 1.572770e+03 ];
omc_error_25 = [ 9.036659e-03 ; 6.556684e-03 ; 1.897656e-02 ];
Tc_error_25  = [ 7.811097e+00 ; 9.851152e+00 ; 2.019860e+01 ];

%-- Image #26:
omc_26 = [ -2.129601e+00 ; -2.209106e+00 ; -1.240947e-01 ];
Tc_26  = [ -1.858831e+02 ; -1.528978e+02 ; 1.473767e+03 ];
omc_error_26 = [ 6.842649e-03 ; 6.264660e-03 ; 1.594006e-02 ];
Tc_error_26  = [ 7.379120e+00 ; 8.798787e+00 ; 1.886997e+01 ];

%-- Image #27:
omc_27 = [ 2.145794e+00 ; 2.153511e+00 ; 2.316911e-01 ];
Tc_27  = [ -2.775697e+02 ; -4.178155e+02 ; 1.298913e+03 ];
omc_error_27 = [ 6.306880e-03 ; 6.517917e-03 ; 1.303320e-02 ];
Tc_error_27  = [ 6.734192e+00 ; 8.002532e+00 ; 1.684997e+01 ];

%-- Image #28:
omc_28 = [ 2.140705e+00 ; 2.078738e+00 ; 1.129326e-01 ];
Tc_28  = [ -3.237707e+02 ; -5.083438e+02 ; 1.474653e+03 ];
omc_error_28 = [ 8.259751e-03 ; 7.263990e-03 ; 1.446931e-02 ];
Tc_error_28  = [ 7.634522e+00 ; 8.872619e+00 ; 1.909042e+01 ];

%-- Image #29:
omc_29 = [ 2.174671e+00 ; 2.109094e+00 ; 1.695724e-01 ];
Tc_29  = [ -4.029749e+02 ; -8.389089e+02 ; 1.457569e+03 ];
omc_error_29 = [ 7.164166e-03 ; 7.275639e-03 ; 1.475189e-02 ];
Tc_error_29  = [ 7.610600e+00 ; 9.099806e+00 ; 1.947213e+01 ];

%-- Image #30:
omc_30 = [ 2.146941e+00 ; 2.070227e+00 ; 2.684191e-01 ];
Tc_30  = [ -3.609744e+02 ; -5.977724e+02 ; 1.365978e+03 ];
omc_error_30 = [ 7.341708e-03 ; 7.156661e-03 ; 1.308098e-02 ];
Tc_error_30  = [ 7.207278e+00 ; 8.509169e+00 ; 1.803698e+01 ];

