% Intrinsic and Extrinsic Camera Parameters
%
% This script file can be directly executed under Matlab to recover the camera intrinsic and extrinsic parameters.
% IMPORTANT: This file contains neither the structure of the calibration objects nor the image coordinates of the calibration points.
%            All those complementary variables are saved in the complete matlab data file Calib_Results.mat.
% For more information regarding the calibration model visit http://www.vision.caltech.edu/bouguetj/calib_doc/


%-- Focal length:
fc = [ 463.036768984339574 ; 114.552796776105012 ];

%-- Principal point:
cc = [ 222.081839222080390 ; 80.012559343345401 ];

%-- Skew coefficient:
alpha_c = 0.000000000000000;

%-- Distortion coefficients:
kc = [ -0.003506781285114 ; -0.013756614619066 ; 0.000000000000000 ; 0.000000000000000 ; 0.000000000000000 ];

%-- Focal length uncertainty:
fc_error = [ 6.030542599012354 ; 1.447913712664638 ];

%-- Principal point uncertainty:
cc_error = [ 2.352800502145937 ; 0.741436140353704 ];

%-- Skew coefficient uncertainty:
alpha_c_error = 0.000000000000000;

%-- Distortion coefficients uncertainty:
kc_error = [ 0.012432273539349 ; 0.033954553246063 ; 0.000000000000000 ; 0.000000000000000 ; 0.000000000000000 ];

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
omc_1 = [ -2.086994e+00 ; -2.131673e+00 ; -2.705511e-01 ];
Tc_1  = [ -1.486964e+02 ; -3.361578e+02 ; 8.724501e+02 ];
omc_error_1 = [ 3.623209e-03 ; 3.159929e-03 ; 8.119142e-03 ];
Tc_error_1  = [ 4.452304e+00 ; 6.060599e+00 ; 1.157509e+01 ];

%-- Image #2:
omc_2 = [ -2.237828e+00 ; -2.120680e+00 ; -2.962816e-01 ];
Tc_2  = [ -2.918904e+02 ; -4.114494e+02 ; 1.216740e+03 ];
omc_error_2 = [ 6.179010e-03 ; 4.875006e-03 ; 1.271835e-02 ];
Tc_error_2  = [ 6.351918e+00 ; 8.362818e+00 ; 1.645151e+01 ];

%-- Image #3:
omc_3 = [ 2.146346e+00 ; 1.873354e+00 ; 6.406397e-01 ];
Tc_3  = [ -3.208114e+02 ; -4.171604e+02 ; 1.219011e+03 ];
omc_error_3 = [ 6.219857e-03 ; 4.969717e-03 ; 1.052904e-02 ];
Tc_error_3  = [ 6.513558e+00 ; 8.551942e+00 ; 1.669647e+01 ];

%-- Image #4:
omc_4 = [ 2.072317e+00 ; 1.620808e+00 ; 8.282730e-01 ];
Tc_4  = [ -2.298477e+02 ; -4.729305e+02 ; 1.345611e+03 ];
omc_error_4 = [ 7.060078e-03 ; 3.971076e-03 ; 1.007048e-02 ];
Tc_error_4  = [ 7.026274e+00 ; 9.350648e+00 ; 1.940709e+01 ];

%-- Image #5:
omc_5 = [ 2.256812e+00 ; 1.701585e+00 ; 9.031919e-01 ];
Tc_5  = [ -1.600484e+01 ; -5.787904e+02 ; 1.580735e+03 ];
omc_error_5 = [ 7.736746e-03 ; 5.533427e-03 ; 1.253767e-02 ];
Tc_error_5  = [ 8.094698e+00 ; 1.134735e+01 ; 2.368882e+01 ];

%-- Image #6:
omc_6 = [ -2.062763e+00 ; -2.017560e+00 ; -6.806688e-01 ];
Tc_6  = [ -3.060972e+01 ; -4.463925e+02 ; 1.260929e+03 ];
omc_error_6 = [ 5.445480e-03 ; 6.549968e-03 ; 1.427161e-02 ];
Tc_error_6  = [ 6.467639e+00 ; 8.934293e+00 ; 1.756499e+01 ];

%-- Image #7:
omc_7 = [ -1.976290e+00 ; -2.143702e+00 ; -2.556248e-01 ];
Tc_7  = [ -2.241998e+02 ; -2.894567e+02 ; 1.326131e+03 ];
omc_error_7 = [ 6.688100e-03 ; 5.027590e-03 ; 1.325467e-02 ];
Tc_error_7  = [ 6.694096e+00 ; 8.989005e+00 ; 1.757928e+01 ];

%-- Image #8:
omc_8 = [ -2.240207e+00 ; -2.123530e+00 ; -1.815110e-01 ];
Tc_8  = [ -1.606729e+02 ; -4.196220e+02 ; 1.339570e+03 ];
omc_error_8 = [ 6.600747e-03 ; 6.005490e-03 ; 1.372499e-02 ];
Tc_error_8  = [ 6.867510e+00 ; 9.072696e+00 ; 1.813837e+01 ];

%-- Image #9:
omc_9 = [ 2.230507e+00 ; 2.119073e+00 ; 1.168102e-01 ];
Tc_9  = [ -2.368112e+02 ; -5.238029e+02 ; 1.305249e+03 ];
omc_error_9 = [ 6.626701e-03 ; 5.988726e-03 ; 1.176457e-02 ];
Tc_error_9  = [ 6.768608e+00 ; 8.731909e+00 ; 1.758356e+01 ];

%-- Image #10:
omc_10 = [ 2.283626e+00 ; 2.069235e+00 ; 5.320824e-01 ];
Tc_10  = [ -2.449137e+02 ; -6.611651e+02 ; 1.204740e+03 ];
omc_error_10 = [ 6.341551e-03 ; 6.396214e-03 ; 1.209095e-02 ];
Tc_error_10  = [ 6.361286e+00 ; 8.647354e+00 ; 1.693804e+01 ];

%-- Image #11:
omc_11 = [ -2.250787e+00 ; -2.030998e+00 ; -4.526584e-01 ];
Tc_11  = [ -2.099776e+02 ; -6.548589e+02 ; 1.628963e+03 ];
omc_error_11 = [ 9.597506e-03 ; 8.375366e-03 ; 1.958840e-02 ];
Tc_error_11  = [ 8.429731e+00 ; 1.152512e+01 ; 2.308238e+01 ];

%-- Image #12:
omc_12 = [ -2.164383e+00 ; -2.090032e+00 ; -9.765066e-02 ];
Tc_12  = [ -2.068215e+02 ; -4.787998e+02 ; 1.665366e+03 ];
omc_error_12 = [ 1.111623e-02 ; 8.808231e-03 ; 2.026005e-02 ];
Tc_error_12  = [ 8.433854e+00 ; 1.119668e+01 ; 2.241439e+01 ];

%-- Image #13:
omc_13 = [ -1.685148e+00 ; -1.960871e+00 ; 4.757698e-03 ];
Tc_13  = [ 1.152263e+02 ; -6.261190e+02 ; 1.413268e+03 ];
omc_error_13 = [ 8.492913e-03 ; 6.669473e-03 ; 1.362007e-02 ];
Tc_error_13  = [ 7.548236e+00 ; 9.382919e+00 ; 1.610440e+01 ];

%-- Image #14:
omc_14 = [ -1.796173e+00 ; -2.025223e+00 ; -1.965989e-01 ];
Tc_14  = [ -5.469437e+01 ; -6.455632e+02 ; 1.410147e+03 ];
omc_error_14 = [ 9.065397e-03 ; 6.400985e-03 ; 1.521105e-02 ];
Tc_error_14  = [ 7.186677e+00 ; 9.792617e+00 ; 1.791320e+01 ];

%-- Image #15:
omc_15 = [ -2.132373e+00 ; -1.873063e+00 ; -6.088318e-01 ];
Tc_15  = [ -2.401971e+02 ; -6.777502e+02 ; 1.319388e+03 ];
omc_error_15 = [ 8.435278e-03 ; 6.840506e-03 ; 1.605267e-02 ];
Tc_error_15  = [ 6.803812e+00 ; 9.492021e+00 ; 1.828121e+01 ];

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
omc_23 = [ 2.084761e+00 ; 1.795480e+00 ; 6.684996e-01 ];
Tc_23  = [ -2.155388e+02 ; -2.154397e+02 ; 1.746277e+03 ];
omc_error_23 = [ 7.918730e-03 ; 5.394749e-03 ; 1.434860e-02 ];
Tc_error_23  = [ 9.079441e+00 ; 1.180887e+01 ; 2.452960e+01 ];

%-- Image #24:
omc_24 = [ 2.218335e+00 ; 2.043220e+00 ; 4.485492e-01 ];
Tc_24  = [ -2.010273e+02 ; -3.189754e+02 ; 1.618915e+03 ];
omc_error_24 = [ 7.976691e-03 ; 7.358079e-03 ; 1.614838e-02 ];
Tc_error_24  = [ 8.392535e+00 ; 1.103894e+01 ; 2.249271e+01 ];

%-- Image #25:
omc_25 = [ -1.949063e+00 ; -2.143474e+00 ; -2.329618e-01 ];
Tc_25  = [ -1.972708e+02 ; -5.026395e+02 ; 1.625102e+03 ];
omc_error_25 = [ 9.855996e-03 ; 7.214421e-03 ; 1.996558e-02 ];
Tc_error_25  = [ 8.209412e+00 ; 1.116138e+01 ; 2.171204e+01 ];

%-- Image #26:
omc_26 = [ -2.156921e+00 ; -2.127825e+00 ; -5.476899e-02 ];
Tc_26  = [ -7.560700e+01 ; -1.755280e+02 ; 1.529810e+03 ];
omc_error_26 = [ 7.416549e-03 ; 6.869788e-03 ; 1.613402e-02 ];
Tc_error_26  = [ 7.792851e+00 ; 9.990373e+00 ; 2.042476e+01 ];

%-- Image #27:
omc_27 = [ 2.206931e+00 ; 2.118274e+00 ; 1.842665e-01 ];
Tc_27  = [ -1.807819e+02 ; -4.342564e+02 ; 1.354471e+03 ];
omc_error_27 = [ 6.869725e-03 ; 6.104647e-03 ; 1.250447e-02 ];
Tc_error_27  = [ 7.010534e+00 ; 9.102398e+00 ; 1.820391e+01 ];

%-- Image #28:
omc_28 = [ 2.189696e+00 ; 2.038278e+00 ; 8.019086e-02 ];
Tc_28  = [ -2.266789e+02 ; -5.252163e+02 ; 1.528771e+03 ];
omc_error_28 = [ 8.878092e-03 ; 6.770983e-03 ; 1.342582e-02 ];
Tc_error_28  = [ 7.920814e+00 ; 1.005665e+01 ; 2.035865e+01 ];

%-- Image #29:
omc_29 = [ 2.224905e+00 ; 2.080265e+00 ; 1.462422e-01 ];
Tc_29  = [ -3.168519e+02 ; -8.560046e+02 ; 1.514733e+03 ];
omc_error_29 = [ 7.904489e-03 ; 7.196352e-03 ; 1.380733e-02 ];
Tc_error_29  = [ 7.941200e+00 ; 1.034077e+01 ; 2.110722e+01 ];

%-- Image #30:
omc_30 = [ 2.194177e+00 ; 2.031267e+00 ; 2.463831e-01 ];
Tc_30  = [ -2.681909e+02 ; -6.135557e+02 ; 1.417463e+03 ];
omc_error_30 = [ 7.838939e-03 ; 6.551939e-03 ; 1.213510e-02 ];
Tc_error_30  = [ 7.455559e+00 ; 9.642255e+00 ; 1.935797e+01 ];

