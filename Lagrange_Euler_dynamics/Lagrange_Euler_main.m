clc
clear

joint_actuated = 1;  % Joint which is moving. You can change it
%% Body Kinematics
syms x_body y_body z_body phi theta psi 
syms x_dot_body y_dot_body z_dot_body phi_dot theta_dot psi_dot


R_earth_to_body = euler_angles(phi-sym(pi/2), theta+sym(pi/2), psi);
q_body = [x_body y_body z_body phi theta psi];

R_dot_earth_to_body = diff(R_earth_to_body,phi)*phi_dot + ...
                      diff(R_earth_to_body,theta)*theta_dot + ...
                      diff(R_earth_to_body,psi)*psi_dot;
omega_all = simplify(R_dot_earth_to_body*inv(R_earth_to_body));
omega_body = [omega_all(3,2) omega_all(1,3) omega_all(2,1)];
v_body = [x_dot_body y_dot_body z_dot_body];

%% Leg kinematics
syms t11 t12 t13 ...          %tij  i=leg,   j=joint
     t21 t22 t23 ...
     t31 t32 t33 ...
     t41 t42 t43

 syms t11dot t12dot t13dot ...   %tdotij  i=leg,   j=joint
     t21dot t22dot t23dot ...
     t31dot t32dot t33dot ...
     t41dot t42dot t43dot

T_all = cell(4,3);
% Leg 1
T01_1 = tr(t11-sym(pi/2), 0    , 0   , -sym(pi/2));
T12_1 = tr(t12          , 0.18 , 0.4 ,       0   );
T23_1 = tr(t13          , 0,     0.41,       0   );

T02_1 = simplify(T01_1 * T12_1);
T03_1 = simplify(T01_1 * T12_1 * T23_1);

T_all{1,1} = T01_1;
T_all{1,2} = T02_1;
T_all{1,3} = T03_1;

% Leg 2
T01_2 = tr(t21-sym(pi/2), 0    , 0   , -sym(pi/2));
T12_2 = tr(t22          , 0.18 , 0.4 ,       0   );
T23_2 = tr(t23          , 0,     0.41,       0   );

T02_2 = simplify(T01_2 * T12_2);
T03_2 = simplify(T01_2 * T12_2 * T23_2);
T_all{2,1} = T01_2;
T_all{2,2} = T02_2;
T_all{2,3} = T03_2;

% Leg 3
T01_3 = tr(t31-sym(pi/2), 0    , 0   , -sym(pi/2));
T12_3 = tr(t32          , 0.18 , 0.4 ,       0   );
T23_3 = tr(t33          , 0,     0.41,       0   );

T02_3 = simplify(T01_3 * T12_3);
T03_3 = simplify(T01_3 * T12_3 * T23_3);
T_all{3,1} = T01_3;
T_all{3,2} = T02_3;
T_all{3,3} = T03_3;

% Leg 4
T01_4 = tr(t41-sym(pi/2), 0    , 0   , -sym(pi/2));
T12_4 = tr(t42          , 0.18 , 0.4 ,       0   );
T23_4 = tr(t43          , 0,     0.41,       0   );

T02_4 = simplify(T01_4 * T12_4);
T03_4 = simplify(T01_4 * T12_4 * T23_4);
T_all{4,1} = T01_4;
T_all{4,2} = T02_4;
T_all{4,3} = T03_4;

%% Inertia matrices

I1 = [0.00070396 0 0
      0           0.00070448 0
      0 0  0.00006850];
  
I2 = [0.00283198	0.00276276	0.00005045
	 0.00276276	 0.00283198	 0.00005045
	 0.00005045	 0.00005045  0.00561620];
 
I3 = [ 0.00252404	 0.00000000	 0.00000000
	 0.00000000	0.00255053	0.00000000
	 0.00000000	0.00000000	0.00004402];
 
I4 = [0.00038211	0.00000000 0.00000000
	0.00000000	 0.00038211	0.00000000
	0.00000000	0.00000000	0.00046935];
%% All legs Kinetic energy
m_all = [0.27 0.35 0.27 0.44;
         0.27 0.35 0.27 0.44;
         0.27 0.35 0.27 0.44;
         0.27 0.35 0.27 0.44];
I_all = cell(4,4);
for i=1:4
    I_all{i,1} = I1;
    I_all{i,2} = I2;
    I_all{i,3} = I3;
    I_all{i,4} = I4;
end

% T_all{i,j} = T0j_i  i=leg j=joint
% P_all{i,j} = P_cj_0_i   i=leg j= joint
% v_all{i,j} = v_cj_0_i   i=leg j= joint
% omega_all{i,j} = omega_cj_0_i   i=leg j= joint

P_all = cell(4,4);
v_all = cell(4,4);
omega_all = cell(4,4);
K_all = cell(4,1);


js = [t11 t12 t13;
      t21 t22 t23;
      t31 t32 t33;
      t41 t42 t43];
  
j_dots = [t11dot t12dot t13dot;
          t21dot t22dot t23dot;
          t31dot t32dot t33dot;
          t41dot t42dot t43dot];

for i=1:4
    % Calculate position of center of mass of leg i with respect to base
    % frame of leg i
    P{i,1} = T_all{i,1}*[0 0 0.08 1]';
    P{i,2} = T_all{i,2}*[0.13 0.13 0.02 1]';
    P{i,3} = T_all{i,3}*[0.18 0.18 0 1]';
    P{i,4} = T_all{i,3}*[0.03 0.03 0 1]';
    
    % Calculate velocity of center of mass of leg i with respect to base
    % frame of leg i
    v_all{i,1} = diff(P{i,1},js(i,1))*j_dots(i,1);
    v_all{i,2} = diff(P{i,2},js(i,1))*j_dots(i,1)+diff(P{i,2},js(i,2))*j_dots(i,2);
    v_all{i,3} = diff(P{i,3},js(i,1))*j_dots(i,1)+diff(P{i,3},js(i,2))*j_dots(i,2)+diff(P{i,3},js(i,3))*j_dots(i,3);
    v_all{i,4} = diff(P{i,4},js(i,1))*j_dots(i,1)+diff(P{i,4},js(i,2))*j_dots(i,2)+diff(P{i,4},js(i,3))*j_dots(i,3);
    
    % Calculate angular velocity of center of mass of leg i with respect to base
    % frame of leg i
    omega_all{i,1} = j_dots(i,1)*T_all{i,1}(1:3,3);
    omega_all{i,2} = j_dots(i,1)*T_all{i,1}(1:3,3)+j_dots(i,2)*T_all{i,2}(1:3,3);
    omega_all{i,3} = j_dots(i,1)*T_all{i,1}(1:3,3)+j_dots(i,2)*T_all{i,2}(1:3,3)+j_dots(i,3)*T_all{i,3}(1:3,3);
    omega_all{i,4} = j_dots(i,1)*T_all{i,1}(1:3,3)+j_dots(i,2)*T_all{i,2}(1:3,3)+j_dots(i,3)*T_all{i,3}(1:3,3);
    
    % Calculate kinetic energy of leg i
    K_all{i} = 0.5*m_all(i,1)*v_all{i,1}.'*v_all{i,1} + ...
               0.5*m_all(i,2)*v_all{i,2}.'*v_all{i,2} + ...
               0.5*m_all(i,3)*v_all{i,3}.'*v_all{i,3} + ...
               0.5*m_all(i,4)*v_all{i,4}.'*v_all{i,4} + ...
               0.5*omega_all{i,1}.'*I_all{i,1}*omega_all{i,1} + ...
               0.5*omega_all{i,2}.'*I_all{i,2}*omega_all{i,2} + ...
               0.5*omega_all{i,3}.'*I_all{i,3}*omega_all{i,3} + ...
               0.5*omega_all{i,4}.'*I_all{i,4}*omega_all{i,4};     
end


%% Leg 1 kinetic energy
syms t11dot t12dot t13dot

m1 = 0.27;
m2 = 0.35;
m3 = 0.27;
m4 = 0.44;

% P_cj_0_i   i=leg j= joint

P_c1_0_1 = T01_1*[0 0 0.08 1]';
P_c2_0_1 = T02_1*[0.13 0.13 0.02 1]';
P_c3_0_1 = T03_1*[0.18 0.18 0 1]';
P_c4_0_1 = T03_1*[0.03 0.03 0 1]';

v_c1_0 = diff(P_c1_0_1, t11)*t11dot;
v_c2_0 = diff(P_c2_0_1, t11)*t11dot + diff(P_c2_0_1, t12)*t12dot;
v_c3_0 = diff(P_c3_0_1, t11)*t11dot + diff(P_c3_0_1, t12)*t12dot + diff(P_c3_0_1, t13)*t13dot;
v_c4_0 = diff(P_c4_0_1, t11)*t11dot + diff(P_c4_0_1, t12)*t12dot + diff(P_c4_0_1, t13)*t13dot;

K1 = 0.5*m1*(v_c1_0.')*v_c1_0;
K2 = 0.5*m2*(v_c2_0.')*v_c2_0;
K3 = 0.5*m3*(v_c3_0.')*v_c3_0;
K4 = 0.5*m4*(v_c4_0.')*v_c4_0;

K = K1+K2+K3+K4;

omega_0_1 = t11dot*T01_1(1:3,3);
omega_0_2 = t11dot*T01_1(1:3,3)+t12dot*T02_1(1:3,3);
omega_0_3 = t11dot*T01_1(1:3,3)+t12dot*T02_1(1:3,3)+t13dot*T03_1(1:3,3);
omega_0_4 = t11dot*T01_1(1:3,3)+t12dot*T02_1(1:3,3)+t13dot*T03_1(1:3,3);

Kr1 = 0.5*(omega_0_1.') * I1 * omega_0_1;
Kr2 = 0.5*(omega_0_2.') * I2 * omega_0_2;
Kr3 = 0.5*(omega_0_3.') * I3 * omega_0_3;
Kr4 = 0.5*(omega_0_4.') * I4 * omega_0_4;

K = K+Kr1+Kr2+Kr3+Kr4;
%% Potential energy of the body

h1 = (0.4*cos(t12) + 0.41*cos(t12+t13))*cos(t11) - 0.18*sin(t11);
h2 = (0.4*cos(t22) + 0.41*cos(t22+t23))*cos(t21) - 0.18*sin(t21);
h3 = (0.4*cos(t32) + 0.41*cos(t32+t33))*cos(t31) - 0.18*sin(t31);
h4 = (0.4*cos(t42) + 0.41*cos(t42+t43))*cos(t41) - 0.18*sin(t41);

h1 = [inv(R_earth_to_body) [0 0 0].'; [0 0 0 1]]*[0 0 h1 1].';
h1=h1(3);

h1 = [inv(R_earth_to_body) [0 0 0].'; [0 0 0 1]]*T03_1(1:4,4);
h2 = [inv(R_earth_to_body) [0 0 0].'; [0 0 0 1]]*T03_2(1:4,4);
h3 = [inv(R_earth_to_body) [0 0 0].'; [0 0 0 1]]*T03_3(1:4,4);
h4 = [inv(R_earth_to_body) [0 0 0].'; [0 0 0 1]]*T03_4(1:4,4);

h1=-h1(3);
h2=-h2(3);
h3=-h3(3);
h4=-h4(3);

mbody=57;

P_body = [mbody*9.81*((h2+h3+h4)/3)
          mbody*9.81*((h1+h3+h4)/3)
          mbody*9.81*((h2+h1+h4)/3)
          mbody*9.81*((h2+h3+h1)/3)];

P_body = P_body(joint_actuated);

%% potential energy of legs
% leg 1
P_c1_1 = m1*[9.81 0 0 0]*[inv(R_earth_to_body) [0 0 0].'; [0 0 0 1]]*T01_1*[0 0 0.08 1]';
P_c2_1 = m2*[9.81 0 0 0]*[inv(R_earth_to_body) [0 0 0].'; [0 0 0 1]]*T02_1*[0.13 0.13 0.02 1]';
P_c3_1 = m3*[9.81 0 0 0]*[inv(R_earth_to_body) [0 0 0].'; [0 0 0 1]]*T03_1*[0.18 0.18 0 1]';
P_c4_1 = m4*[9.81 0 0 0]*[inv(R_earth_to_body) [0 0 0].'; [0 0 0 1]]*T03_1*[0.03 0.03 0 1]';

% leg 2
P_c1_2 = m1*[9.81 0 0 0]*[inv(R_earth_to_body) [0 0 0].'; [0 0 0 1]]*T01_2*[0 0 0.08 1]';
P_c2_2 = m2*[9.81 0 0 0]*[inv(R_earth_to_body) [0 0 0].'; [0 0 0 1]]*T02_2*[0.13 0.13 0.02 1]';
P_c3_2 = m3*[9.81 0 0 0]*[inv(R_earth_to_body) [0 0 0].'; [0 0 0 1]]*T03_2*[0.18 0.18 0 1]';
P_c4_2 = m4*[9.81 0 0 0]*[inv(R_earth_to_body) [0 0 0].'; [0 0 0 1]]*T03_2*[0.03 0.03 0 1]';

% leg 3
P_c1_3 = m1*[9.81 0 0 0]*[inv(R_earth_to_body) [0 0 0].'; [0 0 0 1]]*T01_3*[0 0 0.08 1]';
P_c2_3 = m2*[9.81 0 0 0]*[inv(R_earth_to_body) [0 0 0].'; [0 0 0 1]]*T02_3*[0.13 0.13 0.02 1]';
P_c3_3 = m3*[9.81 0 0 0]*[inv(R_earth_to_body) [0 0 0].'; [0 0 0 1]]*T03_3*[0.18 0.18 0 1]';
P_c4_3 = m4*[9.81 0 0 0]*[inv(R_earth_to_body) [0 0 0].'; [0 0 0 1]]*T03_3*[0.03 0.03 0 1]';

% leg 4
P_c1_4 = m1*[9.81 0 0 0]*[inv(R_earth_to_body) [0 0 0].'; [0 0 0 1]]*T01_4*[0 0 0.08 1]';
P_c2_4 = m2*[9.81 0 0 0]*[inv(R_earth_to_body) [0 0 0].'; [0 0 0 1]]*T02_4*[0.13 0.13 0.02 1]';
P_c3_4 = m3*[9.81 0 0 0]*[inv(R_earth_to_body) [0 0 0].'; [0 0 0 1]]*T03_4*[0.18 0.18 0 1]';
P_c4_4 = m4*[9.81 0 0 0]*[inv(R_earth_to_body) [0 0 0].'; [0 0 0 1]]*T03_4*[0.03 0.03 0 1]';

P_total = P_c1_1+P_c2_1+P_c3_1+P_c4_1+...
          P_c1_2+P_c2_2+P_c3_2+P_c4_2+...
          P_c1_3+P_c2_3+P_c3_3+P_c4_3+...
          P_c1_4+P_c2_4+P_c3_4+P_c4_4+P_body;
%% Lagrangian
 
syms t11_double_dot t12_double_dot t13_double_dot ...
     t21_double_dot t22_double_dot t23_double_dot ...
     t31_double_dot t32_double_dot t33_double_dot ...
     t41_double_dot t42_double_dot t43_double_dot ...

L = K_all{joint_actuated,1}-P_total

%% Joint torques
js = [t11 t12 t13;
      t21 t22 t23;
      t31 t32 t33;
      t41 t42 t43];
  
j_dots = [t11dot t12dot t13dot;
          t21dot t22dot t23dot;
          t31dot t32dot t33dot;
          t41dot t42dot t43dot];

j_double_dots = [t11_double_dot t12_double_dot t13_double_dot;
                  t21_double_dot t22_double_dot t23_double_dot;
                  t31_double_dot t32_double_dot t33_double_dot;
                  t41_double_dot t42_double_dot t43_double_dot];
              
j_torques = cell(4,3);

for i=1:4
    for j=1:3
        a = diff(L, j_dots(i,j));
        
        b = diff(a, js(i,1))*j_dots(i,1) + diff(a,j_dots(i,1))*j_double_dots(i,1) + ...
            diff(a, js(i,2))*j_dots(i,2) + diff(a,j_dots(i,2))*j_double_dots(i,2) + ...
            diff(a, js(i,3))*j_dots(i,3) + diff(a,j_dots(i,3))*j_double_dots(i,3);
        
        j_torques{i,j} = b - diff(L,js(i,j));
        j_torques{i,j} = vpa(simplify(expand(subs(j_torques{i,j},[phi theta psi],[0 0 0]))),4);
        j_torques{i,j} = mapSymType(j_torques{i,j}, 'vpareal', @(x) piecewise(abs(x)<=0.0001, 0, x));
    end
end

%% Import Trajectory DATA

q1 = importdata('q1.mat');
qd1 = importdata('qd1.mat');
qdd1 = importdata('qdd1.mat');
j=[];
for i=1:380
    j1 = subs(j_torques{1,1},[t11 t12 t13 t11dot t12dot t13dot t11_double_dot t12_double_dot t13_double_dot],[q1(1,i) q1(2,i) q1(3,i) qd1(1,i) qd1(2,i) qd1(3,i) qdd1(1,i) qdd1(2,i) qdd1(3,i)]);
    j2 = subs(j_torques{1,2},[t11 t12 t13 t11dot t12dot t13dot t11_double_dot t12_double_dot t13_double_dot],[q1(1,i) q1(2,i) q1(3,i) qd1(1,i) qd1(2,i) qd1(3,i) qdd1(1,i) qdd1(2,i) qdd1(3,i)]);
    j3 = subs(j_torques{1,3},[t11 t12 t13 t11dot t12dot t13dot t11_double_dot t12_double_dot t13_double_dot],[q1(1,i) q1(2,i) q1(3,i) qd1(1,i) qd1(2,i) qd1(3,i) qdd1(1,i) qdd1(2,i) qdd1(3,i)]);
    j = [j [j1 j2 j3]'];
end

j = vpa(j);

%% Plot Torque Profile
t = linspace(0,10,380);
figure, hold on, grid on
plot(t, j(1,:), 'Linewidth', 2); 
plot(t, j(2,:), 'Linewidth', 2);
plot(t, j(3,:), 'Linewidth', 2);
xlim([0 10]);
xlabel('Time [s]'), ylabel('Torque [Nm]');
legend({'Joint 1', 'Joint 2', 'Joint 3'});
set(gca, 'FontSize', 14);