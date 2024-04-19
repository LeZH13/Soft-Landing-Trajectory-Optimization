%% using CVX + mosek
cvx_solver mosek

% dynamics
S_m = [0 -omega(3) omega(2);
       omega(3) 0 -omega(1);
       -omega(2) omega(1) 0];
A_m = [zeros(3,3) eye(3);
           -S_m^2 -2*S_m];
B_m = [zeros(3,3);
           eye(3)];
% Selection matrix (unit vectors)
e1 = [1;0;0]; e2 = [0;1;0]; e3 = [0;0;1];
E_m = [e2';e3'];
% landing position
q = p_ref(2:3);
% glide scope
c = e1/tan(y_gs);

Z0 = log(m_wet);

%% Problem #1 power descent
tic
cvx_begin

% opti var
variable X(6,N) % [r;v]
variable U(3,N) % Tc/m
variable Z(1,N) % ln(m)
variable S(1,N) % slackvar/m

% Objective
minimize( norm(E_m*X(1:3,N)-q) ) % terminal cost
% J = 0;
% for k = 1:N
%     J = J + norm(X(1:3,k))*k/N;
% end
% minimize( J ) % accumulative cost
% Constraints
subject to
    % initial conditions
    X(:,1) == [p0;v0];
    Z(:,1) == Z0;
    % U2(:,1) == S(1,1)*e1;
    for k = 1:N-1
        % dynamics (euler first order)
        % Xd = A_m*X(:,k) + B_m*(g+U(:,k));
        % Zd = -alpha*S(:,k);
        % X_n = X(:,k) + Xd*dt;
        % Z_n = Z(:,k) + Zd*dt;

        % RK4
        % f1x = f_Xd(X(:,k),U(:,k),omega,g); 
        % f2x = f_Xd(X(:,k)+0.5*dt*f1x,U(:,k),omega,g); 
        % f3x = f_Xd(X(:,k)+0.5*dt*f2x,U(:,k),omega,g); 
        % f4x = f_Xd(X(:,k)+dt*f3x,U(:,k),omega,g);
        % X_n = X(:,k)+(dt/6)*(f1x+2*f2x+2*f3x+f4x);
        % f1z = f_Zd(S(:,k),alpha); 
        % f2z = f_Zd(S(:,k)+0.5*dt*f1z,alpha); 
        % f3z = f_Zd(S(:,k)+0.5*dt*f2z,alpha); 
        % f4z = f_Zd(S(:,k)+dt*f3z,alpha);
        % Z_n = Z(:,k)+(dt/6)*(f1z+2*f2z+2*f3z+f4z);
        % 
        % X(:,k+1) == X_n;
        % Z(:,k+1) == Z_n;
        
        % leapfrog
        % X(4:6,k+1) == X(4:6,k)+dt*0.5*(U(:,k)+g-S_m^2*X(1:3,k)-2*S_m*X(4:6,k) + U(:,k+1)+g-S_m^2*X(1:3,k+1)-2*S_m*X(4:6,k+1)); %%
        X(4:6,k+1) == X(4:6,k)+dt*0.5*(U(:,k)+g + U(:,k+1)+g);
        X(1:3,k+1) == X(1:3,k)+dt*0.5*(X(4:6,k+1)+X(4:6,k));
        Z(1,k+1) == Z(1,k) - alpha*dt*0.5*(S(1,k) + S(1,k+1));

        % glide scope
        norm( E_m*(X(1:3,k)-X(1:3,N)) ) <= c'*(X(1:3,k)-X(1:3,N));

        % thrust vector constraints
        norm(U(:,k)) <= S(:,k); % magnitude
        nh'*U(:,k) >= S(:,k)*cos(theta); % direction
        
        Z0t = log(m_wet-alpha*r2*(k-1)*dt);
        Z0t_max = log(m_wet-alpha*r1*(k-1)*dt);
        if (m_wet-alpha*r1*(k-1)*dt)<=m_dry
            error('fuel ran out, decrease t_f')
        end
% tayler series approx r1*exp(-Z(:,k)) <= S(:,k) <= r2*exp(-Z(:,k));

        lb = r1*exp(-Z0t)*( 1-(Z(:,k)-Z0t)+.5*(Z(:,k)-Z0t)^2 );
        % ub = r2*exp(-Z0t_max)*( 1-(Z(:,k)-Z0t_max) );
        ub = r2*exp(-Z0t)*( 1-(Z(:,k)-Z0t) );

        lb <= S(:,k) <= ub; % Slack var bound second order cone approx
        Z0t <= Z(:,k) <= Z0t_max; % physical bounds on Z, [1](36)
        Z(:,k) >= log(m_dry);

        % velocity states constraint
        norm(X(4:6,k)) <= V_max;   

    end

    % end states constraints
    Z(:,N) >= log(m_dry); % fuel
    X(1,N) == p_ref(1); % altitude
    X(4:6,N) == v_final; % velocity
    U(:,N) == [0,0,0]';

cvx_end
toc
%% Get optimal d_p3, N_p2 and t_f for P#2
for k = 1:N
    if norm(X(2:3,k))<10 && X(1,k)<p_ref(1)+1e-3
        d_p1 = X(2:3,k)
        t_f_opt = k*dt
        N_p2 = k
        break
    end
end

%% Problem #2 optimal fuel
% clear X U Z S J
tic
cvx_begin

% opti var
variable X2(6,N_p2) % [r;v]
variable U2(3,N_p2) % Tc/m
variable Z2(1,N_p2) % ln(m)
variable S2(1,N_p2) % slackvar/m

% Objective
J = 0;
for k = 1:N_p2-1
    J = J + S2(:,k); 
end
minimize( J )

subject to

    % initial conditions
    X2(:,1) == [p0;v0];
    Z2(:,1) == Z0;
    % U2(:,1) == S2(1,1)*e1;
    for k = 1:N_p2-1

        % leapfrog
        X2(4:6,k+1) == X2(4:6,k)+dt*0.5*(U2(:,k)+g-S_m^2*X2(1:3,k)-2*S_m*X2(4:6,k) + U2(:,k+1)+g-S_m^2*X2(1:3,k+1)-2*S_m*X2(4:6,k+1)); %%
        X2(1:3,k+1) == X2(1:3,k)+dt*0.5*(X2(4:6,k+1)+X2(4:6,k));
        Z2(1,k+1) == Z2(1,k) - alpha*dt*0.5*(S2(1,k) + S2(1,k+1));

        % glide scope
        norm( E_m*(X2(1:3,k)-X2(1:3,N_p2)) ) <= c'*(X2(1:3,k)-X2(1:3,N_p2));

        % thrust vector constraints
        norm(U2(:,k)) <= S2(:,k); % magnitude
        nh'*U2(:,k) >= S2(:,k)*cos(theta); % direction
        
        Z0t = log(m_wet-alpha*r2*(k-1)*dt);
        Z0t_max = log(m_wet-alpha*r1*(k-1)*dt);

        lb = r1*exp(-Z0t)*( 1-(Z2(:,k)-Z0t)+.5*(Z2(:,k)-Z0t)^2 );
        % ub = r2*exp(-Z0t_max)*( 1-(Z2(:,k)-Z0t_max) );
        ub = r2*exp(-Z0t)*( 1-(Z2(:,k)-Z0t) );
        lb <= S2(:,k) <= ub; % Slack var bound second order cone approx
        Z0t <= Z2(:,k) <= Z0t_max; % physical bounds on Z, [1](36)
        Z2(:,k) >= log(m_dry);

        % velocity states constraint
        norm(X2(4:6,k)) <= V_max;   

    end

    % end states constraints (N_p2)
    Z2(:,N_p2) >= log(m_dry); % fuel
    X2(1,N_p2) == p_ref(1); % altitude
    X2(4:6,N_p2) == v_final; % velocity
    norm(E_m*X2(1:3,N_p2)-q) <= norm(d_p1-q); % [2] (20) convex relaxation of the minimum achievable distance
    % U2(:,N_p2) == S2(1,N_p2)*e1;
    U2(:,N_p2) == [0,0,0]';
cvx_end
toc

%% Trajectory output

% reshape_vec = coord_map*[1;2;3];
% global Tc_opt X_opt dt_opt tf_opt
% X_opt = X2([coord_map*[1;2;3];coord_map*[4;5;6]],:);
% mass_opt = exp(Z2(:,:));
% Tc_opt = U2(coord_map*[1;2;3],:).*mass_opt;
% accel = U2(:,:)+g;
% dt_opt = dt;
% tf_opt = t_f_opt;

