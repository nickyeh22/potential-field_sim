clc;
clear all; close all;
%Setting
map_len = 200;           %地圖square長寬
form_start = [100; 0];        %robot起點
form_goal = [130; 140];       %robot目標點
form_r = 10;                 %碰撞半徑
form_speed = 1;              %robot速度大小
vhc_num = 3;               %隊形中的載具數量
% obs_start = [127; 100];   %obstacle起點
obs_start = [100; 60];
obs_goal = [100; 60];   %obstacle目標點
obs_r = 10;              %碰撞半徑
obs_speed = 1;           %obstacle速度大小

obs_num = 5;
obs_pos = [100; 60];
obs_pos(:,:,2) =  [167; 80];
obs_pos(:,:,3) =  [152; 22];
obs_pos(:,:,4) =  [50; 95];
obs_pos(:,:,5) =  [10; 30];
obs_sz = [22 15 22 20 33];          %障礙物半徑
% obs_sz = [22 15 22]; 
%Variables
form_pos = form_start;
form_ori = form_goal - form_start;
form_ori_uni = form_ori / (sqrt(form_ori(1)^2+form_ori(2)^2));
form_vlc = form_ori_uni * form_speed;


S_1 = [form_r; 0];                              %X_1-X_c
S_2 = [-form_r*cosd(60); form_r*sind(60)];     %X_2-X_c
S_3 = [-form_r*cosd(60); -form_r*sind(60)];      %X_3-X_c
S_12 = S_1 - S_2;
S_13 = S_1 - S_3;
S_23 = S_2 - S_3;

vhc1_pos = S_1 + form_start;
vhc2_pos = S_2 + form_start;
vhc3_pos = S_3 + form_start;
vhc_pos(:,:,1) = vhc1_pos;
vhc_pos(:,:,2) = vhc2_pos;
vhc_pos(:,:,3) = vhc3_pos;

ga_pre = 0;
gamma = 0;
be_pre = 1;
beta = 1;

%%
%%%%%%%%%%%%%%---------------RRT*---------------%%%%%%%%%%%%%%%%%
Thr=12;             %目標達成閥值    
Delta= 12;          %單位步長
radius=24;          %重新布線的圓閥值
goal_rand_thr = 0.1;   %目標取向閥值
T.v(1).x = form_start(1);     %節點座標    
T.v(1).y = form_start(2);     
T.v(1).xPrev = form_start(1); %節點的父節點座標    
T.v(1).yPrev = form_start(2);
T.v(1).dist=0;      %節點與父節點距離    
T.v(1).indPrev = 0; %%%%%%目前未知
T.v(1).cost = 0;      %%%%%節點目前的cost
figure(1);
hold on
plot(form_start(1), form_start(2), 'ro', 'MarkerSize',3, 'MarkerFaceColor','b');
plot(form_goal(1), form_goal(2), 'go', 'MarkerSize',3, 'MarkerFaceColor','y');
for i = 1:1:obs_num
    h = circle(obs_pos(1,1,i), obs_pos(2,1,i), obs_sz(i));
end
axis([0 map_len 0 map_len],'square');
count=1;
for iter = 1:5000
    if rand()<goal_rand_thr
        p_rand(1)=form_goal(1);
        p_rand(2)=form_goal(2);
    else 
        p_rand=[];
        p_rand(1)=ceil(rand()*map_len); 
        p_rand(2)=ceil(rand()*map_len);
    end
 
    p_near=[];
   
    min_distance = 1000;
    for i=1:count
        distance = sqrt( ( T.v(i).x - p_rand(1) )^2 + ( T.v(i).y - p_rand(2) )^2 );
        if distance < min_distance
            min_distance = distance;
            index = i;
        end
    end
    %取得鄰近點p_near座標及其父節點
    p_near(1) = T.v(index).x;
    p_near(2) = T.v(index).y;
    p_near_prev(1) = T.v(index).xPrev;
    p_near_prev(2) = T.v(index).yPrev;
    p_near_cost = T.v(index).cost; %取得鄰近點的cost
    %取得新節點p_new之座標點
    p_new=[];
    p_new(1) = p_near(1) + round( ( p_rand(1)-p_near(1) ) * Delta/min_distance );
    p_new(2) = p_near(2) + round( ( p_rand(2)-p_near(2) ) * Delta/min_distance );
    %碰撞檢測，如果路線有碰撞，就跳過本次迴圈，不增加新節點
%     if ~collisionChecking(p_near,p_new,Imp) 
%        continue;
%     end
    %轉角約束，如果路線轉角過大，就跳過本次迴圈，不增加新節點
%     if ~angleChecking(p_near_prev,p_near,p_new)
%        continue;
%     end
    %--------------------------------------
    % X_near附近圈一個範圍，儲存範圍內所有節點
    inRadius=[];
    inDistance=[];
    for i=1:count
        distance = sqrt( ( T.v(i).x - p_near(1) )^2 + ( T.v(i).y - p_near(2) )^2 );
        if distance < radius
            cost=sqrt( ( T.v(i).x - p_new(1) )^2 + ( T.v(i).y - p_new(2) )^2 );
            inRadius=[inRadius i];
            inDistance=[inDistance cost]; 
        end
    end
    in_Radius_num=size(inRadius,2);
    for i=1:in_Radius_num
        parital_cost = T.v(inRadius(i)).cost + inDistance(i);
        if  parital_cost<p_near_cost+sqrt( ( p_new(1) - p_near(1) )^2 + ( p_new(2) - p_near(2) )^2 ) %有問題 要再改
            p_near(1) = T.v(inRadius(i)).x;
            p_near(2) = T.v(inRadius(i)).y;
            p_near_prev(1) = T.v(inRadius(i)).xPrev;
            p_near_prev(2) = T.v(inRadius(i)).yPrev;
            p_near_cost = T.v(inRadius(i)).cost
        end
        %找出是不是有別的樹節點的cost比鄰近點更小，即找出真正的p_near
    end 
    %重新找到新的鄰近點時也要進行角度約束與碰撞檢測
    if ~collisionChecking(p_near, p_new, obs_num, obs_pos, obs_sz) 
       continue;
    end
    if ~angleChecking(p_near_prev,p_near,p_new)
       continue;
    end

    %-----------------------------------
    count=count+1;
    
    %增加新節點到RRT樹上
    T.v(count).x = p_new(1);         
    T.v(count).y = p_new(2); 
    T.v(count).xPrev = p_near(1);    
    T.v(count).yPrev = p_near(2);
    T.v(count).dist = min_distance;  
    T.v(count).cost = p_near_cost + min_distance; 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %檢查圓內所有節點，若以Xnew作為父節點，是否會使cost降低，有的話就讓該節點的父節點定義為Xnew
    for i=1:in_Radius_num
        if sqrt( ( T.v(inRadius(i)).x - T.v(count).x )^2 + ( T.v(inRadius(i)).y - T.v(count).y )^2 ) + T.v(count).cost <  T.v(inRadius(i)).cost
            t_near_check(1) = T.v(inRadius(i)).x;
            t_near_check(2) = T.v(inRadius(i)).y;
            new_prev(1) =T.v(count).xPrev;
            new_prev(2) = T.v(count).yPrev;
            if ~collisionChecking(t_near_check, p_new, obs_num, obs_pos, obs_sz) 
                continue;
            end
            if ~angleChecking( new_prev,p_new,t_near_check)
                continue;
            end
            T.v(inRadius(i)).xPrev=T.v(count).x;
            T.v(inRadius(i)).yPrev=T.v(count).y;
            T.v(inRadius(i)).dist=sqrt( ( T.v(inRadius(i)).x - T.v(count).x )^2 + ( T.v(inRadius(i)).y - T.v(count).y )^2 );
            T.v(inRadius(i)).cost=sqrt( ( T.v(inRadius(i)).x - T.v(count).x )^2 + ( T.v(inRadius(i)).y - T.v(count).y )^2 ) + T.v(count).cost;
        end
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %判斷是否接觸到終點
    new_distance = sqrt( ( p_new(1) - form_goal(1) )^2 + ( p_new(2) - form_goal(2) )^2 );
    if new_distance <= Thr
        plot(p_new(1), p_new(2), 'bo', 'MarkerSize',2, 'MarkerFaceColor','k'); % 繪製x_new
        line( [p_new(1) p_near(1)], [p_new(2) p_near(2)], 'Marker','.','LineStyle','-'); %連接x_near和x_new
        line( [form_goal(1) p_new(1)], [form_goal(2) p_new(2)], 'Marker','.','LineStyle','-'); %連接x_Target和x_new
        break;
    end
    
   
    plot(p_new(1), p_new(2), 'bo', 'MarkerSize',2, 'MarkerFaceColor','b');
    line( [p_new(1) p_near(1)], [p_new(2) p_near(2)], 'Marker','.','LineStyle','-'); 
    hold on;
%     %錄影
%     frame = getframe(figure(1));            
%     writeVideo(writerObj,frame); 
    pause(0); 
end

%建立樹所有節點的矩陣列表
T_LIST = zeros(size(T.v, 2), 5);
for i=1:size(T.v, 2)
    T_LIST(i,1) = T.v(i).x;
    T_LIST(i,2) = T.v(i).y;
    T_LIST(i,3) = T.v(i).xPrev;
    T_LIST(i,4) = T.v(i).yPrev;
    T_LIST(i,5) = i;
end
%逆向回推路徑 
path = [];
path_count = 1;
path(path_count,1) = form_goal(1);
path(path_count,2) = form_goal(2);
path_count = path_count + 1;
path(path_count,1) = p_new(1);
path(path_count,2) = p_new(2);
n_index = node_index(T_LIST, p_new(1), p_new(2));
path_count = path_count + 1;
path(path_count,1) = T_LIST(i,3);
path(path_count,2) = T_LIST(i,4);
while path(path_count,1) ~= form_start(1) || path(path_count,2) ~= form_start(2)
    new_n_index = node_index(T_LIST, path(path_count,1), path(path_count,2));
    path_count = path_count + 1;
    path(path_count,1) = T_LIST(new_n_index,3);
    path(path_count,2) = T_LIST(new_n_index,4);
    n_index = new_n_index;
end

%b-spline路徑平滑
X=path(:,1);
X=X';
X=[X(1),X(1),X,X(end),X(end)];
Y=path(:,2);
Y=Y';
Y=[Y(1),Y(1),Y,Y(end),Y(end)];
B = [
        [1,4,1,0];
        [-3,0,3,0];
        [3,-6,3,0];
        [-1,3,-3,1];
    ];
A = (1/6)*B;
C = [];
N = size(X,2);
for i=2:N-2
    for t=0:0.01:1
        D = [1,t,t^2,t^3]*A*[[X(i-1),Y(i-1)];[X(i),Y(i)];[X(i+1),Y(i+1)];[X(i+2),Y(i+2)]];
        C = [C;D];    
    end
end
C = flip(C,1);

plot(X,Y,'r',C(:,1),C(:,2),'bl');

% figure(2);
% plot(form_start(1), form_start(2), 'ro', 'MarkerSize',3, 'MarkerFaceColor','b');
% plot(form_goal(1), form_goal(2), 'go', 'MarkerSize',3, 'MarkerFaceColor','y');
% for i = 1:1:obs_num
%     h = circle(obs_pos(1,1,i), obs_pos(2,1,i), obs_sz(i));
% end
% axis([0 map_len 0 map_len],'square');
% hold on
% plot(C(:,1),C(:,2),'bl');
%%
%%%%%%%%%%%%%%---------------Potential---------------%%%%%%%%%%%%%%%%%
str="";         %顯示結果
%Program
run = 1;
jump = 0;
iter = 1;
hd_ori = atan2(C(iter+1,2)-C(iter,2), C(iter+1,1)-C(iter,1));
while run == 1

    pre_ori = hd_ori;
%     hd_ori = (C(iter+1,2)-C(iter,2))/(C(iter+1,1)-C(iter,1));
    hd_ori = atan2(C(iter+1,2)-C(iter,2), C(iter+1,1)-C(iter,1));
%     if C(iter+1,1)-C(iter,1) < 0 && C(iter+1,2)-C(iter,2) > 0       %第二象限角
%         hd_ori = (pi/2) - hd_ori;
    if (C(iter+1,2)-C(iter,2)) < 0   %第三四象限角
        hd_ori = 2*pi + hd_ori;
    elseif C(iter+1,1)-C(iter,1) == 0 && C(iter+1,2)-C(iter,2) > 0
        hd_ori = pi/2;
    elseif C(iter+1,2)-C(iter,2) == 0
        hd_ori = pre_ori;
    elseif C(iter+1,1)-C(iter,1) == 0 && C(iter+1,2)-C(iter,2) < 0
        hd_ori = 3*pi/4;
    end

    if hd_ori - pre_ori > 5*pi/180          %隊形轉角過大，維持上一時刻轉角
        hd_ori = pre_ori;
    end
disp(hd_ori*180/pi);

    veh_pos(:,:,1) = vhc1_pos;
    veh_pos(:,:,2) = vhc2_pos;
    veh_pos(:,:,3) = vhc3_pos;

    R = [cos(hd_ori) -sin(hd_ori); sin(hd_ori) cos(hd_ori)];

    i = -1; j = -1;         %i, j為min的所在座標
    g_shift = 0.05; b_shift = 0.05;

    if jump == 0
        ga_srh = gamma;
        be_srh = beta;
    else
        ga_srh = ga_min;
        be_srh = be_min;
    end

    while i<5 || i>95 || j<5 || j>95

        for g = (ga_srh-g_shift):0.002:(ga_srh-g_shift+0.1)
            for b = (be_srh-b_shift):0.002:(be_srh-b_shift+0.1)
               
                F = [1 0; g b];
                
                i = int32((g-(ga_srh - g_shift))*500 + 1);
                j = int32((b-(be_srh - b_shift))*500 + 1);
                J(i, j) = ((del(F*R*S_12)/del(S_12))^(-2) + (del(F*R*S_12)/del(S_12))^(2) + ...
                          (del(F*R*S_13)/del(S_13))^(-2) + (del(F*R*S_13)/del(S_13))^(2) + ...
                          (del(F*R*S_23)/del(S_23))^(-2) + (del(F*R*S_23)/del(S_23))^(2))*3;
                for n = 1:1:obs_num
%                     for m = 1:1:3
%                         dist = del(obs_pos(:,:,n) - vhc_pos(:,:,m));        %載具與障礙物形心間的距離
%                         if dist - obs_sz(n) < 10                            %載具與障礙物邊界的距離
%                             J(i, j) = J(i, j) + ((del(obs_pos(:,:,n) - F*R*veh_pos - form_pos) / (2*form_r))^(-2))*obs_sz(n);
%                         end
%                     end
                    J(i, j) = J(i, j) + ...
                              ((del(obs_pos(:,:,n) - F*R*S_1 - form_pos) / (2*form_r))^(-2) + ...
                              (del(obs_pos(:,:,n) - F*R*S_2 - form_pos) / (2*form_r))^(-2) + ...
                              (del(obs_pos(:,:,n) - F*R*S_3 - form_pos) / (2*form_r))^(-2))*obs_sz(n);
                    
                end      
            end
        end
    
        [M,I] = min(J(:));
        [i, j] = ind2sub(size(J),I);

        if i<5
            g_shift = g_shift+0.05;
        end
        if i>95
            g_shift = g_shift-0.05;
        end

        if j<5
            b_shift = b_shift+0.05;
        end
        if j>95
            b_shift = b_shift-0.05;
        end
    end

% disp([i, j, M]);
    ga_pre = gamma;
    be_pre = beta;
    gamma = (i-1)/500 + (ga_srh - g_shift);
    beta = (j-1)/500 + (be_srh - b_shift);
% disp([gamma, beta, M]);

    ro = sqrt((ga_pre - gamma)^(2) + (be_pre - beta)^(2));
% dist = sqrt((ga_pre - gamma)^(2) + (be_pre - beta)^(2));
% disp([ro, "before"]);
    if ro == 0
        step = 0;
    else
        step = 0.02/ro;
        if step > ro
            step = ro;
        end
    end

    jump = 0;
% if sqrt((ga_pre - gamma)^(2) + (be_pre - beta)^(2)) > 0.3      %如果更新幅度過大(出現跳點)
%     jump = 1;

    gdn_ga = (ptnl(ga_pre + 10.^(-12), be_pre, form_pos, obs_num, obs_pos, obs_sz, R) - ptnl(ga_pre - 10.^(-12), be_pre, form_pos, obs_num, obs_pos, obs_sz, R)) / (2*10.^(-12));
    gdn_be = (ptnl(ga_pre, be_pre + 10.^(-12), form_pos, obs_num, obs_pos, obs_sz, R) - ptnl(ga_pre, be_pre - 10.^(-12), form_pos, obs_num, obs_pos, obs_sz, R)) / (2*10.^(-12));
    vec = [gdn_ga gdn_be];
    
    vec_uni = [vec(1)/del(vec), vec(2)/del(vec)]; 
    
    ga_min = gamma;
    be_min = beta;
    gamma = ga_pre + (ga_min - ga_pre)/abs(ga_min - ga_pre)*abs(vec_uni(1))*step;       %step scale: 0.05
    beta = be_pre + (be_min - be_pre)/abs(be_min - be_pre)*abs(vec_uni(2))*step;  
    if ga_min == ga_pre
        gamma = ga_pre;
    end
    if be_min == be_pre
        beta = be_pre;
    end

if sqrt((ga_min - gamma)^(2) + (be_min - beta)^(2)) > 0.001      %如果更新幅度過大(出現跳點)
    jump = 1;
end

dist = sqrt((ga_pre - gamma)^(2) + (be_pre - beta)^(2));
% disp([dist, "after"]);
disp([gamma, beta, M]);

    F = [1 0; gamma beta];

    vhc1_pos = form_pos + F*R*S_1;
    vhc2_pos = form_pos + F*R*S_2;
    vhc3_pos = form_pos + F*R*S_3;

    figure(2);
    plot(form_start(1), form_start(2), 'ro', 'MarkerSize',3, 'MarkerFaceColor','b');
    plot(form_goal(1), form_goal(2), 'go', 'MarkerSize',3, 'MarkerFaceColor','y');
    for i = 1:1:obs_num
        h = circle(obs_pos(1,1,i), obs_pos(2,1,i), obs_sz(i));
    end
    plot(C(:,1),C(:,2),'bl');
    plot(form_start(1),form_start(2),'*b',form_goal(1),form_goal(2),'pr',...         %start and goal
        form_pos(1),form_pos(2),'.b',...
        vhc1_pos(1),vhc1_pos(2),'+r',...
        vhc2_pos(1),vhc2_pos(2),'+k',...
        vhc3_pos(1),vhc3_pos(2),'+b',...
        obs_pos(1),obs_pos(2),'*r')                                          %robot position/半徑
        
    axis([0 map_len 0 map_len],'square');     %顯示方形
    hold off
    drawnow;

%     if jump == 0       %形變參數更新幅度過，形心不動
%         form_pos = form_pos + form_vlc;
%     else
%         form_pos = form_pos + 0.2*form_vlc;
%     end
    form_pos(1) = C(iter, 1) - form_r*cos(hd_ori);
    form_pos(2) = C(iter, 2) - form_r*sin(hd_ori);

    %抵達終點
    if form_pos(1)==form_goal(1) && form_pos(2)==form_goal(2)
        run=0;
        str="Arrival";
        disp(str);
    end

    iter = iter + 5;
end
disp([gamma, beta, M]);

%%
%Function
function delta = del(vec)   %計算向量大小
    delta = sqrt(vec(1)^2 + vec(2)^2); 
end

function J = ptnl(ga, be, f_pos, o_num, o_pos, o_sz, R)          %form_size = 10
    F = [1 0; ga be];
    S_1 = [0; 10];                          %X_1-X_c
    S_2 = [-10*cosd(30); -10*sind(30)];     %X_2-X_c
    S_3 = [10*cosd(30); -10*sind(30)];      %X_3-X_c
    S_12 = S_1 - S_2;
    S_13 = S_1 - S_3;
    S_23 = S_2 - S_3;
    J = ((del(F*R*S_12)/del(S_12))^(-2) + (del(F*R*S_12)/del(S_12))^(2) + ...
        (del(F*R*S_13)/del(S_13))^(-2) + (del(F*R*S_13)/del(S_13))^(2) + ...
        (del(F*R*S_23)/del(S_23))^(-2) + (del(F*R*S_23)/del(S_23))^(2))*3;
    for n = 1:1:o_num
%         for m = 1:1:3
%             dist = del(o_pos(:,:,n) - v_pos(:,:,m));
%             if dist - o_sz(n) < 10
%                 J = J + ((del(o_pos(:,:,n) - F*R*form_S(:,:,m) - f_pos) / (2*10))^(-2))*o_sz(n);
%             end
%         end

        J = J + ((del(o_pos(:,:,n) - F*R*S_1 - f_pos) / (2*10))^(-2) + ...
                (del(o_pos(:,:,n) - F*R*S_2 - f_pos) / (2*10))^(-2) + ...
                (del(o_pos(:,:,n) - F*R*S_3 - f_pos) / (2*10))^(-2))*o_sz(n);
    end
end

%%%%%%%%%%%%%%%%%%%%%%%-----collision checking---------------%%%%%%%%%%%%%%%
function feasible=collisionChecking(start_p, end_p, obs_num, obs_pos, obs_sz)
    feasible=true;
    dir=atan2(end_p(1)-start_p(1),end_p(2)-start_p(2));
    for r=0:0.5:sqrt(sum((start_p-end_p).^2))

        check_p = start_p + r.*[sin(dir) cos(dir)];
        for i = 1:1:obs_num
            dist = sqrt((check_p(1)-obs_pos(1,1,i))^2 + (check_p(2)-obs_pos(2,1,i))^2);
            if(dist < obs_sz(i)+9)          %save distance = 9
                feasible=false;
                break;
            end
        end
    end
end
 
%%%%%%%%%%%%%%%%%%%%%%%-----angle checking---------------%%%%%%%%%%%%%%%
function inconstraint = angleChecking(preStartPose,startPose,goalPose)
    inconstraint = true;
    vector1=[(startPose(2)-preStartPose(2)),(startPose(1)-preStartPose(1))];
    vector2=[(goalPose(2)-startPose(2)),(goalPose(1)-startPose(1))];
    if vector1 == [0,0]
        inconstraint = true;
        return
    end
    angle = mod(atan2d(det([vector1;vector2]),dot(vector1,vector2)),360);
    if ~(angle<60 || angle>300)
        inconstraint = false;
    end
end

%%%%%%%%%%%%%%%%%%%%%%%-----func 4 b-spline---------------%%%%%%%%%%%%%%%
function n_index = node_index(T_LIST,xval,yval)
    i=1;
    while ( T_LIST(i,1) ~= xval || T_LIST(i,2) ~= yval )
        i=i+1;
    end
    n_index=i;
end

function h = circle(x,y,r)
    hold on
    th = 0:pi/50:2*pi;
    xunit = r * cos(th) + x;
    yunit = r * sin(th) + y;
    h = plot(xunit, yunit);
end

 


    


