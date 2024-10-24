clc;
clear all;
z_mat = [0.1,0,0,0,0,0; 0,0,0,0,0,0];

theta1 = [0,0,0,0,0,0];
theta_tri = [pi,pi,pi,pi,pi,pi];
theta_cate = [2*pi/3, 2*pi/3, 2*pi/3, 2*pi/3, 2*pi/3, 2*pi/3];
theta_lurch = [pi,pi,0,pi,pi,0];
theta_metach = [pi/3,pi/3,pi/3,pi/3,pi/3,pi/3];
dt = 0;
state_vec1 = [];
state_vec2 = [];
state_vec3 = [];
state_vec4 = [];
state_vec5 = [];
state_vec6 = [];
time_list = [];

gamma = 1;

beta = 0.0;
current_theta = theta_tri;

r = rateControl(200);
start_code = tic;
duration = 0;
while 1

    start_loop = tic;
    
    if duration<=5
        z_mat = my_diffusive_hopf(z_mat,dt, gamma,theta_tri);
    
    else
        target_theta = theta_lurch;
        % new_theta = beta*current_theta + (1-beta)*target_theta;
        % z_mat = my_diffusive_hopf(z_mat,dt, gamma,new_theta);
        z_mat = my_diffusive_hopf(z_mat,dt, gamma,target_theta);
        % current_theta = new_theta;
    end
    
    
    state_vec1 = [state_vec1,z_mat(1,1)];
    state_vec2 = [state_vec2,z_mat(1,2)];
    state_vec3 = [state_vec3,z_mat(1,3)];
    state_vec4 = [state_vec4,z_mat(1,4)];
    state_vec5 = [state_vec5,z_mat(1,5)];
    state_vec6 = [state_vec6,z_mat(1,6)];
    time_list = [time_list, duration];
    
    disp(duration)

    if duration>=10
        break;
    end

    waitfor(r);
    dt = toc(start_loop);
    duration = toc(start_code);


end

figure(1)
hold on

bx1 = [0 0 5 5];
by1 = [1.5 -1.5 -1.5 1.5];
fill(bx1,by1,'black','FaceAlpha',0.1, 'LineStyle','none')

txt = '\theta_{tripod}';
text(4.5,1.35,txt)

txt = '\theta_{lurch}';
text(7.5,1.35,txt)

bx2 = [5 5 10 10];
by2 = [1.5 -1.5 -1.5 1.5];
fill(bx2,by2,'black','FaceAlpha',0.2, 'LineStyle','none')


x1=plot(time_list,state_vec1, 'color',[1,0.2,0], 'LineWidth',6)
x2=plot(time_list,state_vec2, 'color',[1,0.7,0], 'LineWidth',5)
x3=plot(time_list,state_vec3, 'color',[0.5,1,0], 'LineWidth',4)
x4=plot(time_list,state_vec4, 'color',[0,0.7,1], 'LineWidth',3)
x5=plot(time_list,state_vec5, 'color',[0.478,0,1], 'LineWidth',2)
x6=plot(time_list,state_vec6, 'color',[0.957,0,1], 'LineWidth',1)
x0=10;
y0=10;
width=600;
height=200;
set(gcf,'position',[x0,y0,width,height])
ax = gca; 
ax.FontSize = 10; 
leg = legend([x1,x2,x3,x4,x5,x6], ["x1","x2","x3","x4","x5","x6"], 'Orientation','horizontal')
leg.ItemTokenSize = [9,1];
leg.Location = 'southwest';
xlabel('Time(s)','FontSize',10)
% ylabel('CPG outputs')
% 300，600  - 300,500

xlim([4, 10]);
ylim([-1.5,1.5]);

