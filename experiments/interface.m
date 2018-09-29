function varargout = interface(varargin)
% INTERFACE MATLAB code for interface.fig
%      INTERFACE, by itself, creates a new INTERFACE or raises the existing
%      singleton*.
%
%      H = INTERFACE returns the handle to a new INTERFACE or the handle to
%      the existing singleton*.
%
%      INTERFACE('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in INTERFACE.M with the given input arguments.
%
%      INTERFACE('Property','Value',...) creates a new INTERFACE or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before interface_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to interface_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help interface

% Last Modified by GUIDE v2.5 13-Sep-2018 22:45:29

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @interface_OpeningFcn, ...
                   'gui_OutputFcn',  @interface_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before interface is made visible.
function [] = interface_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to interface (see VARARGIN)

% Choose default command line output for interface
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes interface wait for user response (see UIRESUME)
% uiwait(handles.figure1);
addpath ../derivation
addpath ../derivation/generated
clc

% --- Outputs from this function are returned to the command line.
function varargout = interface_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

% ---------------------------------------------------------------------
%       User Defined Functionalities
% ---------------------------------------------------------------------


% --- Executes on button press in BTN_EXP_Init_ROS.
function BTN_EXP_Init_ROS_Callback(hObject, eventdata, handles)
% hObject    handle to BTN_EXP_Init_ROS (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global reset_client read_obj_pose_client get_robot_pose_client move_tool_client
global move_hybrid_client move_until_touch_client

% rosinit;

reset_client         = rossvcclient('/robot_bridge/reset');
% read_obj_pose_client = rossvcclient('/robot_bridge/read_obj_pose');
get_robot_pose_client = rossvcclient('/robot_bridge/get_pose');
move_tool_client     = rossvcclient('/robot_bridge/move_tool');
move_hybrid_client        = rossvcclient('/robot_bridge/move_hybrid');
move_until_touch_client           = rossvcclient('/robot_bridge/move_until_touch');

set(handles.BTN_EXP_Init_ROS, 'Enable', 'off');
set(handles.BTN_EXP_Reset, 'Enable', 'on');
set(handles.BTN_EXP_read_obj_pose, 'Enable', 'on');
set(handles.BTN_EXP_Pre_Grasp, 'Enable', 'off');
set(handles.BTN_EXP_Engage, 'Enable', 'off');
set(handles.BTN_EXP_Planning, 'Enable', 'off');
% set(handles.BTN_EXP_Run, 'Enable', 'off');
set(handles.BTN_EXP_Release_Reset, 'Enable', 'off');

disp('Initialization is done. Service clients are ready to use.');

% --- Executes on button press in BTN_EXP_Reset.
function BTN_EXP_Reset_Callback(hObject, eventdata, handles)
% hObject    handle to BTN_EXP_Reset (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global reset_client
disp('Calling Reset_service:');
call(reset_client);
disp('Reset is done.');

set(handles.BTN_EXP_Init_ROS, 'Enable', 'off');
set(handles.BTN_EXP_Reset, 'Enable', 'off');
set(handles.BTN_EXP_read_obj_pose, 'Enable', 'on');
set(handles.BTN_EXP_Pre_Grasp, 'Enable', 'off');
set(handles.BTN_EXP_Engage, 'Enable', 'off');
set(handles.BTN_EXP_Planning, 'Enable', 'off');
set(handles.BTN_EXP_Release_Reset, 'Enable', 'off');

% --- Executes on button press in BTN_EXP_read_obj_pose.
function BTN_EXP_read_obj_pose_Callback(hObject, eventdata, handles)
% hObject    handle to BTN_EXP_read_obj_pose (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global read_obj_pose_client inputs

% disp('Calling read_obj_pose service:');
% call(read_obj_pose_client);

switch get(get(handles.BTNGROUP_select_experiment,'SelectedObject'),'Tag')
    case 'RBTN_block_tilting'
        inputs.ObjectEdgeLength = 0.075;
        inputs.kGoalRotationVelocity = 0.1; % rad/s
        inputs.zTableTop = 0.259;
        p_WCenter0 = [30.75 439.5 259]'/1000;
        switch get(get(handles.BTNGROUP_block_tilting_direction,'SelectedObject'),'Tag')
            case 'RBTN_X_plus'
                inputs.p_WO0 = p_WCenter0 + ...
                        [inputs.ObjectEdgeLength/2, 0, 0]';
                inputs.TiltDirection = [1 0 0]';
                inputs.m_project = diag([1 0 1]);
            case 'RBTN_Y_minus'
                inputs.p_WO0 = p_WCenter0 + ...
                        [inputs.ObjectEdgeLength, -inputs.ObjectEdgeLength/2, 0]';
                inputs.TiltDirection = [0 -1 0]';
                inputs.m_project = diag([0 1 1]);
            case 'RBTN_X_minus'
                inputs.p_WO0 = p_WCenter0 + ...
                        [inputs.ObjectEdgeLength/2, -inputs.ObjectEdgeLength, 0]';
                inputs.TiltDirection = [-1 0 0]';
                inputs.m_project = diag([1 0 1]);
            case 'RBTN_Y_plus'
                inputs.p_WO0 = p_WCenter0 + ...
                        [0, -inputs.ObjectEdgeLength/2, 0]';
                inputs.TiltDirection = [0 1 0]';
                inputs.m_project = diag([0 1 1]);
            otherwise
                error('[Error] unknown selected object.');
        end
        inputs.p_WH0 = inputs.p_WO0 - ...
                0.15*inputs.ObjectEdgeLength*inputs.TiltDirection + ...
                [0 0 inputs.ObjectEdgeLength]';
        inputs.p_WHPrepare_mm = 1000*(inputs.p_WH0 + [0 0 0.2*inputs.ObjectEdgeLength]');
    case 'RBTN_flip_against_corner'
        inputs.kObjectLength         = 0.075;
        inputs.kObjectThickness      = 0.035;
        inputs.kGoalRotationVelocity = 0.05;
        inputs.p_WH0                 = [400.4 268]'/1000;
        inputs.p_WO0                 = [356.5 277.5]'/1000;
        inputs.p_WHPrepare_mm = [30 410 268];
    otherwise
        error('[Error] unknown selected object.');
end

set(handles.BTN_EXP_Pre_Grasp, 'Enable', 'on');
set(handles.BTN_EXP_Engage, 'Enable', 'off');
set(handles.BTN_EXP_Planning, 'Enable', 'off');
set(handles.BTN_EXP_Release_Reset, 'Enable', 'off');

disp('Read obj pose is done.');


% --- Executes on button press in BTN_EXP_Pre_Grasp.
function BTN_EXP_Pre_Grasp_Callback(hObject, eventdata, handles)
% hObject    handle to BTN_EXP_Pre_Grasp (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global move_tool_client get_robot_pose_client inputs

% obtain orientation
call(get_robot_pose_client);
fp_feedback   = fopen('../results/pose_feedback.txt','r');
pose_feedback = fscanf(fp_feedback, '%f');
fclose(fp_feedback);

assert(length(pose_feedback) == 7);

pose_set      = pose_feedback;
pose_set(1:3) = inputs.p_WHPrepare_mm;
% warning('Hacked here!!!!');
% pose_set = [30 410 268 0.11626 0.5394 -0.81523 0.17587];

fp = fopen('../results/pose_set.txt','w');
fprintf(fp, '%f ', pose_set);
fclose(fp);

disp('Calling move_tool_service:');
call(move_tool_client);

set(handles.BTN_EXP_Init_ROS, 'Enable', 'off');
set(handles.BTN_EXP_Reset, 'Enable', 'on');
set(handles.BTN_EXP_read_obj_pose, 'Enable', 'off');
set(handles.BTN_EXP_Pre_Grasp, 'Enable', 'off');
set(handles.BTN_EXP_Engage, 'Enable', 'on');
set(handles.BTN_EXP_Planning, 'Enable', 'off');
set(handles.BTN_EXP_Release_Reset, 'Enable', 'off');

disp('Pre Grasp is done.');

% --- Executes on button press in BTN_EXP_Engage.
function BTN_EXP_Engage_Callback(hObject, eventdata, handles)
% hObject    handle to BTN_EXP_Engage (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global move_until_touch_client get_robot_pose_client

switch get(get(handles.BTNGROUP_select_experiment,'SelectedObject'),'Tag')
    case 'RBTN_block_tilting'
        velocity_set = 5*[0 0 -1]'; % mm/s
    case 'RBTN_flip_against_corner'
        velocity_set = 5*[0 -1 0]'; % mm/s
    otherwise
        error('[Error] unknown selected object.');
end

fp = fopen('../results/velocity_set.txt','w');
fprintf(fp, '%f ', velocity_set);
fclose(fp);

disp('Calling move_until_touch_service:');
call(move_until_touch_client);

% % feedback
% call(get_robot_pose_client);
% fp_feedback = fopen(['../results/pose_feedback.txt'],'r');
% pose_feedback = fscanf(fp_feedback, '%f');
% fclose(fp_feedback);
% pose_feedback(1:3) = pose_feedback(1:3)/1000;

set(handles.BTN_EXP_Init_ROS, 'Enable', 'off');
set(handles.BTN_EXP_Reset, 'Enable', 'on');
set(handles.BTN_EXP_read_obj_pose, 'Enable', 'off');
set(handles.BTN_EXP_Pre_Grasp, 'Enable', 'off');
set(handles.BTN_EXP_Engage, 'Enable', 'on');
set(handles.BTN_EXP_Planning, 'Enable', 'on');
set(handles.BTN_EXP_Release_Reset, 'Enable', 'off');

disp('Engaging is done.');

% --- Executes on button press in BTN_EXP_Planning.
function BTN_EXP_Planning_Callback(hObject, eventdata, handles)
% hObject    handle to BTN_EXP_Planning (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
clc;
global get_robot_pose_client move_hybrid_client move_tool_client inputs

disp('[Planning] Planning begin.');

switch get(get(handles.BTNGROUP_select_experiment,'SelectedObject'),'Tag')
    case 'RBTN_block_tilting'
        kTotalRotationAngle = 50*pi/180;
    case 'RBTN_flip_against_corner'
        kTotalRotationAngle = 50*pi/180;
    otherwise
        error('[Error] unknown selected object.');
end

total_time_s = kTotalRotationAngle/inputs.kGoalRotationVelocity;
kNumberOfSteps = 20;

for steps = 1:kNumberOfSteps+5
    % read feedback
    call(get_robot_pose_client);
    fp_feedback = fopen(['../results/pose_feedback.txt'],'r');
    pose_feedback = fscanf(fp_feedback, '%f');
    fclose(fp_feedback);
    pose_feedback(1:3) = pose_feedback(1:3)/1000;

    % feedback and check stop condition
    switch get(get(handles.BTNGROUP_select_experiment,'SelectedObject'),'Tag')
        case 'RBTN_block_tilting'
            inputs.p_WH = pose_feedback(1:3);
            p_WO = inputs.p_WO0;
            v_C2C0 = inputs.p_WH0 - inputs.p_WO0;
            angle_rotated = angBTVec(inputs.m_project*v_C2C0, inputs.m_project*(inputs.p_WH - p_WO));
        case 'RBTN_flip_against_corner'
            inputs.p_WH = pose_feedback(2:3);
            a = inputs.p_WH0(2) - inputs.p_WO0(2) + inputs.kObjectThickness/2;
            b = inputs.kObjectLength;
            c = inputs.p_WH(2) - inputs.p_WO0(2) + inputs.kObjectThickness/2;
            phi = atan2(a, b);
            theta_plus_phi = asin(c/norm([a, b]));
            angle_rotated = theta_plus_phi - phi;
    end
    if (angle_rotated > kTotalRotationAngle)
        break;
    end

    % run the algorithm
    tic
    switch get(get(handles.BTNGROUP_select_experiment,'SelectedObject'),'Tag')
        case 'RBTN_block_tilting'
            [n_av, n_af, R_a, w_v, eta_f] = example_2D_block_tilting(inputs);
        case 'RBTN_flip_against_corner'
            [n_av, n_af, R_a, w_v, eta_f] = example_flip_against_corner(inputs);
    end
    toc

    linear_velocity = norm(w_v) * 1000;
    w_set = [zeros(n_af, 1); w_v];
    v_set = R_a^-1*w_set;
    pose_set = pose_feedback;
    switch get(get(handles.BTNGROUP_select_experiment,'SelectedObject'),'Tag')
        case 'RBTN_block_tilting'
            pose_set(1:3) = pose_set(1:3) + v_set*total_time_s/kNumberOfSteps;
            pose_set(1:3) = pose_set(1:3)*1000;
        case 'RBTN_flip_against_corner'
            pose_set(2:3) = pose_set(2:3) + v_set*total_time_s/kNumberOfSteps;
            pose_set(1:3) = pose_set(1:3)*1000;
            R_a = [[0;0] R_a; 1 0 0];
            n_av = n_av + 1; % for X
    end
    force_set = zeros(6,1);
    force_set(1:n_af) = eta_f;
    % move hybrid
    fp = fopen(['../results/hybrid_action.txt'],'w');
    fprintf(fp, '%d %f ', n_av, linear_velocity);
    fprintf(fp, '%f ', pose_set);
    fprintf(fp, '%f %f %f %f %f %f %f %f %f ', R_a(1,1), R_a(1,2), R_a(1,3), ...
                                               R_a(2,1), R_a(2,2), R_a(2,3), ...
                                               R_a(3,1), R_a(3,2), R_a(3,3));
    fprintf(fp, '%f ', force_set);
    fclose(fp);

    disp('Calling move_hybrid_service:');
    call(move_hybrid_client);
end

% disengage
switch get(get(handles.BTNGROUP_select_experiment,'SelectedObject'),'Tag')
    case 'RBTN_block_tilting'
        pose_set = pose_feedback;
        pose_set(1:3) = pose_set(1:3) + inputs.TiltDirection*0.04;
        pose_set(1:3) = pose_set(1:3)*1000;
        fp = fopen(['../results/pose_set.txt'],'w');
        fprintf(fp, '%f ', pose_set);
        fclose(fp);
        disp('Calling move_tool_service:');
        call(move_tool_client);

    case 'RBTN_flip_against_corner'
end

set(handles.BTN_EXP_Planning, 'Enable', 'off');
set(handles.BTN_EXP_Release_Reset, 'Enable', 'on');

disp('Planning is done.')

% --- Executes on button press in BTN_EXP_Release_Reset.
function BTN_EXP_Release_Reset_Callback(hObject, eventdata, handles)
% hObject    handle to BTN_EXP_Release_Reset (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global get_robot_pose_client move_tool_client inputs reset_client

% read feedback
call(get_robot_pose_client);
fp_feedback = fopen('../results/pose_feedback.txt','r');
pose_feedback = fscanf(fp_feedback, '%f');
fclose(fp_feedback);

% move up
pose_set = pose_feedback;
pose_set(3) = pose_set(3) + 30;
fp = fopen('../results/pose_set.txt','w');
fprintf(fp, '%f ', pose_set);
fclose(fp);

disp('Calling move_tool_service:');
call(move_tool_client);

% call reset
call(reset_client);

set(handles.BTN_EXP_Init_ROS, 'Enable', 'off');
set(handles.BTN_EXP_Reset, 'Enable', 'on');
set(handles.BTN_EXP_read_obj_pose, 'Enable', 'on');
set(handles.BTN_EXP_Pre_Grasp, 'Enable', 'off');
set(handles.BTN_EXP_Engage, 'Enable', 'off');
set(handles.BTN_EXP_Planning, 'Enable', 'off');
set(handles.BTN_EXP_Release_Reset, 'Enable', 'off');
disp('Release_reset is done.');

% --- Executes on button press in RBTN_X_plus.
function RBTN_X_plus_Callback(hObject, eventdata, handles)
% hObject    handle to RBTN_X_plus (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of RBTN_X_plus
