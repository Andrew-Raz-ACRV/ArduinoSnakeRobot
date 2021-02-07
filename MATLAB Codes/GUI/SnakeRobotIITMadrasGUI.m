function varargout = SnakeRobotIITMadrasGUI(varargin)
% SNAKEROBOTIITMADRASGUI MATLAB code for SnakeRobotIITMadrasGUI.fig
%      SNAKEROBOTIITMADRASGUI, by itself, creates a new SNAKEROBOTIITMADRASGUI or raises the existing
%      singleton*.
%
%      H = SNAKEROBOTIITMADRASGUI returns the handle to a new SNAKEROBOTIITMADRASGUI or the handle to
%      the existing singleton*.
%
%      SNAKEROBOTIITMADRASGUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in SNAKEROBOTIITMADRASGUI.M with the given input arguments.
%
%      SNAKEROBOTIITMADRASGUI('Property','Value',...) creates a new SNAKEROBOTIITMADRASGUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before SnakeRobotIITMadrasGUI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to SnakeRobotIITMadrasGUI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help SnakeRobotIITMadrasGUI

% Last Modified by GUIDE v2.5 04-Jun-2018 21:46:54

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @SnakeRobotIITMadrasGUI_OpeningFcn, ...
                   'gui_OutputFcn',  @SnakeRobotIITMadrasGUI_OutputFcn, ...
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


% --- Executes just before SnakeRobotIITMadrasGUI is made visible.
function SnakeRobotIITMadrasGUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to SnakeRobotIITMadrasGUI (see VARARGIN)

% Choose default command line output for SnakeRobotIITMadrasGUI
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes SnakeRobotIITMadrasGUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);


%Andrew, open new serial communication
% delete(instrfind({'Port'},{'COM5'}));
% close all
% clc
% 
% s=serial('COM5','BAUD', 9600); % Make sure the baud rate and COM port is 
%                               % same as in Arduino IDE
% fopen(s);

clear all
global q1 q2 q3 q4 q5 q6 target;
q1 = 0;
q2 = 0;
q3 = 0;
q4 = 0;
q5 = 0;
q6 = 0;
target = [0, 0, 0]';



% --- Outputs from this function are returned to the command line.
function varargout = SnakeRobotIITMadrasGUI_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


function q1_box_Callback(hObject, eventdata, handles)
% hObject    handle to q1_box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of q1_box as text
%        str2double(get(hObject,'String')) returns contents of q1_box as a double
global q1 q2 q3 q4 q5 q6 target;
q1 = str2double(get(hObject,'String'));
display(q1)
plot_snake_robot(q1,q2,q3,q4,q5,q6,target);

% --- Executes during object creation, after setting all properties.
function q1_box_CreateFcn(hObject, eventdata, handles)
% hObject    handle to q1_box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function q2_box_Callback(hObject, eventdata, handles)
% hObject    handle to q2_box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of q2_box as text
%        str2double(get(hObject,'String')) returns contents of q2_box as a double
global q1 q2 q3 q4 q5 q6 target;
q2 = str2double(get(hObject,'String'));
display(q2)
plot_snake_robot(q1,q2,q3,q4,q5,q6,target);

% --- Executes during object creation, after setting all properties.
function q2_box_CreateFcn(hObject, eventdata, handles)
% hObject    handle to q2_box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function q3_box_Callback(hObject, eventdata, handles)
% hObject    handle to q3_box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of q3_box as text
%        str2double(get(hObject,'String')) returns contents of q3_box as a double
global q1 q2 q3 q4 q5 q6 target;
q3 = str2double(get(hObject,'String'));
display(q3)
plot_snake_robot(q1,q2,q3,q4,q5,q6,target);

% --- Executes during object creation, after setting all properties.
function q3_box_CreateFcn(hObject, eventdata, handles)
% hObject    handle to q3_box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function q4_box_Callback(hObject, eventdata, handles)
% hObject    handle to q4_box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of q4_box as text
%        str2double(get(hObject,'String')) returns contents of q4_box as a double
global q1 q2 q3 q4 q5 q6 target;
q4 = deg2rad(str2double(get(hObject,'String')));
display(q4)
plot_snake_robot(q1,q2,q3,q4,q5,q6,target);

% --- Executes during object creation, after setting all properties.
function q4_box_CreateFcn(hObject, eventdata, handles)
% hObject    handle to q4_box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function q5_box_Callback(hObject, eventdata, handles)
% hObject    handle to q5_box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of q5_box as text
%        str2double(get(hObject,'String')) returns contents of q5_box as a double
global q1 q2 q3 q4 q5 q6 target;
q5 = deg2rad(str2double(get(hObject,'String')));
display(q5)
plot_snake_robot(q1,q2,q3,q4,q5,q6,target);

% --- Executes during object creation, after setting all properties.
function q5_box_CreateFcn(hObject, eventdata, handles)
% hObject    handle to q5_box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function q6_box_Callback(hObject, eventdata, handles)
% hObject    handle to q6_box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of q6_box as text
%        str2double(get(hObject,'String')) returns contents of q6_box as a double
global q1 q2 q3 q4 q5 q6 target;
q6 = deg2rad(str2double(get(hObject,'String')));
display(q6)
plot_snake_robot(q1,q2,q3,q4,q5,q6,target);

% --- Executes during object creation, after setting all properties.
function q6_box_CreateFcn(hObject, eventdata, handles)
% hObject    handle to q6_box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in move_sim.
function move_sim_Callback(hObject, eventdata, handles)
% hObject    handle to move_sim (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in move_arm.
function move_arm_Callback(hObject, eventdata, handles)
% hObject    handle to move_arm (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



function X_box_Callback(hObject, eventdata, handles)
% hObject    handle to X_box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of X_box as text
%        str2double(get(hObject,'String')) returns contents of X_box as a double
global q1 q2 q3 q4 q5 q6 target;
x = str2double(get(hObject,'String'));
display(x)
target(1) = x;
plot_snake_robot(q1,q2,q3,q4,q5,q6,target);

% --- Executes during object creation, after setting all properties.
function X_box_CreateFcn(hObject, eventdata, handles)
% hObject    handle to X_box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Y_box_Callback(hObject, eventdata, handles)
% hObject    handle to Y_box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Y_box as text
%        str2double(get(hObject,'String')) returns contents of Y_box as a double
global q1 q2 q3 q4 q5 q6 target;
y = str2double(get(hObject,'String'));
display(y)
target(2) = y;
plot_snake_robot(q1,q2,q3,q4,q5,q6,target);

% --- Executes during object creation, after setting all properties.
function Y_box_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Y_box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Z_box_Callback(hObject, eventdata, handles)
% hObject    handle to Z_box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Z_box as text
%        str2double(get(hObject,'String')) returns contents of Z_box as a double
global q1 q2 q3 q4 q5 q6 target;
z = str2double(get(hObject,'String'));
display(z)
target(3) = z;
plot_snake_robot(q1,q2,q3,q4,q5,q6,target);

% --- Executes during object creation, after setting all properties.
function Z_box_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Z_box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in invkin_move_sim.
function invkin_move_sim_Callback(hObject, eventdata, handles)
% hObject    handle to invkin_move_sim (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%Variables
global q1 q2 q3 q4 q5 q6 target;

%Constants
%For joint limit avoidance
Lower_Joint_Limits = [0, 0, 0, -4*pi]';
Upper_Joint_Limits = [100, 100, 100, 4*pi]';
r = 175.73; %mm

%Run loop
success = false;
disp('Starting Snake Robot Simulation')
timer = 0;
speed_limit = 5; %mm/loop

while (success==false)
    
    %Plot robot arm and find forward Kinematics
    [Base_T_tool] = plot_snake_robot(q1,q2,q3,q4,q5,q6,target);
    
    if rem(timer,3)==0
        pause(0.001)
    end
    
    
    %Extract position
    X = Base_T_tool(1:3,4);
    
    %Compute Error
    dX = target - X;
    
    %Logic to stop simulation based on target position
    if (norm(dX) < 0.1)
        success = true;
        disp('Simulation finished')
    end   
    
    %Control speed in case of big magnitude
    if (norm(dX)>speed_limit)
        dX = speed_limit*(dX/norm(dX));
    end
    
    %Compute Jacobian
    J = SnakeRobotJacobian(q2,q3,q5,r);
    
    %Compute Inverse
    inv_J = dampedLeastSquaresInverse(J,[q1,q2,q3,q5]',Lower_Joint_Limits,Upper_Joint_Limits);
    
    %Compute Update step
    dQ = inv_J*dX;
    
    %Compute new joint targets
    q1 = q1 + dQ(1);
    q2 = q2 + dQ(2);
    q3 = q3 + dQ(3);
    q5 = q5 + dQ(4);
    
    %Enforce Joint Limits in the model
    [q1,q2,q3] = enforceJointLimits(q1,q2,q3,Lower_Joint_Limits,Upper_Joint_Limits);
    
    %Increment timer
    timer = timer + 1;
    
    %Simulation time out
    if (timer>10000)
        success = true;
        disp('Failed to converge, Time out')
    end
          
end

function [q1,q2,q3] = enforceJointLimits(q1,q2,q3,Lower_Joint_Limits,Upper_Joint_Limits)
    %Enforces the prismatic tube joint limits
    Q = [q1,q2,q3];
    for ii = 1:3
        qmin = Lower_Joint_Limits(ii);
        qmax = Upper_Joint_Limits(ii);
        q = Q(ii);
        
        if (q<qmin)
            Q(ii) = qmin;
        elseif (q>qmax)
            Q(ii) = qmax;
        end
    end
    
    q1 = Q(1); q2 = Q(2); q3 = Q(3);




% --- Executes on button press in invkin_mov_arm.
function invkin_mov_arm_Callback(hObject, eventdata, handles)
% hObject    handle to invkin_mov_arm (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in clear_fig.
function clear_fig_Callback(hObject, eventdata, handles)
% hObject    handle to clear_fig (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in save_fig.
function save_fig_Callback(hObject, eventdata, handles)
% hObject    handle to save_fig (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in recordbutton.
function recordbutton_Callback(hObject, eventdata, handles)
% hObject    handle to recordbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in reset_button.
function reset_button_Callback(hObject, eventdata, handles)
% hObject    handle to reset_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global q1 q2 q3 q4 q5 q6 target;
q1 = 0;
q2 = 0;
q3 = 0;
q4 = 0;
q5 = 0;
q6 = 0;
target = [0, 0, 0]';
plot_snake_robot(q1,q2,q3,q4,q5,q6,target);

% --- Executes during object creation, after setting all properties.
function Axes1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Axes1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate Axes1

global q1 q2 q3 q4 q5 q6 target;
q1 = 0;
q2 = 0;
q3 = 0;
q4 = 0;
q5 = 0;
q6 = 0;
target = [0, 0, 0]';
plot_snake_robot(q1,q2,q3,q4,q5,q6,target);



