%%BROWNKINDEF12         -This program was created to solve the motion
%%identification problem for flapping wing flight in bats.  While the code
%%below was specifically created for this problem, it can be adapted to
%%solve any motion identification problem for an open kinematic chain.  The
%%data supplied to this program is 3D point trajectories of all joint links
%%and robotic configuration information in the form of a DH table.  The
%%output is graphs of the joint variable for each degree of freedom, plots 
%%of motion identificaiton verification (i.e. experimental vs. identified
%%trajectories in cartesian coordinates, and a plot of all experimental
%%points and all identified points on the kinematic chain for each time
%%step.

%%Future revisions of this code will contain the following:
%%------1. GUI based kinematic configuration definition.  Currently this is
%%done manually, but a GUI will make the process easier and less prone to
%%bugs. 

%%------2. GUI based data import.  Data will be associated to the correct
%%links and a configuration file will be created for future processing so
%%that the definition does not have to be performed for each run.

%%------3. Trajectory smoothing.  All of the identified trajectories 

%% Define Kinematic Chain Structure For Motion Identification Software
%This program is a manual assignment of the hierarchy implemented for the
%motion identification code which will follow.  It assigns node numbers and
%string identifiers, as well as the parent child relationships between
%nodes.  In the future this portion of the motion identification software
%will be automated.  

fprintf('Creating Kinematic Definition...\n')
% Node Names and Parent-Child Info
%             Node Name   Parent    Child       Connecting Point       Group   ID Kernal  
NodeNames   = {'BB_1'       ,   [] ,   [2,3],          [],               1,      'DH';        %1
               'BB_2'       ,   [1] ,  [4],            [1],              1,      'DH';        %2
               'BB_3'       ,   [1],   [6],            [1],              1,      'DH';        %3
               'RHum'       ,   [2],   [5],            [1],              2,      'DH';        %4
               'RRad'       ,   [4],   [8,9,10]        [1],              2,      'DH';        %5
               'LHum'       ,   [3],   [7],            [1],              2,      'DH';        %6
               'LRad'       ,   [6],   [11,12,13],     [1],              2,      'DH';        %7 'RWrist'     ,   [5],   [9,10,11],      [1],              3,      'DH';        %8
               'RD3Met'     ,   [5],   [],             [1],              3,      'DH';        %8
               'RD4Met'     ,   [5],   [],             [1],              3,      'DH';        %9
               'RD5Met'     ,   [5],   [],             [1],              3,      'DH';        %10 'LWrist'     ,   [7],   [13,14,15],     [1],              3,      'DH';        %12
               'LD3Met'     ,   [7],  [16],            [1],              3,      'DH';        %11
               'LD4Met'     ,   [7],  [],              [1],              3,      'DH';        %12
               'LD5Met'     ,   [7],  [],              [1],              3,      'DH';        %13
               'LD3Phal1'   ,   [11],  [20],           [1],              4,      'DH';        %14%%%%%%%%%%%
               'LD4Phal1'   ,   [12],  [21],           [1],              4,      'DH';        %15
               'LD5Phal1'   ,   [13],  [22],           [1],              4,      'DH';        %16
               'RD3Phal1'   ,   [8],   [23],           [1],              4,      'DH';        %17%%%%%%%%%%%
               'RD4Phal1'   ,   [9],   [24],           [1],              4,      'DH';        %18
               'RD5Phal1'   ,   [10],  [25],           [1],              4,      'DH';        %19
               'LD3Phal2'   ,   [14],  [],             [1],              5,      'DH';        %20
               'LD4Phal2'   ,   [15],  [],             [1],              5,      'DH';        %21
               'LD5Phal2'   ,   [16],  [],             [1],              5,      'DH';        %22
               'RD3Phal2'   ,   [17],  [],             [1],              5,      'DH';        %23
               'RD4Phal2'   ,   [18],  [],             [1],              5,      'DH';        %24
               'RD5Phal2'   ,   [19],  [],             [1],              5,      'DH'};       %25
           
nnodes = size(NodeNames,1);

for nn = 1:nnodes 
    
synthConfig.link(nn).nnames = NodeNames{nn,1};
synthConfig.link(nn).parent = NodeNames{nn,2};
synthConfig.link(nn).child  = NodeNames{nn,3};
synthConfig.link(nn).ConPt  = NodeNames{nn,4};
synthConfig.link(nn).Group  = NodeNames{nn,5};
synthConfig.link(nn).IDkern = NodeNames{nn,6};

end


%% Create DOF Tree
DOFAssign = {'BB_1', 5; 'BB_2', 1; 'BB_3', 1; 'Hum', 3; 'Rad', 1; 'Met', 2; 'Wrist', 1; 'Phal', 2};

%Cycle through labels and assign NDOFs
for ii = 1:length(DOFAssign)
    %Get current node label 
    cmpstr = strcat('.*',DOFAssign(ii,1),'.*');
    nlist = [];
    %find nodes with label identifer
    for kk = 1:nnodes
        TF = regexp(synthConfig.link(kk).nnames, cmpstr{1});
        %if node label contains identifier
        if TF
            %add node to list
            nlist = [nlist, kk];
        end
    end
    %for all matched nodes
    for jj = nlist
        %assign number of Dofs
        synthConfig.link(jj).nDof = DOFAssign{ii,2};
    end
    
end

%Dof Type Specification (1 for translational, 0 for rotational)
for ii = 1:length(synthConfig.link)
    if synthConfig.link(ii).nDof == 5
        synthConfig.link(ii).tDof = [1;1;1;0;0]';
    else
        synthConfig.link(ii).tDof = zeros(synthConfig.link(ii).nDof,1)';
    end
end

fprintf('---------------------------------------------------\n')
fprintf(' Link \t Name \t nDof \t Group \t Parent \tIdKern \n')
fprintf('---------------------------------------------------\n')

for ii = 1:length(synthConfig.link)
    fprintf(' %d \t\t %s',ii, synthConfig.link(ii).nnames)
    if length(synthConfig.link(ii).nnames)<3
        fprintf('\t\t %d \t\t\t %d \t\t %d \t\t %s \n', synthConfig.link(ii).nDof,synthConfig.link(ii).Group,synthConfig.link(ii).parent,synthConfig.link(ii).IDkern)
    elseif length(synthConfig.link(ii).nnames)<6
        fprintf('\t %d \t\t\t %d \t\t %d \t\t %s \n', synthConfig.link(ii).nDof,synthConfig.link(ii).Group,synthConfig.link(ii).parent,synthConfig.link(ii).IDkern)
    else
        fprintf('  %d \t\t\t %d \t\t %d \t\t %s \n', synthConfig.link(ii).nDof,synthConfig.link(ii).Group,synthConfig.link(ii).parent,synthConfig.link(ii).IDkern)
    end
end
fprintf('---------------------------------------------------\n')


%% Create Body Fixed Vectors
%specifiy the points which are on each link
synthConfig.link(1).pt_nums   = [105];
synthConfig.link(2).pt_nums   = [100,105];
synthConfig.link(3).pt_nums   = [141,105];

synthConfig.link(4).pt_nums = [93,100];
synthConfig.link(5).pt_nums = [87,89,91,93];

synthConfig.link(6).pt_nums  = [144,141];
synthConfig.link(7).pt_nums = [174,172,144];

%synthConfig.link(8).pt_nums = [87,87];
synthConfig.link(8).pt_nums = [47,54,87];
synthConfig.link(9).pt_nums = [41,56,87];
synthConfig.link(10).pt_nums = [44,58,87];

%synthConfig.link(12).pt_nums = [174,174];
synthConfig.link(11).pt_nums = [208,212,174];
synthConfig.link(12).pt_nums = [190,192,174];
synthConfig.link(13).pt_nums = [167,170,174];

synthConfig.link(14).pt_nums = [203,208];
synthConfig.link(15).pt_nums = [188,190];
synthConfig.link(16).pt_nums = [166,167];
synthConfig.link(17).pt_nums = [37,47];
synthConfig.link(18).pt_nums = [21,41];
synthConfig.link(19).pt_nums = [25,44];

synthConfig.link(20).pt_nums = [201,203];
synthConfig.link(21).pt_nums = [187,188];
synthConfig.link(22).pt_nums = [165,166];
synthConfig.link(23).pt_nums = [1,37];
synthConfig.link(24).pt_nums = [4,21];
synthConfig.link(25).pt_nums = [8,25];

%load the BFVectors
load('C:\ShandongData2016\BatFlight_20160717\config\Bat_I\BFVECS.mat')

%% Define DH params
nn = 1;
%----------------------------------Base Body CF Defn---------------------------------------
synthConfig.link(nn).thetas  = [0;-pi/2;0;0;-pi/2];
synthConfig.link(nn).alphas  = [-pi/2; -pi/2; 0;-pi/2;-pi/2];
synthConfig.link(nn).disps   = [0;0;0;0;0];
synthConfig.link(nn).offsets = [0;0;0;0;0];
synthConfig.link(nn).H       = DHTransforms(synthConfig.link(nn).thetas,synthConfig.link(nn).alphas,synthConfig.link(nn).disps,synthConfig.link(nn).offsets);
synthConfig.link(nn).BFvecs  = BFvecs{nn}(:,1:end-1);
nn = nn+1;

synthConfig.link(nn).thetas  = [0];
synthConfig.link(nn).alphas  = [0];
synthConfig.link(nn).disps   = 0;
synthConfig.link(nn).offsets = 1.1*[norm(BFvecs{nn}(:,end))];
synthConfig.link(nn).H       = DHTransforms(synthConfig.link(nn).thetas,synthConfig.link(nn).alphas,synthConfig.link(nn).disps,synthConfig.link(nn).offsets);
synthConfig.link(nn).BFvecs  = 1.1*BFvecs{nn}(:,1:end-1);
nn = nn+1;

synthConfig.link(nn).thetas  = [0];
synthConfig.link(nn).alphas  = [0];
synthConfig.link(nn).disps   = 0;
synthConfig.link(nn).offsets = 1*[norm(BFvecs{nn}(:,end))];
synthConfig.link(nn).H       = DHTransforms(synthConfig.link(nn).thetas,synthConfig.link(nn).alphas,synthConfig.link(nn).disps,synthConfig.link(nn).offsets);
synthConfig.link(nn).BFvecs  = 1*BFvecs{nn}(:,1:end-1);
nn = nn+1;

%----------------------------------------------------------------------------------------
%----------------------------------- Right Arm -----------------------------------------
%----------------------------------------------------------------------------------------

%----------------------------------Humerus CF Defn---------------------------------------
%hum_len = 60;
synthConfig.link(nn).thetas  = [0;0;0];%pi/2;-pi/2];
synthConfig.link(nn).alphas  = [pi/2; -pi/2; pi/2];
synthConfig.link(nn).disps   = [0;0;1.25*norm(BFvecs{nn}(:,end))];
synthConfig.link(nn).offsets = [0;0;0];
synthConfig.link(nn).H       = DHTransforms(synthConfig.link(nn).thetas,synthConfig.link(nn).alphas,synthConfig.link(nn).disps,synthConfig.link(nn).offsets);
synthConfig.link(nn).BFvecs  = 1.25*BFvecs{nn}(:,1:end-1);
nn = nn+1;

%----------------------------------Raduis CF Defn---------------------------------------
%rad_len = 70;
gamma = 85/180*pi;
synthConfig.link(nn).thetas  = [0];
synthConfig.link(nn).alphas  = [0];
synthConfig.link(nn).disps   = [0];
synthConfig.link(nn).offsets = norm(BFvecs{nn}(:,end));
synthConfig.link(nn).H       = DHTransforms(synthConfig.link(nn).thetas,synthConfig.link(nn).alphas,synthConfig.link(nn).disps,synthConfig.link(nn).offsets);
synthConfig.link(nn).BFvecs  = BFvecs{nn}(:,1:end-1);
synthConfig.link(nn).BFvecs(2,2) = -synthConfig.link(nn).BFvecs(2,2);
synthConfig.link(nn).BFvecs(2,3) = -synthConfig.link(nn).BFvecs(2,3);
nn = nn+1;

%----------------------------------------------------------------------------------------
%----------------------------------- Left Arm -----------------------------------------
%----------------------------------------------------------------------------------------

%----------------------------------Humerus CF Defn---------------------------------------
%hum_len = 60;
synthConfig.link(nn).thetas  = [0;0;0];%pi/2;-pi/2];
synthConfig.link(nn).alphas  = [pi/2; -pi/2; pi/2];
synthConfig.link(nn).disps   = [0;0;norm(BFvecs{nn}(:,end))];
synthConfig.link(nn).offsets = [0;0;0];
synthConfig.link(nn).H       = DHTransforms(synthConfig.link(nn).thetas,synthConfig.link(nn).alphas,synthConfig.link(nn).disps,synthConfig.link(nn).offsets);
synthConfig.link(nn).BFvecs  = BFvecs{nn}(:,1:end-1);
nn = nn+1;

%----------------------------------Raduis CF Defn---------------------------------------
%rad_len = 70;
gamma = 85/180*pi;
synthConfig.link(nn).thetas  = [0];
synthConfig.link(nn).alphas  = [0];
synthConfig.link(nn).disps   = [0];
synthConfig.link(nn).offsets = norm(BFvecs{nn}(:,end));
synthConfig.link(nn).H       = DHTransforms(synthConfig.link(nn).thetas,synthConfig.link(nn).alphas,synthConfig.link(nn).disps,synthConfig.link(nn).offsets);
synthConfig.link(nn).BFvecs  = BFvecs{nn}(:,1:end-1);
synthConfig.link(nn).BFvecs(2,2) = -synthConfig.link(nn).BFvecs(2,2);
nn = nn+1;

%----------------------------------------------------------------------------------------
%----------------------------------- Right Hand -----------------------------------------
%----------------------------------------------------------------------------------------

%----------------------------------Wrist CF Defn---------------------------------------
%rad_len = 70;
% gamma = 85/180*pi;
% synthConfig.link(nn).thetas  = [0];
% synthConfig.link(nn).alphas  = [-pi/2];
% synthConfig.link(nn).disps   = [0];
% synthConfig.link(nn).offsets = 0;
% synthConfig.link(nn).H       = DHTransforms(synthConfig.link(nn).thetas,synthConfig.link(nn).alphas,synthConfig.link(nn).disps,synthConfig.link(nn).offsets);
% synthConfig.link(nn).BFvecs  = [];
% nn = nn+1;

%----------------------------------Digit 3 Metacarpal CF Defn---------------------------------------
met3_len   = norm(BFvecs{nn}(:,end));
synthConfig.link(nn).thetas  = [pi/4;0];
synthConfig.link(nn).alphas  = [pi/2;0];
synthConfig.link(nn).disps   = [0;0];
synthConfig.link(nn).offsets = [0;met3_len];
synthConfig.link(nn).H = DHTransforms(synthConfig.link(nn).thetas,synthConfig.link(nn).alphas,synthConfig.link(nn).disps,synthConfig.link(nn).offsets);
synthConfig.link(nn).BFvecs = BFvecs{nn}(:,1:end-1);
nn = nn+1;

%----------------------------------Digit 4 Metacarpal CF Defn---------------------------------------
met4_len   = norm(BFvecs{nn}(:,end));
synthConfig.link(nn).thetas  = [pi/2;0];
synthConfig.link(nn).alphas  = [pi/2;0];
synthConfig.link(nn).disps   = [0;0];
synthConfig.link(nn).offsets = [0;met4_len];
synthConfig.link(nn).H = DHTransforms(synthConfig.link(nn).thetas,synthConfig.link(nn).alphas,synthConfig.link(nn).disps,synthConfig.link(nn).offsets);
synthConfig.link(nn).BFvecs = BFvecs{nn}(:,1:end-1);
nn = nn+1;
%----------------------------------Digit 5 Metacarpal CF Defn---------------------------------------
met5_len   = norm(BFvecs{nn}(:,end));
synthConfig.link(nn).thetas  = [3*pi/4;0];
synthConfig.link(nn).alphas  = [pi/2;0];
synthConfig.link(nn).disps   = [0;0];
synthConfig.link(nn).offsets = [0;met5_len];
synthConfig.link(nn).H = DHTransforms(synthConfig.link(nn).thetas,synthConfig.link(nn).alphas,synthConfig.link(nn).disps,synthConfig.link(nn).offsets);
synthConfig.link(nn).BFvecs = BFvecs{nn}(:,1:end-1);
nn = nn+1;
%----------------------------------------------------------------------------------------
%------------------------------------ Left Hand -----------------------------------------
%----------------------------------------------------------------------------------------

%----------------------------------Wrist CF Defn---------------------------------------
%rad_len = 70;
% gamma = 85/180*pi;
% synthConfig.link(nn).thetas  = [0];
% synthConfig.link(nn).alphas  = [-pi/2];
% synthConfig.link(nn).disps   = [0];
% synthConfig.link(nn).offsets = 0;
% synthConfig.link(nn).H       = DHTransforms(synthConfig.link(nn).thetas,synthConfig.link(nn).alphas,synthConfig.link(nn).disps,synthConfig.link(nn).offsets);
% synthConfig.link(nn).BFvecs  = [];
% nn = nn+1;

%----------------------------------Digit 3 Metacarpal CF Defn---------------------------------------
met3_len   = norm(BFvecs{nn}(:,end));
synthConfig.link(nn).thetas  = [pi/4;0];
synthConfig.link(nn).alphas  = [pi/2;0];
synthConfig.link(nn).disps   = [0;0];
synthConfig.link(nn).offsets = [0;met3_len];
synthConfig.link(nn).H = DHTransforms(synthConfig.link(nn).thetas,synthConfig.link(nn).alphas,synthConfig.link(nn).disps,synthConfig.link(nn).offsets);
synthConfig.link(nn).BFvecs = BFvecs{nn}(:,1:end-1);
nn = nn+1;

%----------------------------------Digit 4 Metacarpal CF Defn---------------------------------------
met4_len   = norm(BFvecs{nn}(:,end));
synthConfig.link(nn).thetas  = [pi/2;0];
synthConfig.link(nn).alphas  = [pi/2;0];
synthConfig.link(nn).disps   = [0;0];
synthConfig.link(nn).offsets = [0;met4_len];
synthConfig.link(nn).H = DHTransforms(synthConfig.link(nn).thetas,synthConfig.link(nn).alphas,synthConfig.link(nn).disps,synthConfig.link(nn).offsets);
synthConfig.link(nn).BFvecs = BFvecs{nn}(:,1:end-1);
nn = nn+1;
%----------------------------------Digit 5 Metacarpal CF Defn---------------------------------------
met5_len   = norm(BFvecs{nn}(:,end));
synthConfig.link(nn).thetas  = [3*pi/4;0];
synthConfig.link(nn).alphas  = [pi/2;0];
synthConfig.link(nn).disps   = [0;0];
synthConfig.link(nn).offsets = [0;met5_len];
synthConfig.link(nn).H = DHTransforms(synthConfig.link(nn).thetas,synthConfig.link(nn).alphas,synthConfig.link(nn).disps,synthConfig.link(nn).offsets);
synthConfig.link(nn).BFvecs = BFvecs{nn}(:,1:end-1);
nn = nn+1;

%----------------------------------------------------------------------------------------
%------------------------------------ Left phal1 --------------------------------------
%----------------------------------------------------------------------------------------
%------------------------------------ Digit3 Phal 1 -----------------------------------------
phal_len   = norm(BFvecs{nn}(:,end));
synthConfig.link(nn).thetas  = [0;0];
synthConfig.link(nn).alphas  = [pi/2;-pi/2];
synthConfig.link(nn).disps   = [0;0];
synthConfig.link(nn).offsets = [0;29.54];
synthConfig.link(nn).H       = DHTransforms(synthConfig.link(nn).thetas,synthConfig.link(nn).alphas,synthConfig.link(nn).disps,synthConfig.link(nn).offsets);
synthConfig.link(nn).BFvecs  = zeros(3,1);
nn = nn+1;

%------------------------------------ Digit4 Phal 1 -----------------------------------------
phal_len   = norm(BFvecs{nn}(:,end));
synthConfig.link(nn).thetas  = [0;0];
synthConfig.link(nn).alphas  = [pi/2;-pi/2];
synthConfig.link(nn).disps   = [0;0];
synthConfig.link(nn).offsets = [0;22.06];
synthConfig.link(nn).H       = DHTransforms(synthConfig.link(nn).thetas,synthConfig.link(nn).alphas,synthConfig.link(nn).disps,synthConfig.link(nn).offsets);
synthConfig.link(nn).BFvecs  = zeros(3,1);
nn = nn+1;

%------------------------------------ Digit5 Phal 1 -----------------------------------------
phal_len   = norm(BFvecs{nn}(:,end));
synthConfig.link(nn).thetas  = [0;0];
synthConfig.link(nn).alphas  = [pi/2;-pi/2];
synthConfig.link(nn).disps   = [0;0];
synthConfig.link(nn).offsets = [0;18.2];
synthConfig.link(nn).H       = DHTransforms(synthConfig.link(nn).thetas,synthConfig.link(nn).alphas,synthConfig.link(nn).disps,synthConfig.link(nn).offsets);
synthConfig.link(nn).BFvecs  = zeros(3,1);
nn = nn+1;

%----------------------------------------------------------------------------------------
%------------------------------------ Right Phal1 --------------------------------------
%----------------------------------------------------------------------------------------
%------------------------------------ Digit3 Phal 1 -----------------------------------------
phal_len   = norm(BFvecs{nn}(:,end));
synthConfig.link(nn).thetas  = [0;0];
synthConfig.link(nn).alphas  = [pi/2;-pi/2];
synthConfig.link(nn).disps   = [0;0];
synthConfig.link(nn).offsets = [0;27.8];
synthConfig.link(nn).H       = DHTransforms(synthConfig.link(nn).thetas,synthConfig.link(nn).alphas,synthConfig.link(nn).disps,synthConfig.link(nn).offsets);
synthConfig.link(nn).BFvecs  = zeros(3,1);
nn = nn+1;

%------------------------------------ Digit4 Phal 1 -----------------------------------------
phal_len   = norm(BFvecs{nn}(:,end));
synthConfig.link(nn).thetas  = [0;0];
synthConfig.link(nn).alphas  = [pi/2;-pi/2];
synthConfig.link(nn).disps   = [0;0];
synthConfig.link(nn).offsets = [0;20.2];
synthConfig.link(nn).H       = DHTransforms(synthConfig.link(nn).thetas,synthConfig.link(nn).alphas,synthConfig.link(nn).disps,synthConfig.link(nn).offsets);
synthConfig.link(nn).BFvecs  = zeros(3,1);
nn = nn+1;

%------------------------------------ Digit5 Phal 1 -----------------------------------------
phal_len   = norm(BFvecs{nn}(:,end));
synthConfig.link(nn).thetas  = [0;0];
synthConfig.link(nn).alphas  = [pi/2;-pi/2];
synthConfig.link(nn).disps   = [0;0];
synthConfig.link(nn).offsets = [0;20.24];
synthConfig.link(nn).H       = DHTransforms(synthConfig.link(nn).thetas,synthConfig.link(nn).alphas,synthConfig.link(nn).disps,synthConfig.link(nn).offsets);
synthConfig.link(nn).BFvecs  = zeros(3,1);
nn = nn+1;

%----------------------------------------------------------------------------------------
%------------------------------------ Left Phal2 --------------------------------------
%----------------------------------------------------------------------------------------
%------------------------------------ Digit3 Phal2 -----------------------------------------
phal_len   = norm(BFvecs{nn}(:,end));
synthConfig.link(nn).thetas  = [0;0];
synthConfig.link(nn).alphas  = [pi/2;-pi/2];
synthConfig.link(nn).disps   = [0;0];
synthConfig.link(nn).offsets = [0;28.16];
synthConfig.link(nn).H       = DHTransforms(synthConfig.link(nn).thetas,synthConfig.link(nn).alphas,synthConfig.link(nn).disps,synthConfig.link(nn).offsets);
synthConfig.link(nn).BFvecs  = zeros(3,1);
nn = nn+1;

%------------------------------------ Digit4 Phal2 -----------------------------------------
phal_len   = norm(BFvecs{nn}(:,end));
synthConfig.link(nn).thetas  = [0;0];
synthConfig.link(nn).alphas  = [pi/2;-pi/2];
synthConfig.link(nn).disps   = [0;0];
synthConfig.link(nn).offsets = [0;13.28];
synthConfig.link(nn).H       = DHTransforms(synthConfig.link(nn).thetas,synthConfig.link(nn).alphas,synthConfig.link(nn).disps,synthConfig.link(nn).offsets);
synthConfig.link(nn).BFvecs  = zeros(3,1);
nn = nn+1;

%------------------------------------ Digit5 Phal2 -----------------------------------------
phal_len   = norm(BFvecs{nn}(:,end));
synthConfig.link(nn).thetas  = [0;0];
synthConfig.link(nn).alphas  = [pi/2;-pi/2];
synthConfig.link(nn).disps   = [0;0];
synthConfig.link(nn).offsets = [0;15.4];
synthConfig.link(nn).H       = DHTransforms(synthConfig.link(nn).thetas,synthConfig.link(nn).alphas,synthConfig.link(nn).disps,synthConfig.link(nn).offsets);
synthConfig.link(nn).BFvecs  = zeros(3,1);
nn = nn+1;
%----------------------------------------------------------------------------------------
%------------------------------------ Right Phal2 --------------------------------------
%----------------------------------------------------------------------------------------
%------------------------------------ Digit3 Phal2 -----------------------------------------
phal_len   = norm(BFvecs{nn}(:,end));
synthConfig.link(nn).thetas  = [0;0];
synthConfig.link(nn).alphas  = [pi/2;-pi/2];
synthConfig.link(nn).disps   = [0;0];
synthConfig.link(nn).offsets = [0;28.77];
synthConfig.link(nn).H       = DHTransforms(synthConfig.link(nn).thetas,synthConfig.link(nn).alphas,synthConfig.link(nn).disps,synthConfig.link(nn).offsets);
synthConfig.link(nn).BFvecs  = zeros(3,1);
nn = nn+1;

%------------------------------------ Digit4 Phal2 -----------------------------------------
phal_len   = norm(BFvecs{nn}(:,end));
synthConfig.link(nn).thetas  = [0;0];
synthConfig.link(nn).alphas  = [pi/2;-pi/2];
synthConfig.link(nn).disps   = [0;0];
synthConfig.link(nn).offsets = [0;15.26];
synthConfig.link(nn).H       = DHTransforms(synthConfig.link(nn).thetas,synthConfig.link(nn).alphas,synthConfig.link(nn).disps,synthConfig.link(nn).offsets);
synthConfig.link(nn).BFvecs  = zeros(3,1);
nn = nn+1;

%------------------------------------ Digit5 Phal2 -----------------------------------------
phal_len   = norm(BFvecs{nn}(:,end));
synthConfig.link(nn).thetas  = [0;0];
synthConfig.link(nn).alphas  = [pi/2;-pi/2];
synthConfig.link(nn).disps   = [0;0];
synthConfig.link(nn).offsets = [0;15.7];
synthConfig.link(nn).H       = DHTransforms(synthConfig.link(nn).thetas,synthConfig.link(nn).alphas,synthConfig.link(nn).disps,synthConfig.link(nn).offsets);
synthConfig.link(nn).BFvecs  = zeros(3,1);
nn = nn+1;


fprintf('------------------- DH  Table ---------------------\n')
fprintf('---------------------------------------------------\n')
fprintf(' DOF # \t Link Name \t  theta \t d \t\t a \t\t alpha \n')
fprintf('---------------------------------------------------\n')
ndof = 0;
for ll = 1:length(synthConfig.link)
    for dd = 1:length(synthConfig.link(ll).thetas)
        ndof = ndof+1;
        if ndof>9
            fprintf(' %d \t %s',ndof, synthConfig.link(ll).nnames)
        else
            fprintf(' %d \t\t %s',ndof, synthConfig.link(ll).nnames)
        end
        if length(synthConfig.link(ll).nnames)<3
            fprintf('\t\t\t %2.1f \t\t %2.1f \t %2.1f \t\t %2.1f \n', synthConfig.link(ll).thetas(dd),synthConfig.link(ll).disps(dd),synthConfig.link(ll).offsets(dd),synthConfig.link(ll).alphas(dd))
        elseif length(synthConfig.link(ll).nnames)<6
            fprintf('\t\t %2.1f \t\t %2.1f \t %2.1f \t\t %2.1f \n', synthConfig.link(ll).thetas(dd),synthConfig.link(ll).disps(dd),synthConfig.link(ll).offsets(dd),synthConfig.link(ll).alphas(dd))
        else
            fprintf('\t\t  %2.1f \t\t %2.1f \t %2.1f \t\t %2.1f \n', synthConfig.link(ll).thetas(dd),synthConfig.link(ll).disps(dd),synthConfig.link(ll).offsets(dd),synthConfig.link(ll).alphas(dd))
        end
    end
end
fprintf('---------------------------------------------------\n')

figure
plot_kin_chain(synthConfig,synthConfig,1,1)

axis equal