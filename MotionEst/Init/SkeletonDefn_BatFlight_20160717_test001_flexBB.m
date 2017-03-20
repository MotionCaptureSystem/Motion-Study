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
NodeNames   = {'BB_1'     ,   [] ,    [],             [],                1,      'DH';
               'BB_2'     ,   [1] ,   [3],            [1],               1,      'DH';
               'LHum'     ,   [1]+1,  [3]+1,          [1],               2,      'DH';
               'LRad'     ,   [2]+1,  [4]+1           [1],               2,      'DH';
               'LWrist'   ,   [3]+1,  [5,6,7]+1,      [1],               3,      'DH';
               'RD3Met'   ,   [4]+1,  [],             [1],               3,      'DH'; 
               'RD4Met'   ,   [4]+1,  [],             [1],               3,      'DH';
               'RD5Met'   ,   [4]+1,  [],             [1],               3,      'DH'};
           
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
DOFAssign = {'BB_1', 6; 'BB_2', 1; 'Hum', 3; 'Rad', 1; 'Met', 2; 'Wrist', 1; 'Phal', 1};

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
    if synthConfig.link(ii).nDof == 6
        synthConfig.link(ii).tDof = [1;1;1;0;0;0]';
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
synthConfig.link(1).pt_nums   = [105,141];
synthConfig.link(2).pt_nums   = [100,105];
synthConfig.link(2+1).pt_nums = [93,100];
synthConfig.link(3+1).pt_nums = [87,89,91,93];
synthConfig.link(4+1).pt_nums = [87,87];
synthConfig.link(5+1).pt_nums = [46,54,87];
synthConfig.link(6+1).pt_nums = [49,56,87];
synthConfig.link(7+1).pt_nums = [44,58,87];
%load the stereo triangulation data
load([options.path,filesep,'StereoStruct.mat']);
npair = length(Stereo);         %determine the number of cameras pairs

for ll = 1:length(synthConfig.link)
    vectors = [];
    nvecs = length(synthConfig.link(ll).pt_nums);
    delta = zeros(3,size(Stereo(1).pts,2),nvecs,npair);
    for vec = 1:nvecs
        for pair = 1:npair
            delta(:,:,vec,pair) = Stereo(pair).pts(:,:,synthConfig.link(ll).pt_nums(vec))-Stereo(pair).pts(:,:,synthConfig.link(ll).pt_nums(1));
        end
    end
    
    mean_delta = nanmedian(delta,4);
    std_delta_pair = nanstd(delta,0,4);
    std_delta_time = nanstd(delta,0,2);
    mean_delta = nanmedian(mean_delta,2);
    mean_delta = squeeze(mean_delta);

    %align BF basis with body points
%     if ll ==1 
%         z_hat = cross(mean_delta(:,3),mean_delta(:,2))/norm(cross(mean_delta(:,2),mean_delta(:,3)));
%         b     = (mean_delta(:,2)+mean_delta(:,3))/2;
%         y_hat = cross(z_hat,b)/norm(cross(z_hat,b));
%         x_hat = cross(y_hat,z_hat);
%         vectors = 1*[x_hat,y_hat,z_hat]'*mean_delta;
        %vectors = vectors - repmat(vectors(:,2),1,3);
    if ll==1
        vectors(:,1) = [0,0,0]';
        vectors(:,2) = [0,norm(mean_delta(:,2)),0]';
    elseif ll==2
        vectors(:,1) = [0,0,0]';
        vectors(:,2) = [-norm(mean_delta(:,2)),0,0]';
    elseif ll==2+1
        vectors(:,1) = [0,0,0]';
        vectors(:,2) = [0,-norm(mean_delta(:,2)),0]';
        %vectors(:,2) = [0,-norm(mean_delta(:,2)),0]';
    elseif ll==3+1
        vectors(:,1) = [norm(mean_delta(:,1)),0,0]';
        vectors(:,2) = [-norm(mean_delta(:,2)),0,0]';
        vectors(:,3) = [-norm(mean_delta(:,3)),0,0]';
        vectors(:,4) = [-norm(mean_delta(:,4)),0,0]';
%         vectors(:,3) = [-norm(mean_delta(:,3)),0,0]';
    elseif ll == 4+1
        vectors(:,1) = [0,0,0]';
        vectors(:,2) = [0,0,0]';
    elseif ll==5+1
        vectors(:,1) = [-norm(mean_delta(:,1)),0,0]';
        vectors(:,2) = [-norm(mean_delta(:,2)),0,0]';
        vectors(:,3) = [-norm(mean_delta(:,3)),0,0]';
%         vectors(:,2) = [-norm(mean_delta(:,2)),0,0]';
%         vectors(:,3) = [-norm(mean_delta(:,3)),0,0]';
    elseif ll==6+1
        vectors(:,1) = [-norm(mean_delta(:,1)),0,0]';
        vectors(:,2) = [-norm(mean_delta(:,2)),0,0]';
        vectors(:,3) = [-norm(mean_delta(:,3)),0,0]';
    elseif ll==7+1
        vectors(:,1) = [-norm(mean_delta(:,1)),0,0]';
        vectors(:,2) = [-norm(mean_delta(:,2)),0,0]';
        vectors(:,3) = [-norm(mean_delta(:,3)),0,0]';
    end
    
    BFvecs{ll} = vectors;
end

%% Define DH params
nn = 1;
%----------------------------------Base Body CF Defn---------------------------------------
synthConfig.link(nn).thetas  = [0;-pi/2;-pi/2;0;pi/2;pi/2];
synthConfig.link(nn).alphas  = [-pi/2; -pi/2; -pi/2;pi/2;pi/2;pi/2];
synthConfig.link(nn).disps   = [0;0;0;0;0;-norm(BFvecs{nn}(:,end))];
synthConfig.link(nn).offsets = [0;0;0;0;0;0];
synthConfig.link(nn).H       = DHTransforms(synthConfig.link(nn).thetas,synthConfig.link(nn).alphas,synthConfig.link(nn).disps,synthConfig.link(nn).offsets);
synthConfig.link(nn).BFvecs  = BFvecs{nn}(:,1:end);
nn = nn+1;

synthConfig.link(nn).thetas  = [0];
synthConfig.link(nn).alphas  = [0];
synthConfig.link(nn).disps   = 0;
synthConfig.link(nn).offsets = [norm(BFvecs{nn}(:,end))];
synthConfig.link(nn).H       = DHTransforms(synthConfig.link(nn).thetas,synthConfig.link(nn).alphas,synthConfig.link(nn).disps,synthConfig.link(nn).offsets);
synthConfig.link(nn).BFvecs  = BFvecs{nn}(:,1:end-1);
nn = nn+1;

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
synthConfig.link(nn).alphas  = [pi/2];
synthConfig.link(nn).disps   = [0];
synthConfig.link(nn).offsets = norm(BFvecs{nn}(:,end));
synthConfig.link(nn).H       = DHTransforms(synthConfig.link(nn).thetas,synthConfig.link(nn).alphas,synthConfig.link(nn).disps,synthConfig.link(nn).offsets);
synthConfig.link(nn).BFvecs  = BFvecs{nn}(:,1:end-1);
nn = nn+1;

%----------------------------------Wrist CF Defn---------------------------------------
%rad_len = 70;
gamma = 85/180*pi;
synthConfig.link(nn).thetas  = [0];
synthConfig.link(nn).alphas  = [-pi/2];
synthConfig.link(nn).disps   = [0];
synthConfig.link(nn).offsets = 0;
synthConfig.link(nn).H       = DHTransforms(synthConfig.link(nn).thetas,synthConfig.link(nn).alphas,synthConfig.link(nn).disps,synthConfig.link(nn).offsets);
synthConfig.link(nn).BFvecs  = [];
nn = nn+1;

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
plot_kin_chain(synthConfig,synthConfig,1)

axis equal