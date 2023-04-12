% This class uses vehicl kinematics similar to what you saw in Part 01 of
% this module.
%
% The model assumes that the vehicle speed is specified in the vehicle
% frame and is then projected into the world frame. Specifically,
%
% M = [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0;0 0 1];
%
% The process model has the form:
%
% x = x + M * [vx;vy;theta]
%
% where vx, vy and vtheta are the velocities.
%
% The error model
% eTheta =

classdef VehicleKinematicsEdge < g2o.core.BaseBinaryEdge
    
    properties(Access = protected)
        % The length of the time step
        dT;
    end
    
    % Intialize the
    
    methods(Access = public)
        
        % Store the prediction interval
        function this = VehicleKinematicsEdge(dT)
            assert(dT >= 0);
            this = this@g2o.core.BaseBinaryEdge(3);
            this.dT = dT;
        end
        
        % This method predicts what the robot pose will be at the next
        % time step, given the pose at the current step, the control inputs
        % and the prediction interval. It is only used when creating
        % vertices - the updates are carried out in computeError and
        % linearizeOplus.
        
        function initialize(this)
            
            
            priorX = this.edgeVertices{1}.x;
            
            c = cos(priorX(3));
            s = sin(priorX(3));
            
            M = this.dT * [c -s 0;
                s c 0;
                0 0 1];
            
            % Compute the posterior assming no noise
            this.edgeVertices{2}.x = this.edgeVertices{1}.x + M * this.z;
            
            % Wrap the heading to -pi to pi
            this.edgeVertices{2}.x(3) = g2o.stuff.normalize_theta(this.edgeVertices{2}.x(3));
            
        end
        %Q1b
        function computeError(this)
        
            priorX = this.edgeVertices{1}.x;
            
            c = cos(priorX(3));
            s = sin(priorX(3));
            
            Mi =[c s 0;
                -s c 0;
                0 0 1] / this.dT;
            
            dx = this.edgeVertices{2}.x - priorX;
            %Q1b wrap the difference in heading to -pi to pi 
            dx(3) =  g2o.stuff.normalize_theta(dx(3));
            % Q1b Compute the error.
            this.errorZ = Mi * (dx) - this.z;

%             % Wrap the heading error to -pi to pi
%             this.errorZ(3) = g2o.stuff.normalize_theta(this.errorZ(3));

        end
        
        % Compute the Jacobians
        %Q1b
        function linearizeOplus(this)
            %Q1b x_k
            priorX = this.edgeVertices{1}.x;
            
            c = cos(priorX(3));
            s = sin(priorX(3));
        
            dx = this.edgeVertices{2}.x - priorX; %Q1b x_{k+1}-x_k
            Mi =[c s 0;
                -s c 0;
                0 0 1] / this.dT;

            this.J{2} = Mi;
            this.J{1}(1, 1) = - c / this.dT;
            this.J{1}(1, 2) = - s / this.dT;
            this.J{1}(1, 3) = (-dx(1) * s + dx(2) * c)/ this.dT;
            this.J{1}(2, 1) = s/ this.dT;
            this.J{1}(2, 2) = - c/ this.dT;
            this.J{1}(2, 3) = (-dx(1) * c - dx(2) * s)/ this.dT;
            this.J{1}(3, 3) = -1/ this.dT;

        end
    end
end