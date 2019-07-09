classdef MahonyAHRS < handle
%MAYHONYAHRS Madgwick's implementation of Mayhony's AHRS algorithm
%
%   For more information see:
%   http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
%
%   Date          Author          Notes
%   28/09/2011    SOH Madgwick    Initial release
 
    %% Public properties
    properties (Access = public)
        SamplePeriod = 1/256;
        Quaternion = [1 0 0 0];     % output quaternion describing the Earth relative to the sensor
        Kp = 1;                     % algorithm proportional gain
        Ki = 0;                     % algorithm integral gain
        unwrapper;
    end
    
    %% Public properties
    properties (Access = private)
        eInt = [0 0 0];             % integral error
    end    
 
    %% Public methods
    methods (Access = public)
        function obj = MahonyAHRS(varargin)
            for i = 1:2:nargin
                if  strcmp(varargin{i}, 'SamplePeriod'), obj.SamplePeriod = varargin{i+1};
                elseif  strcmp(varargin{i}, 'Quaternion'), obj.Quaternion = varargin{i+1};
                elseif  strcmp(varargin{i}, 'Kp'), obj.Kp = varargin{i+1};
                elseif  strcmp(varargin{i}, 'Ki'), obj.Ki = varargin{i+1};
                else error('Invalid argument');
                end
            end;
            obj.unwrapper = RealtimeUnwrap();
        end
        
        function obj = Init(obj, Gyroscope, Accelerometer, UWB_psi)
            obj.unwrapper.UpdateLastPhase(UWB_psi);
            obj.Quaternion = yaw2quatern(UWB_psi);
        end
        
        function obj = Update(obj, Gyroscope, Accelerometer, UWB_psi)
            q = obj.Quaternion; % short name local variable for readability
 
            % Normalise accelerometer measurement
            if(norm(Accelerometer) == 0), return; end   % handle NaN
            Accelerometer = Accelerometer / norm(Accelerometer);    % normalise magnitude
 
            % psi = atan2(2*q_2*q_3 - 2*q_1*q_4, 2*q_1^2 + 2*q_2^2 - 1)
            y = 2*q(2)*q(3)+2*q(1)*q(4);
            x = 2*q(1)^2+2*q(2)^2-1;
            
            % Estimated direction of gravity
            v = [2*(q(2)*q(4) - q(1)*q(3))
                 2*(q(1)*q(2) + q(3)*q(4))
                 q(1)^2 - q(2)^2 - q(3)^2 + q(4)^2];
             
            % Eclasstimated psi
            psi = atan2(y, x); 
            psi = obj.unwrapper.unwrap(psi);
%             disp('psi:');
%             disp(psi);
%             disp('UWB_psi:');
%             disp(UWB_psi);
            % Error is sum of cross product between estimated direction and measured direction of fields
            e = cross(Accelerometer, v) + [0 0 UWB_psi-psi]; 
%             e = [0 0 UWB_psi-psi]; 
            if(obj.Ki > 0)
                obj.eInt = obj.eInt + e * obj.SamplePeriod;   
            else
                obj.eInt = [0 0 0];
            end
            
            % Apply feedback terms
            Gyroscope = Gyroscope + obj.Kp * e + obj.Ki * obj.eInt;            
            
            % Compute rate of change of quaternion
            qDot = 0.5 * quaternProd(q, [0 Gyroscope(1) Gyroscope(2) Gyroscope(3)]);
 
            % Integrate to yield quaternion
            q = q + qDot * obj.SamplePeriod;
            obj.Quaternion = q / norm(q); % normalise quaternion
        end
        function obj = UpdateIMU(obj, Gyroscope, Accelerometer)
            q = obj.Quaternion; % short name local variable for readability
 
            % Normalise accelerometer measurement
            if(norm(Accelerometer) == 0), return; end   % handle NaN
            Accelerometer = Accelerometer / norm(Accelerometer);	% normalise magnitude
 
            % Estimated direction of gravity and magnetic flux
            v = [2*(q(2)*q(4) - q(1)*q(3))
                 2*(q(1)*q(2) + q(3)*q(4))
                 q(1)^2 - q(2)^2 - q(3)^2 + q(4)^2];
 
            % Error is sum of cross product between estimated direction and measured direction of field
            e = cross(Accelerometer, v); 
            if(obj.Ki > 0)
                obj.eInt = obj.eInt + e * obj.SamplePeriod;   
            else
                obj.eInt = [0 0 0];
            end
            
            % Apply feedback terms
            Gyroscope = Gyroscope + obj.Kp * e + obj.Ki * obj.eInt;            
            
            % Compute rate of change of quaternion
            qDot = 0.5 * quaternProd(q, [0 Gyroscope(1) Gyroscope(2) Gyroscope(3)]);
 
            % Integrate to yield quaternion
            q = q + qDot * obj.SamplePeriod;
            obj.Quaternion = q / norm(q); % normalise quaternion
        end
        function obj = UpdateGyro(obj, Gyroscope)
            q = obj.Quaternion; % short name local variable for readability
            
            % Compute rate of change of quaternion
            qDot = 0.5 * quaternProd(q, [0 Gyroscope(1) Gyroscope(2) Gyroscope(3)]);
 
            % Integrate to yield quaternion
            q = q + qDot * obj.SamplePeriod;
            obj.Quaternion = q / norm(q); % normalise quaternion
        end
        function obj = UpdateUWB(obj, Gyroscope, UWB_psi)
            q = obj.Quaternion; % short name local variable for readability
 
            % psi = atan2(2*q_2*q_3 - 2*q_1*q_4, 2*q_1^2 + 2*q_2^2 - 1)
            y = 2*q(2)*q(3)+2*q(1)*q(4);
            x = 2*q(1)^2+2*q(2)^2-1;
             
            % Eclasstimated psi
            psi = atan2(y, x); 
            psi = obj.unwrapper.unwrap(psi);
%             disp('psi:');
%             disp(psi);
%             disp('UWB_psi:');
%             disp(UWB_psi);
            % Error is sum of cross product between estimated direction and measured direction of fields
            e = [0 0 UWB_psi-psi]; 
%             e = [0 0 UWB_psi-psi]; 
            if(obj.Ki > 0)
                obj.eInt = obj.eInt + e * obj.SamplePeriod;   
            else
                obj.eInt = [0 0 0];
            end
            
            % Apply feedback terms
            Gyroscope = Gyroscope + obj.Kp * e + obj.Ki * obj.eInt;            
            
            % Compute rate of change of quaternion
            qDot = 0.5 * quaternProd(q, [0 Gyroscope(1) Gyroscope(2) Gyroscope(3)]);
 
            % Integrate to yield quaternion
            q = q + qDot * obj.SamplePeriod;
            obj.Quaternion = q / norm(q); % normalise quaternion
        end
    end
end