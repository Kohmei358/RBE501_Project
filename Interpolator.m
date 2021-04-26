classdef Interpolator
    %This class is the base for the smooth tranjectory planning. Construct
    %a instance of Interpolator by calling I = Interpolator("Type", TotalTime)
    %Then use I.get(currentTime) to get a scaled value between 0 and 1 that
    %follows a (Linear/Cubit/Quintric) curve that goes from 0 to 1 on the Y-axis
    %and 0 -> TotalTime on the X axis. If a time less than 0 is given the
    %output of the get funciton is 0, if the time given is more than
    %TotalTime, the output of the get function is 1.

    properties
        coeffs; %none    3x1     5x1
        type; % Linear, Cubic, Quintic
        T = 0; %in secconds
    end

    methods
        function obj = Interpolator(InterpolationType,Time)
            %Type is wither Linear, Cubic, Quintic, Time is the total
            %desired path time
            obj.type = InterpolationType;
            obj.T = Time;

            %Set Known Parameters
            t0 = 0;
            tf = obj.T;
            v0 = 0;
            vf = 0;
            q0 = 0;
            qf = 1;
            alpha0 = 0;
            alphaf = 0;

            %Calculate Coefficients for Cubic Trajectory and store in obj.coeffs
            if InterpolationType == "Cubic"
                MCubic = [1 t0  t0^2    t0^3;
                          0 1   2*t0    3*t0^2;
                          1 tf  tf^2    tf^3;
                          0 1   2*tf    3*tf^2];

                AnsCubic = [q0;
                            v0;
                            qf;
                            vf];

                tempCubic = linsolve(MCubic,AnsCubic);
                obj.coeffs = tempCubic;

            %Calculate Coefficients for Quintic Trajectory and store in obj.coeffs
            elseif InterpolationType == "Quintic"
                MQuintic = [1   t0  t0^2    t0^3    t0^4        t0^5;
                            0   1   2*t0    3*t0^2  4*t0^3      5*t0^4;
                            0   0   2       6*t0    12*t0^2     20*t0^3;
                            1   tf  tf^2    tf^3    tf^4        tf^5;
                            0   1   2*tf    3*tf^2  4*tf^3      5*tf^4;
                            0   0   2       6*tf    12*tf^2     20*tf^3];


                AnsQuintic = [q0;
                              v0;
                              alpha0;
                              qf;
                              vf;
                              alphaf];

                tempQuintic = linsolve(MQuintic,AnsQuintic);
                obj.coeffs = tempQuintic;
            end

        end

        function pos = get(obj, deltaT)
            %This evaultes 'Type' equation at the time given
            A = obj.coeffs;

            if obj.type == "Linear"
               pos = (deltaT/obj.T);

            elseif obj.type == "Cubic"

                pos = A(1) + A(2)*deltaT + A(3)*deltaT^2 + A(4)*deltaT^3;

            elseif obj.type == "Quintic"

                pos = A(1) + A(2)*deltaT + A(3)*deltaT^2 + A(4)*deltaT^3 + ...
                    A(5)*deltaT^4 + A(6)*deltaT^5;
            end
            
            %If time is out of bounds (0 < time < obj.T) then give 0 or 1
            if deltaT < 0
                pos = 0;
            elseif deltaT > obj.T
                pos = 1;
            end
            
        end

    end
end