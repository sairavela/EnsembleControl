classdef EnsControl
    properties
        target_ens;
        pred_ens;
        horizon;
        control_mean;
        state_fnc;
        params;
        control_ens
    end
    methods
        function ensupd = calc_update(obj)
        % Calculates updated controller
            xens  = obj.pred_ens; %position ensemble 
            yens = obj.target_ens;
            
            dxens = xens - mean(xens,2);
            err_ens = yens+randn(2,size(xens,2))*0.075-xens;
            [u,s,~] = svd(dxens);
            spr = rank(s); u=u(:,1:spr); s=s(1:spr,1:spr);
            %s = s/(size(xens,2)-1);
            obj.control_ens = obj.control_ens*(eye(size(xens,2))+...
                dxens'*(u*(pinv(s.*s +max(0.0025,min(diag(s)).^2) )*u' * ...
                err_ens)));
            ensupd = median(obj.control_ens, 2);
        end

        function state_ens = Forecast(obj, xin, nens, state_ens, control_ens)
        % Prediction of state from t=0 to horizon (t+horizon)
            ic = [repmat(xin, [1, nens]); control_ens];

            % Change dynamics for different systems
            for i = 1:nens
                x0 = [ic(:,i); obj.params];
                [~,x] = ode45(@obj.state_fnc,[0 obj.horizon],x0);
                x(:,1:2) = atan2(sin(x(:,1:2)),cos(x(:,1:2)));
                state_ens(:,i) = x(end,1:4)';
            end          
        end

        function yout = ObsOperator(obj, xin)%xin = state, parameters          
        % Defines equations that provide current state of system
        % Input xin = current state
        %       parameters = parameters defining system
        % Output yout = state transformed to relevant coordinates
            state = xin;
            parameters = obj.params;
            
            % Change for different systems
            L1 = parameters(2); L2 = parameters(4);
            th1 = state(1,:)-pi/2; th2 = state(2,:)-pi/2;
            yout = [L1*cos(th1)+L2*cos(th2); L1*sin(th1)+L2*sin(th2)];
        end
        
%         function failsafe()
%         % Create rules for position arguments that we know will fail
%         end
%         function anti_wind_up()
          % Set checkers to reset controller in case fo failure
%         end
% 
    end
end
