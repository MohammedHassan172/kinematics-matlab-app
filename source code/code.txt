classdef app1 < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        UIFigure                  matlab.ui.Figure
        constraintsLabel          matlab.ui.control.Label
        UITable6                  matlab.ui.control.Table
        UITable4_2                matlab.ui.control.Table
        UITable5                  matlab.ui.control.Table
        dCheckBox                 matlab.ui.control.CheckBox
        UITable4                  matlab.ui.control.Table
        UITable3                  matlab.ui.control.Table
        robottyperorpEditField    matlab.ui.control.EditField
        robottyperorpLabel        matlab.ui.control.Label
        LinksSpinner              matlab.ui.control.Spinner
        LinksSpinnerLabel         matlab.ui.control.Label
        ProblemtypeDropDown       matlab.ui.control.DropDown
        ProblemtypeDropDownLabel  matlab.ui.control.Label
        UITable2                  matlab.ui.control.Table
        calculateButton           matlab.ui.control.Button
        DHparamatersLabel         matlab.ui.control.Label
        UITable                   matlab.ui.control.Table
    end

    
    properties (Access = private)
        myax % Description
    end

    
    methods (Access = private)
        
        function draw_bot(app)
            q_mat = zeros(app.LinksSpinner.Value,1);
            mat_alpha = app.UITable.Data;
            mat_alpha(:,2) = mat_alpha(:,2) * pi;
            mat_theta = app.UITable2.Data;
            mat_theta(:,2) = mat_theta(:,2) * pi;
            dh_mat = horzcat(mat_alpha,mat_theta);
            roboot = rigidBodyTree;
            bodyes = rigidBody.empty();
            jntes = rigidBodyJoint.empty();

            bodyes(1) = rigidBody('body1');
            if lower(app.robottyperorpEditField.Value(1)) == 'r'
                jntes(1) = rigidBodyJoint('jnt1','revolute');
                q_mat(1) = dh_mat(1,4);
            else
                jntes(1) = rigidBodyJoint('jnt1','prismatic');
                q_mat(1) = dh_mat(1,3);
            end

            setFixedTransform(jntes(1),dh_mat(1,:),'dh');
            bodyes(1).Joint = jntes(1);

            addBody(roboot,bodyes(1),'base');
            for i = 2:app.LinksSpinner.Value
                bodyes(i) = rigidBody('body' + string(i));

                if lower(app.robottyperorpEditField.Value(i)) == 'r'
                    jntes(i) = rigidBodyJoint('jnt' + string(i),'revolute');
                    q_mat(i) = dh_mat(i,4);
                else
                    jntes(i) = rigidBodyJoint('jnt' + string(i),'prismatic');
                    q_mat(i) = dh_mat(i,3);
                end

                setFixedTransform(jntes(i),dh_mat(i,:),'dh');
                bodyes(i).Joint = jntes(i);

                addBody(roboot,bodyes(i),'body' + string(i - 1));           
            end
            q = homeConfiguration(roboot);
            if app.ProblemtypeDropDown.Value == "Forward Kinematics Solver"
                for i = 1:app.LinksSpinner.Value
                    q(1,i).JointPosition = q_mat(i);
                end
            elseif app.ProblemtypeDropDown.Value == "Inverse Kinematics Solver"
                for i = 1:app.LinksSpinner.Value
                    q(1,i).JointPosition = app.UITable3.Data(i);
                end
            end
            
            show(roboot,q,"Parent",app.myax);
        
            
            
            % axis([-0.5,0.5,-0.5,0.5,-0.5,0.5]);
        end
        
        
        function forward_kei(app,mat_1,mat_2,links,fk)
            for i = 1:links
               a_mat = zeros(4,4);
               a_i = mat_1(i,1);
               alpha_i = mat_1(i,2) * pi;
               d_i = mat_2(i,1);
               theta_i = mat_2(i,2) * pi;
               a_mat(1,1) = cos(theta_i);
               a_mat(1,2) = -sin(theta_i) * cos(alpha_i);
               a_mat(1,3) = sin(theta_i) * sin(alpha_i);
               a_mat(1,4) = a_i * cos(theta_i);
               a_mat(2,1) = sin(theta_i);
               a_mat(2,2) = cos(theta_i) * cos(alpha_i);
               a_mat(2,3) = -cos(theta_i) * sin(alpha_i);
               a_mat(2,4) = a_i * sin(theta_i);
               a_mat(3,2) = sin(alpha_i);
               a_mat(3,3) = cos(alpha_i);
               a_mat(3,4) = d_i;
               a_mat(4,4) = 1;
               fk(:,:,i) = a_mat;
            end
            res = eye(4);
            for i = 1:links
                res = res * fk(:,:,i);
            end
            app.UITable3.Data = res;
            % for i = 1:4
            %     for j = 1:4
            %         if app.ProblemtypeDropDown.Value == "Forward Kinematics Solver"
            %             str = sprintf('%.4f', res(i,j));
            %             app.UITable3.Data{i,j} = str;
            %         else
            %             % str = evalc('disp(res(i,j))');
            %             % app.UITable3.Data{i,j} = str;
            %         end
            % 
            %     end
            % end
            draw_bot(app);
            hold(app.myax,'off');
            
        end

% *******************************************************************************************************

        function inverse_kei(app,mat_1,mat_2,links,fk)
            q_v = sym(zeros(links,1));
            for i = 1:links
               a_mat = sym(zeros(4,4));
               a_i = mat_1(i,1);
               alpha_i = mat_1(i,2) * pi;

               if lower(app.robottyperorpEditField.Value(i)) == 'r'
                   theta_i = sym("q" + string(i));
                   d_i = mat_2(i,1);
                   q_v(i) = theta_i;
               elseif lower(app.robottyperorpEditField.Value(i)) == 'p'
                   d_i = sym("d" + string(i));
                   theta_i = mat_2(i,2) * pi;
                   q_v(i) = d_i;
               else
                   d_i = mat_2(i,1);
                   theta_i = mat_2(i,2) * pi;
               end
               
               a_mat(1,1) = cos(theta_i);
               a_mat(1,2) = -sin(theta_i) * cos(alpha_i);
               a_mat(1,3) = sin(theta_i) * sin(alpha_i);
               a_mat(1,4) = a_i * cos(theta_i);
               a_mat(2,1) = sin(theta_i);
               a_mat(2,2) = cos(theta_i) * cos(alpha_i);
               a_mat(2,3) = -cos(theta_i) * sin(alpha_i);
               a_mat(2,4) = a_i * sin(theta_i);
               a_mat(3,2) = sin(alpha_i);
               a_mat(3,3) = cos(alpha_i);
               a_mat(3,4) = d_i;
               a_mat(4,4) = 1;
               fk(:,:,i) = a_mat;
            end
            res = eye(4);
            for i = 1:links
                res = res * fk(:,:,i);
            end

            % for i = 1:4
            %     for j = 1:4
            %         if app.ProblemtypeDropDown.Value == "Forward Kinematics Solver"
            %             str = sprintf('%.4f', res(i,j));
            %             app.UITable3.Data{i,j} = str;
            %         else
            %             % str = evalc('disp(res(i,j))');
            %             % app.UITable3.Data{i,j} = str;
            %         end
            % 
            %     end
            % end


            % syms q1 d2 q3
            % goal_points = [3 -2];
            goal_points = app.UITable4.Data;
            % xf = transpose(goal_points);
            xf = goal_points;
            xi = 0.1;
            error_margin = 0.01;
            init_config = app.UITable5.Data;
            init_config = init_config + 0.00001;
            disp(init_config);
            
            % q0 = transpose(init_config);
            q0 = init_config;
            if app.dCheckBox.Value == true
                kinematics = res(1:3,4);
            else
                kinematics = res(1:2,4);
            end
            Ja = jacobian(kinematics,q_v);
            Jt = transpose(Ja);
            JJt = Ja*Jt;
            % JJt_inverse = inv(JJt);
            J_hash = Jt/JJt;
            q = q0;


            
            for i = 1:5000
                % *** you must replace symbolic variables q1, q2 with q(:,i) ***
                kinematics_ = subs(kinematics, transpose(q_v),q(:,i).');
                

                J_hash_ = subs(J_hash, transpose(q_v), q(:,i).');
                q(:,i+1) = q(:,i) + xi*J_hash_*(xf - kinematics_);
                dist = norm(xf - kinematics_);

                % *** This boils up into an infinite loop
                % while dist >= error_margin
                %      i = i + 1;
                % end
                if dist < error_margin
                    % q(:,i) = q(:,i) + 5 * ones(app.LinksSpinner.Value,1);
                    % num_sol = num_sol + 1;
                    % if num_sol >= 5
                    %     break;
                    % end
                    
                    % app.UITable3.Data(:,num_sol) = q(:,i);
                    app.UITable3.Data = q(:,i);
                    % app.UITable3.Data(height(app.UITable2.DisplayData),:) = [];
                    break; 
                end
                % *** You cannot use disp in this way: use fprintf instead
                fprintf('Number of iterations = %d\n',i);
            end
            assignin('base','q',q);
            out = zeros(size(kinematics_,1),1);
            for i=1:size(kinematics_,1)
                out(i) = str2double(sprintf('%f', kinematics_(i)));
            end
            
            app.UITable4_2.Data = out;
            
            % fprintf('kinematics = [%f , %f]\n', kinematics_(1), kinematics_(2));
            draw_bot(app);
            hold(app.myax,'on');
        end

% *******************************************************************************************************        
        
        function workspace(app,mat_1,mat_2,links,fk)
            q_v = sym(zeros(links,1));
            for i = 1:links
               a_mat = sym(zeros(4,4));
               a_i = mat_1(i,1);
               alpha_i = mat_1(i,2) * pi;

               if lower(app.robottyperorpEditField.Value(i)) == 'r'
                   theta_i = sym("q" + string(i));
                   d_i = mat_2(i,1);
                   q_v(i) = theta_i;
               elseif lower(app.robottyperorpEditField.Value(i)) == 'p'
                   d_i = sym("d" + string(i));
                   theta_i = mat_2(i,2) * pi;
                   q_v(i) = d_i;
               else
                   d_i = mat_2(i,1);
                   theta_i = mat_2(i,2) * pi;
               end
               
               a_mat(1,1) = cos(theta_i);
               a_mat(1,2) = -sin(theta_i) * cos(alpha_i);
               a_mat(1,3) = sin(theta_i) * sin(alpha_i);
               a_mat(1,4) = a_i * cos(theta_i);
               a_mat(2,1) = sin(theta_i);
               a_mat(2,2) = cos(theta_i) * cos(alpha_i);
               a_mat(2,3) = -cos(theta_i) * sin(alpha_i);
               a_mat(2,4) = a_i * sin(theta_i);
               a_mat(3,2) = sin(alpha_i);
               a_mat(3,3) = cos(alpha_i);
               a_mat(3,4) = d_i;
               a_mat(4,4) = 1;
               fk(:,:,i) = a_mat;
            end
            res = eye(4);
            for i = 1:links
                res = res * fk(:,:,i);
            end

            test_vaules =cell(1,links);
            
            draw_value = cell(1,links);
            for i=1:links
                test_vaules{i} = linspace(app.UITable6.Data(i,1),app.UITable6.Data(i,2),30) * pi;
            end
            [draw_value{:}] = ndgrid(test_vaules{:});
            
            disp("done");
            xm = subs(res(1,4),transpose(q_v),draw_value);
            ym = subs(res(2,4),transpose(q_v),draw_value);
            zm = subs(res(3,4),transpose(q_v),draw_value);
            plot3(app.myax,xm(:),ym(:),zm(:),'.')
           
            
            
            
        end
    end
    

    % Callbacks that handle component events
    methods (Access = private)

        % Code that executes after component creation
        function startupFcn(app)

            app.myax = axes(app.UIFigure);
            app.myax.Position = [0.48 0.45 0.5 0.5];
            
            app.UITable4.Visible = "off";
            app.UITable4_2.Visible = "off";
            app.UITable4.ColumnWidth = {76};
            app.UITable4_2.ColumnWidth = {76};
            app.UITable5.Visible = "off";
            app.dCheckBox.Visible= "off";
            app.UITable6.Visible = "off";
            app.constraintsLabel.Visible = "off";
        end

        % Button pushed function: calculateButton
        function calculateButtonPushed(app, event)
            mat_1 = app.UITable.Data;
            mat_2 = app.UITable2.Data;
            links = app.LinksSpinner.Value;
            fk = zeros(4,4,links);
            if app.ProblemtypeDropDown.Value == "Forward Kinematics Solver"
                forward_kei(app,mat_1,mat_2,links,fk);
            elseif app.ProblemtypeDropDown.Value == "Inverse Kinematics Solver"
                inverse_kei(app,mat_1,mat_2,links,sym(fk))
            elseif app.ProblemtypeDropDown.Value == "Robot workspace drawing"
                workspace(app,mat_1,mat_2,links,sym(fk));
            end
        end

        % Value changing function: LinksSpinner
        function LinksSpinnerValueChanging(app, event)
            changingValue = event.Value;
            if height(app.UITable.DisplayData) > changingValue
                for c = 1:(height(app.UITable.DisplayData) - changingValue)
                    app.UITable.Data(height(app.UITable.DisplayData),:) = [];
                    app.UITable2.Data(height(app.UITable2.DisplayData),:) = [];
                    app.UITable5.Data(height(app.UITable5.DisplayData),:) = [];
                    app.UITable6.Data(height(app.UITable6.DisplayData),:) = [];
                end
            else
                for c = 1:(changingValue - height(app.UITable.DisplayData))
                    input = [0 0];
                    app.UITable.Data = [app.UITable.Data; input];
                    app.UITable2.Data = [app.UITable2.Data; input];
                    app.UITable5.Data = [app.UITable5.Data; input(1)];
                    app.UITable6.Data = [app.UITable6.Data; input];
                end
            end
            
        end

        % Value changed function: ProblemtypeDropDown
        function ProblemtypeDropDownValueChanged(app, event)
            value = app.ProblemtypeDropDown.Value;
            if value == "Inverse Kinematics Solver"
                app.UITable4.Visible = "on";
                app.UITable4_2.Visible = "on";
                app.dCheckBox.Visible= "on";
                app.UITable5.Visible= "on";
                app.UITable4.Data = zeros(2,1);
                app.UITable3.Data = [];
                app.UITable5.Data= zeros(app.LinksSpinner.Value,1);
                disp(app.UITable4.Data);
            elseif value == "Robot workspace drawing"
                app.UITable6.Visible = "on";
                app.constraintsLabel.Visible = "on";

                app.UITable4.Visible = "off";
                app.UITable4_2.Visible = "off";
                app.dCheckBox.Visible= "off";
                app.UITable5.Visible= "off";
                app.UITable4_2.Visible = "off";
            else
                app.UITable4.Visible = "off";
                app.UITable4_2.Visible = "off";
                app.dCheckBox.Visible= "off";
                app.UITable5.Visible= "off";
                app.UITable4_2.Visible = "off";
                app.UITable6.Visible = "off";
                app.constraintsLabel.Visible = "off";
            end
        end

        % Value changed function: dCheckBox
        function dCheckBoxValueChanged(app, event)
            value = app.dCheckBox.Value;
            if value == true
               app.UITable4.Data = zeros(3,1);
            else
               app.UITable4.Data = zeros(2,1);
            end
        end
    end

    % Component initialization
    methods (Access = private)

        % Create UIFigure and components
        function createComponents(app)

            % Create UIFigure and hide until all components are created
            app.UIFigure = uifigure('Visible', 'off');
            app.UIFigure.Position = [100 100 731 551];
            app.UIFigure.Name = 'MATLAB App';

            % Create UITable
            app.UITable = uitable(app.UIFigure);
            app.UITable.ColumnName = {'a_i'; 'alpha_i/pi'};
            app.UITable.RowName = {};
            app.UITable.ColumnEditable = true;
            app.UITable.Position = [19 209 148 166];

            % Create DHparamatersLabel
            app.DHparamatersLabel = uilabel(app.UIFigure);
            app.DHparamatersLabel.Position = [19 384 90 22];
            app.DHparamatersLabel.Text = 'DH paramaters:';

            % Create calculateButton
            app.calculateButton = uibutton(app.UIFigure, 'push');
            app.calculateButton.ButtonPushedFcn = createCallbackFcn(app, @calculateButtonPushed, true);
            app.calculateButton.Position = [179 434 100 23];
            app.calculateButton.Text = 'calculate';

            % Create UITable2
            app.UITable2 = uitable(app.UIFigure);
            app.UITable2.ColumnName = {'d_i'; 'theta_i/pi'};
            app.UITable2.RowName = {};
            app.UITable2.ColumnEditable = true;
            app.UITable2.Position = [19 14 148 185];

            % Create ProblemtypeDropDownLabel
            app.ProblemtypeDropDownLabel = uilabel(app.UIFigure);
            app.ProblemtypeDropDownLabel.HorizontalAlignment = 'right';
            app.ProblemtypeDropDownLabel.Position = [19 488 76 22];
            app.ProblemtypeDropDownLabel.Text = 'Problem type';

            % Create ProblemtypeDropDown
            app.ProblemtypeDropDown = uidropdown(app.UIFigure);
            app.ProblemtypeDropDown.Items = {'Forward Kinematics Solver', 'Inverse Kinematics Solver', 'Robot workspace drawing'};
            app.ProblemtypeDropDown.ValueChangedFcn = createCallbackFcn(app, @ProblemtypeDropDownValueChanged, true);
            app.ProblemtypeDropDown.Position = [106 488 186 22];
            app.ProblemtypeDropDown.Value = 'Forward Kinematics Solver';

            % Create LinksSpinnerLabel
            app.LinksSpinnerLabel = uilabel(app.UIFigure);
            app.LinksSpinnerLabel.HorizontalAlignment = 'right';
            app.LinksSpinnerLabel.Position = [19 434 33 22];
            app.LinksSpinnerLabel.Text = 'Links';

            % Create LinksSpinner
            app.LinksSpinner = uispinner(app.UIFigure);
            app.LinksSpinner.ValueChangingFcn = createCallbackFcn(app, @LinksSpinnerValueChanging, true);
            app.LinksSpinner.Limits = [0 Inf];
            app.LinksSpinner.RoundFractionalValues = 'on';
            app.LinksSpinner.Position = [67 434 100 22];

            % Create robottyperorpLabel
            app.robottyperorpLabel = uilabel(app.UIFigure);
            app.robottyperorpLabel.HorizontalAlignment = 'right';
            app.robottyperorpLabel.Position = [177 405 98 22];
            app.robottyperorpLabel.Text = 'robot type(r or p):';

            % Create robottyperorpEditField
            app.robottyperorpEditField = uieditfield(app.UIFigure, 'text');
            app.robottyperorpEditField.Position = [180 374 100 22];

            % Create UITable3
            app.UITable3 = uitable(app.UIFigure);
            app.UITable3.ColumnName = '';
            app.UITable3.RowName = {};
            app.UITable3.Position = [401 14 302 157];

            % Create UITable4
            app.UITable4 = uitable(app.UIFigure);
            app.UITable4.ColumnName = {'x;y;z'};
            app.UITable4.ColumnWidth = {75};
            app.UITable4.RowName = {};
            app.UITable4.ColumnEditable = true;
            app.UITable4.Position = [182 14 78 157];

            % Create dCheckBox
            app.dCheckBox = uicheckbox(app.UIFigure);
            app.dCheckBox.ValueChangedFcn = createCallbackFcn(app, @dCheckBoxValueChanged, true);
            app.dCheckBox.Text = '3d';
            app.dCheckBox.Position = [182 177 35 22];

            % Create UITable5
            app.UITable5 = uitable(app.UIFigure);
            app.UITable5.ColumnName = {'intit values'};
            app.UITable5.RowName = {};
            app.UITable5.ColumnEditable = true;
            app.UITable5.Position = [279 14 89 157];

            % Create UITable4_2
            app.UITable4_2 = uitable(app.UIFigure);
            app.UITable4_2.ColumnName = {'x;y;z_res'};
            app.UITable4_2.RowName = {};
            app.UITable4_2.ColumnEditable = true;
            app.UITable4_2.Position = [180 209 78 157];

            % Create UITable6
            app.UITable6 = uitable(app.UIFigure);
            app.UITable6.ColumnName = {'lower/pi'; 'upper/pi'};
            app.UITable6.RowName = {};
            app.UITable6.ColumnEditable = true;
            app.UITable6.Position = [178 14 153 185];

            % Create constraintsLabel
            app.constraintsLabel = uilabel(app.UIFigure);
            app.constraintsLabel.Position = [181 198 184 44];
            app.constraintsLabel.Text = {'constraints: '; '{note: divide by pi only in R joints '; 'no need for P joints constrains} '};

            % Show the figure after all components are created
            app.UIFigure.Visible = 'on';
        end
    end

    % App creation and deletion
    methods (Access = public)

        % Construct app
        function app = app1

            % Create UIFigure and components
            createComponents(app)

            % Register the app with App Designer
            registerApp(app, app.UIFigure)

            % Execute the startup function
            runStartupFcn(app, @startupFcn)

            if nargout == 0
                clear app
            end
        end

        % Code that executes before app deletion
        function delete(app)

            % Delete UIFigure when app is deleted
            delete(app.UIFigure)
        end
    end
end