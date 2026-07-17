classdef plotPowertrainEnergyResultTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addProjectPaths(testCase)
            projectRoot = fileparts(fileparts(fileparts(mfilename("fullpath"))));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(projectRoot, "src"), IncludingSubfolders=true));
        end
    end

    methods (TestMethodSetup)
        function hideFigures(testCase)
            original = get(groot, "DefaultFigureVisible");
            set(groot, "DefaultFigureVisible", "off");
            testCase.addTeardown(@() set(groot, ...
                "DefaultFigureVisible", original));
            testCase.addTeardown(@() close("all", "force"));
        end
    end

    methods (Test)
        function testAllEngineeringViewsDoNotMutateResult(testCase)
            result = plotPowertrainEnergyResultTest.makeResult();
            before = result;

            fig = plot_powertrain_energy_result(result);

            axesHandles = findall(fig, Type="axes");
            titleText = string(arrayfun(@(ax) ax.Title.String, ...
                axesHandles, UniformOutput=false));
            powerRule = findobj(fig, Tag="fsg-rule-power");
            currentRule = findobj(fig, Tag="fsg-rule-current");
            activeImage = findobj(fig, Tag="active-constraints");
            energyLine = findobj(fig, Tag="cumulative-energy");
            limiterShare = findobj(fig, Tag="limiter-share");
            testCase.verifyTrue(isgraphics(fig, "figure"));
            testCase.verifyNumElements(axesHandles, 10);
            testCase.verifyEqual(sort(titleText), sort([ ...
                "Speed and Acceleration"; "Curvature"; ...
                "Primary Limiter"; "Active Constraints"; ...
                "Motor Speed"; "Motor Torque"; ...
                "TSAC Power"; "TSAC Current"; ...
                "Cumulative Lap Energy"; "Limiter Distance Share"]));
            testCase.verifyEqual(powerRule.YData, [80e3 80e3], ...
                AbsTol=0);
            testCase.verifyEqual(string(powerRule.DisplayName), ...
                "FSG rule reference (80 kW)");
            testCase.verifyEqual(currentRule.YData, [500 500], ...
                AbsTol=0);
            testCase.verifyEqual(string(currentRule.DisplayName), ...
                "FSG rule reference (500 A)");
            testCase.verifySize(activeImage.CData, [14 4]);
            testCase.verifyEqual(energyLine.XData(:), ...
                [0; cumsum(result.track.ds_m)], AbsTol=0);
            testCase.verifyEqual(sum(limiterShare.YData), 100, ...
                AbsTol=1e-12);
            testCase.verifyTrue(isequaln(result, before));
        end

        function testMissingPowertrainContractErrors(testCase)
            result = plotPowertrainEnergyResultTest.makeResult();
            result = rmfield(result, "powertrain");

            action = @() plot_powertrain_energy_result(result);

            testCase.verifyError(action, "QSSLTS:PlotResultContract");
        end

        function testNodeShapeMismatchErrors(testCase)
            result = plotPowertrainEnergyResultTest.makeResult();
            result.powertrain.motor_speed_rpm = [1000; 2000];

            action = @() plot_powertrain_energy_result(result);

            testCase.verifyError(action, "QSSLTS:PlotResultShape");
        end

        function testSegmentShapeMismatchErrors(testCase)
            result = plotPowertrainEnergyResultTest.makeResult();
            result.energy.cumulative_energy_ts_kWh = [0; 0.1; 0.2];

            action = @() plot_powertrain_energy_result(result);

            testCase.verifyError(action, "QSSLTS:PlotResultShape");
        end
    end

    methods (Static, Access=private)
        function result = makeResult()
            nPoint = 4;
            result.s_m = [0; 10; 25; 40];
            result.v_mps = [10; 15; 20; 12];
            result.ax_mps2 = [1; 2; -3; 0];
            result.ay_mps2 = [0; 4; -5; 1];
            result.limiter = ["traction"; "rule_power"; "brake"; "lateral"];
            result.track.s_m = result.s_m;
            result.track.ds_m = [10; 15; 15; 10];
            result.track.kappa_1pm = [0; 0.02; -0.015; 0.005];
            result.track.is_closed = true;
            result.powertrain.motor_speed_rpm = [1000; 2000; 3000; 1800];
            result.powertrain.motor_torque_used_Nm = [40; 60; 20; 10];
            result.powertrain.motor_torque_available_Nm = [80; 75; 65; 70];
            result.powertrain.tsac_power_used_W = [20000; 50000; 10000; 5000];
            result.powertrain.tsac_power_cap_W = [80000; 80000; 70000; 80000];
            result.powertrain.tsac_dc_current_used_A = [40; 100; 20; 10];
            result.energy.cumulative_energy_ts_kWh = [0; 0.03; 0.09; 0.11; 0.12];
            result.active_constraints = ...
                plotPowertrainEnergyResultTest.makeActive(nPoint);
        end

        function active = makeActive(nPoint)
            names = ["lateral", "brake", "traction", "motor_torque", ...
                "motor_power", "motor_speed", "motor_voltage", ...
                "rule_power", "rule_current", "battery_power", ...
                "battery_current", "inverter_power", ...
                "inverter_current", "top_speed"];
            values = false(nPoint, numel(names));
            values(1, 3) = true;
            values(2, 8) = true;
            values(3, 2) = true;
            values(4, 1) = true;
            active = cell2struct(num2cell(values, 1), ...
                cellstr(names), 2);
        end
    end
end
