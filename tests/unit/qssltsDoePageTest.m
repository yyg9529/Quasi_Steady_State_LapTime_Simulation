classdef qssltsDoePageTest < matlab.unittest.TestCase
    properties (SetAccess = private)
        ProjectRoot
    end

    methods (TestClassSetup)
        function addProjectPaths(testCase)
            testCase.ProjectRoot = fileparts(fileparts(fileparts( ...
                mfilename("fullpath"))));
            folders = ["app", "src", "data", "preprocessing"];
            for folder = folders
                testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                    fullfile(testCase.ProjectRoot, folder), ...
                    IncludingSubfolders=true));
            end
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                testCase.ProjectRoot));
        end
    end

    methods (TestMethodSetup)
        function closeFigures(testCase)
            testCase.addTeardown(@() close("all", "force"));
        end
    end

    methods (Test)
        function testDoeIsIndependentTopLevelPage(testCase)
            app = QssltsApp(false);
            testCase.addTeardown(@() delete(app));

            app.selectPage("doe");
            page = findobj(app.UIFigure, Tag="page-doe");

            testCase.verifyNotEmpty(page);
            testCase.verifyEqual(app.ActivePage, "doe");
            testCase.verifyEqual(string(page.Visible), "on");
            testCase.verifyNotEmpty(findobj(app.UIFigure, Tag="run-doe"));
            testCase.verifyNotEmpty(findobj(app.UIFigure, ...
                Tag="doe-input-table"));
            testCase.verifyNotEmpty(findobj(app.UIFigure, ...
                Tag="doe-ranking-table"));
            testCase.verifyNotEmpty(findobj(app.UIFigure, ...
                Tag="doe-response-axes"));
        end

        function testDoeResultStateStartsEmptyAndSeparate(testCase)
            app = QssltsApp(false);
            testCase.addTeardown(@() delete(app));

            doeResult = app.getLastDoeResult();
            lapResult = app.getLastResult();

            testCase.verifyEmpty(fieldnames(doeResult));
            testCase.verifyEmpty(fieldnames(lapResult));
        end


        function testDoeInputTableExposesRequestedParameters(testCase)
            app = QssltsApp(false);
            testCase.addTeardown(@() delete(app));
            inputTable = findobj(app.UIFigure, Tag="doe-input-table");
            expected = ["mass_kg", "cg_height_m", ...
                "inertia_Iz_kgm2", "front_static_frac", ...
                "wheelbase_m", "track_front_m", "track_rear_m", ...
                "cla_m2", "cda_m2", "front_downforce_frac", ...
                "gear_ratio"];

            testCase.verifyEqual( ...
                string(inputTable.Data.Properties.VariableNames), expected);
            testCase.verifyTrue(all(inputTable.ColumnEditable));
        end

        function testDoeInputTableUsesCurrentBaselineForNewParameters( ...
                testCase)
            app = QssltsApp(false);
            testCase.addTeardown(@() delete(app));
            inputTable = findobj(app.UIFigure, Tag="doe-input-table");
            state = app.getParameterState();
            expectedBaseline = [state.vehicle_mass_total_kg, ...
                state.vehicle_cg_height_m, ...
                state.vehicle_inertia_Iz_kgm2, ...
                state.vehicle_front_static_frac, ...
                state.vehicle_wheelbase_m, ...
                state.vehicle_track_front_m, ...
                state.vehicle_track_rear_m, state.aero_CLA_m2, ...
                state.aero_CDA_m2, ...
                state.aero_front_downforce_frac, ...
                state.powertrain_gear_ratio];

            testCase.verifyEqual(inputTable.Data{2, :}, ...
                expectedBaseline, AbsTol=1e-12);
            testCase.verifyEqual(inputTable.Data.mass_kg, ...
                state.vehicle_mass_total_kg + [-10; 0; 10], ...
                AbsTol=1e-12);
            testCase.verifyEqual(inputTable.Data{1, 2:end}, ...
                inputTable.Data{2, 2:end}, AbsTol=1e-12);
            testCase.verifyEqual(inputTable.Data{3, 2:end}, ...
                inputTable.Data{2, 2:end}, AbsTol=1e-12);
        end

        function testDoePageStatesYawInertiaModelBoundary(testCase)
            app = QssltsApp(false);
            testCase.addTeardown(@() delete(app));
            note = findobj(app.UIFigure, Tag="doe-model-boundary");

            testCase.verifyTrue(contains(string(note.Text), ...
                "当前固定赛线 QSS 圈时未使用"));
        end


        function testUntouchedDoeColumnsFollowCurrentParameters(testCase)
            app = QssltsApp(false);
            testCase.addTeardown(@() delete(app));
            massField = findobj(app.UIFigure, ...
                Tag="param-vehicle_mass_total_kg");
            cgField = findobj(app.UIFigure, ...
                Tag="param-vehicle_cg_height_m");
            gearField = findobj(app.UIFigure, ...
                Tag="param-powertrain_gear_ratio");
            massField.Value = 315;
            cgField.Value = 0.29;
            gearField.Value = 4.75;

            app.selectPage("doe");
            inputTable = findobj(app.UIFigure, Tag="doe-input-table");

            testCase.verifyEqual(inputTable.Data.mass_kg, ...
                [305; 315; 325], AbsTol=1e-12);
            testCase.verifyEqual(inputTable.Data.cg_height_m, ...
                0.29 * ones(3, 1), AbsTol=1e-12);
            testCase.verifyEqual(inputTable.Data.gear_ratio, ...
                4.75 * ones(3, 1), AbsTol=1e-12);
        end

        function testEditedDoeColumnIsPreservedWhenBaselineChanges(testCase)
            app = QssltsApp(false);
            testCase.addTeardown(@() delete(app));
            inputTable = findobj(app.UIFigure, Tag="doe-input-table");
            editedCg = [0.22; 0.27; 0.32];
            inputTable.Data.cg_height_m = editedCg;
            cgField = findobj(app.UIFigure, ...
                Tag="param-vehicle_cg_height_m");
            gearField = findobj(app.UIFigure, ...
                Tag="param-powertrain_gear_ratio");
            cgField.Value = 0.30;
            gearField.Value = 4.80;

            app.selectPage("doe");

            testCase.verifyEqual(inputTable.Data.cg_height_m, ...
                editedCg, AbsTol=1e-12);
            testCase.verifyEqual(inputTable.Data.gear_ratio, ...
                4.80 * ones(3, 1), AbsTol=1e-12);
        end
    end
end
