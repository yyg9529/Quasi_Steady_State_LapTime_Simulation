classdef QssltsApp < handle
    %QSSLTSAPP Programmatic engineering GUI for the QSSLTS workflow.

    properties (SetAccess = private)
        UIFigure matlab.ui.Figure
        ActivePage (1,1) string = "dashboard"
    end

    properties (Access = private)
        ProjectRoot (1,1) string
        RootGrid matlab.ui.container.GridLayout
        PageStack matlab.ui.container.GridLayout
        Pages struct = struct()
        NavButtons struct = struct()
        StatusLabel matlab.ui.control.Label
        ParameterStatusLabel matlab.ui.control.Label
        ParameterControls struct = struct()
        DefaultParameterState struct = struct()
        KpiCards
        RunButton
        RunGauge
        RunProgressLabel
        RunStatusLabel
        RunDetailLabel
        RunTimer = []
        RunFuture = []
        RunStartTime = []
        IsRunning (1,1) logical = false
        DoeRunButton
        DoeStatusLabel
        DoeInputTable
        DoeRankingTable
        DoeResponseAxes
        DoeTimer = []
        DoeFuture = []
        IsDoeRunningFlag (1,1) logical = false
        LastResult struct = struct()
        LastSummary struct = struct()
        LastDoeResult struct = struct()
        LastHandlingResult struct = struct()
        ResultAxes struct = struct()
    end

    properties (Constant, Access = private)
        Navy = [0.018 0.039 0.055]
        NavyRaised = [0.035 0.105 0.120]
        Accent = [0.000 0.690 0.675]
        Canvas = [0.910 0.955 0.965]
        Surface = [0.965 0.990 0.992]
        Field = [0.925 0.975 0.978]
        Ink = [0.035 0.090 0.110]
        Muted = [0.310 0.410 0.455]
        Border = [0.720 0.860 0.875]
        Warning = [0.930 0.590 0.120]
    end

    methods
        function app = QssltsApp(visible)
            arguments
                visible (1,1) logical = true
            end

            app.ProjectRoot = string(fileparts(fileparts( ...
                mfilename("fullpath"))));
            app.initializeProjectPaths();
            app.createComponents();
            app.DefaultParameterState = app.getParameterState();
            app.selectPage("dashboard");

            if visible
                movegui(app.UIFigure, "center");
                app.UIFigure.Visible = "on";
            end
            if nargout == 0
                clear app
            end
        end

        function selectPage(app, pageName)
            pageName = string(pageName);
            validPages = ["dashboard", "parameters", "run", "results", ...
                "doe"];
            if ~ismember(pageName, validPages)
                error("QSSLTS:AppPage", ...
                    "Unknown GUI page: %s", pageName);
            end

            for name = validPages
                isActive = name == pageName;
                app.Pages.(name).Visible = app.onOff(isActive);
                if isActive
                    app.NavButtons.(name).BackgroundColor = app.Accent;
                    app.NavButtons.(name).FontColor = [1 1 1];
                else
                    app.NavButtons.(name).BackgroundColor = app.NavyRaised;
                    app.NavButtons.(name).FontColor = [0.78 0.90 0.91];
                end
            end
            app.ActivePage = pageName;
        end

        function state = getParameterState(app)
            names = string(fieldnames(app.ParameterControls));
            state = struct();
            for name = names.'
                state.(name) = app.ParameterControls.(name).Value;
            end
            defaults = qsslts_gui_default_state();
            state.vehicle_preset = defaults.vehicle_preset;
            state.tire_preset = defaults.tire_preset;
            state.aero_preset = defaults.aero_preset;
            state.brake_preset = defaults.brake_preset;
            state = validate_qsslts_gui_state(state);
        end

        function config = buildRuntimeConfig(app)
            config = qsslts_gui_state_to_runtime_config( ...
                app.getParameterState(), app.ProjectRoot);
        end

        function exportParameters(app, filePath)
            write_qsslts_gui_config(filePath, app.getParameterState());
            [~, name, extension] = fileparts(string(filePath));
            app.setParameterStatus("已保存参数：" + name + extension, ...
                app.Accent);
        end

        function importParameters(app, filePath)
            state = read_qsslts_gui_config(filePath);
            app.applyParameterState(state);
            [~, name, extension] = fileparts(string(filePath));
            app.setParameterStatus("已载入参数：" + name + extension, ...
                app.Accent);
        end

        function runSimulation(app, synchronous)
            arguments
                app
                synchronous (1,1) logical = false
            end

            if app.IsRunning || app.IsDoeRunningFlag
                error("QSSLTS:AnalysisRunning", ...
                    "A QSSLTS simulation or DOE is already running.");
            end

            config = app.buildRuntimeConfig();
            app.beginSimulation();
            if synchronous
                try
                    result = run_analysis_case(config);
                    app.finishSimulation(result);
                catch exception
                    app.failSimulation(exception, false);
                    rethrow(exception)
                end
                return
            end

            try
                app.RunFuture = parfeval( ...
                    backgroundPool, @run_analysis_case, 1, config);
                app.RunTimer = timer( ...
                    ExecutionMode="fixedSpacing", Period=0.25, ...
                    BusyMode="drop", ...
                    TimerFcn=@(~,~) app.pollSimulation());
                start(app.RunTimer);
            catch exception
                app.failSimulation(exception, false);
                rethrow(exception)
            end
        end

        function cancelSimulation(app)
            app.stopRunTimer();
            if ~isempty(app.RunFuture)
                try
                    if isvalid(app.RunFuture) ...
                            && string(app.RunFuture.State) ~= "finished"
                        cancel(app.RunFuture);
                    end
                catch
                end
            end
            app.RunFuture = [];
            app.IsRunning = false;
            app.restoreRunControls("仿真已取消", 0, app.Warning);
            if ~isempty(app.StatusLabel) && isgraphics(app.StatusLabel)
                app.StatusLabel.Text = "●  仿真已由用户取消";
                app.StatusLabel.FontColor = app.Warning;
            end
        end

        function tf = isSimulationRunning(app)
            tf = app.IsRunning;
        end

        function result = getLastResult(app)
            result = app.LastResult;
        end

        function summary = getLastSummary(app)
            summary = app.LastSummary;
        end

        function runDoe(app, doeTable, synchronous)
            arguments
                app
                doeTable table
                synchronous (1,1) logical = false
            end

            if app.IsDoeRunningFlag || app.IsRunning
                error("QSSLTS:AnalysisRunning", ...
                    "A QSSLTS simulation or DOE is already running.");
            end

            config = app.buildRuntimeConfig();
            app.beginDoe();
            if synchronous
                try
                    result = run_doe(config, doeTable);
                    app.finishDoe(result);
                catch exception
                    app.failDoe(exception, false);
                    rethrow(exception)
                end
                return
            end

            try
                app.DoeFuture = parfeval(backgroundPool, @run_doe, 1, ...
                    config, doeTable);
                app.DoeTimer = timer(ExecutionMode="fixedSpacing", ...
                    Period=0.25, BusyMode="drop", ...
                    TimerFcn=@(~,~) app.pollDoe());
                start(app.DoeTimer);
            catch exception
                app.failDoe(exception, false);
                rethrow(exception)
            end
        end

        function cancelDoe(app)
            app.stopDoeTimer();
            if ~isempty(app.DoeFuture)
                try
                    if isvalid(app.DoeFuture) ...
                            && string(app.DoeFuture.State) ~= "finished"
                        cancel(app.DoeFuture);
                    end
                catch
                end
            end
            app.DoeFuture = [];
            app.IsDoeRunningFlag = false;
            app.restoreDoeControls("DOE cancelled", app.Warning);
        end

        function tf = isDoeRunning(app)
            tf = app.IsDoeRunningFlag;
        end

        function result = getLastDoeResult(app)
            result = app.LastDoeResult;
        end

        function result = getLastHandlingResult(app)
            result = app.LastHandlingResult;
        end

        function showHandlingResult(app, ymd, understeer)
            arguments
                app
                ymd (1,1) struct
                understeer (1,1) struct
            end
            app.LastHandlingResult = struct( ...
                "ymd", ymd, "understeer", understeer);
            app.renderHandlingResult(ymd, understeer);
            app.selectPage("results");
        end

        function delete(app)
            app.stopRunTimer();
            app.stopDoeTimer();
            if ~isempty(app.RunFuture)
                try
                    if isvalid(app.RunFuture) ...
                            && string(app.RunFuture.State) ~= "finished"
                        cancel(app.RunFuture);
                    end
                catch
                end
            end
            if ~isempty(app.DoeFuture)
                try
                    if isvalid(app.DoeFuture) ...
                            && string(app.DoeFuture.State) ~= "finished"
                        cancel(app.DoeFuture);
                    end
                catch
                end
            end
            if ~isempty(app.UIFigure) && isgraphics(app.UIFigure)
                delete(app.UIFigure);
            end
        end
    end

    methods (Access = private)
        function initializeProjectPaths(app)
            addpath(app.ProjectRoot);
            folders = ["app", "src", "data", "examples", ...
                "preprocessing"];
            for folder = folders
                folderPath = fullfile(app.ProjectRoot, folder);
                if isfolder(folderPath)
                    addpath(genpath(folderPath));
                end
            end
        end

        function createComponents(app)
            app.UIFigure = uifigure( ...
                Name="QSSLTS Engineering Studio", ...
                Position=[100 100 1440 900], ...
                Color=app.Canvas, ...
                Visible="off", ...
                AutoResizeChildren="off");
            app.UIFigure.CloseRequestFcn = @(~,~) delete(app);

            app.RootGrid = uigridlayout(app.UIFigure, [1 2]);
            app.RootGrid.ColumnWidth = {232, "1x"};
            app.RootGrid.RowHeight = {"1x"};
            app.RootGrid.Padding = [0 0 0 0];
            app.RootGrid.ColumnSpacing = 0;

            app.createSidebar();
            app.createPageStack();
        end

        function createSidebar(app)
            sidebar = uipanel(app.RootGrid, ...
                BorderType="none", BackgroundColor=app.Navy);
            sidebar.Layout.Row = 1;
            sidebar.Layout.Column = 1;

            grid = uigridlayout(sidebar, [10 1]);
            grid.RowHeight = {112, 48, 48, 48, 48, 48, "1x", 48, 24, 28};
            grid.Padding = [18 20 18 16];
            grid.RowSpacing = 8;
            grid.BackgroundColor = app.Navy;

            brand = uigridlayout(grid, [2 1]);
            brand.RowHeight = {48, 24};
            brand.Padding = [0 8 0 8];
            brand.BackgroundColor = app.Navy;
            uilabel(brand, Text="QSSLTS", FontName="Segoe UI", ...
                FontSize=27, FontWeight="bold", FontColor=[1 1 1], ...
                HorizontalAlignment="left");
            uilabel(brand, Text="ENGINEERING STUDIO", ...
                FontName="Segoe UI", FontSize=10, ...
                FontColor=app.Accent, HorizontalAlignment="left");

            app.NavButtons.dashboard = app.createNavButton( ...
                grid, 2, "概览  Dashboard", "dashboard");
            app.NavButtons.parameters = app.createNavButton( ...
                grid, 3, "参数  Parameters", "parameters");
            app.NavButtons.run = app.createNavButton( ...
                grid, 4, "运行  Simulation", "run");
            app.NavButtons.results = app.createNavButton( ...
                grid, 5, "结果  Results", "results");

            app.NavButtons.doe = app.createNavButton( ...
                grid, 6, "DOE  Design Study", "doe");

            scopePanel = uipanel(grid, BorderType="none", ...
                BackgroundColor=app.NavyRaised);
            scopePanel.Layout.Row = 8;
            scopeGrid = uigridlayout(scopePanel, [1 2]);
            scopeGrid.ColumnWidth = {8, "1x"};
            scopeGrid.Padding = [10 8 10 8];
            scopeGrid.BackgroundColor = app.NavyRaised;
            lamp = uilamp(scopeGrid, Color=app.Warning);
            lamp.Layout.Column = 1;
            uilabel(scopeGrid, Text="QSS 求解器 · 后台运行已接入", ...
                FontName="Microsoft YaHei UI", FontSize=10, ...
                FontColor=[0.88 0.90 0.94]);

            disciplineLabel = uilabel(grid, ...
                Text="Formula Student / FSAE", ...
                FontName="Segoe UI", FontSize=9, ...
                FontColor=[0.48 0.55 0.66], ...
                HorizontalAlignment="center");
            disciplineLabel.Layout.Row = 9;
            versionLabel = uilabel(grid, Text="TU TAIYUAN · Team Edition", ...
                FontName="Segoe UI", FontSize=9, ...
                FontColor=[0.48 0.55 0.66], ...
                HorizontalAlignment="center");
            versionLabel.Layout.Row = 10;
        end

        function button = createNavButton(app, parent, row, text, page)
            button = uibutton(parent, "push", Text=text, ...
                FontName="Microsoft YaHei UI", FontSize=12, ...
                FontWeight="bold", HorizontalAlignment="left", ...
                BackgroundColor=app.NavyRaised, ...
                FontColor=[0.78 0.90 0.91], ...
                ButtonPushedFcn=@(~,~) app.selectPage(page));
            button.Layout.Row = row;
        end

        function createPageStack(app)
            host = uipanel(app.RootGrid, BorderType="none", ...
                BackgroundColor=app.Canvas);
            host.Layout.Row = 1;
            host.Layout.Column = 2;
            app.PageStack = uigridlayout(host, [1 1]);
            app.PageStack.Padding = [0 0 0 0];
            app.PageStack.BackgroundColor = app.Canvas;

            app.Pages.dashboard = app.createDashboardPage();
            app.Pages.parameters = app.createParametersPage();
            app.Pages.run = app.createRunPage();
            app.Pages.results = app.createResultsPage();
            app.addHandlingResultTabs(app.Pages.results);
            app.Pages.doe = app.createDoePage();
        end

        function page = createDashboardPage(app)
            page = app.createPagePanel("page-dashboard");
            grid = uigridlayout(page, [4 1]);
            grid.RowHeight = {84, 146, "1x", 44};
            grid.Padding = [28 22 28 20];
            grid.RowSpacing = 16;
            grid.BackgroundColor = app.Canvas;

            app.createPageHeader(grid, "工程概览", ...
                "固定赛线 QSS · 配置检查、仿真状态与关键结果");

            app.KpiCards = uihtml(grid, ...
                HTMLSource=fullfile(app.ProjectRoot, "app", ...
                    "components", "kpi_cards.html"), ...
                Tag="kpi-cards");
            app.KpiCards.Data = struct( ...
                lapTime="—", maxSpeed="—", lapEnergy="—", ...
                convergence="尚未运行");

            content = uigridlayout(grid, [1 2]);
            content.ColumnWidth = {"2x", "1x"};
            content.Padding = [0 0 0 0];
            content.ColumnSpacing = 16;
            content.BackgroundColor = app.Canvas;

            trackPanel = app.createCard(content, "赛道预览");
            trackPanel.Layout.Column = 1;
            trackGrid = uigridlayout(trackPanel, [1 1]);
            trackGrid.Padding = [14 10 14 14];
            trackGrid.BackgroundColor = app.Surface;
            trackAxes = uiaxes(trackGrid, Tag="track-preview");
            app.styleAxes(trackAxes);
            app.drawTrackPreview(trackAxes);

            workflowPanel = app.createCard(content, "当前工作流");
            workflowPanel.Layout.Column = 2;
            workflow = uigridlayout(workflowPanel, [8 1]);
            workflow.RowHeight = {32, 50, 50, 50, 50, 16, 42, "1x"};
            workflow.Padding = [18 12 18 16];
            workflow.RowSpacing = 8;
            workflow.BackgroundColor = app.Surface;
            app.addWorkflowStep(workflow, 2, "01", "输入检查", ...
                "赛道已纳入仓库 · 参数待确认", app.Accent);
            app.addWorkflowStep(workflow, 3, "02", "GGV 生成", ...
                "尚未运行", app.Muted);
            app.addWorkflowStep(workflow, 4, "03", "速度剖面", ...
                "尚未运行", app.Muted);
            app.addWorkflowStep(workflow, 5, "04", "能量与约束", ...
                "尚未运行", app.Muted);
            goButton = uibutton(workflow, "push", ...
                Text="检查参数  →", FontName="Microsoft YaHei UI", ...
                FontWeight="bold", BackgroundColor=app.Accent, ...
                FontColor=[1 1 1], ...
                ButtonPushedFcn=@(~,~) app.selectPage("parameters"));
            goButton.Layout.Row = 7;

            statusGrid = uigridlayout(grid, [1 2]);
            statusGrid.ColumnWidth = {"1x", "fit"};
            statusGrid.Padding = [14 0 14 0];
            statusGrid.BackgroundColor = app.Surface;
            app.StatusLabel = uilabel(statusGrid, ...
                Text="●  系统就绪：等待参数确认与 QSS 求解", ...
                FontName="Microsoft YaHei UI", FontSize=11, ...
                FontColor=app.Muted);
            uilabel(statusGrid, Text="SI units  ·  X前 / Y左 / Z上", ...
                FontName="Segoe UI", FontSize=10, ...
                FontColor=app.Muted, HorizontalAlignment="right");
        end

        function page = createParametersPage(app)
            page = app.createPagePanel("page-parameters");
            grid = uigridlayout(page, [3 1]);
            grid.RowHeight = {84, "1x", 56};
            grid.Padding = [28 22 28 22];
            grid.RowSpacing = 16;
            grid.BackgroundColor = app.Canvas;

            app.createPageHeader(grid, "参数配置", ...
                "车辆、轮胎、制动与电驱参数统一采用 SI 单位并写入版本化 JSON");

            defaults = qsslts_gui_default_state();
            scrollPanel = uipanel(grid, BorderType="none", ...
                BackgroundColor=app.Canvas, Scrollable="on");
            cards = uigridlayout(scrollPanel, [4 2]);
            cards.ColumnWidth = {"1x", "1x"};
            cards.RowHeight = {430, 430, 800, 270};
            cards.Padding = [0 0 12 0];
            cards.RowSpacing = 16;
            cards.ColumnSpacing = 16;
            cards.BackgroundColor = app.Canvas;

            trackCard = app.createCard(cards, "赛道与工况");
            app.populateTrackInputs(trackCard, defaults);
            vehicleCard = app.createCard(cards, "车辆");
            app.populateVehicleInputs(vehicleCard, defaults);
            tireCard = app.createCard(cards, "轮胎");
            app.populateTireInputs(tireCard, defaults);
            aeroBrakeCard = app.createCard(cards, "空气动力学与制动");
            app.populateAeroBrakeInputs(aeroBrakeCard, defaults);
            motorCard = app.createCard(cards, "电机与传动");
            app.populateMotorInputs(motorCard, defaults);
            batteryCard = app.createCard(cards, "电池");
            app.populateBatteryInputs(batteryCard, defaults);
            inverterCard = app.createCard(cards, "逆变器");
            app.populateInverterInputs(inverterCard, defaults);
            solverCard = app.createCard(cards, "求解器");
            app.populateSolverInputs(solverCard, defaults);

            actions = uigridlayout(grid, [1 5]);
            actions.ColumnWidth = {"1x", 120, 120, 140, 170};
            actions.Padding = [0 4 0 4];
            actions.BackgroundColor = app.Canvas;
            app.ParameterStatusLabel = uilabel(actions, ...
                Text="参数尚未保存 · 当前值仍为概念或继承值", ...
                FontName="Microsoft YaHei UI", FontColor=app.Muted);
            loadButton = uibutton(actions, "push", Text="载入 JSON", ...
                Tag="load-parameters", FontName="Microsoft YaHei UI", ...
                BackgroundColor=app.Surface, FontColor=app.Ink, ...
                ButtonPushedFcn=@(~,~) app.loadParametersDialog());
            loadButton.Layout.Column = 2;
            saveButton = uibutton(actions, "push", Text="保存 JSON", ...
                Tag="save-parameters", FontName="Microsoft YaHei UI", ...
                BackgroundColor=app.Surface, FontColor=app.Ink, ...
                ButtonPushedFcn=@(~,~) app.saveParametersDialog());
            saveButton.Layout.Column = 3;
            resetButton = uibutton(actions, "push", Text="恢复基准值", ...
                FontName="Microsoft YaHei UI", BackgroundColor=app.Surface, ...
                FontColor=app.Ink, ...
                ButtonPushedFcn=@(~,~) app.resetParameters());
            resetButton.Layout.Column = 4;
            nextButton = uibutton(actions, "push", Text="进入运行检查  →", ...
                FontName="Microsoft YaHei UI", FontWeight="bold", ...
                BackgroundColor=app.Accent, FontColor=[1 1 1], ...
                ButtonPushedFcn=@(~,~) app.selectPage("run"));
            nextButton.Layout.Column = 5;
        end

        function page = createRunPage(app)
            page = app.createPagePanel("page-run");
            grid = uigridlayout(page, [3 1]);
            grid.RowHeight = {84, "1x", 54};
            grid.Padding = [28 22 28 22];
            grid.RowSpacing = 16;
            grid.BackgroundColor = app.Canvas;
            app.createPageHeader(grid, "仿真运行", ...
                "参数经适配层校验后，由后台任务调用 QSS 分析入口");

            body = uigridlayout(grid, [1 2]);
            body.ColumnWidth = {"1x", "1x"};
            body.Padding = [0 0 0 0];
            body.ColumnSpacing = 16;
            body.BackgroundColor = app.Canvas;

            summaryCard = app.createCard(body, "配置摘要");
            summary = uigridlayout(summaryCard, [7 2]);
            summary.RowHeight = {38, 38, 38, 38, 38, 38, "1x"};
            summary.ColumnWidth = {155, "1x"};
            summary.Padding = [18 16 18 18];
            summary.BackgroundColor = app.Surface;
            app.addSummaryRow(summary, 1, "赛道", "Tianji closed track");
            app.addSummaryRow(summary, 2, "车辆", "Baseline FSAE · 300 kg");
            app.addSummaryRow(summary, 3, "驱动", "RWD · EMRAX 228 HV CC");
            app.addSummaryRow(summary, 4, "GGV 网格", "0–45 m/s · ±4 g");
            app.addSummaryRow(summary, 5, "耐久", "26 laps · 1.10 safety factor");
            app.addSummaryRow(summary, 6, "参数可信度", "概念基准 / 待实测");

            runCard = app.createCard(body, "运行控制");
            runGrid = uigridlayout(runCard, [7 1]);
            runGrid.RowHeight = {40, 28, 80, 28, 22, 52, "1x"};
            runGrid.Padding = [24 20 24 20];
            runGrid.RowSpacing = 10;
            runGrid.BackgroundColor = app.Surface;
            uilabel(runGrid, Text="SOLVER READY", ...
                FontName="Segoe UI", FontSize=11, FontWeight="bold", ...
                FontColor=app.Accent);
            app.RunStatusLabel = uilabel(runGrid, Text="等待开始仿真", ...
                FontName="Microsoft YaHei UI", FontSize=15, ...
                FontWeight="bold", FontColor=app.Ink);
            app.RunGauge = uigauge(runGrid, "semicircular", ...
                Limits=[0 100], Value=0, ...
                ScaleColors=[app.Accent; app.Warning], ...
                ScaleColorLimits=[0 70; 70 100]);
            app.RunGauge.Layout.Row = 3;
            app.RunProgressLabel = uilabel(runGrid, ...
                Text="尚未运行", ...
                FontName="Microsoft YaHei UI", ...
                FontColor=app.Muted, HorizontalAlignment="center");
            app.RunDetailLabel = uilabel(runGrid, ...
                Text="运行期间显示阶段与耗时，不伪造非线性百分比", ...
                FontName="Microsoft YaHei UI", FontSize=10, ...
                FontColor=app.Muted, HorizontalAlignment="center");
            app.RunButton = uibutton(runGrid, "push", ...
                Text="开始仿真", Tag="run-simulation", ...
                FontName="Microsoft YaHei UI", FontWeight="bold", ...
                BackgroundColor=app.NavyRaised, FontColor=[1 1 1], ...
                ButtonPushedFcn=@(~,~) app.toggleSimulation());
            app.RunButton.Layout.Row = 6;

            uilabel(grid, ...
                Text="求解在后台执行；完成后自动进入结果页，运行中可安全取消", ...
                FontName="Microsoft YaHei UI", FontColor=app.Muted, ...
                HorizontalAlignment="center");
        end

        function page = createResultsPage(app)
            page = app.createPagePanel("page-results");
            grid = uigridlayout(page, [2 1]);
            grid.RowHeight = {84, "1x"};
            grid.Padding = [28 22 28 22];
            grid.RowSpacing = 16;
            grid.BackgroundColor = app.Canvas;
            app.createPageHeader(grid, "结果分析", ...
                "真实求解结果 · 速度、动力学、电驱能量与 GGV 能力边界");

            tabs = uitabgroup(grid);
            tabs.Tag = "results-tabs";
            app.ResultAxes.track = app.createResultTab(tabs, ...
                "赛道与速度", "Track speed map", ...
                "运行仿真后显示赛道速度云图");
            app.ResultAxes.dynamics = app.createResultTab(tabs, ...
                "车辆动力学", ["Speed / Ax / Ay", "Limiter share"], ...
                ["速度和加速度剖面", "主导约束的赛道占比"]);
            app.ResultAxes.energy = app.createResultTab(tabs, ...
                "动力与能量", ...
                ["Motor speed", "Motor torque", ...
                "TSAC power / current", "Cumulative energy"], ...
                ["电机转速", "使用与可用扭矩", ...
                "储能系统功率和电流", "单圈累计能量"]);
            app.ResultAxes.ggv = app.createResultTab(tabs, ...
                "GGV", "GGV capability", ...
                "速度切片能力边界与实际工况点");
        end

        function addHandlingResultTabs(app, resultsPage)
            tabs = findobj(resultsPage, Tag="results-tabs");
            app.ResultAxes.ymd = app.createResultTab(tabs, ...
                "YMD", "Yaw moment diagram", ...
                "Yaw moment versus sideslip and road-wheel steer");
            app.ResultAxes.ymd{1}.Tag = "result-ymd-axes";
            app.ResultAxes.understeer = app.createResultTab(tabs, ...
                "Understeer", "Understeer gradient", ...
                "Road-wheel steer and gradient versus lateral acceleration");
            app.ResultAxes.understeer{1}.Tag = ...
                "result-understeer-axes";
        end

        function page = createDoePage(app)
            page = app.createPagePanel("page-doe");
            grid = uigridlayout(page, [3 1]);
            grid.RowHeight = {84, "1x", 52};
            grid.Padding = [28 22 28 22];
            grid.RowSpacing = 16;
            grid.BackgroundColor = app.Canvas;
            app.createPageHeader(grid, "Design of Experiments", ...
                "Independent DOE result state, ranking, and response view");

            content = uigridlayout(grid, [1 2]);
            content.ColumnWidth = {420, "1x"};
            content.Padding = [0 0 0 0];
            content.ColumnSpacing = 16;
            content.BackgroundColor = app.Canvas;

            setupCard = app.createCard(content, "DOE cases");
            setupGrid = uigridlayout(setupCard, [3 1]);
            setupGrid.RowHeight = {"1x", 44, 32};
            setupGrid.Padding = [14 12 14 12];
            setupGrid.RowSpacing = 10;
            setupGrid.BackgroundColor = app.Surface;
            app.DoeInputTable = uitable(setupGrid, ...
                Data=table([290; 300; 310], ...
                    VariableNames="mass_kg"), ...
                ColumnEditable=true, Tag="doe-input-table");
            app.DoeRunButton = uibutton(setupGrid, "push", ...
                Text="Run DOE", Tag="run-doe", ...
                FontWeight="bold", BackgroundColor=app.Accent, ...
                FontColor=[1 1 1], ...
                ButtonPushedFcn=@(~,~) app.toggleDoe());
            app.DoeStatusLabel = uilabel(setupGrid, ...
                Text="DOE has not run", FontColor=app.Muted, ...
                HorizontalAlignment="center");

            responseCard = app.createCard(content, "DOE response");
            responseGrid = uigridlayout(responseCard, [2 1]);
            responseGrid.RowHeight = {"1x", "1x"};
            responseGrid.Padding = [14 12 14 12];
            responseGrid.RowSpacing = 12;
            responseGrid.BackgroundColor = app.Surface;
            app.DoeResponseAxes = uiaxes(responseGrid, ...
                Tag="doe-response-axes");
            app.styleAxes(app.DoeResponseAxes);
            title(app.DoeResponseAxes, "Lap-time response");
            xlabel(app.DoeResponseAxes, "Rank");
            ylabel(app.DoeResponseAxes, "Lap time (s)");
            app.DoeRankingTable = uitable(responseGrid, Data=table(), ...
                Tag="doe-ranking-table");

            uilabel(grid, Text= ...
                "DOE results are stored separately from the latest lap result.", ...
                FontColor=app.Muted, HorizontalAlignment="center");
        end

        function page = createPagePanel(app, tag)
            page = uipanel(app.PageStack, BorderType="none", ...
                BackgroundColor=app.Canvas, Tag=tag, Visible="off");
            page.Layout.Row = 1;
            page.Layout.Column = 1;
        end

        function createPageHeader(app, parent, titleText, subtitleText)
            header = uigridlayout(parent, [2 2]);
            header.RowHeight = {42, 24};
            header.ColumnWidth = {"1x", "fit"};
            header.Padding = [0 0 0 0];
            header.BackgroundColor = app.Canvas;
            titleLabel = uilabel(header, Text=titleText, ...
                FontName="Microsoft YaHei UI", FontSize=24, ...
                FontWeight="bold", FontColor=app.Ink);
            titleLabel.Layout.Row = 1;
            subtitle = uilabel(header, Text=subtitleText, ...
                FontName="Microsoft YaHei UI", FontSize=11, ...
                FontColor=app.Muted);
            subtitle.Layout.Row = 2;
            subtitle.Layout.Column = 1;
            badge = uilabel(header, Text="  TU TAIYUAN  ", ...
                FontName="Segoe UI", FontSize=10, FontWeight="bold", ...
                FontColor=app.Ink, BackgroundColor=[0.72 0.95 0.94], ...
                HorizontalAlignment="center");
            badge.Layout.Row = 1;
            badge.Layout.Column = 2;
        end

        function panel = createCard(app, parent, titleText)
            panel = uipanel(parent, Title=titleText, ...
                FontName="Microsoft YaHei UI", FontSize=12, ...
                FontWeight="bold", ForegroundColor=app.Ink, ...
                BackgroundColor=app.Surface, BorderType="line", ...
                BorderColor=app.Border, BorderWidth=1);
        end

        function drawTrackPreview(app, ax)
            trackFile = fullfile(app.ProjectRoot, "data", "track", ...
                "tianji_kart_QSS_track_closed.csv");
            if isfile(trackFile)
                track = readtable(trackFile, VariableNamingRule="preserve");
                plot(ax, track.x_m, track.y_m, Color=app.Accent, ...
                    LineWidth=2.6);
                hold(ax, "on");
                scatter(ax, track.x_m(1), track.y_m(1), 52, ...
                    app.Warning, "filled");
                hold(ax, "off");
                title(ax, "Tianji Kart Track · 857.46 m", ...
                    Color=app.Ink, FontWeight="bold");
                axis(ax, "equal");
            else
                text(ax, 0.5, 0.5, "赛道文件不可用", ...
                    Units="normalized", HorizontalAlignment="center", ...
                    Color=app.Muted);
            end
        end

        function styleAxes(app, ax)
            ax.FontName = "Segoe UI";
            ax.FontSize = 10;
            ax.Color = app.Surface;
            ax.XColor = app.Muted;
            ax.YColor = app.Muted;
            ax.GridColor = app.Border;
            ax.Box = "off";
            ax.Toolbar.Visible = "off";
            grid(ax, "on");
        end

        function addWorkflowStep(app, parent, row, index, titleText, ...
                detailText, color)
            item = uigridlayout(parent, [2 2]);
            item.Layout.Row = row;
            item.RowHeight = {22, 20};
            item.ColumnWidth = {34, "1x"};
            item.Padding = [0 0 0 0];
            item.BackgroundColor = app.Surface;
            badge = uilabel(item, Text=index, ...
                FontName="Segoe UI", FontSize=10, FontWeight="bold", ...
                FontColor=[1 1 1], BackgroundColor=color, ...
                HorizontalAlignment="center");
            badge.Layout.Row = [1 2];
            titleLabel = uilabel(item, Text=titleText, ...
                FontName="Microsoft YaHei UI", FontSize=11, ...
                FontWeight="bold", FontColor=app.Ink);
            titleLabel.Layout.Row = 1;
            titleLabel.Layout.Column = 2;
            detail = uilabel(item, Text=detailText, ...
                FontName="Microsoft YaHei UI", FontSize=9, ...
                FontColor=app.Muted);
            detail.Layout.Row = 2;
            detail.Layout.Column = 2;
        end

        function populateTrackInputs(app, panel, defaults)
            grid = app.inputGrid(panel, 5);
            app.addDropdown(grid, 1, "track_preset", "赛道", ...
                defaults.track_preset, ...
                "Tianji closed track", ...
                "tianji_kart_QSS_track_closed");
            app.addTextField(grid, 2, "track_source_file", "赛道文件", ...
                defaults.track_source_file);
            app.addNumericField(grid, 3, "endurance_num_laps", ...
                "耐久圈数", defaults.endurance_num_laps, "lap");
            app.addNumericField(grid, 4, "endurance_safety_factor", ...
                "安全系数", defaults.endurance_safety_factor, "—");
            app.addDisplayField(grid, 5, "闭环长度", "857.461 m");
        end

        function populateVehicleInputs(app, panel, defaults)
            grid = app.inputGrid(panel, 9);
            app.addNumericField(grid, 1, "vehicle_mass_total_kg", ...
                "整车质量", defaults.vehicle_mass_total_kg, "kg");
            app.addNumericField(grid, 2, "vehicle_wheelbase_m", ...
                "轴距", defaults.vehicle_wheelbase_m, "m");
            app.addNumericField(grid, 3, "vehicle_track_front_m", ...
                "前轮距", defaults.vehicle_track_front_m, "m");
            app.addNumericField(grid, 4, "vehicle_track_rear_m", ...
                "后轮距", defaults.vehicle_track_rear_m, "m");
            app.addNumericField(grid, 5, "vehicle_cg_height_m", ...
                "质心高度", defaults.vehicle_cg_height_m, "m");
            app.addNumericField(grid, 6, ...
                "vehicle_front_static_frac", ...
                "前轴静载比例", defaults.vehicle_front_static_frac, "—");
            app.addNumericField(grid, 7, "vehicle_inertia_Iz_kgm2", ...
                "横摆转动惯量", defaults.vehicle_inertia_Iz_kgm2, "kg·m²");
            app.addNumericField(grid, 8, ...
                "vehicle_front_lateral_load_transfer_frac", ...
                "前轴侧向载荷转移", ...
                defaults.vehicle_front_lateral_load_transfer_frac, "—");
            app.addDropdown(grid, 9, "vehicle_drivetrain_layout", ...
                "驱动布局", defaults.vehicle_drivetrain_layout, ...
                ["RWD", "FWD", "AWD"], ...
                ["RWD", "FWD", "AWD"]);
        end

        function populateTireInputs(app, panel, defaults)
            grid = app.inputGrid(panel, 7);
            app.addNumericField(grid, 1, "tire_Fz_ref_N", ...
                "参考垂向载荷", defaults.tire_Fz_ref_N, "N");
            app.addNumericField(grid, 2, "tire_mu_x_ref", ...
                "参考 μx", defaults.tire_mu_x_ref, "—");
            app.addNumericField(grid, 3, "tire_mu_y_ref", ...
                "参考 μy", defaults.tire_mu_y_ref, "—");
            app.addNumericField(grid, 4, "tire_load_sensitivity_x", ...
                "纵向载荷敏感性", defaults.tire_load_sensitivity_x, "—");
            app.addNumericField(grid, 5, "tire_load_sensitivity_y", ...
                "横向载荷敏感性", defaults.tire_load_sensitivity_y, "—");
            app.addNumericField(grid, 6, "tire_combined_n", ...
                "联合附着指数", defaults.tire_combined_n, "—");
            app.addNumericField(grid, 7, "tire_rolling_radius_m", ...
                "滚动半径", defaults.tire_rolling_radius_m, "m");
        end

        function populateAeroBrakeInputs(app, panel, defaults)
            grid = app.inputGrid(panel, 9);
            app.addNumericField(grid, 1, "aero_CLA_m2", ...
                "CLA", defaults.aero_CLA_m2, "m²");
            app.addNumericField(grid, 2, "aero_CDA_m2", ...
                "CDA", defaults.aero_CDA_m2, "m²");
            app.addNumericField(grid, 3, "aero_front_downforce_frac", ...
                "前轴下压力比例", defaults.aero_front_downforce_frac, "—");
            app.addCheckbox(grid, 4, "brake_enabled", ...
                "启用机械制动", defaults.brake_enabled);
            app.addCheckbox(grid, 5, "brake_force_limit_enabled", ...
                "启用总制动力限制", defaults.brake_force_limit_enabled);
            app.addNumericField(grid, 6, ...
                "brake_max_total_brake_force_N", "最大总制动力", ...
                defaults.brake_max_total_brake_force_N, "N");
            app.addNumericField(grid, 7, ...
                "brake_max_decel_g_mechanical", "机械减速度限制", ...
                defaults.brake_max_decel_g_mechanical, "g");
            app.addNumericField(grid, 8, ...
                "brake_max_total_brake_torque_Nm", "最大总制动扭矩", ...
                defaults.brake_max_total_brake_torque_Nm, "N·m");
            app.addNumericField(grid, 9, "brake_front_bias", ...
                "前制动力分配", defaults.brake_front_bias, "—");
        end

        function populateMotorInputs(app, panel, defaults)
            grid = app.inputGrid(panel, 17);
            app.addDropdown(grid, 1, "powertrain_preset", ...
                "基础动力预设", defaults.powertrain_preset, ...
                "EMRAX 228 HV CC", ...
                "powertrain_emrax228_hvcc_demo");
            app.addNumericField(grid, 2, "powertrain_gear_ratio", ...
                "总传动比", defaults.powertrain_gear_ratio, "—");
            app.addNumericField(grid, 3, ...
                "powertrain_drivetrain_efficiency", "传动效率", ...
                defaults.powertrain_drivetrain_efficiency, "—");
            app.addNumericField(grid, 4, "motor_max_mechanical_speed_rpm", ...
                "最高机械转速", defaults.motor_max_mechanical_speed_rpm, "rpm");
            app.addNumericField(grid, 5, "motor_physical_peak_power_W", ...
                "峰值功率", defaults.motor_physical_peak_power_W, "W");
            app.addNumericField(grid, 6, "motor_physical_peak_power_rpm", ...
                "峰值功率转速", defaults.motor_physical_peak_power_rpm, "rpm");
            app.addNumericField(grid, 7, "motor_physical_cont_power_W", ...
                "持续功率", defaults.motor_physical_cont_power_W, "W");
            app.addNumericField(grid, 8, "motor_peak_torque_Nm", ...
                "峰值扭矩", defaults.motor_peak_torque_Nm, "N·m");
            app.addNumericField(grid, 9, "motor_cont_torque_Nm", ...
                "持续扭矩", defaults.motor_cont_torque_Nm, "N·m");
            app.addNumericField(grid, 10, ...
                "motor_required_voltage_peak_power_V", "峰值功率所需电压", ...
                defaults.motor_required_voltage_peak_power_V, "V");
            app.addNumericField(grid, 11, ...
                "motor_peak_phase_current_Arms", "峰值相电流", ...
                defaults.motor_peak_phase_current_Arms, "Arms");
            app.addNumericField(grid, 12, ...
                "motor_cont_phase_current_Arms", "持续相电流", ...
                defaults.motor_cont_phase_current_Arms, "Arms");
            app.addNumericField(grid, 13, ...
                "motor_Kv_no_load_rpm_per_V", "空载 Kv", ...
                defaults.motor_Kv_no_load_rpm_per_V, "rpm/V");
            app.addNumericField(grid, 14, ...
                "motor_Kv_nominal_load_rpm_per_V", "额定负载 Kv", ...
                defaults.motor_Kv_nominal_load_rpm_per_V, "rpm/V");
            app.addNumericField(grid, 15, ...
                "motor_Kv_peak_load_rpm_per_V", "峰值负载 Kv", ...
                defaults.motor_Kv_peak_load_rpm_per_V, "rpm/V");
            app.addNumericField(grid, 16, "motor_Kt_Nm_per_Arms", ...
                "扭矩常数 Kt", defaults.motor_Kt_Nm_per_Arms, "N·m/A");
            app.addNumericField(grid, 17, "motor_eta_const", ...
                "电机效率", defaults.motor_eta_const, "—");
        end

        function populateBatteryInputs(app, panel, defaults)
            grid = app.inputGrid(panel, 11);
            app.addNumericField(grid, 1, "battery_V_max_V", ...
                "最高电压", defaults.battery_V_max_V, "V");
            app.addNumericField(grid, 2, "battery_V_nominal_V", ...
                "标称电压", defaults.battery_V_nominal_V, "V");
            app.addNumericField(grid, 3, "battery_V_min_V", ...
                "最低电压", defaults.battery_V_min_V, "V");
            app.addNumericField(grid, 4, "battery_V_bus_assumed_V", ...
                "仿真假定母线电压", defaults.battery_V_bus_assumed_V, "V");
            app.addNumericField(grid, 5, "battery_E_nominal_kWh", ...
                "标称能量", defaults.battery_E_nominal_kWh, "kWh");
            app.addNumericField(grid, 6, "battery_SOC_init", ...
                "初始 SOC", defaults.battery_SOC_init, "—");
            app.addNumericField(grid, 7, "battery_SOC_min", ...
                "最低 SOC", defaults.battery_SOC_min, "—");
            app.addNumericField(grid, 8, ...
                "battery_P_discharge_peak_W", "峰值放电功率", ...
                defaults.battery_P_discharge_peak_W, "W");
            app.addNumericField(grid, 9, ...
                "battery_I_discharge_peak_A", "峰值放电电流", ...
                defaults.battery_I_discharge_peak_A, "A");
            app.addNumericField(grid, 10, "battery_eta_discharge", ...
                "放电效率", defaults.battery_eta_discharge, "—");
            app.addNumericField(grid, 11, "battery_P_ts_aux_W", ...
                "高压附件功率", defaults.battery_P_ts_aux_W, "W");
        end

        function populateInverterInputs(app, panel, defaults)
            grid = app.inputGrid(panel, 5);
            app.addNumericField(grid, 1, "inverter_V_dc_max_V", ...
                "最高直流电压", defaults.inverter_V_dc_max_V, "V");
            app.addNumericField(grid, 2, "inverter_P_dc_peak_W", ...
                "峰值直流功率", defaults.inverter_P_dc_peak_W, "W");
            app.addNumericField(grid, 3, "inverter_I_dc_peak_A", ...
                "峰值直流电流", defaults.inverter_I_dc_peak_A, "A");
            app.addNumericField(grid, 4, ...
                "inverter_I_phase_peak_Arms", "峰值相电流", ...
                defaults.inverter_I_phase_peak_Arms, "Arms");
            app.addNumericField(grid, 5, "inverter_eta_const", ...
                "逆变器效率", defaults.inverter_eta_const, "—");
        end

        function populateSolverInputs(app, panel, defaults)
            grid = app.inputGrid(panel, 3);
            app.addNumericField(grid, 1, "options_v_max_mps", ...
                "最高车速", defaults.options_v_max_mps, "m/s");
            app.addNumericField(grid, 2, "options_v_grid_step_mps", ...
                "GGV Δv", defaults.options_v_grid_step_mps, "m/s");
            app.addNumericField(grid, 3, ...
                "options_solver_tolerance_mps", "求解容差", ...
                defaults.options_solver_tolerance_mps, "m/s");
        end

        function grid = inputGrid(app, panel, rowCount)
            grid = uigridlayout(panel, [rowCount 3]);
            grid.RowHeight = repmat({38}, 1, rowCount);
            grid.ColumnWidth = {175, "1x", 58};
            grid.Padding = [18 14 18 14];
            grid.RowSpacing = 6;
            grid.BackgroundColor = app.Surface;
        end

        function addNumericField(app, grid, row, key, ...
                labelText, value, unit)
            label = uilabel(grid, Text=labelText, ...
                FontName="Microsoft YaHei UI", FontColor=app.Muted);
            label.Layout.Row = row;
            field = uieditfield(grid, "numeric", Value=value, ...
                FontName="Segoe UI", FontColor=app.Ink, ...
                BackgroundColor=app.Field);
            field.Layout.Row = row;
            field.Layout.Column = 2;
            field.Tag = "param-" + key;
            app.ParameterControls.(key) = field;
            unitLabel = uilabel(grid, Text=unit, FontName="Segoe UI", ...
                FontColor=app.Muted, HorizontalAlignment="right");
            unitLabel.Layout.Row = row;
            unitLabel.Layout.Column = 3;
        end

        function addCheckbox(app, grid, row, key, labelText, value)
            label = uilabel(grid, Text=labelText, ...
                FontName="Microsoft YaHei UI", FontColor=app.Muted);
            label.Layout.Row = row;
            checkbox = uicheckbox(grid, Text="", Value=logical(value), ...
                FontName="Microsoft YaHei UI", FontColor=app.Ink);
            checkbox.Layout.Row = row;
            checkbox.Layout.Column = [2 3];
            checkbox.Tag = "param-" + key;
            app.ParameterControls.(key) = checkbox;
        end

        function addTextField(app, grid, row, key, labelText, value)
            label = uilabel(grid, Text=labelText, ...
                FontName="Microsoft YaHei UI", FontColor=app.Muted);
            label.Layout.Row = row;
            field = uieditfield(grid, "text", Value=value, ...
                FontName="Microsoft YaHei UI", ...
                FontColor=app.Muted, ...
                BackgroundColor=app.Field, Editable="off");
            field.Layout.Row = row;
            field.Layout.Column = [2 3];
            field.Tag = "param-" + key;
            app.ParameterControls.(key) = field;
        end

        function addDropdown(app, grid, row, key, labelText, value, ...
                items, itemsData)
            label = uilabel(grid, Text=labelText, ...
                FontName="Microsoft YaHei UI", FontColor=app.Muted);
            label.Layout.Row = row;
            dropdown = uidropdown(grid, Items=string(items), ...
                ItemsData=string(itemsData), ...
                Value=string(value), ...
                FontName="Microsoft YaHei UI", ...
                FontColor=app.Ink, ...
                BackgroundColor=app.Field);
            dropdown.Layout.Row = row;
            dropdown.Layout.Column = [2 3];
            dropdown.Tag = "param-" + key;
            app.ParameterControls.(key) = dropdown;
        end

        function addDisplayField(app, grid, row, labelText, value)
            label = uilabel(grid, Text=labelText, ...
                FontName="Microsoft YaHei UI", FontColor=app.Muted);
            label.Layout.Row = row;
            field = uieditfield(grid, "text", Value=value, ...
                FontName="Microsoft YaHei UI", ...
                FontColor=app.Muted, ...
                BackgroundColor=app.Field, Editable="off");
            field.Layout.Row = row;
            field.Layout.Column = [2 3];
        end

        function addSummaryRow(app, grid, row, labelText, valueText)
            label = uilabel(grid, Text=labelText, ...
                FontName="Microsoft YaHei UI", FontColor=app.Muted);
            label.Layout.Row = row;
            value = uilabel(grid, Text=valueText, ...
                FontName="Microsoft YaHei UI", FontWeight="bold", ...
                FontColor=app.Ink);
            value.Layout.Row = row;
            value.Layout.Column = 2;
        end

        function axesHandles = createResultTab(app, tabGroup, ...
                titleText, axesTitles, notes)
            axesTitles = string(axesTitles);
            notes = string(notes);
            axesCount = numel(axesTitles);
            if numel(notes) ~= axesCount
                error("QSSLTS:ResultTabDefinition", ...
                    "Each result axes requires one placeholder note.");
            end

            tab = uitab(tabGroup, Title=titleText, ...
                BackgroundColor=app.Surface);
            if axesCount == 1
                gridSize = [1 1];
            else
                gridSize = [2 2];
            end
            grid = uigridlayout(tab, gridSize);
            grid.Padding = [18 18 18 18];
            grid.RowSpacing = 14;
            grid.ColumnSpacing = 14;
            grid.BackgroundColor = app.Surface;
            axesHandles = cell(axesCount, 1);
            for index = 1:axesCount
                ax = uiaxes(grid);
                axesHandles{index} = ax;
                app.styleAxes(ax);
                title(ax, axesTitles(index));
                xlabel(ax, "Distance / operating point");
                ylabel(ax, "Result");
                xlim(ax, [0 1]);
                ylim(ax, [0 1]);
                text(ax, 0.5, 0.53, "NO RESULT DATA", ...
                    HorizontalAlignment="center", FontName="Segoe UI", ...
                    FontSize=18, FontWeight="bold", Color=app.Border);
                text(ax, 0.5, 0.45, notes(index), ...
                    HorizontalAlignment="center", ...
                    FontName="Microsoft YaHei UI", FontSize=11, ...
                    Color=app.Muted);
            end
        end

        function applyParameterState(app, state)
            state = validate_qsslts_gui_state(state);
            names = string(fieldnames(state));
            for name = names.'
                if isfield(app.ParameterControls, name)
                    app.ParameterControls.(name).Value = state.(name);
                end
            end
        end

        function resetParameters(app)
            app.applyParameterState(app.DefaultParameterState);
            app.setParameterStatus("已恢复 GUI 基准参数", app.Accent);
        end

        function saveParametersDialog(app)
            defaultFile = fullfile(app.ProjectRoot, ...
                "qsslts_parameters.json");
            [fileName, folder] = uiputfile( ...
                {"*.json", "QSSLTS parameter files (*.json)"}, ...
                "保存 QSSLTS 参数", defaultFile);
            if isequal(fileName, 0)
                return
            end
            try
                app.exportParameters(fullfile(folder, fileName));
            catch exception
                uialert(app.UIFigure, exception.message, ...
                    "参数保存失败", Icon="error");
            end
        end

        function loadParametersDialog(app)
            [fileName, folder] = uigetfile( ...
                {"*.json", "QSSLTS parameter files (*.json)"}, ...
                "载入 QSSLTS 参数", app.ProjectRoot);
            if isequal(fileName, 0)
                return
            end
            try
                app.importParameters(fullfile(folder, fileName));
            catch exception
                uialert(app.UIFigure, exception.message, ...
                    "参数载入失败", Icon="error");
            end
        end

        function setParameterStatus(app, text, color)
            if ~isempty(app.ParameterStatusLabel) ...
                    && isgraphics(app.ParameterStatusLabel)
                app.ParameterStatusLabel.Text = text;
                app.ParameterStatusLabel.FontColor = color;
            end
        end

        function toggleDoe(app)
            if app.IsDoeRunningFlag
                app.cancelDoe();
                return
            end
            try
                app.runDoe(app.DoeInputTable.Data, false);
            catch exception
                if isgraphics(app.UIFigure)
                    uialert(app.UIFigure, exception.message, ...
                        "DOE failed to start", Icon="error");
                end
            end
        end

        function beginDoe(app)
            app.IsDoeRunningFlag = true;
            app.DoeRunButton.Text = "Cancel DOE";
            app.DoeRunButton.BackgroundColor = app.Warning;
            app.DoeStatusLabel.Text = "DOE running";
            app.DoeStatusLabel.FontColor = app.Accent;
            drawnow limitrate
        end

        function pollDoe(app)
            if ~app.IsDoeRunningFlag || isempty(app.DoeFuture)
                return
            end
            if string(app.DoeFuture.State) ~= "finished"
                return
            end

            future = app.DoeFuture;
            app.DoeFuture = [];
            app.stopDoeTimer();
            try
                result = fetchOutputs(future);
                app.finishDoe(result);
            catch exception
                app.failDoe(exception, true);
            end
        end

        function finishDoe(app, result)
            app.LastDoeResult = result;
            app.IsDoeRunningFlag = false;
            app.DoeFuture = [];
            app.stopDoeTimer();
            app.renderDoeResult(result);
            app.restoreDoeControls(sprintf( ...
                "DOE complete: %d cases", height(result.ranking)), ...
                app.Accent);
            app.selectPage("doe");
        end

        function failDoe(app, exception, showAlert)
            app.IsDoeRunningFlag = false;
            app.DoeFuture = [];
            app.stopDoeTimer();
            app.restoreDoeControls("DOE failed", app.Warning);
            if showAlert && isgraphics(app.UIFigure)
                uialert(app.UIFigure, exception.message, ...
                    "DOE failed", Icon="error");
            end
        end

        function restoreDoeControls(app, statusText, color)
            if ~isempty(app.DoeRunButton) && isgraphics(app.DoeRunButton)
                app.DoeRunButton.Text = "Run DOE";
                app.DoeRunButton.BackgroundColor = app.Accent;
            end
            if ~isempty(app.DoeStatusLabel) ...
                    && isgraphics(app.DoeStatusLabel)
                app.DoeStatusLabel.Text = statusText;
                app.DoeStatusLabel.FontColor = color;
            end
        end

        function stopDoeTimer(app)
            if isempty(app.DoeTimer)
                return
            end
            try
                if isvalid(app.DoeTimer)
                    stop(app.DoeTimer);
                    delete(app.DoeTimer);
                end
            catch
            end
            app.DoeTimer = [];
        end

        function renderDoeResult(app, result)
            app.DoeRankingTable.Data = result.ranking;
            ax = app.DoeResponseAxes;
            cla(ax);
            plot(ax, 1:height(result.ranking), ...
                result.ranking.lap_time_s, "-o", ...
                Color=app.Accent, MarkerFaceColor=app.Accent, ...
                LineWidth=1.4, Tag="doe-lap-time-response");
            xlabel(ax, "Rank");
            ylabel(ax, "Lap time (s)");
            title(ax, "DOE ranked lap-time response");
            grid(ax, "on");
        end

        function toggleSimulation(app)
            if app.IsRunning
                app.cancelSimulation();
                return
            end
            try
                app.runSimulation(false);
            catch exception
                if isgraphics(app.UIFigure)
                    uialert(app.UIFigure, exception.message, ...
                        "仿真启动失败", Icon="error");
                end
            end
        end

        function beginSimulation(app)
            app.IsRunning = true;
            app.RunStartTime = tic;
            app.RunGauge.Value = 20;
            app.RunButton.Text = "取消仿真";
            app.RunButton.BackgroundColor = app.Warning;
            app.RunStatusLabel.Text = "QSS 后台求解中";
            app.RunStatusLabel.FontColor = app.Accent;
            app.RunProgressLabel.Text = "正在构建 GGV 与速度剖面";
            app.RunDetailLabel.Text = ...
                "已用时 0.0 s · 求解器未提供线性进度回调";
            app.StatusLabel.Text = "●  QSS 求解正在后台运行";
            app.StatusLabel.FontColor = app.Accent;
            drawnow limitrate
        end

        function pollSimulation(app)
            if ~app.IsRunning || isempty(app.RunFuture)
                return
            end
            elapsed_s = toc(app.RunStartTime);
            if isgraphics(app.RunDetailLabel)
                app.RunDetailLabel.Text = sprintf( ...
                    "已用时 %.1f s · 求解器未提供线性进度回调", ...
                    elapsed_s);
            end
            if string(app.RunFuture.State) ~= "finished"
                return
            end

            future = app.RunFuture;
            app.RunFuture = [];
            app.stopRunTimer();
            try
                result = fetchOutputs(future);
                app.finishSimulation(result);
            catch exception
                app.failSimulation(exception, true);
            end
        end

        function finishSimulation(app, result)
            summary = summarize_lap_result(result);
            app.LastResult = result;
            app.LastSummary = summary;
            app.IsRunning = false;
            app.RunFuture = [];
            app.stopRunTimer();
            app.renderResult(result);
            app.updateKpiCards(result, summary);

            if result.solver.converged
                solverText = sprintf("已收敛 · %d iter", ...
                    result.solver.iterations);
                solverColor = app.Accent;
            else
                solverText = sprintf("未收敛 · %d iter", ...
                    result.solver.iterations);
                solverColor = app.Warning;
            end
            app.restoreRunControls(solverText, 100, solverColor);
            app.StatusLabel.Text = sprintf( ...
                "●  仿真完成：单圈 %.3f s · 最高车速 %.2f m/s", ...
                summary.lap_time_s, summary.max_speed_mps);
            app.StatusLabel.FontColor = solverColor;
            app.selectPage("results");
        end

        function failSimulation(app, exception, showAlert)
            app.IsRunning = false;
            app.RunFuture = [];
            app.stopRunTimer();
            app.restoreRunControls("仿真失败", 0, app.Warning);
            app.RunDetailLabel.Text = string(exception.message);
            app.StatusLabel.Text = "●  仿真失败：请检查参数与求解器输入";
            app.StatusLabel.FontColor = app.Warning;
            if showAlert && isgraphics(app.UIFigure)
                uialert(app.UIFigure, exception.message, ...
                    "QSS 求解失败", Icon="error");
            end
        end

        function restoreRunControls(app, statusText, gaugeValue, color)
            if ~isempty(app.RunButton) && isgraphics(app.RunButton)
                if isempty(fieldnames(app.LastResult))
                    app.RunButton.Text = "开始仿真";
                else
                    app.RunButton.Text = "重新运行仿真";
                end
                app.RunButton.BackgroundColor = app.NavyRaised;
            end
            if ~isempty(app.RunGauge) && isgraphics(app.RunGauge)
                app.RunGauge.Value = gaugeValue;
            end
            if ~isempty(app.RunStatusLabel) && isgraphics(app.RunStatusLabel)
                app.RunStatusLabel.Text = statusText;
                app.RunStatusLabel.FontColor = color;
            end
            if ~isempty(app.RunProgressLabel) ...
                    && isgraphics(app.RunProgressLabel)
                if gaugeValue == 100
                    app.RunProgressLabel.Text = "求解与结果回传完成";
                elseif gaugeValue == 0
                    app.RunProgressLabel.Text = "当前未运行";
                end
            end
        end

        function stopRunTimer(app)
            if isempty(app.RunTimer)
                return
            end
            try
                if isvalid(app.RunTimer)
                    stop(app.RunTimer);
                    delete(app.RunTimer);
                end
            catch
            end
            app.RunTimer = [];
        end

        function updateKpiCards(app, result, summary)
            if isfield(result, "energy") ...
                    && isfield(result.energy, "E_lap_stored_kWh")
                lapEnergy_kWh = result.energy.E_lap_stored_kWh;
            elseif isfield(result, "energy") ...
                    && isfield(result.energy, "E_lap_ts_kWh")
                lapEnergy_kWh = result.energy.E_lap_ts_kWh;
            else
                lapEnergy_kWh = NaN;
            end
            if isfinite(lapEnergy_kWh)
                energyText = sprintf("%.3f", lapEnergy_kWh);
            else
                energyText = "—";
            end
            if result.solver.converged
                convergenceText = sprintf("已收敛 · %d iter", ...
                    result.solver.iterations);
            else
                convergenceText = sprintf("未收敛 · %d iter", ...
                    result.solver.iterations);
            end
            app.KpiCards.Data = struct( ...
                lapTime=sprintf("%.3f", summary.lap_time_s), ...
                maxSpeed=sprintf("%.2f", summary.max_speed_mps), ...
                lapEnergy=energyText, convergence=convergenceText);
        end

        function renderResult(app, result)
            app.renderTrackResult(result);
            app.renderDynamicsResult(result);
            app.renderEnergyResult(result);
            app.renderGgvResult(result);
        end

        function renderTrackResult(app, result)
            ax = app.ResultAxes.track{1};
            cla(ax);
            x_m = result.track.x_m(:);
            y_m = result.track.y_m(:);
            speed_mps = result.v_mps(:);
            if isfield(result.track, "is_closed") ...
                    && logical(result.track.is_closed)
                x_m(end + 1) = x_m(1);
                y_m(end + 1) = y_m(1);
                speed_mps(end + 1) = speed_mps(1);
            end
            surface(ax, [x_m x_m], [y_m y_m], ...
                zeros(numel(x_m), 2), [speed_mps speed_mps], ...
                FaceColor="none", EdgeColor="interp", LineWidth=2.8, ...
                Tag="result-track-speed-line");
            axis(ax, "equal");
            grid(ax, "on");
            xlabel(ax, "x (m)");
            ylabel(ax, "y (m)");
            title(ax, "赛道速度云图");
            colorScale = colorbar(ax);
            colorScale.Label.String = "Speed (m/s)";
        end

        function renderDynamicsResult(app, result)
            ax = app.ResultAxes.dynamics{1};
            cla(ax);
            yyaxis(ax, "left");
            plot(ax, result.s_m, result.v_mps, ...
                Color=app.Accent, LineWidth=1.5, ...
                DisplayName="Speed");
            ylabel(ax, "Speed (m/s)");
            yyaxis(ax, "right");
            plot(ax, result.s_m, result.ax_mps2, ...
                Color=app.Warning, LineWidth=1.1, DisplayName="Ax");
            hold(ax, "on");
            plot(ax, result.s_m, result.ay_mps2, ...
                Color=[0.89 0.17 0.26], LineWidth=1.1, ...
                DisplayName="Ay");
            hold(ax, "off");
            ylabel(ax, "Acceleration (m/s^2)");
            xlabel(ax, "Distance (m)");
            title(ax, "速度与加速度剖面");
            grid(ax, "on");
            legend(ax, Location="best");

            ax = app.ResultAxes.dynamics{2};
            cla(ax);
            usage = summarize_limiter_usage(result);
            bars = bar(ax, usage.percent_distance, ...
                FaceColor=app.Accent, Tag="result-limiter-share");
            bars.EdgeColor = app.NavyRaised;
            xticks(ax, 1:height(usage));
            xticklabels(ax, usage.limiter);
            xtickangle(ax, 25);
            ylabel(ax, "Distance share (%)");
            title(ax, "主导约束占比");
            grid(ax, "on");
        end

        function renderEnergyResult(app, result)
            s_m = result.s_m(:);
            powertrain = result.powertrain;

            ax = app.ResultAxes.energy{1};
            cla(ax);
            plot(ax, s_m, powertrain.motor_speed_rpm(:), ...
                Color=app.Accent, LineWidth=1.25);
            xlabel(ax, "Distance (m)");
            ylabel(ax, "Speed (rpm)");
            title(ax, "电机转速");
            grid(ax, "on");

            ax = app.ResultAxes.energy{2};
            cla(ax);
            plot(ax, s_m, powertrain.motor_torque_used_Nm(:), ...
                Color=app.Warning, LineWidth=1.25, ...
                DisplayName="Used");
            hold(ax, "on");
            plot(ax, s_m, powertrain.motor_torque_available_Nm(:), ...
                "--", Color=app.Accent, LineWidth=1.15, ...
                DisplayName="Available");
            hold(ax, "off");
            xlabel(ax, "Distance (m)");
            ylabel(ax, "Torque (N·m)");
            title(ax, "电机扭矩");
            grid(ax, "on");
            legend(ax, Location="best");

            ax = app.ResultAxes.energy{3};
            cla(ax);
            yyaxis(ax, "left");
            plot(ax, s_m, powertrain.tsac_power_used_W(:) / 1000, ...
                Color=app.Accent, LineWidth=1.25, ...
                DisplayName="TSAC power");
            ylabel(ax, "Power (kW)");
            yyaxis(ax, "right");
            plot(ax, s_m, powertrain.tsac_dc_current_used_A(:), ...
                Color=[0.89 0.17 0.26], LineWidth=1.1, ...
                DisplayName="DC current");
            ylabel(ax, "Current (A)");
            xlabel(ax, "Distance (m)");
            title(ax, "TSAC 功率与电流");
            grid(ax, "on");

            ax = app.ResultAxes.energy{4};
            cla(ax);
            segmentDistance_m = [0; cumsum(result.track.ds_m(:))];
            plot(ax, segmentDistance_m, ...
                result.energy.cumulative_energy_ts_kWh(:), ...
                Color=app.Accent, LineWidth=1.5, ...
                Tag="result-energy-line");
            xlabel(ax, "Distance (m)");
            ylabel(ax, "Energy (kWh)");
            title(ax, "累计单圈能量");
            grid(ax, "on");
        end

        function renderHandlingResult(app, ymd, understeer)
            ax = app.ResultAxes.ymd{1};
            cla(ax);
            surfaceHandle = surf(ax, rad2deg(ymd.steer_rad), ...
                rad2deg(ymd.beta_rad), ymd.yaw_moment_cg_Nm, ...
                ymd.ay_mps2, EdgeColor="none", ...
                Tag="result-ymd-surface");
            surfaceHandle.FaceAlpha = 0.92;
            xlabel(ax, "Road-wheel steer (deg)");
            ylabel(ax, "Body sideslip (deg)");
            zlabel(ax, "Yaw moment at CG (N m)");
            title(ax, "Yaw Moment Diagram (color: Ay)");
            view(ax, 3);
            grid(ax, "on");
            colorbar(ax);

            ax = app.ResultAxes.understeer{1};
            cla(ax);
            plot(ax, understeer.ay_g, ...
                rad2deg(understeer.roadwheel_steer_rad), "-o", ...
                Color=app.Accent, MarkerFaceColor=app.Accent, ...
                LineWidth=1.4, Tag="result-understeer-line");
            xlabel(ax, "Lateral acceleration (g)");
            ylabel(ax, "Road-wheel steer (deg)");
            title(ax, sprintf("Understeer gradient: %.3f deg/g", ...
                understeer.linear_fit_gradient_deg_per_g));
            grid(ax, "on");
        end

        function renderGgvResult(app, result)
            ax = app.ResultAxes.ggv{1};
            cla(ax);
            ggv = result.ggv_used;
            axMax_g = ggv.ax_max_g;
            axMin_g = ggv.ax_min_g;
            axMax_g(~ggv.feasible) = NaN;
            axMin_g(~ggv.feasible) = NaN;
            [ayGrid_g, speedGrid_mps] = meshgrid( ...
                ggv.ay_g, ggv.v_mps);
            hold(ax, "on");
            surf(ax, ayGrid_g, speedGrid_mps, axMax_g, speedGrid_mps, ...
                EdgeColor="none", FaceAlpha=0.78, ...
                Tag="result-ggv-upper-surface", ...
                DisplayName="Acceleration boundary");
            surf(ax, ayGrid_g, speedGrid_mps, axMin_g, speedGrid_mps, ...
                EdgeColor="none", FaceAlpha=0.78, ...
                Tag="result-ggv-lower-surface", ...
                DisplayName="Braking boundary");
            scatter3(ax, result.ay_mps2(:) / ggv.gravity_mps2, ...
                result.v_mps(:), ...
                result.ax_mps2(:) / ggv.gravity_mps2, 13, ...
                result.v_mps(:), "filled", ...
                Tag="result-actual-ggv-points", ...
                DisplayName="Actual lap");
            hold(ax, "off");
            xlabel(ax, "Ay (g)");
            ylabel(ax, "Speed (m/s)");
            zlabel(ax, "Ax (g)");
            title(ax, "GGV 能力边界与实际工况");
            grid(ax, "on");
            view(ax, 3);
            legend(ax, Location="best");
            colorbar(ax);
        end
    end

    methods (Static, Access = private)
        function value = onOff(condition)
            if condition
                value = "on";
            else
                value = "off";
            end
        end
    end
end
