classdef mfevalQssEnvelopeTest < matlab.unittest.TestCase
    %mfevalQssEnvelopeTest Offline MFeval-to-QSS reduction contract.

    properties (SetAccess = private)
        ProjectRoot
        RealTirPath
        EnvelopeFile
    end

    methods (TestClassSetup)
        function exportRepresentativeEnvelope(testCase)
            testCase.ProjectRoot = fileparts(fileparts(fileparts( ...
                mfilename("fullpath"))));
            testCase.RealTirPath = fullfile(testCase.ProjectRoot, ...
                "data", "tire", "local", ...
                "Hoosier_16x75_10_R20.tir");
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(testCase.ProjectRoot, "preprocessing"), ...
                IncludingSubfolders=true));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(testCase.ProjectRoot, "src"), ...
                IncludingSubfolders=true));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(testCase.ProjectRoot, "data"), ...
                IncludingSubfolders=true));
            testCase.assumeTrue(isfile(testCase.RealTirPath));
            temporaryFolder = testCase.applyFixture( ...
                matlab.unittest.fixtures.TemporaryFolderFixture);
            copiedTir = fullfile(temporaryFolder.Folder, ...
                "Hoosier_16x75_10_R20.tir");
            copyfile(testCase.RealTirPath, copiedTir);

            manifest = export_mfeval_qss_envelope( ...
                temporaryFolder.Folder);
            testCase.EnvelopeFile = manifest.envelope_file;
        end
    end

    methods (Test)
        function testExportIsHashBoundAndTraceable(testCase)
            saved = load(testCase.EnvelopeFile, "qss_envelope");
            artifact = saved.qss_envelope;

            testCase.verifyTrue(isfile(testCase.EnvelopeFile));
            testCase.verifyEqual(artifact.schema_version, 1);
            testCase.verifyEqual(artifact.source.sha256, ...
                "6CE561640F15CE2DC5CCBA3CC2622978933E4CBE0C6FB1F15EC87520DC42F023");
            testCase.verifyEqual(artifact.mfeval.version, "4.3.1");
            testCase.verifyEqual(artifact.tire.model_type, ...
                "load_sensitive");
            testCase.verifyGreaterThan(artifact.tire.combined_n, 1.5);
            testCase.verifyLessThan(artifact.tire.combined_n, 2.2);
            testCase.verifyFalse(testCase.containsFunctionHandle(artifact.tire));
        end

        function testPureSlipReductionNeverExceedsSampledPeaks(testCase)
            saved = load(testCase.EnvelopeFile, "qss_envelope");
            artifact = saved.qss_envelope;
            [muX, muY] = tire_load_sensitive_mu( ...
                artifact.pure_boundary.Fz_N, artifact.tire);

            testCase.verifyLessThanOrEqual( ...
                muX .* artifact.pure_boundary.Fz_N, ...
                artifact.pure_boundary.Fx_peak_N + 1e-8);
            testCase.verifyLessThanOrEqual( ...
                muY .* artifact.pure_boundary.Fz_N, ...
                artifact.pure_boundary.Fy_peak_N + 1e-8);
        end

        function testCombinedSlipFitNeverExceedsSampledBoundary(testCase)
            saved = load(testCase.EnvelopeFile, "qss_envelope");
            artifact = saved.qss_envelope;
            exponent = artifact.tire.combined_n;
            predictedX = max(0, ...
                1 - artifact.combined_boundary.y_fraction .^ exponent) ...
                .^ (1 / exponent);

            testCase.verifyLessThanOrEqual(predictedX, ...
                artifact.combined_boundary.x_boundary_fraction + 1e-12);
            testCase.verifyLessThanOrEqual( ...
                artifact.fit.combined.max_boundary_excess, 0);
        end

        function testRuntimeLoaderReturnsNumericQssTireOnly(testCase)
            tire = load_qss_tire_envelope(testCase.EnvelopeFile);

            testCase.verifyEqual(tire.model_type, "load_sensitive");
            testCase.verifyEqual(tire.provenance.source_tir_sha256, ...
                "6CE561640F15CE2DC5CCBA3CC2622978933E4CBE0C6FB1F15EC87520DC42F023");
            testCase.verifyFalse(isfield(tire, "evaluate"));
            testCase.verifyFalse(isfield(tire, "mfeval"));
            testCase.verifyFalse(testCase.containsFunctionHandle(tire));
        end

        function testExtractedEnvelopeRunsLightweightGgv(testCase)
            tire = load_qss_tire_envelope(testCase.EnvelopeFile);
            tire.rolling_radius_m = 0.195;
            options = default_qss_options(struct( ...
                v_grid_mps=[5; 10], ay_grid_g=(-2:0.5:2).'));

            ggv = generate_model_ggv(vehicle_baseline(), tire, ...
                struct("enabled", false), struct("enabled", false), ...
                struct("enabled", false), options);

            testCase.verifySize(ggv.ax_max_g, [2 9]);
            testCase.verifyTrue(any(ggv.feasible, "all"));
            testCase.verifyFalse(testCase.containsFunctionHandle( ...
                ggv.provenance.tire));
        end
    end

    methods (Static, Access = private)
        function result = containsFunctionHandle(value)
            if isa(value, "function_handle")
                result = true;
                return
            end
            if isstruct(value)
                names = fieldnames(value);
                result = any(cellfun(@(name) ...
                    mfevalQssEnvelopeTest.containsFunctionHandle( ...
                    value.(name)), names));
                return
            end
            if iscell(value)
                result = any(cellfun(@(item) ...
                    mfevalQssEnvelopeTest.containsFunctionHandle(item), ...
                    value));
                return
            end
            result = false;
        end
    end
end
