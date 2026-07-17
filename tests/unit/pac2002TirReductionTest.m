classdef pac2002TirReductionTest < matlab.unittest.TestCase
    properties (SetAccess = private)
        ProjectRoot
    end

    properties (TestParameter)
        RepresentativeLoad_N = struct( ...
            "load300", 300, ...
            "load667", 667, ...
            "load1000", 1000, ...
            "load1500", 1500)
    end

    methods (TestClassSetup)
        function addProjectPaths(testCase)
            testCase.ProjectRoot = fileparts(fileparts( ...
                fileparts(mfilename("fullpath"))));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(testCase.ProjectRoot, "preprocessing"), ...
                IncludingSubfolders=true));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(testCase.ProjectRoot, "data"), ...
                IncludingSubfolders=true));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(testCase.ProjectRoot, "src"), ...
                IncludingSubfolders=true));
        end
    end

    methods (Test)
        function test_parser_ignores_commented_coefficients(testCase)
            tirPath = testCase.writeTemporaryTir( ...
                testCase.validTirText("meter", "PAC2002", true));

            parsed = read_pac2002_tir(tirPath);

            testCase.verifyEqual( ...
                parsed.sections.LATERAL_COEFFICIENTS.PDY1, 2.506, ...
                AbsTol=1e-12);
            testCase.verifyEqual( ...
                parsed.sections.LATERAL_COEFFICIENTS.PDY2, -0.17373, ...
                AbsTol=1e-12);
            testCase.verifyTrue(isfield(parsed.sections, "MODEL"));
            testCase.verifyMatches(parsed.source.sha256, "^[0-9A-F]{64}$");
        end

        function test_tir_units_reject_non_si_length(testCase)
            tirPath = testCase.writeTemporaryTir( ...
                testCase.validTirText("millimeter", "PAC2002", true));

            testCase.verifyError(@() read_pac2002_tir(tirPath), ...
                "read_pac2002_tir:InvalidUnits");
        end

        function test_tir_format_rejects_non_pac2002(testCase)
            tirPath = testCase.writeTemporaryTir( ...
                testCase.validTirText("meter", "PAC2012", true));

            testCase.verifyError(@() read_pac2002_tir(tirPath), ...
                "read_pac2002_tir:InvalidPropertyFileFormat");
        end

        function test_missing_reduction_field_has_clear_error(testCase)
            tirPath = testCase.writeTemporaryTir( ...
                testCase.validTirText("meter", "PAC2002", false));

            testCase.verifyError(@() read_pac2002_tir(tirPath), ...
                "read_pac2002_tir:MissingField");
        end

        function test_fixed_qss_mapping_and_assumption(testCase)
            tirPath = testCase.writeTemporaryTir( ...
                testCase.validTirText("meter", "PAC2002", true));
            parsed = read_pac2002_tir(tirPath);

            reduced = derive_qss_tire_parameters(parsed);

            testCase.verifyEqual(reduced.Fz_ref_N, 667, AbsTol=1e-12);
            testCase.verifyEqual(reduced.mu_x_ref, 2.3415 * 0.57, ...
                AbsTol=1e-12);
            testCase.verifyEqual(reduced.mu_y_ref, 2.506 * 0.72, ...
                AbsTol=1e-12);
            testCase.verifyEqual(reduced.load_sensitivity_x, ...
                -0.03943 / 2.3415, AbsTol=1e-12);
            testCase.verifyEqual(reduced.load_sensitivity_y, ...
                -0.17373 / 2.506, AbsTol=1e-12);
            testCase.verifyEqual(reduced.combined_n, 2, AbsTol=1e-12);
            testCase.verifyEqual(reduced.provenance.source_tir_sha256, ...
                parsed.source.sha256);
            testCase.verifyEqual(reduced.provenance.combined_n_basis, ...
                "QSS p-norm model-reduction assumption; " + ...
                "not directly extracted from TIR");
            testCase.verifyFalse(isfield(reduced, "rolling_radius_m"));
        end

        function test_reduced_peak_mu_matches_pac2002( ...
                testCase, RepresentativeLoad_N)
            tirPath = testCase.writeTemporaryTir( ...
                testCase.validTirText("meter", "PAC2002", true));
            reduced = derive_qss_tire_parameters( ...
                read_pac2002_tir(tirPath));
            normalizedLoad = (RepresentativeLoad_N - 667) / 667;
            expectedMuX = (2.3415 - 0.03943 * normalizedLoad) * 0.57;
            expectedMuY = (2.506 - 0.17373 * normalizedLoad) * 0.72;

            [actualMuX, actualMuY] = tire_load_sensitive_mu( ...
                RepresentativeLoad_N, reduced);

            testCase.verifyEqual(actualMuX, expectedMuX, AbsTol=1e-12);
            testCase.verifyEqual(actualMuY, expectedMuY, AbsTol=1e-12);
        end

        function test_factory_requires_measured_rolling_radius(testCase)
            rollingRadius = testCase.validRollingRadius();

            tire = tire_hoosier_16x75_10_r20_qss(rollingRadius);

            testCase.verifyEqual(tire.model_type, "load_sensitive");
            testCase.verifyEqual(tire.Fz_ref_N, 667, AbsTol=1e-12);
            testCase.verifyEqual(tire.mu_x_ref, 1.334655, AbsTol=1e-12);
            testCase.verifyEqual(tire.mu_y_ref, 1.804320, AbsTol=1e-12);
            testCase.verifyEqual(tire.load_sensitivity_x, ...
                -0.0168396327140722, AbsTol=1e-12);
            testCase.verifyEqual(tire.load_sensitivity_y, ...
                -0.0693256185155627, AbsTol=1e-12);
            testCase.verifyEqual(tire.combined_n, 2, AbsTol=1e-12);
            testCase.verifyEqual(tire.rolling_radius_m, ...
                rollingRadius.value_m, AbsTol=1e-12);
            testCase.verifyEqual(tire.provenance.rolling_radius, ...
                rollingRadius);
            testCase.verifyGreaterThan( ...
                abs(tire.rolling_radius_m - 0.20066), 1e-12);
            testCase.verifyEqual(tire.provenance.source_tir_sha256, ...
                "6AB8AA1219B7910A1660966AAD1B6C8FC7EF2FDB46F40BF0FA9B6FC172E53A2A");
        end

        function test_factory_rejects_missing_rolling_radius_field(testCase)
            rollingRadius = rmfield(testCase.validRollingRadius(), "load_N");

            testCase.verifyError( ...
                @() tire_hoosier_16x75_10_r20_qss(rollingRadius), ...
                "tire_hoosier_16x75_10_r20_qss:MissingRollingRadiusField");
        end

        function test_factory_rejects_nan_rolling_radius(testCase)
            rollingRadius = testCase.validRollingRadius();
            rollingRadius.value_m = NaN;

            testCase.verifyError( ...
                @() tire_hoosier_16x75_10_r20_qss(rollingRadius), ...
                "tire_hoosier_16x75_10_r20_qss:InvalidRollingRadius");
        end

        function test_factory_rejects_unloaded_radius_source(testCase)
            rollingRadius = testCase.validRollingRadius();
            rollingRadius.value_m = 0.20066;
            rollingRadius.source = "TIR UNLOADED_RADIUS";

            testCase.verifyError( ...
                @() tire_hoosier_16x75_10_r20_qss(rollingRadius), ...
                "tire_hoosier_16x75_10_r20_qss:UnvalidatedRollingRadius");
        end

        function test_no_runtime_reference_dependency(testCase)
            sourceText = fileread(which( ...
                "tire_hoosier_16x75_10_r20_qss"));

            testCase.verifyFalse(contains(lower(sourceText), "reference"));
            testCase.verifyFalse(contains(lower(sourceText), "fileread"));
            testCase.verifyFalse(contains(lower(sourceText), ".tir"));
        end

        function test_real_hoosier_tir_when_available(testCase)
            tirPath = fullfile(testCase.ProjectRoot, "reference", ...
                "FSAE-VD-Personal-Scripts-main", ...
                "FSAE-VD-Personal-Scripts-clean", "Yaw Dynamics", ...
                "Tire Model", "Hoosier_16x75_10_R20.tir");
            testCase.assumeTrue(isfile(tirPath), ...
                "Local-only Hoosier TIR is unavailable.");

            parsed = read_pac2002_tir(tirPath);
            reduced = derive_qss_tire_parameters(parsed);

            testCase.verifyEqual(parsed.source.sha256, ...
                "6AB8AA1219B7910A1660966AAD1B6C8FC7EF2FDB46F40BF0FA9B6FC172E53A2A");
            testCase.verifyEqual(reduced.Fz_ref_N, 667, AbsTol=1e-12);
            testCase.verifyEqual(reduced.mu_x_ref, 1.334655, AbsTol=1e-12);
            testCase.verifyEqual(reduced.mu_y_ref, 1.804320, AbsTol=1e-12);
            testCase.verifyEqual(reduced.load_sensitivity_x, ...
                -0.0168396327140722, AbsTol=1e-12);
            testCase.verifyEqual(reduced.load_sensitivity_y, ...
                -0.0693256185155627, AbsTol=1e-12);
        end
    end

    methods (Access = private)
        function tirPath = writeTemporaryTir(testCase, text)
            tempFolder = string(tempname);
            mkdir(tempFolder);
            testCase.addTeardown(@() rmdir(tempFolder, "s"));
            tirPath = fullfile(tempFolder, "synthetic.tir");
            fileId = fopen(tirPath, "w");
            cleanup = onCleanup(@() fclose(fileId));
            fwrite(fileId, text, "char");
        end
    end

    methods (Static, Access = private)
        function text = validTirText(lengthUnit, format, includePdy2)
            lines = [
                "[UNITS]"
                "LENGTH = '" + lengthUnit + "'"
                "FORCE = 'newton'"
                "ANGLE = 'radians'"
                "MASS = 'kg'"
                "TIME = 'second'"
                "[MODEL]"
                "PROPERTY_FILE_FORMAT = '" + format + "'"
                "FITTYP = 6"
                "[DIMENSION]"
                "UNLOADED_RADIUS = 0.20066"
                "[VERTICAL]"
                "FNOMIN = 667"
                "[SCALING_COEFFICIENTS]"
                "LFZO = 1"
                "LMUX = 0.57"
                "LMUY = 0.72"
                "[LONGITUDINAL_COEFFICIENTS]"
                "PDX1 = 2.3415"
                "PDX2 = -0.03943"
                "[LATERAL_COEFFICIENTS]"
                "PDY1 = 2.506 $ active coefficient"
                "$ PDY1 = 99"
                "$PDY2 = 99"
            ];
            if includePdy2
                lines(end + 1) = "PDY2 = -0.17373";
            end
            text = join(lines, newline) + newline;
        end

        function rollingRadius = validRollingRadius()
            rollingRadius.value_m = 0.195;
            rollingRadius.load_N = 667;
            rollingRadius.pressure_kPa = 82.7;
            rollingRadius.speed_mps = 10;
            rollingRadius.source = "loaded rolling-circumference measurement";
        end
    end
end
