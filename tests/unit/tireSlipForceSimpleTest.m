classdef tireSlipForceSimpleTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addProjectPaths(testCase)
            projectRoot = fileparts(fileparts(fileparts(mfilename("fullpath"))));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(projectRoot, "src"), IncludingSubfolders=true));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(projectRoot, "data"), IncludingSubfolders=true));
        end
    end

    methods (Test)
        function testZeroSlipProducesZeroForce(testCase)
            tire = tire_load_sensitive_baseline();

            force = tire_force_from_slip(0, 0, 750, 0, tire);

            testCase.verifyEqual(force.Fx_N, 0, AbsTol=0);
            testCase.verifyEqual(force.Fy_N, 0, AbsTol=0);
        end

        function testForceSignsOpposeRelativeSlip(testCase)
            tire = tire_load_sensitive_baseline();

            force = tire_force_from_slip(0.02, 0.04, 750, 0, tire);

            testCase.verifyGreaterThan(force.Fx_N, 0);
            testCase.verifyLessThan(force.Fy_N, 0);
        end

        function testCombinedForceStaysInsideEnvelope(testCase)
            tire = tire_load_sensitive_baseline();
            env = tire_envelope(750, 0, tire);

            force = tire_force_from_slip(1, 1, 750, 0, tire);
            usage = (abs(force.Fx_N) / env.Fx_max_N)^tire.combined_n ...
                + (abs(force.Fy_N) / env.Fy_max_N)^tire.combined_n;

            testCase.verifyLessThanOrEqual(usage, 1 + 1e-12);
            testCase.verifyEqual(force.combined_usage, 1, AbsTol=1e-12);
        end

        function testZeroLoadProducesZeroForce(testCase)
            tire = tire_load_sensitive_baseline();

            force = tire_force_from_slip(0.2, -0.2, 0, 0, tire);

            testCase.verifyEqual(force.Fx_N, 0, AbsTol=0);
            testCase.verifyEqual(force.Fy_N, 0, AbsTol=0);
        end

        function testForcesDissipateRelativeSlip(testCase)
            tire = tire_load_sensitive_baseline();
            slipRatio = 0.05;
            slipAngle_rad = 0.08;
            force = tire_force_from_slip( ...
                slipRatio, slipAngle_rad, 750, 0, tire);

            longitudinalRelativeSpeed_mps = -slipRatio;
            lateralRelativeSpeed_mps = slipAngle_rad;
            dissipatedPower = force.Fx_N * longitudinalRelativeSpeed_mps ...
                + force.Fy_N * lateralRelativeSpeed_mps;

            testCase.verifyLessThan(dissipatedPower, 0);
        end

        function testNaNStiffnessIsRejected(testCase)
            tire = tire_load_sensitive_baseline();
            tire.slip_force.longitudinal_stiffness_ref_N = NaN;

            action = @() tire_force_from_slip(0, 0, 750, 0, tire);

            testCase.verifyError(action, "QSSLTS:TireForceParameters");
        end
    end
end
