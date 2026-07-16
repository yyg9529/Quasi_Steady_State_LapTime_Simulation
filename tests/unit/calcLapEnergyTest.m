classdef calcLapEnergyTest < matlab.unittest.TestCase
    properties
        Vehicle
        Aero
        Powertrain
        Endurance
    end

    methods (TestClassSetup)
        function buildFixture(testCase)
            projectRoot = fileparts(fileparts(fileparts(mfilename("fullpath"))));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(projectRoot, "src"), IncludingSubfolders=true));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(projectRoot, "data"), IncludingSubfolders=true));

            testCase.Vehicle = vehicle_baseline();
            testCase.Aero = struct("enabled", false);
            testCase.Powertrain = powertrain_emrax228_hvcc_demo();
            testCase.Endurance = struct("num_laps", 20, ...
                "safety_factor", 1.10);
        end
    end

    methods (Test)
        function testEnergyIntegrationConstantPower(testCase)
            track = calcLapEnergyTest.makeOpenTrack([10; 10]);
            speed_mps = 10 * ones(3, 1);
            aero = struct("enabled", true, "rho_kgpm3", 1, ...
                "CDA_m2", 2, "CLA_m2", 0, ...
                "front_downforce_frac", 0.5);
            powertrain = testCase.Powertrain;
            powertrain.drivetrain_efficiency = 0.8;
            powertrain.inverter.eta_const = 0.9;
            powertrain.motor.eta_const = 0.9;
            powertrain.battery.eta_discharge = 0.95;
            powertrain.battery.P_ts_aux_W = 400;
            expectedWheelPower_W = 1000;
            expectedTsPower_W = expectedWheelPower_W / (0.8 * 0.9 * 0.9) + 400;
            expectedTsEnergy_kWh = expectedTsPower_W * 2 / 3.6e6;

            energy = calc_lap_energy(track, speed_mps, ...
                testCase.Vehicle, aero, powertrain, testCase.Endurance);

            testCase.verifyEqual(energy.segment_wheel_power_W, ...
                expectedWheelPower_W * ones(2, 1), AbsTol=1e-10);
            testCase.verifyEqual(energy.segment_tsac_power_W, ...
                expectedTsPower_W * ones(2, 1), AbsTol=1e-10);
            testCase.verifyEqual(energy.E_lap_ts_kWh, ...
                expectedTsEnergy_kWh, AbsTol=1e-12);
            testCase.verifyEqual(energy.E_lap_stored_kWh, ...
                expectedTsEnergy_kWh / 0.95, AbsTol=1e-12);
        end

        function testClosedTrackEnergyIncludesLastSegment(testCase)
            closedTrack = calcLapEnergyTest.makeClosedTrack([10; 10; 10]);
            openTrack = calcLapEnergyTest.makeOpenTrack([10; 10]);
            speed_mps = 10 * ones(3, 1);
            powertrain = calcLapEnergyTest.makeLosslessPowertrain( ...
                testCase.Powertrain, 3600);

            closed = calc_lap_energy(closedTrack, speed_mps, ...
                testCase.Vehicle, testCase.Aero, powertrain, ...
                testCase.Endurance);
            open = calc_lap_energy(openTrack, speed_mps, ...
                testCase.Vehicle, testCase.Aero, powertrain, ...
                testCase.Endurance);

            testCase.verifySize(closed.segment_energy_ts_kWh, [3 1]);
            testCase.verifySize(open.segment_energy_ts_kWh, [2 1]);
            testCase.verifyEqual(closed.E_lap_ts_kWh, 0.003, ...
                AbsTol=1e-12);
            testCase.verifyEqual(open.E_lap_ts_kWh, 0.002, ...
                AbsTol=1e-12);
            testCase.verifyEqual(closed.cumulative_energy_ts_kWh, ...
                [0; 0.001; 0.002; 0.003], AbsTol=1e-12);
        end

        function testAuxEnergyAppliesForFullLap(testCase)
            track = calcLapEnergyTest.makeClosedTrack([7; 11; 13]);
            speed_mps = 7 * ones(3, 1);
            auxPower_W = 720;
            powertrain = calcLapEnergyTest.makeLosslessPowertrain( ...
                testCase.Powertrain, auxPower_W);
            expectedLapTime_s = sum(track.ds_m) / speed_mps(1);
            expectedEnergy_kWh = auxPower_W * expectedLapTime_s / 3.6e6;

            energy = calc_lap_energy(track, speed_mps, ...
                testCase.Vehicle, testCase.Aero, powertrain, ...
                testCase.Endurance);

            testCase.verifyEqual(energy.segment_tsac_power_W, ...
                auxPower_W * ones(3, 1), AbsTol=1e-12);
            testCase.verifyEqual(energy.E_lap_ts_kWh, ...
                expectedEnergy_kWh, AbsTol=1e-12);
        end

        function testEnduranceBatteryRequirement(testCase)
            track = calcLapEnergyTest.makeClosedTrack([10; 10; 10]);
            speed_mps = 10 * ones(3, 1);
            powertrain = calcLapEnergyTest.makeLosslessPowertrain( ...
                testCase.Powertrain, 3600);
            powertrain.battery.eta_discharge = 0.8;
            expectedLapStored_kWh = 0.003 / 0.8;
            expectedEndurance_kWh = expectedLapStored_kWh ...
                * testCase.Endurance.num_laps ...
                * testCase.Endurance.safety_factor;
            expectedRequired_kWh = expectedEndurance_kWh ...
                / (powertrain.battery.SOC_init ...
                - powertrain.battery.SOC_min);
            expectedSocEnd = powertrain.battery.SOC_init ...
                - expectedEndurance_kWh / powertrain.battery.E_nominal_kWh;

            energy = calc_lap_energy(track, speed_mps, ...
                testCase.Vehicle, testCase.Aero, powertrain, ...
                testCase.Endurance);

            testCase.verifyEqual(energy.E_endurance_stored_kWh, ...
                expectedEndurance_kWh, AbsTol=1e-12);
            testCase.verifyEqual(energy.E_nominal_required_kWh, ...
                expectedRequired_kWh, AbsTol=1e-12);
            testCase.verifyEqual(energy.SOC_end_estimated, ...
                expectedSocEnd, AbsTol=1e-12);
            testCase.verifyTrue(energy.can_finish_endurance_estimated);
        end

        function testEnduranceFeasibilityUsesSocBoundaryTolerance(testCase)
            track = calcLapEnergyTest.makeClosedTrack([10; 10]);
            speed_mps = 10 * ones(2, 1);
            endurance = struct("num_laps", 1, "safety_factor", 1);
            powertrain = calcLapEnergyTest.makeLosslessPowertrain( ...
                testCase.Powertrain, 1530);
            powertrain.battery.E_nominal_kWh = 0.001;

            boundary = calc_lap_energy(track, speed_mps, ...
                testCase.Vehicle, testCase.Aero, powertrain, endurance);
            powertrain.battery.P_ts_aux_W = 1530 * (1 + 1e-9);
            overdrawn = calc_lap_energy(track, speed_mps, ...
                testCase.Vehicle, testCase.Aero, powertrain, endurance);

            testCase.verifyEqual(boundary.SOC_end_estimated, ...
                powertrain.battery.SOC_min, AbsTol=1e-12);
            testCase.verifyTrue(boundary.can_finish_endurance_estimated);
            testCase.verifyFalse(overdrawn.can_finish_endurance_estimated);
        end

        function testSegmentReconstructionAndNoRegen(testCase)
            track = calcLapEnergyTest.makeOpenTrack([10; 10]);
            speed_mps = [10; sqrt(120); 10];
            vehicle = testCase.Vehicle;
            vehicle.mass.total_kg = 100;
            powertrain = calcLapEnergyTest.makeLosslessPowertrain( ...
                testCase.Powertrain, 0);
            expectedMeanSpeed_mps = 0.5 * (10 + sqrt(120));

            energy = calc_lap_energy(track, speed_mps, vehicle, ...
                testCase.Aero, powertrain, testCase.Endurance);

            testCase.verifyEqual(energy.segment_ax_mps2, [1; -1], ...
                AbsTol=1e-12);
            testCase.verifyEqual(energy.segment_speed_mean_mps, ...
                expectedMeanSpeed_mps * ones(2, 1), AbsTol=1e-12);
            testCase.verifyEqual(energy.segment_drive_force_N, ...
                [100; 0], AbsTol=1e-10);
            testCase.verifyEqual(energy.segment_wheel_power_W, ...
                [100 * expectedMeanSpeed_mps; 0], AbsTol=1e-10);
        end

        function testOutputShapesAndCumulativeOrigins(testCase)
            track = calcLapEnergyTest.makeClosedTrack([5; 7; 9]);
            speed_mps = 8 * ones(3, 1);

            energy = calc_lap_energy(track, speed_mps, ...
                testCase.Vehicle, testCase.Aero, testCase.Powertrain, ...
                testCase.Endurance);

            testCase.verifySize(energy.segment_ax_mps2, [3 1]);
            testCase.verifySize(energy.segment_speed_mean_mps, [3 1]);
            testCase.verifySize(energy.segment_drive_force_N, [3 1]);
            testCase.verifySize(energy.segment_wheel_power_W, [3 1]);
            testCase.verifySize(energy.segment_tsac_power_W, [3 1]);
            testCase.verifySize(energy.segment_energy_ts_kWh, [3 1]);
            testCase.verifySize(energy.segment_energy_stored_kWh, [3 1]);
            testCase.verifySize(energy.cumulative_energy_ts_kWh, [4 1]);
            testCase.verifySize(energy.cumulative_energy_stored_kWh, [4 1]);
            testCase.verifyEqual(energy.cumulative_energy_ts_kWh(1), ...
                0, AbsTol=0);
            testCase.verifyEqual(energy.cumulative_energy_stored_kWh(1), ...
                0, AbsTol=0);
        end

        function testNonintegerLapCountErrors(testCase)
            track = calcLapEnergyTest.makeOpenTrack([10; 10]);
            endurance = struct("num_laps", 2.5, "safety_factor", 1.1);

            action = @() calc_lap_energy(track, 10 * ones(3, 1), ...
                testCase.Vehicle, testCase.Aero, testCase.Powertrain, ...
                endurance);

            testCase.verifyError(action, "QSSLTS:EnduranceConfig");
        end

        function testSafetyFactorBelowOneErrors(testCase)
            track = calcLapEnergyTest.makeOpenTrack([10; 10]);
            endurance = struct("num_laps", 2, "safety_factor", 0.9);

            action = @() calc_lap_energy(track, 10 * ones(3, 1), ...
                testCase.Vehicle, testCase.Aero, testCase.Powertrain, ...
                endurance);

            testCase.verifyError(action, "QSSLTS:EnduranceConfig");
        end
    end

    methods (Static, Access=private)
        function track = makeOpenTrack(ds_m)
            track.ds_m = ds_m(:);
            track.s_m = [0; cumsum(track.ds_m)];
            track.kappa_1pm = zeros(numel(track.s_m), 1);
            track.is_closed = false;
        end

        function track = makeClosedTrack(ds_m)
            track.ds_m = ds_m(:);
            track.s_m = [0; cumsum(track.ds_m(1:end-1))];
            track.kappa_1pm = zeros(numel(track.s_m), 1);
            track.is_closed = true;
        end

        function powertrain = makeLosslessPowertrain(powertrain, auxPower_W)
            powertrain.drivetrain_efficiency = 1;
            powertrain.inverter.eta_const = 1;
            powertrain.motor.eta_const = 1;
            powertrain.battery.eta_discharge = 1;
            powertrain.battery.P_ts_aux_W = auxPower_W;
        end
    end
end
