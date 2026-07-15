classdef fsgRulesTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addProjectDataPath(testCase)
            projectRoot = fileparts(fileparts(fileparts(mfilename("fullpath"))));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(projectRoot, "data"), IncludingSubfolders=true));
        end
    end

    methods (Test)
        function testDocumentIdentity(testCase)
            expectedUrl = "https://www.formulastudent.de/fileadmin/user_upload/all/2026/rules/FS-Rules_2026_v1.1.pdf";

            rules = fsg_2026_ev_rules();

            testCase.verifyEqual(rules.name, "Formula Student Rules");
            testCase.verifyEqual(rules.season, 2026, AbsTol=0);
            testCase.verifyEqual(rules.version, "1.1");
            testCase.verifyEqual(rules.source, ...
                "Formula Student Rules 2026 v1.1");
            testCase.verifyEqual(rules.source_url, expectedUrl);
        end

        function testTsacOutletElectricalLimits(testCase)
            rules = fsg_2026_ev_rules();

            testCase.verifyEqual(rules.max_ts_voltage_V, 600, AbsTol=0);
            testCase.verifyEqual(rules.max_ts_power_W, 80e3, AbsTol=0);
            testCase.verifyEqual(rules.max_ts_current_A, 500, AbsTol=0);
            testCase.verifyEqual(rules.ts_limit_location, "TSAC outlet");
            testCase.verifyEqual(rules.ts_current_type, "DC");
        end

        function testRegenRuleAndModelPolicyAreSeparate(testCase)
            rules = fsg_2026_ev_rules();

            testCase.verifyTrue(rules.regen_allowed);
            testCase.verifyFalse(rules.regen_enabled_in_model);
        end

        function testRuleTraceability(testCase)
            rules = fsg_2026_ev_rules();

            testCase.verifyEqual(rules.rule_ids.power, "EV2.2.1");
            testCase.verifyEqual(rules.rule_ids.current, "EV2.2.2");
            testCase.verifyEqual(rules.rule_ids.voltage, "EV4.1.1");
        end

        function testLegacyLimitAliasesAreAbsent(testCase)
            rules = fsg_2026_ev_rules();

            testCase.verifyFalse(isfield(rules, "max_voltage_V"));
            testCase.verifyFalse(isfield(rules, "max_power_W"));
            testCase.verifyFalse(isfield(rules, "max_current_A"));
        end
    end
end
