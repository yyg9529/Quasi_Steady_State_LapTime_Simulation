function rules = fsg_2026_ev_rules()
%FSG_2026_EV_RULES Return Formula Student 2026 EV tractive-system limits.

rules.name = "Formula Student Rules";
rules.season = 2026;
rules.version = "1.1";
rules.source = "Formula Student Rules 2026 v1.1";
rules.source_url = "https://www.formulastudent.de/fileadmin/user_upload/all/2026/rules/FS-Rules_2026_v1.1.pdf";

rules.max_ts_voltage_V = 600;
rules.max_ts_power_W = 80e3;
rules.max_ts_current_A = 500;
rules.ts_limit_location = "TSAC outlet";
rules.ts_current_type = "DC";

rules.regen_allowed = true;
rules.regen_enabled_in_model = false;

rules.rule_ids.power = "EV2.2.1";
rules.rule_ids.current = "EV2.2.2";
rules.rule_ids.voltage = "EV4.1.1";
end
