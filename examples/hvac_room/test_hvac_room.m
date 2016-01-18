% deterministic case
HR = hvac_room();
HR = init_control(HR, 12, 0, 40);
HR.run_deterministic(HR.controller);


% Adversarial (reactive) case
HR2 = hvac_room();
HR2 = init_control(HR2, 12, 5, 40);
HR2.run_adversarial(HR2.controller, HR2.adversary);