HR1 = hvac_room_det();
HR1 = init_control(HR1,12,5);
HR1.h=[];
%HR1.run_open_loop(HR1.controller);
%HR1.run_open_loop_adv(HR1.controller, HR1.adversary);
%HR1= HR1.run_deterministic(HR1.controller);
HR1= HR1.run_adversarial(HR1.controller, HR1.adversary);


HR2 = hvac_room();
HR2 = init_control(HR2,10);
%HR2.run_adversarial(HR2.controller, HR2.adversary)

