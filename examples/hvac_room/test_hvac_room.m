HR1 = hvac_room;
HR1 = init_control(HR1,12,0, 0);
HR1= HR1.run_adversarial(HR1.controller, HR1.adversary)
save2pdf('hvac_det2.pdf', gcf)

HR2 = hvac_room();
HR2 = init_control(HR2,12,5, 400);
HR2.run_adversarial(HR2.controller, HR2.adversary)
save2pdf('hvac_nondet2.pdf', gcf)
