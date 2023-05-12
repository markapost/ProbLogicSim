set(0, 'defaultlinelinewidth', 5);
set(0, 'defaultaxesfontname', 'Times-Roman')
set(0, 'defaultaxesfontsize', 18)
set(0, 'defaulttextfontname', 'Times-Roman')
set(0, 'defaulttextfontsize', 18)

figure(1);
plot(lt, lC); title("Collision"); xlabel("time {/Times-Italic t}"); ylabel("logic value"); legend({'F'}, 'location', 'west'); ylim([-0.1,1.1]);

figure(2);
subplot(2,1,1);
plot(lt, lT(:,1), lt, lT(:,2), lt, lT(:,3), lt, lT(:,4), lt, lT(:,5)); title("Target"); xlabel("time {/Times-Italic t}"); ylabel("logic value"); legend({'L','FL','F','FR','R'}, 'location', 'west'); ylim([-0.1,1.1]);
subplot(2,1,2);
plot(bt, bT(:,1), bt, bT(:,2), bt, bT(:,3), bt, bT(:,4), bt, bT(:,5)); xlabel("time {/Times-Italic t}"); ylabel("probability {/Times-Italic p}"); legend({'L','FL','F','FR','R'}, 'location', 'west'); ylim([-0.1,1.1]);

figure(3);
subplot(2,1,1);
plot(lt, lnO(:,1), lt, lnO(:,2), lt, lnO(:,3), lt, lnO(:,4), lt, lnO(:,5)); title("not-Obstacle"); xlabel("time {/Times-Italic t}"); ylabel("logic value"); legend({'L','FL','F','FR','R'}, 'location', 'west'); ylim([-0.1,1.1]);
subplot(2,1,2);
plot(bt, bnO(:,1), bt, bnO(:,2), bt, bnO(:,3), bt, bnO(:,4), bt, bnO(:,5)); xlabel("time {/Times-Italic t}"); ylabel("probability {/Times-Italic p}"); legend({'L','FL','F','FR','R'}, 'location', 'west'); ylim([-0.1,1.1]);

figure(4);
subplot(2,1,1);
plot(lt, lS); title("Speed"); xlabel("time {/Times-Italic t}"); ylabel("logic value"); legend({'F'}, 'location', 'west'); ylim([-0.1,1.1]);
subplot(2,1,2);
plot(bt, bS); xlabel("time {/Times-Italic t}"); ylabel("probability {/Times-Italic p}"); legend({'F'}, 'location', 'west'); ylim([-0.1,1.1]);

figure(5);
subplot(2,1,1);
plot(lt, lD(:,1), lt, lD(:,2), lt, lD(:,3), lt, lD(:,4), lt, lD(:,5)); title("Direction"); xlabel("time {/Times-Italic t}"); ylabel("logic value"); legend({'L','FL','F','FR','R'}, 'location', 'northeast'); ylim([-0.1,1.1]);
subplot(2,1,2);
plot(bt, bD(:,1), bt, bD(:,2), bt, bD(:,3), bt, bD(:,4), bt, bD(:,5)); xlabel("time {/Times-Italic t}"); ylabel("probability {/Times-Italic p}"); legend({'L','FL','F','FR','R'}, 'location', 'northeast'); ylim([-0.1,1.1]);

figure(6);
subplot(2,1,1);
plot(lt, lMD(:,1), lt, lMD(:,2)); title("Turn"); xlabel("time {/Times-Italic t}"); ylabel("logic value"); legend({'L','R'}, 'location', 'northeast'); ylim([-0.1,1.1]);
subplot(2,1,2);
plot(bt, bMD(:,1), bt, bMD(:,2)); xlabel("time {/Times-Italic t}"); ylabel("probability {/Times-Italic p}"); legend({'L','R'}, 'location', 'northeast'); ylim([-0.1,1.1]);

figure(7);
subplot(2,1,1);
plot(lt, lMF(:,1), lt, lMF(:,2)); title("Motor Forward"); xlabel("time {/Times-Italic t}"); ylabel("logic value"); legend({'L','R'}, 'location', 'east'); ylim([-0.1,1.1]);
subplot(2,1,2);
plot(bt, bMF(:,1), bt, bMF(:,2)); xlabel("time {/Times-Italic t}"); ylabel("probability {/Times-Italic p}"); legend({'L','R'}, 'location', 'northeast'); ylim([-0.1,1.1]);

figure(8);
subplot(2,1,1);
plot(lt, lMB(:,1), lt, lMB(:,2)); title("Motor Reverse"); xlabel("time {/Times-Italic t}"); ylabel("logic value"); legend({'L','R'}, 'location', 'east'); ylim([-0.1,1.1]);
subplot(2,1,2);
plot(bt, bMB(:,1), bt, bMB(:,2)); xlabel("time {/Times-Italic t}"); ylabel("probability {/Times-Italic p}"); legend({'L','R'}, 'location', 'northeast'); ylim([-0.1,1.1]);

saveas (1, 'collision.eps', 'epsc');
saveas (2, 'target.eps', 'epsc');
saveas (3, 'notobstacle.eps', 'epsc');
saveas (4, 'speed.eps', 'epsc');
saveas (5, 'direction.eps', 'epsc');
saveas (6, 'turn.eps', 'epsc');
saveas (7, 'motorf.eps', 'epsc');
saveas (8, 'motorb.eps', 'epsc');

