angs = [1 3 5 7 9 11 13 15];
angs = angs * 3.1415926 / 180;

for i=1:8,
  radius = cot(angs(i));
  disp(sprintf('radius: %0.3f', radius));
  disp(sprintf('dis: %0.3f', radius*2/180*3.14));
  #disp(angs(i));
end
