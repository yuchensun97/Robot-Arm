function q_new = shorterD(qrand, q_clostest, minD, max)
   q_new = [0 0];
   
   % Steer towards qn with maximum step size of eps
   if minD >= max
       q_new(1) = q_clostest(1) + (qrand(1)-q_clostest(1))*max/minD;
       q_new(2) = q_clostest(2) + (qrand(2)-q_clostest(2))*max/minD;
   else
       q_new(1) = qrand(1);
       q_new(2) = qrand(2);
   end   
end