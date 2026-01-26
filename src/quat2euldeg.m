function [roll, pitch, head] = quat2euldeg(quat)
    q = quat;
    q = q / norm(q);
    
    qx = q(1);
    qy = q(2);
    qz = q(3);
    qw = q(4);

    roll  = atan2(2*(qw*qx + qy*qz), 1 - 2*(qx^2 + qy^2)) * 180/pi;
    pitch = asin (2*(qw*qy - qz*qx)) * 180/pi;
    head = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy^2 + qz^2)) * 180/pi;

end