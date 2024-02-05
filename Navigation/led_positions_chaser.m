function [xnc] = led_positions_chaser(d_initial, Az, El, alpha, beta, gamma, relative_state_chaser)

    % Position of the LED on the camera sensor
    xy1 = d_initial*((cosd(Az+gamma)*cosd(alpha))-(sind(El+beta)*sind(Az+gamma)*sind(alpha)));
    
    xz1 = -d_initial*cosd(El+beta)*sind(alpha);
    
    xy2 = d_initial*((cosd(Az+gamma)*sind(alpha))+(sind(El+beta)*sind(Az+gamma)*cosd(alpha)));
    
    xz2 = d_initial*cosd(El+beta)*cosd(alpha);
    
    xy3 = -d_initial*((cosd(Az+gamma)*cosd(alpha))-(sind(El+beta)*sind(Az+gamma)*sind(alpha)));
    
    xz3 = d_initial*cosd(El+beta)*sind(alpha);
    
    xy4 = -d_initial*((cosd(Az+gamma)*sind(alpha))+(sind(El+beta)*sind(Az+gamma)*cosd(alpha)));
    
    xz4 = -d_initial*cosd(El+beta)*cosd(alpha);
    
    xy5 = d_initial*(cosd(El+beta)*sind(Az+gamma));
    
    xz5 = -d_initial*sind(El+beta);

    led1_pos_chaser = [xy1; relative_state_chaser(2,:); xz1];

    led2_pos_chaser = [xy2; relative_state_chaser(2,:); xz2];

    led3_pos_chaser = [xy3; relative_state_chaser(2,:); xz3];

    led4_pos_chaser = [xy4; relative_state_chaser(2,:); xz4];

    led5_pos_chaser = [xy5; relative_state_chaser(2,:); xz5];

    % LED Pattern Matrix on the Chaser Frame 
    xnc = [led1_pos_chaser, led2_pos_chaser, led3_pos_chaser, led4_pos_chaser, led5_pos_chaser];

end