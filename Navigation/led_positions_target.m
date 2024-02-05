function [xnt] = led_positions_target(d_initial)

    led1_pos_target = [0; -d_initial; 0]; % LED-1
    led2_pos_target = [0; 0; -d_initial]; % LED-2
    led3_pos_target = [0; d_initial; 0]; % LED-3
    led4_pos_target = [0; 0; d_initial]; % LED-4
    led5_pos_target = [d_initial; 0; 0]; % LED-5
    
    % LED Pattern Matrix on the Target Frame
    xnt = [led1_pos_target, led2_pos_target, led3_pos_target, led4_pos_target, led5_pos_target];

end