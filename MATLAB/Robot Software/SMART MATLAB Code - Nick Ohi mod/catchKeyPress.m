function catchKeyPress()
    global fig
    global keyboardStruct;
    set(fig,'KeyPressFcn',@KeyPressCb);
    set(fig,'KeyReleaseFcn',@KeyReleaseCb);
    function y = KeyPressCb(~,evnt)
        %fprintf('key pressed: %s\n',evnt.Key);
        if strcmp(evnt.Key,'rightarrow')==1
            keyboardStruct.leftSpeed = keyboardStruct.maxSpeed;
            keyboardStruct.rightSpeed = -keyboardStruct.maxSpeed;
            keyboardStruct.quit = 0;
        elseif strcmp(evnt.Key, 'leftarrow')==1
            keyboardStruct.leftSpeed = -keyboardStruct.maxSpeed;
            keyboardStruct.rightSpeed = keyboardStruct.maxSpeed;
            keyboardStruct.quit = 0;
        elseif strcmp(evnt.Key,'uparrow')==1
            keyboardStruct.leftSpeed = keyboardStruct.maxSpeed;
            keyboardStruct.rightSpeed = keyboardStruct.maxSpeed;
            keyboardStruct.quit = 0;
        elseif strcmp(evnt.Key,'downarrow')==1
            keyboardStruct.leftSpeed = -keyboardStruct.maxSpeed;
            keyboardStruct.rightSpeed = -keyboardStruct.maxSpeed;
            keyboardStruct.quit = 0;
        elseif strcmp(evnt.Key,'q')==1
            keyboardStruct.leftSpeed = 0;
            keyboardStruct.rightSpeed = 0;
            keyboardStruct.quit = 1;
        end
    end

    function y = KeyReleaseCb(~,evnt)
        %fprintf('key released')
        keyboardStruct.leftSpeed = 0;
        keyboardStruct.rightSpeed = 0;
    end
end

