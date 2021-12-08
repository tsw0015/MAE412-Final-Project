% Function to decode range information transmitted using SCIP2.0 protocol.
% Works for only two bit encoding.
% Author- Shikhar Shrestha, IIT Bhubaneswar. Modified by Gu
function rangeval=decodeSCIP(rangeenc)
% Check for 2 or 3 Character Encoding
% if rangeenc(1)=='0' && rangeenc(2)=='0' && rangeenc(3)=='0'
%     rangeval=0;
%     return;
% end
% if rangeenc(1)=='0'
    dig1sub=(rangeenc(2)-'!')-15;    % dig1=((rangeenc(2)-'!')+33); dig1sub=dig1-48;
    dig2sub=(rangeenc(3)-'!')-15;
    dig1bin=d2b2(dig1sub);
    dig2bin=d2b2(dig2sub);
    rangeval=b2d2([dig1bin dig2bin]);
%    temp=[dig1bin dig2bin]
%     return;
% else
%     dig1sub=(rangeenc(1)-'!')-15;
%     dig2sub=(rangeenc(2)-'!')-15;
%     dig3sub=(rangeenc(3)-'!')-15;
%     dig1bin=d2b(dig1sub,6);
%     dig2bin=d2b(dig2sub,6);
%     dig3bin=d2b(dig3sub,6);
%     rangeval=b2d([dig1bin dig2bin dig3bin])
%     return;
    
% end
    
end