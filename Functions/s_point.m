function s_i = s_point(v,PB,s,psi)

if v(1) == 0
    s_i(2) = PB(2)+sign(psi)*v(2)*s/norm(v);
    s_i(1) = (v(1)/v(2))*(s_i(2)-PB(2))+PB(1);
    s_i(3) = (v(3)/v(2))*(s_i(2)-PB(2))+PB(3);
else
    s_i(1) = PB(1)+v(1)*s/norm(v);
    s_i(2) = (v(2)/v(1))*(s_i(1)-PB(1))+PB(2);
    s_i(3) = (v(3)/v(1))*(s_i(1)-PB(1))+PB(3);
end

end